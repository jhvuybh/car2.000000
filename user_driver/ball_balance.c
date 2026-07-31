#include "ball_balance.h"

#include "delay.h"
#include "system_time.h"
#include "zdt_stepper.h"

static uint16_t target_position = BALL_TARGET_CENTER;
static k230_ball_result_t last_result = {0U, false, 0U, 0U, 0U};
static bool have_result = false;
static bool filter_initialized = false;
static float filtered_position = (float)BALL_TARGET_CENTER;
static float previous_filtered_position = (float)BALL_TARGET_CENTER;
static uint32_t previous_measurement_ms = 0U;
static uint32_t last_command_ms = 0U;
static int32_t last_command_pulses = 0;
static bool control_enabled = false;
static ball_balance_state_t controller_state = BALL_BALANCE_DISABLED;

volatile uint16_t ball_debug_position = BALL_TARGET_CENTER;
volatile uint8_t ball_debug_position_valid = 0U;
volatile int32_t ball_debug_command_pulses = 0;
volatile uint32_t ball_debug_command_count = 0U;

static float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t clamp_actuator(float command)
{
    if (command > (float)BALL_ACTUATOR_LIMIT_PULSES) {
        command = (float)BALL_ACTUATOR_LIMIT_PULSES;
    } else if (command < -(float)BALL_ACTUATOR_LIMIT_PULSES) {
        command = -(float)BALL_ACTUATOR_LIMIT_PULSES;
    }
    return (int32_t)command;
}

static void reset_controller_history(void)
{
    filter_initialized = false;
    filtered_position = (float)BALL_TARGET_CENTER;
    previous_filtered_position = filtered_position;
    previous_measurement_ms = 0U;
}

bool ball_balance_init(void)
{
    bool ok;

    k230_init();
    zdt_stepper_init();

    /* Keep about 10 ms between configuration commands, as recommended by the
     * motor vendor examples.  These delays occur only once at startup. */
    ok = zdt_stepper_enable(true);
    delay_ms(10U);
    /* 平衡控制使用FD标准绝对位置帧，每条命令自带全部运动参数，
     * 不再依赖F1快速位置配置是否被驱动器成功保存。 */

    target_position = BALL_TARGET_CENTER;
    ball_debug_position = BALL_TARGET_CENTER;
    ball_debug_position_valid = 0U;
    ball_debug_command_pulses = 0;
    ball_debug_command_count = 0U;
    reset_controller_history();
    /* 上电后只接收K230坐标，不在任务选择界面控制步进电机。 */
    control_enabled = false;
    controller_state = BALL_BALANCE_DISABLED;
    return ok;
}

bool ball_balance_calibrate_level_zero(void)
{
    bool ok;

    ok = zdt_stepper_stop();
    delay_ms(10U);
    ok = zdt_stepper_set_current_position_zero() && ok;
    delay_ms(10U);
    ok = zdt_stepper_move_absolute(
        0,
        BALL_STEPPER_SPEED_RPM,
        BALL_STEPPER_ACCELERATION
    ) && ok;

    last_command_pulses = 0;
    last_command_ms = SystemTime_GetMs();
    reset_controller_history();
    /* KEY2标定只执行一次回零动作，不自动打开钢球闭环。 */
    controller_state = control_enabled ?
        BALL_BALANCE_LEVEL_HOLD : BALL_BALANCE_DISABLED;
    return ok;
}

void ball_balance_set_target(uint16_t new_target)
{
    if (new_target > K230_BALL_POSITION_MAX) {
        new_target = K230_BALL_POSITION_MAX;
    }
    target_position = new_target;
}

uint16_t ball_balance_get_target(void)
{
    return target_position;
}

void ball_balance_enable_control(void)
{
    control_enabled = true;
    last_command_ms = SystemTime_GetMs();
    reset_controller_history();
    controller_state = BALL_BALANCE_LEVEL_HOLD;
}

void ball_balance_disable_control(void)
{
    /* 停止当前运动，但保留驱动器使能和已经标定的机械零点。 */
    control_enabled = false;
    zdt_stepper_stop();
    ball_debug_command_pulses = 0;
    reset_controller_history();
    controller_state = BALL_BALANCE_DISABLED;
}

bool ball_balance_update(void)
{
    k230_ball_result_t result;
    uint32_t now = SystemTime_GetMs();
    uint32_t dt_ms;
    float velocity = 0.0f;
    float error;
    float command;
    int32_t command_pulses;
    int32_t command_difference;

    zdt_stepper_poll();

    if (!k230_get_ball_result(&result)) {
        /* 待机时即使K230掉线，也不能把步进电机自动拉回零点。 */
        if (!control_enabled) {
            controller_state = BALL_BALANCE_DISABLED;
            return false;
        }
        if (!k230_is_online()) {
            if (controller_state != BALL_BALANCE_NO_VISION) {
                zdt_stepper_move_absolute(
                    0,
                    BALL_STEPPER_SPEED_RPM,
                    BALL_STEPPER_ACCELERATION
                );
                last_command_pulses = 0;
                reset_controller_history();
            }
            controller_state = BALL_BALANCE_NO_VISION;
        }
        return false;
    }

    last_result = result;
    have_result = true;
    ball_debug_position_valid = result.valid ? 1U : 0U;

    /*
     * READY/任务选择界面仍接收并显示钢球坐标，但不向步进电机发送
     * 位置命令。闭环只由任务3、任务4显式开启。
     */
    if (!control_enabled) {
        if (result.valid) {
            ball_debug_position = result.position;
        }
        controller_state = BALL_BALANCE_DISABLED;
        return true;
    }

    if (!result.valid) {
        /* The camera is online but the ball is temporarily lost.  Level the
         * beam rather than continuing with an old unsafe tilt command. */
        if (controller_state != BALL_BALANCE_NO_VISION) {
            zdt_stepper_move_absolute(
                0,
                BALL_STEPPER_SPEED_RPM,
                BALL_STEPPER_ACCELERATION
            );
            last_command_pulses = 0;
        }
        reset_controller_history();
        controller_state = BALL_BALANCE_NO_VISION;
        return true;
    }

    ball_debug_position = result.position;

    if (!filter_initialized) {
        filtered_position = (float)result.position;
        previous_filtered_position = filtered_position;
        previous_measurement_ms = now;
        filter_initialized = true;
    } else {
        previous_filtered_position = filtered_position;
        filtered_position += BALL_POSITION_FILTER_ALPHA *
            ((float)result.position - filtered_position);

        dt_ms = now - previous_measurement_ms;
        if (dt_ms >= 10U && dt_ms <= 200U) {
            velocity = (filtered_position - previous_filtered_position) *
                1000.0f / (float)dt_ms;
        }
        previous_measurement_ms = now;
    }

    error = (float)target_position - filtered_position;
    command = BALL_CONTROL_DIRECTION *
        (BALL_KP_PULSES_PER_POSITION * error -
         BALL_KD_PULSES_PER_POSITION_PER_S * velocity);
    command_pulses = clamp_actuator(command);
    ball_debug_command_pulses = command_pulses;
    command_difference = command_pulses - last_command_pulses;
    if (command_difference < 0) {
        command_difference = -command_difference;
    }

    if ((now - last_command_ms) >= BALL_COMMAND_MIN_INTERVAL_MS &&
        command_difference >= BALL_ACTUATOR_COMMAND_DEADBAND) {
        if (zdt_stepper_move_absolute(
                command_pulses,
                BALL_STEPPER_SPEED_RPM,
                BALL_STEPPER_ACCELERATION)) {
            last_command_pulses = command_pulses;
            last_command_ms = now;
            ball_debug_command_count++;
        }
    }

    controller_state = BALL_BALANCE_TRACKING;
    return true;
}

void ball_balance_hold_level(void)
{
    target_position = BALL_TARGET_CENTER;
    zdt_stepper_move_absolute(
        0,
        BALL_STEPPER_SPEED_RPM,
        BALL_STEPPER_ACCELERATION
    );
    last_command_pulses = 0;
    last_command_ms = SystemTime_GetMs();
    reset_controller_history();
    controller_state = control_enabled ?
        BALL_BALANCE_LEVEL_HOLD : BALL_BALANCE_DISABLED;
}

ball_balance_state_t ball_balance_get_state(void)
{
    return controller_state;
}

bool ball_balance_get_last_result(k230_ball_result_t *result)
{
    if (result == NULL || !have_result) {
        return false;
    }
    *result = last_result;
    return true;
}

bool ball_balance_is_at_target(uint16_t tolerance)
{
    float error;

    if (controller_state != BALL_BALANCE_TRACKING || !last_result.valid) {
        return false;
    }
    error = filtered_position - (float)target_position;
    return abs_float(error) <= (float)tolerance;
}
