#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "ball_balance.h"
#include "delay.h"
#include "gyro_angle_stop.h"
#include "huidu.h"
#include "hwt101.h"
#include "key.h"
#include "motor.h"
#include "oled.h"
#include "system_time.h"

/*
 * H-problem programs implemented here:
 *   P1: wireless ball video (the K230 RTSP program runs continuously)
 *   P2: clockwise lap, stop at A and freeze total time
 *   P3: stationary ball motion: center -> +5 cm -> -5 cm and stabilize
 *   P4: clockwise A -> B while holding the ball at center
 *   P5: clockwise lap through A while holding the ball at center
 *   P6: clockwise lap through A while holding the ball at its start position
 *
 * Keys (hardware RC debounce is already installed):
 *   KEY1: start; press again while running to abort
 *   KEY2: calibrate the current beam height as the level actuator zero
 *   KEY3: next program
 *   KEY4: previous program
 *
 * OLED is hardware I2C1: PB3=SDA, PB2=SCL.
 * HWT101 is hardware UART1: PA18=RX, PB6=TX, 115200 baud.
 */

/* 0：启用完整任务1~6，并初始化K230和步进电机平衡控制模块。
 * 1：仅锁定任务2，用于单独调试循迹。 */
#define PROGRAM_2_ONLY_TEST             0U
/* 1：初始化并持续轮询HWT101，同时在OLED上显示累计转角。 */
#define HWT101_ENABLE                   1U
/* 1：任务2使用“编码里程门槛 + HWT101累计转角”联合判断终点。 */
#define PROGRAM_2_ENABLE_GYRO_STOP      1U
#define PROGRAM_MIN                     1U
#define PROGRAM_MAX                     6U

/* 任务2的20秒是比赛评分要求，不作为强制停车条件。
 * 自动停车只由编码里程和累计转角联合触发，避免20秒时停在赛道中间。 */
#define PROGRAM_3_TIME_LIMIT_MS         5000U
#define PROGRAM_3_TIMEOUT_ENABLE        0U  /* 0=关闭5秒停止，调试完成后改回1 */
#define PROGRAM_4_TIME_LIMIT_MS         8000U
#define PROGRAM_5_6_TIME_LIMIT_MS      30000U

/* 任务2终点低速区：到此里程后将基础循迹速度限制到300mm/s。 */
#define PROGRAM_2_APPROACH_DISTANCE_MM  5900.0f
#define PROGRAM_2_APPROACH_SPEED_MM_S    280.0f

/*
 * 任务2自动停车参数（实车调试主要修改这两个值）：
 * 两个条件必须同时满足才下达停车命令，没有额外隐藏提前量。
 *   STOP_DISTANCE增大 -> 停得更晚；减小 -> 停得更早。
 *   STOP_ANGLE增大    -> 停得更晚；减小 -> 停得更早。
 * 当前角度330.875度沿用此前333.875度目标减3度惯性提前量的实测值。
 */
#define PROGRAM_2_STOP_DISTANCE_MM      6050.0f//终点距离
#define PROGRAM_2_STOP_ANGLE_DEG         325.0f//角度

/*
 * 任务4：AB标称1.5m。在B点冻结计时并降速，之后持续低速循迹，
 * 不再自动停车；需要结束时长按KEY1或切换任务。
 */
#define PROGRAM_4_B_PASS_DISTANCE_MM       1500.0f
#define PROGRAM_4_AFTER_B_SPEED_MM_S        160.0f

/*
 * 任务5/6：角度和里程同时达到时认为已经通过A，并立即冻结整圈时间。
 * 随后持续低速循迹，避免停在A基准线上，不再自动停车。
 * 当前通过A阈值采用此前整圈实测均值附近，可继续按实车修改。
 */
#define PROGRAM_5_6_A_PASS_DISTANCE_MM     6280.0f
#define PROGRAM_5_6_A_PASS_ANGLE_DEG        332.0f
#define PROGRAM_5_6_AFTER_A_SPEED_MM_S       160.0f

/* P3 must really settle at each target, not merely cross it once. */
#define PROGRAM_3_PLUS_STABLE_MS        150U
#define PROGRAM_3_MINUS_STABLE_MS       400U

#define OLED_TIME_UPDATE_MS             200U
#define OLED_TELEMETRY_UPDATE_MS        400U

/* 运行中连续按住KEY1达到此时间才中止，短暂电机干扰不会触发。 */
#define KEY1_ABORT_HOLD_MS               800U

typedef enum {
    RUN_READY = 0,
    RUN_ACTIVE,
    RUN_DONE,
    RUN_ERROR
} run_state_t;

typedef enum {
    P3_GO_TO_PLUS_5 = 0,
    P3_GO_TO_MINUS_5
} program3_phase_t;

typedef enum {
    COURSE_BEFORE_PASS = 0,
    COURSE_AFTER_PASS
} course_pass_phase_t;

static uint8_t selected_program = PROGRAM_MIN;
static run_state_t run_state = RUN_READY;
static program3_phase_t program3_phase = P3_GO_TO_PLUS_5;
static course_pass_phase_t program4_phase = COURSE_BEFORE_PASS;
static course_pass_phase_t lap_phase = COURSE_BEFORE_PASS;
static uint32_t run_start_ms = 0U;
static uint32_t run_stop_ms = 0U;
static bool run_timer_frozen = false;
static uint32_t target_stable_start_ms = 0U;
static uint32_t last_oled_time_ms = 0U;
static uint32_t last_oled_telemetry_ms = 0U;
static bool balance_hardware_ready = false;
static bool program2_hwt_warning_shown = false;
static uint16_t program6_target_position = BALL_TARGET_CENTER;

static void oled_draw_time(void);

static uint32_t elapsed_time_ms(void)
{
    if (run_state == RUN_ACTIVE && !run_timer_frozen) {
        return SystemTime_GetMs() - run_start_ms;
    }
    if (run_timer_frozen || run_state == RUN_DONE || run_state == RUN_ERROR) {
        return run_stop_ms - run_start_ms;
    }
    return 0U;
}

/* Freeze the scored time while allowing the car to continue past B/A. */
static void freeze_run_timer(void)
{
    if (!run_timer_frozen) {
        run_stop_ms = SystemTime_GetMs();
        run_timer_frozen = true;
        oled_draw_time();
    }
}

static void oled_draw_program(void)
{
    OLED_ShowString(0U, 0U, (u8 *)"PROGRAM:", 16U);
    OLED_ShowNum(64U, 0U, selected_program, 1U, 16U);
    OLED_RefreshArea(0U, 0U, 80U, 16U);
}

static void oled_draw_time(void)
{
    uint32_t elapsed = elapsed_time_ms();
    uint32_t seconds = elapsed / 1000U;
    uint32_t hundredths = (elapsed % 1000U) / 10U;

    if (seconds > 99U) {
        seconds = 99U;
        hundredths = 99U;
    }

    OLED_ShowString(0U, 16U, (u8 *)"TIME:", 16U);
    OLED_ShowNum(40U, 16U, seconds, 2U, 16U);
    OLED_ShowChar(56U, 16U, '.', 16U);
    OLED_ShowNum(64U, 16U, hundredths, 2U, 16U);
    OLED_ShowChar(80U, 16U, 's', 16U);
    OLED_RefreshArea(0U, 16U, 88U, 32U);
}

static void oled_draw_status(const char *text)
{
    /* Fifteen spaces overwrite the previous fixed-width status line. */
    OLED_ShowString(0U, 32U, (u8 *)"               ", 16U);
    OLED_ShowString(0U, 32U, (u8 *)text, 16U);
    OLED_RefreshArea(0U, 32U, 120U, 48U);
}

static void oled_draw_telemetry(void)
{
#if HWT101_ENABLE != 0U
    float angle;
#endif
    float distance_mm;
    uint32_t distance_integer_mm;
    uint32_t command_magnitude;
#if HWT101_ENABLE != 0U
    uint32_t angle_magnitude;
#endif

    /* 任务6显示当前钢球位置和启动时锁存的任意指定目标位置。 */
    if (selected_program == 6U) {
        OLED_ShowString(0U, 48U, (u8 *)"P:---- T:----   ", 16U);
        if (ball_debug_position_valid != 0U) {
            OLED_ShowNum(16U, 48U, ball_debug_position, 4U, 16U);
        }
        OLED_ShowNum(72U, 48U, program6_target_position, 4U, 16U);
        OLED_RefreshArea(0U, 48U, 120U, 64U);
        return;
    }

    /* 任务3显示钢球位置和步进目标，便于区分视觉/控制/驱动故障。 */
    if (selected_program == 3U) {
        OLED_ShowString(0U, 48U, (u8 *)"P:---- M:----   ", 16U);
        if (ball_debug_position_valid != 0U) {
            OLED_ShowNum(16U, 48U, ball_debug_position, 4U, 16U);
        }
        if (ball_debug_command_pulses >= 0) {
            OLED_ShowChar(72U, 48U, '+', 16U);
            command_magnitude = (uint32_t)ball_debug_command_pulses;
        } else {
            OLED_ShowChar(72U, 48U, '-', 16U);
            command_magnitude = (uint32_t)(-ball_debug_command_pulses);
        }
        if (command_magnitude > 999U) {
            command_magnitude = 999U;
        }
        OLED_ShowNum(80U, 48U, command_magnitude, 3U, 16U);
        OLED_RefreshArea(0U, 48U, 120U, 64U);
        return;
    }

    /*
     * Compact last line:
     *   D:5600 A:+0357
     * D is the encoder odometer in mm. A is the HWT101 accumulated yaw
     * since gyro_lap_stop_start(), not the raw yaw limited to -180..180.
     * The odometer is reset whenever a new program starts.
     * The accumulated value remains visible after stopping so the real angle
     * at finish line A can be recorded and used to tune
     * PROGRAM_2_STOP_ANGLE_DEG and PROGRAM_2_STOP_DISTANCE_MM.
     */
    OLED_ShowString(0U, 48U, (u8 *)"D:0000 A:-----  ", 16U);
    distance_mm = motor_odometer_get_mm();
    if (distance_mm < 0.0f) {
        distance_mm = 0.0f;
    }
    distance_integer_mm = (uint32_t)(distance_mm + 0.5f);
    if (distance_integer_mm > 9999U) {
        distance_integer_mm = 9999U;
    }
    OLED_ShowNum(16U, 48U, distance_integer_mm, 4U, 16U);

#if HWT101_ENABLE != 0U
    if (hwt101_is_online() != 0U) {
        angle = gyro_angle_stop_get_delta();
        if (angle >= 0.0f) {
            OLED_ShowChar(80U, 48U, '+', 16U);
            angle_magnitude = (uint32_t)(angle + 0.5f);
        } else {
            OLED_ShowChar(80U, 48U, '-', 16U);
            angle_magnitude = (uint32_t)(-angle + 0.5f);
        }
        if (angle_magnitude > 9999U) {
            angle_magnitude = 9999U;
        }
        OLED_ShowNum(88U, 48U, angle_magnitude, 4U, 16U);
    }
#endif
    OLED_RefreshArea(0U, 48U, 120U, 64U);
}

static void oled_show_ready_screen(void)
{
    OLED_Clear();
    oled_draw_program();
    oled_draw_time();
    oled_draw_status("STATE:READY");
    oled_draw_telemetry();
}

static void set_run_state(run_state_t new_state, const char *status)
{
    if ((new_state == RUN_DONE || new_state == RUN_ERROR) &&
        run_state == RUN_ACTIVE && !run_timer_frozen) {
        run_stop_ms = SystemTime_GetMs();
        run_timer_frozen = true;
    }
    run_state = new_state;
    oled_draw_time();
    oled_draw_status(status);
    oled_draw_telemetry();
}

static void abort_program(void)
{
    car_stop();
    huidu_soft_start_cancel();
    huidu_set_runtime_speed_cap(0.0f);
    motor_odometer_set_enabled(0U);
    gyro_angle_stop_cancel();
#if PROGRAM_2_ONLY_TEST == 0U
    /* 返回任务选择界面后关闭闭环，不再跟随K230坐标驱动电机。 */
    ball_balance_disable_control();
#endif
    run_start_ms = 0U;
    run_stop_ms = 0U;
    run_timer_frozen = false;
    target_stable_start_ms = 0U;
    program4_phase = COURSE_BEFORE_PASS;
    lap_phase = COURSE_BEFORE_PASS;
    program2_hwt_warning_shown = false;
    set_run_state(RUN_READY, "STATE:READY");
}

static void finish_program(bool success, const char *status)
{
    if (selected_program == 2U || selected_program >= 4U) {
        car_stop();
    }
    huidu_soft_start_cancel();
    huidu_set_runtime_speed_cap(0.0f);
    motor_odometer_set_enabled(0U);
    gyro_angle_stop_cancel();
    set_run_state(success ? RUN_DONE : RUN_ERROR, status);
}

static bool vision_ball_is_ready(void)
{
    k230_ball_result_t result;

    return balance_hardware_ready &&
        k230_is_online() &&
        ball_balance_get_last_result(&result) &&
        result.valid;
}

static void start_selected_program(void)
{
    k230_ball_result_t ball_result;

    car_stop();
    huidu_soft_start_cancel();
    huidu_set_runtime_speed_cap(0.0f);
    gyro_angle_stop_cancel();
    finish_detector_reset();
    motor_odometer_set_enabled(0U);
    motor_odometer_reset();
    target_stable_start_ms = 0U;
    program3_phase = P3_GO_TO_PLUS_5;
    program4_phase = COURSE_BEFORE_PASS;
    lap_phase = COURSE_BEFORE_PASS;
    program2_hwt_warning_shown = false;

    /* P3/P4 are not allowed to start blind because that could leave the beam
     * at an unsafe old tilt. */
    if (selected_program >= 3U && selected_program <= 6U &&
        !vision_ball_is_ready()) {
        run_start_ms = SystemTime_GetMs();
        run_stop_ms = run_start_ms;
        set_run_state(RUN_ERROR, "ERR:NO BALL");
        return;
    }

    run_start_ms = SystemTime_GetMs();
    run_stop_ms = run_start_ms;
    run_timer_frozen = false;
    last_oled_time_ms = run_start_ms;
    last_oled_telemetry_ms = run_start_ms;
    run_state = RUN_ACTIVE;
    oled_draw_time();

    switch (selected_program) {
        case 1U:
            ball_balance_disable_control();
            oled_draw_status("RUN:VIDEO");
            break;

        case 2U:
            motor_odometer_set_enabled(1U);
#if PROGRAM_2_ONLY_TEST == 0U
            ball_balance_disable_control();
#endif
#if PROGRAM_2_ENABLE_GYRO_STOP != 0U
            if (gyro_lap_stop_start(PROGRAM_2_STOP_ANGLE_DEG,
                                    PROGRAM_2_STOP_DISTANCE_MM) == 0U) {
                run_stop_ms = SystemTime_GetMs();
                set_run_state(RUN_ERROR, "ERR:HWT101");
                return;
            }
#else
            /* Tracking-only test: reset the displayed relative angle when the
             * sensor is available, but never block starting the motors. */
#if HWT101_ENABLE != 0U
            if (hwt101_is_online() != 0U) {
                hwt101_yaw_tracker_reset();
            }
#endif
#endif
            car_forward();
            huidu_soft_start_begin();
            adjust_motor();
            oled_draw_status("RUN:TRACK");
            break;

        case 3U:
            ball_balance_set_target(BALL_TARGET_PLUS_5CM);
            ball_balance_enable_control();
            oled_draw_status("RUN:TO +5CM");
            break;

        case 4U:
            motor_odometer_set_enabled(1U);
            ball_balance_set_target(BALL_TARGET_CENTER);
            ball_balance_enable_control();
            car_forward();
            huidu_soft_start_begin();
            adjust_motor();
            oled_draw_status("RUN:A-B");
            break;

        case 5U:
            motor_odometer_set_enabled(1U);
            ball_balance_set_target(BALL_TARGET_CENTER);
            ball_balance_enable_control();
            if (gyro_lap_stop_start(PROGRAM_5_6_A_PASS_ANGLE_DEG,
                                    PROGRAM_5_6_A_PASS_DISTANCE_MM) == 0U) {
                run_stop_ms = SystemTime_GetMs();
                set_run_state(RUN_ERROR, "ERR:HWT101");
                return;
            }
            car_forward();
            huidu_soft_start_begin();
            adjust_motor();
            oled_draw_status("RUN:LAP CENTER");
            break;

        case 6U:
            /* 题目允许任意指定位置：按KEY1时锁存当前识别位置。 */
            if (!ball_balance_get_last_result(&ball_result) ||
                !ball_result.valid) {
                run_stop_ms = SystemTime_GetMs();
                set_run_state(RUN_ERROR, "ERR:NO BALL");
                return;
            }
            program6_target_position = ball_result.position;
            motor_odometer_set_enabled(1U);
            ball_balance_set_target(program6_target_position);
            ball_balance_enable_control();
            if (gyro_lap_stop_start(PROGRAM_5_6_A_PASS_ANGLE_DEG,
                                    PROGRAM_5_6_A_PASS_DISTANCE_MM) == 0U) {
                run_stop_ms = SystemTime_GetMs();
                set_run_state(RUN_ERROR, "ERR:HWT101");
                return;
            }
            car_forward();
            huidu_soft_start_begin();
            adjust_motor();
            oled_draw_status("RUN:LAP ANY");
            oled_draw_telemetry();
            break;

        default:
            finish_program(false, "ERR:PROGRAM");
            break;
    }
}

static void update_program_2(void)
{
#if PROGRAM_2_ENABLE_GYRO_STOP != 0U
    gyro_angle_stop_state_t stop_state;
#endif

    if (motor_odometer_get_mm() >= PROGRAM_2_APPROACH_DISTANCE_MM) {
        huidu_set_runtime_speed_cap(PROGRAM_2_APPROACH_SPEED_MM_S);
    }
    adjust_motor(); /* Keep the existing, experimentally verified tracking. */
#if PROGRAM_2_ENABLE_GYRO_STOP != 0U
    stop_state = gyro_angle_stop_update();

    /* 陀螺仪暂时掉线只提示，不停止循迹、计时和编码器里程。
     * 恢复收到有效角度帧后自动清除提示。 */
    if (hwt101_is_online() == 0U) {
        if (!program2_hwt_warning_shown) {
            program2_hwt_warning_shown = true;
            oled_draw_status("WARN:HWT LOST");
        }
    } else if (program2_hwt_warning_shown) {
        program2_hwt_warning_shown = false;
        oled_draw_status("RUN:TRACK");
    }

    if (stop_state == GYRO_ANGLE_STOP_REACHED) {
        finish_program(true, "DONE:A-A");
    }
#else
    /* 不使用陀螺仪停车时，仅按KEY1人工停止任务2。 */
#endif
}

static void update_program_3(bool new_ball_frame)
{
    uint32_t now = SystemTime_GetMs();

    if (new_ball_frame && ball_balance_is_at_target(BALL_ONE_CM_TOLERANCE)) {
        if (target_stable_start_ms == 0U) {
            target_stable_start_ms = now;
        }
    } else if (new_ball_frame) {
        target_stable_start_ms = 0U;
    }

    if (program3_phase == P3_GO_TO_PLUS_5 &&
        target_stable_start_ms != 0U &&
        (now - target_stable_start_ms) >= PROGRAM_3_PLUS_STABLE_MS) {
        program3_phase = P3_GO_TO_MINUS_5;
        target_stable_start_ms = 0U;
        ball_balance_set_target(BALL_TARGET_MINUS_5CM);
        oled_draw_status("RUN:TO -5CM");
    } else if (program3_phase == P3_GO_TO_MINUS_5 &&
               target_stable_start_ms != 0U &&
               (now - target_stable_start_ms) >= PROGRAM_3_MINUS_STABLE_MS) {
        /* 到-5cm并稳定后立即冻结计时，同时继续闭环保持在-5cm。 */
        finish_program(true, "DONE:HOLD -5");
    }

    if (PROGRAM_3_TIMEOUT_ENABLE != 0U &&
        run_state == RUN_ACTIVE &&
        elapsed_time_ms() >= PROGRAM_3_TIME_LIMIT_MS) {
        /* Keep the -5 cm target active so the final position remains stable. */
        ball_balance_set_target(BALL_TARGET_MINUS_5CM);
        finish_program(false, "ERR:OVER 5S");
    }
}

static void update_program_4(void)
{
    float distance_mm = motor_odometer_get_mm();

    adjust_motor(); /* Existing line tracking is intentionally unchanged. */

    if (program4_phase == COURSE_BEFORE_PASS) {
        if (distance_mm >= PROGRAM_4_B_PASS_DISTANCE_MM) {
            /* AB计时在通过B时结束，但车辆不停在B线上。 */
            freeze_run_timer();
            program4_phase = COURSE_AFTER_PASS;
            huidu_set_runtime_speed_cap(PROGRAM_4_AFTER_B_SPEED_MM_S);
            oled_draw_status("PASS:B SLOW");
        } else if (elapsed_time_ms() >= PROGRAM_4_TIME_LIMIT_MS) {
            finish_program(false, "ERR:OVER 8S");
        }
    }
}

static void update_program_5_6(void)
{
    float distance_mm = motor_odometer_get_mm();

    adjust_motor();

    if (lap_phase == COURSE_BEFORE_PASS) {
        /*
         * 不调用gyro_angle_stop_update()，因为该旧接口达到阈值会立即停车。
         * 这里直接使用累计角度和里程判断，通过A时只冻结计时并降速。
         */
        if (gyro_angle_stop_get_delta() >= PROGRAM_5_6_A_PASS_ANGLE_DEG &&
            distance_mm >= PROGRAM_5_6_A_PASS_DISTANCE_MM) {
            freeze_run_timer();
            lap_phase = COURSE_AFTER_PASS;
            gyro_angle_stop_cancel();
            huidu_set_runtime_speed_cap(PROGRAM_5_6_AFTER_A_SPEED_MM_S);
            oled_draw_status("PASS:A SLOW");
        } else if (elapsed_time_ms() >= PROGRAM_5_6_TIME_LIMIT_MS) {
            finish_program(false, "ERR:OVER 30S");
        } else if (hwt101_is_online() == 0U &&
                   !program2_hwt_warning_shown) {
            program2_hwt_warning_shown = true;
            oled_draw_status("WARN:HWT LOST");
        } else if (hwt101_is_online() != 0U &&
                   program2_hwt_warning_shown) {
            program2_hwt_warning_shown = false;
            oled_draw_status(selected_program == 5U ?
                             "RUN:LAP CENTER" : "RUN:LAP ANY");
        }
    }
}

static void update_oled_runtime(void)
{
    uint32_t now = SystemTime_GetMs();

    if (run_state == RUN_ACTIVE &&
        (now - last_oled_time_ms) >= OLED_TIME_UPDATE_MS) {
        last_oled_time_ms = now;
        oled_draw_time();
    }

    if ((now - last_oled_telemetry_ms) >= OLED_TELEMETRY_UPDATE_MS) {
        last_oled_telemetry_ms = now;
        oled_draw_telemetry();
    }
}

int main(void)
{
    uint8_t key1_last = 0U;
    uint8_t key2_last = 0U;
    uint8_t key3_last = 0U;
    uint8_t key4_last = 0U;
    uint8_t key1_abort_armed = 1U;
    uint32_t key1_abort_press_ms = 0U;

    SYSCFG_DL_init();

    /* Draw READY before external UART devices are initialized. */
    OLED_Init();
    oled_show_ready_screen();

    motor_init(1U);
    motor_init(2U);
#if HWT101_ENABLE != 0U
    hwt101_init();
#endif

#if PROGRAM_2_ONLY_TEST == 0U
    balance_hardware_ready = ball_balance_init();
    /* 冷启动默认处于任务选择状态，钢球闭环保持关闭。 */
    ball_balance_disable_control();
#endif
    oled_show_ready_screen();

    while (1) {
        uint8_t key1_now;
        uint8_t key2_now;
        uint8_t key3_now;
        uint8_t key4_now;
        bool new_ball_frame;

#if HWT101_ENABLE != 0U
        hwt101_poll();
#endif
#if PROGRAM_2_ONLY_TEST == 0U
        new_ball_frame = ball_balance_update();
#else
        new_ball_frame = false;
#endif

        key1_now = key1_is_pressed();
        key2_now = key2_is_pressed();
        key3_now = key3_is_pressed();
        key4_now = key4_is_pressed();

        if (key3_now != 0U && key3_last == 0U) {
            abort_program();
            selected_program = (selected_program < PROGRAM_MAX) ?
                (uint8_t)(selected_program + 1U) : PROGRAM_MIN;
            oled_show_ready_screen();
        }

        if (key4_now != 0U && key4_last == 0U) {
            abort_program();
            selected_program = (selected_program > PROGRAM_MIN) ?
                (uint8_t)(selected_program - 1U) : PROGRAM_MAX;
            oled_show_ready_screen();
        }

        if (key2_now != 0U && key2_last == 0U &&
            run_state != RUN_ACTIVE) {
            car_stop();
            balance_hardware_ready = ball_balance_calibrate_level_zero();
            oled_draw_status(balance_hardware_ready ? "ZERO:OK" : "ZERO:FAIL");
        }

        if (run_state == RUN_ACTIVE) {
            /*
             * 本次启动按键必须先松开，之后再次连续按住0.8秒才人工中止。
             * 单次短干扰或电容充放电边沿不会再产生STATE:ABORT。
             */
            if (key1_now == 0U) {
                key1_abort_armed = 1U;
                key1_abort_press_ms = 0U;
            } else if (key1_abort_armed != 0U) {
                if (key1_abort_press_ms == 0U) {
                    key1_abort_press_ms = SystemTime_GetMs();
                } else if ((SystemTime_GetMs() - key1_abort_press_ms) >=
                           KEY1_ABORT_HOLD_MS) {
#if PROGRAM_2_ENABLE_GYRO_STOP == 0U
                    if (selected_program == 2U) {
                        finish_program(true, "STOP:KEY1");
                    } else {
                        abort_program();
                        oled_draw_status("STATE:ABORT");
                    }
#else
                    abort_program();
                    oled_draw_status("STATE:ABORT");
#endif
                    key1_abort_armed = 0U;
                    key1_abort_press_ms = 0U;
                }
            }
        } else if (key1_now != 0U && key1_last == 0U) {
            start_selected_program();
            /* 启动后先等待KEY1释放，不能把同一次按压当作中止。 */
            key1_abort_armed = 0U;
            key1_abort_press_ms = 0U;
        }

        if (run_state == RUN_ACTIVE) {
            switch (selected_program) {
                case 1U:
                    /* RTSP is generated on K230.  This timer brackets recording. */
                    break;
                case 2U:
                    update_program_2();
                    break;
                case 3U:
                    update_program_3(new_ball_frame);
                    break;
                case 4U:
                    update_program_4();
                    break;
                case 5U:
                case 6U:
                    update_program_5_6();
                    break;
                default:
                    finish_program(false, "ERR:PROGRAM");
                    break;
            }
        }

        update_oled_runtime();

        key1_last = key1_now;
        key2_last = key2_now;
        key3_last = key3_now;
        key4_last = key4_now;

        /* Buttons already have hardware RC debounce. */
        delay_ms(2U);
    }
}
