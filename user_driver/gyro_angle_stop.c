#include "gyro_angle_stop.h"

#include "hwt101.h"
#include "motor.h"

static gyro_angle_stop_state_t angle_stop_state = GYRO_ANGLE_STOP_IDLE;
/* 0表示普通角度任务；1表示任务2的“角度+里程”联合停车。 */
static uint8_t lap_stop_mode = 0U;
static float lap_stop_angle_degrees = 0.0f;
static float lap_stop_distance_mm = 0.0f;

uint8_t gyro_angle_stop_start(float target_degrees)
{
    /* 传感器未开始输出或已经掉线时禁止启动车辆角度任务。 */
    hwt101_poll();
    if (hwt101_is_online() == 0U) {
        angle_stop_state = GYRO_ANGLE_STOP_SENSOR_LOST;
        car_stop();
        return 0U;
    }

    /* 记录此刻航向为相对0度；累计器会自动处理±180度跨界。 */
    hwt101_yaw_tracker_reset();
    hwt101_set_turn_target(target_degrees);
    lap_stop_mode = 0U;
    angle_stop_state = GYRO_ANGLE_STOP_RUNNING;
    return 1U;
}

uint8_t gyro_lap_stop_start(float stop_angle_degrees,
                            float stop_distance_mm)
{
    if (stop_angle_degrees <= 0.0f || stop_distance_mm <= 0.0f) {
        return 0U;
    }

    if (gyro_angle_stop_start(stop_angle_degrees) == 0U) {
        return 0U;
    }

    /* 从A点启动时将车轮编码器累计距离清零。 */
    motor_odometer_reset();
    lap_stop_angle_degrees = stop_angle_degrees;
    lap_stop_distance_mm = stop_distance_mm;
    lap_stop_mode = 1U;
    return 1U;
}

gyro_angle_stop_state_t gyro_angle_stop_update(void)
{
    if (angle_stop_state != GYRO_ANGLE_STOP_RUNNING) {
        return angle_stop_state;
    }

    hwt101_poll();

    /*
     * 整圈任务恢复使用精确转角，并与编码里程联合判断。
     * 两个阈值都由main.c传入；任意一个未达到都不会停车。
     */
    if (lap_stop_mode != 0U) {
        if (gyro_angle_stop_get_delta() >= lap_stop_angle_degrees &&
            motor_odometer_get_mm() >= lap_stop_distance_mm) {
            car_stop();
            lap_stop_mode = 0U;
            angle_stop_state = GYRO_ANGLE_STOP_REACHED;
        }
        return angle_stop_state;
    }

    /*
     * HWT101短时掉线不能让任务2的计时、里程和循迹一起停止。
     * 保持RUNNING状态，主循环会继续驱动车辆；串口恢复后角度跟踪器
     * 会从最新帧继续累计。任务2不再使用20秒强制停车。
     */
    if (hwt101_is_online() == 0U) {
        return angle_stop_state;
    }

    if (hwt101_yaw_tracker_target_reached() != 0U) {
        car_stop();
        angle_stop_state = GYRO_ANGLE_STOP_REACHED;
    }

    return angle_stop_state;
}

void gyro_angle_stop_cancel(void)
{
    lap_stop_mode = 0U;
    lap_stop_angle_degrees = 0.0f;
    lap_stop_distance_mm = 0.0f;
    angle_stop_state = GYRO_ANGLE_STOP_IDLE;
}

gyro_angle_stop_state_t gyro_angle_stop_get_state(void)
{
    return angle_stop_state;
}

float gyro_angle_stop_get_delta(void)
{
    return hwt101_yaw_tracker_get_delta();
}
