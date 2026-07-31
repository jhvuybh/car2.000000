#ifndef GYRO_ANGLE_STOP_H
#define GYRO_ANGLE_STOP_H

#include <stdint.h>

/* 角度停车控制器状态。 */
typedef enum {
    GYRO_ANGLE_STOP_IDLE = 0,       /* 尚未启动或已经取消 */
    GYRO_ANGLE_STOP_RUNNING,        /* 正在累计偏航角 */
    GYRO_ANGLE_STOP_REACHED,        /* 已达到目标并调用car_stop() */
    GYRO_ANGLE_STOP_SENSOR_LOST     /* HWT101超时，已安全停车 */
} gyro_angle_stop_state_t;

/*
 * 开始一次角度停车任务：
 *   target_degrees > 0：按本工程定义的右转方向累计；
 *   target_degrees < 0：按左转方向累计。
 * 顺时针跑完整个椭圆赛道时使用+360.0f。
 * 只有HWT101已经收到有效角度数据时才能成功启动。
 */
uint8_t gyro_angle_stop_start(float target_degrees);

/*
 * 题目整圈专用：清零角度和里程，之后必须同时达到传入的两个阈值才停车。
 * 参数统一在main.c顶部定义，便于实车修改。
 */
uint8_t gyro_lap_stop_start(float stop_angle_degrees,
                            float stop_distance_mm);

/*
 * 主循环持续调用。达到目标或传感器超时后函数会自动调用car_stop()。
 * 返回当前状态，便于任务状态机进入下一步。
 */
gyro_angle_stop_state_t gyro_angle_stop_update(void);

/* 取消当前任务，不主动停车；需要停车时先调用car_stop()。 */
void gyro_angle_stop_cancel(void);

gyro_angle_stop_state_t gyro_angle_stop_get_state(void);
float gyro_angle_stop_get_delta(void);

#endif
