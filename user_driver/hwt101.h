#ifndef HWT101_H
#define HWT101_H

#include <stdint.h>

/*
 * HWT101 UART姿态模块
 * ================================================================
 * 接线：
 *   HWT101 TX -> MSPM0 PA18 / UART1_RX（读取数据必须连接）
 *   HWT101 RX <- MSPM0 PB6  / UART1_TX（只读取时可以不接）
 *   HWT101 GND -- MSPM0 GND（必须共地）
 *   HWT101 VCC -> 按模块铭牌要求供电
 *
 * SysConfig名称：HWT101；串口：115200、8N1；RX FIFO中断接收。
 * 主循环必须持续调用hwt101_poll()。
 */

/*
 * 安装方向修正：本工程规定累计偏航角“右转为正、左转为负”。
 * 实测右转时delta若为负数，把1.0f改成-1.0f。
 */
#define HWT101_YAW_DIRECTION_SIGN      -1.0f

/*
 * 提前停车补偿机械惯性。目标90度、提前量3度时，在累计约87度发出停车。
 * 实测转过头就增大，转不到就减小；建议范围1~8度。
 */
#define HWT101_STOP_ADVANCE_DEG         3.0f

/* 超过此时间没有新角度帧，认为传感器掉线并立即停车。 */
/* OLED刷新和电机中断可能造成短时处理延迟。1秒内没有角度帧只认为
 * 通信暂时中断；任务2不会因此立即停车，恢复收到数据后可继续工作。 */
#define HWT101_DATA_TIMEOUT_MS          1000U

/* 1：主控每次启动时把HWT101配置成仅输出0x53角度帧。
 * 若后续需要用上角速度/加速度，可改成0并重新编译。 */
#define HWT101_CONFIGURE_ANGLE_ONLY_ON_BOOT  0U

/* 可在CCS Expressions/Watch中观察：
 * uart_error增长 -> 线路噪声、供电波动或波特率不匹配；
 * buffer_overflow增长 -> 主循环处理不及时或模块输出内容过多。 */
extern volatile uint32_t hwt101_debug_uart_error_count;
extern volatile uint32_t hwt101_debug_buffer_overflow_count;

typedef struct {
    float roll;                         /* 绕X轴角度，单位度 */
    float pitch;                        /* 绕Y轴角度，单位度 */
    float yaw;                          /* 绕Z轴偏航角，范围约-180~180度 */
} hwt101_angles_t;

typedef struct {
    float wx;                           /* X轴角速度，度/秒 */
    float wy;                           /* Y轴角速度，度/秒 */
    float wz;                           /* Z轴角速度，度/秒 */
} hwt101_gyro_t;

typedef struct {
    float ax;                           /* X轴加速度，单位g */
    float ay;                           /* Y轴加速度，单位g */
    float az;                           /* Z轴加速度，单位g */
} hwt101_accel_t;

typedef struct {
    float hx;
    float hy;
    float hz;
} hwt101_mag_t;

/* CCS Watch调试变量。 */
extern volatile uint32_t hwt101_debug_rx_byte_count;
extern volatile uint32_t hwt101_debug_valid_frame_count;
extern volatile uint32_t hwt101_debug_rejected_frame_count;
extern volatile uint8_t hwt101_debug_parser_state;
extern volatile uint8_t hwt101_debug_last_rx_byte;
extern volatile uint8_t hwt101_debug_last_data_type;
extern volatile hwt101_angles_t hwt101_debug_last_angles;
extern volatile hwt101_gyro_t hwt101_debug_last_gyro;

/* 在SYSCFG_DL_init()之后调用一次。 */
void hwt101_init(void);

/* 非阻塞处理已接收数据；主循环应每次都调用。 */
void hwt101_poll(void);

/* 有新数据返回1；没有新数据返回0。 */
uint8_t hwt101_get_angles(hwt101_angles_t *angles);
uint8_t hwt101_get_gyro(hwt101_gyro_t *gyro);
uint8_t hwt101_get_accel(hwt101_accel_t *accel);
uint8_t hwt101_get_mag(hwt101_mag_t *mag);

/* 直接读取最近一次有效值。 */
float hwt101_get_yaw(void);
float hwt101_get_pitch(void);
float hwt101_get_roll(void);

/* 是否至少收到过一帧有效角度，以及最近数据是否仍在有效期内。 */
uint8_t hwt101_has_valid_angle(void);
uint8_t hwt101_is_online(void);

/*
 * 偏航角累计器：自动处理+180/-180度跳变，累计结果可以超过360度。
 * reset()在转弯开始前调用；之后delta正数表示右转、负数表示左转。
 */
void hwt101_yaw_tracker_reset(void);
float hwt101_yaw_tracker_get_delta(void);

/* 设置目标角度；正数右转、负数左转。 */
void hwt101_set_turn_target(float target_degrees);

/* 达到“目标角度减提前停车量”时返回1，并自动清除本次目标。 */
uint8_t hwt101_yaw_tracker_target_reached(void);

#endif
