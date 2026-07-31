#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

#include "k230.h"

/*
 * Ball-on-beam controller configuration
 * -------------------------------------
 * All values that normally need tuning on the real mechanism are here.
 */

/* 右侧单电机机构和当前K230坐标定义：
 *   正执行器命令 = CCW = 右端上升 -> 钢球向画面左侧滚动；
 *   K230物理正方向也在画面左侧：X=247对应P=700(+5cm)。
 * 因此error=target-P为正时应输出正命令，控制方向系数使用+1。
 */
#define BALL_CONTROL_DIRECTION             -1.0f // 控制方向系数：1表示正向控制，-1表示反向控制

/* Camera coordinate conversion for the specified 25 cm tube. */
#define BALL_TARGET_CENTER                 K230_BALL_POSITION_CENTER
/*
 * 最新三点标定：X=394为中心，X=234为+5cm，X=542为-5cm。
 * K230归一化后对应：中心P=500，+5cm P=700，-5cm P=300。
 */
#define BALL_TARGET_PLUS_5CM               (BALL_TARGET_CENTER + 5U * K230_BALL_POSITION_PER_CM)
#define BALL_TARGET_MINUS_5CM              (BALL_TARGET_CENTER - 5U * K230_BALL_POSITION_PER_CM)
#define BALL_ONE_CM_TOLERANCE               K230_BALL_POSITION_PER_CM

/* Position PD: actuator pulses = Kp*position_error - Kd*ball_velocity. */
#define BALL_KP_PULSES_PER_POSITION         1.4f   // 比例系数：越大纠偏越快，但更容易越过目标
#define BALL_KD_PULSES_PER_POSITION_PER_S   0.26f   // 速度阻尼微分系数：越大抑制越过目标，但响应变慢,反向制动
#define BALL_POSITION_FILTER_ALPHA          0.25f  // 低通滤波系数：越大响应越快，但噪声越大,越小越平滑，延迟也会增加

/* Maximum endpoint travel relative to the manually calibrated level zero. */
#define BALL_ACTUATOR_LIMIT_PULSES           500L  // 水管最大倾斜量
#define BALL_ACTUATOR_COMMAND_DEADBAND       10L   // 小于该变化量时，不重新发送电机位置
#define BALL_COMMAND_MIN_INTERVAL_MS         35U // 电机命令最小间隔，避免过快发送导致K230丢帧

/* Closed-loop stepper position-mode settings. */
#define BALL_STEPPER_SPEED_RPM               60U   // 电机目标速度，单位RPM
#define BALL_STEPPER_ACCELERATION            40U    // 电机加速度，单位RPM/s

typedef enum {
    BALL_BALANCE_NO_VISION = 0,
    BALL_BALANCE_TRACKING,
    BALL_BALANCE_LEVEL_HOLD,
    BALL_BALANCE_DISABLED
} ball_balance_state_t;

/* Initializes UART reception and the Emm_V5 quick absolute position mode. */
bool ball_balance_init(void);

/*
 * Set the current physical beam position as the level/center actuator zero.
 * Call only while the car is stopped and the beam has been manually leveled.
 */
bool ball_balance_calibrate_level_zero(void);

void ball_balance_set_target(uint16_t target_position);
uint16_t ball_balance_get_target(void);

/*
 * Enable/disable automatic ball-position control.
 * When disabled, K230 coordinates are still received for display and start
 * checks, but no new stepper motion command is generated.
 */
void ball_balance_enable_control(void);
void ball_balance_disable_control(void);

/* Non-blocking update; returns true when a new K230 frame was processed. */
bool ball_balance_update(void);

/* Command the calibrated level position and clear controller history. */
void ball_balance_hold_level(void);

ball_balance_state_t ball_balance_get_state(void);
bool ball_balance_get_last_result(k230_ball_result_t *result);
bool ball_balance_is_at_target(uint16_t tolerance);

/* 任务3实车诊断量：OLED和CCS均可观察。 */
extern volatile uint16_t ball_debug_position;       /* K230位置0~1000 */
extern volatile uint8_t ball_debug_position_valid;  /* 1=当前识别有效 */
extern volatile int32_t ball_debug_command_pulses;  /* 最近计算/发送的绝对目标脉冲 */
extern volatile uint32_t ball_debug_command_count;  /* 成功写入UART的运动命令数 */

#endif
