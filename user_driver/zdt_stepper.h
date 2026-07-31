#ifndef ZDT_STEPPER_H
#define ZDT_STEPPER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 张大头 Emm_V5 串口闭环步进电机驱动
 * ================================================================
 * 接线：
 *   MSPM0 PA10 / UART0_TX -> 电机 RX
 *   MSPM0 PA11 / UART0_RX <- 电机 TX
 *   MSPM0 GND             -- 电机 GND（必须共地）
 *
 * 电机菜单：Emm_V5协议、115200波特率、8N1、电机地址1。
 * 电机功率端必须使用独立电源，不能由MSPM0的3.3V/5V供电。
 *
 * 时序说明：驱动函数只保证一条UART帧内的字节发送完整，不会自动在两条命令
 * 之间阻塞延时10ms。原厂例程的delay_ms(10)主要用于等待整帧回复进入FIFO，
 * 不是依靠空闲时间划分协议帧。本驱动使用中断环形缓冲区和固定帧长状态机，
 * 主循环持续调用zdt_stepper_poll()即可。初始化阶段连续发送不同配置命令时，
 * 建议调用者在两条命令之间保留10ms；平衡阶段每个视觉帧最多发送一次新目标。
 *
 * 后续更换电机地址或发现运动方向相反时，通常只需修改下面两个宏。
 */

/* 电机在驱动器菜单中设置的通信地址，默认是1。 */
#define ZDT_STEPPER_ADDRESS       1U

/*
 * 本工程统一规定：正脉冲=右侧滑轨向上，负脉冲=右侧滑轨向下。
 * 当前机械实测：CCW=上升、CW=下降；Emm_V5协议0=CW、1=CCW，
 * 因此正方向必须配置为1U。普通位置和快速位置命令都会使用此配置。
 */
#define ZDT_STEPPER_POSITIVE_DIR  1U

/*
 * 默认16细分、1.8度电机时为3200命令脉冲/圈。
 * 修改驱动器细分后必须同步修改这个值，它用于角度和滑轨距离换算。
 */
#define ZDT_STEPPER_COMMAND_PULSES_PER_REV  3200L

/* 电机实时位置回复固定使用65536位置单位/圈。 */
#define ZDT_STEPPER_FEEDBACK_UNITS_PER_REV  65536L

typedef struct {
    int32_t position_units;       /* 实时位置，65536单位/电机一圈 */
    int16_t speed_rpm;            /* 实时转速，带正负方向，单位RPM */
    uint8_t homing_status;        /* 驱动器返回的回零状态标志 */
    uint8_t last_function;        /* 最近一帧返回数据的功能码 */
    uint8_t last_status;          /* 最近一条控制命令的执行状态 */
    uint32_t last_response_ms;    /* 最近收到有效回复的系统时间 */
    bool position_valid;          /* 已经成功收到过位置反馈 */
    bool speed_valid;             /* 已经成功收到过转速反馈 */
    bool homing_status_valid;     /* 已经成功收到过回零状态 */
} zdt_stepper_feedback_t;

/* SYSCFG_DL_init()之后调用一次：打开UART中断并清空接收解析器。 */
void zdt_stepper_init(void);

/* 电机使能与急停；急停后是否保持锁定由驱动器菜单设置决定。 */
bool zdt_stepper_enable(bool enable);
bool zdt_stepper_stop(void);

/* 将驱动器的当前实时位置设为0，不能代替机械限位回零。 */
bool zdt_stepper_set_current_position_zero(void);

/*
 * 普通位置命令参数：
 *   pulses/target_pulses：命令脉冲数，不是反馈的65536单位。
 *   speed_rpm：0~5000 RPM；滑轨首次测试建议30~60 RPM。
 *   acceleration：0~255；0表示直接启动，首次测试不要使用0。
 *   正数按“向上”处理，负数按“向下”处理。
 */
bool zdt_stepper_move_relative(
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration
);
bool zdt_stepper_move_absolute(
    int32_t target_pulses,
    uint16_t speed_rpm,
    uint8_t acceleration
);

/*
 * 快速位置模式：初始化时配置一次，平衡循环中反复发送目标位置。
 * absolute_mode=true：quick_move参数是相对回零点的绝对位置（推荐）。
 * absolute_mode=false：quick_move参数是相对电机当前位置的增量。
 */
bool zdt_stepper_configure_quick_position(
    uint16_t speed_rpm,
    uint8_t acceleration,
    bool absolute_mode
);
bool zdt_stepper_quick_move(int32_t signed_pulses);

/*
 * 回零模式：
 *   mode=0：单圈就近回零，不适合多圈丝杆滑轨；
 *   mode=1：单圈指定方向回零；
 *   mode=2：多圈碰撞回零，存在撞击机构风险；
 *   mode=3：多圈外部限位开关回零，竖直滑轨推荐使用。
 */
bool zdt_stepper_start_homing(uint8_t mode);
bool zdt_stepper_abort_homing(void);

/*
 * 非阻塞反馈查询：request函数只发送命令，不等待回复。
 * 主循环必须持续调用zdt_stepper_poll()，然后用get_feedback()读取结果。
 */
bool zdt_stepper_request_position(void);
bool zdt_stepper_request_speed(void);
bool zdt_stepper_request_homing_status(void);
void zdt_stepper_poll(void);
bool zdt_stepper_get_feedback(zdt_stepper_feedback_t *feedback);

#endif
