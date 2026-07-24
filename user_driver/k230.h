#ifndef K230_H
#define K230_H

/*
 * K230视觉模块通信接口
 *
 * MSPM0向K230发送固定4字节任务命令：
 *
 *     0xA5  TASK  TARGET  0x5A
 *
 * TASK=1：初次识别画面中央房号，此时TARGET传0。
 * TASK=2：寻找指定目标房号，此时TARGET传1~8。
 *
 * K230向MSPM0发送固定6字节识别结果：
 *
 *     0xAA  ROOM  SIDE  FOUND  TASK  0x55
 *
 * ROOM取值：FOUND=0时为0；FOUND=1时为识别到的房号1~8。
 * SIDE取值：0=画面中间，1=画面左侧，2=画面右侧。
 * FOUND取值：0=尚无稳定结果，1=已经连续多帧稳定识别。
 * TASK取值：1=初次识别，2=寻找指定目标。
 *
 * 所有字段都是二进制数值，不是ASCII字符。例如TASK2识别到3号房在左边：
 *     AA 03 01 01 02 55
 */

/* bool、true和false的类型定义。 */
#include <stdbool.h>
/* uint8_t等固定宽度整数类型定义。 */
#include <stdint.h>

/*
 * 数字在K230摄像头画面中的横向位置。
 * 这些枚举值必须与K230端Python程序中的SIDE_CENTER/LEFT/RIGHT保持一致，
 * 因为枚举值会直接对应串口帧中的SIDE字节。
 */
typedef enum
{
    /* 检测框中心位于画面中线附近的死区内。 */
    K230_SIDE_CENTER = 0,
    /* 检测框中心位于画面左侧。 */
    K230_SIDE_LEFT = 1,
    /* 检测框中心位于画面右侧。 */
    K230_SIDE_RIGHT = 2
} k230_side_t;

/*
 * K230视觉工作模式。
 * 枚举值必须与K230端Python程序的TASK_INITIAL_ROOM/TASK_FIND_TARGET一致。
 */
typedef enum
{
    /* 初次识别：只接受画面中央区域的房号。 */
    K230_TASK_INITIAL_ROOM = 1,
    /* 定向寻找：只寻找target_room指定的数字，并判断左/中/右。 */
    K230_TASK_FIND_TARGET = 2
} k230_task_t;

/* 一次完整的K230视觉识别结果。 */
typedef struct
{
    /* FOUND为true时范围为1~8；FOUND为false时为0。 */
    uint8_t room;
    /* 房号在摄像头画面中的位置：中间、左侧或右侧。 */
    k230_side_t side;
    /* true表示结果已经通过K230连续帧确认；false表示当前没有稳定结果。 */
    bool found;
    /* 产生本结果时K230正在执行的视觉任务。 */
    k230_task_t task;
} k230_result_t;

/*
 * UART接收诊断量，可直接加入CCS Watch；只用于观察，不参与控制逻辑。
 * rx_byte_count持续增加说明物理串口确实收到数据；valid_frame_count增加说明
 * AA ROOM SIDE FOUND TASK 55完整帧已经通过校验；last_result保存最近的有效帧。
 */
extern volatile uint32_t k230_debug_rx_byte_count;
extern volatile uint32_t k230_debug_build_id;
extern volatile uint32_t k230_debug_poll_count;
extern volatile uint32_t k230_debug_init_count;
extern volatile uint32_t k230_debug_irq_count;
extern volatile uint32_t k230_debug_valid_frame_count;
extern volatile uint32_t k230_debug_rejected_frame_count;
extern volatile uint8_t k230_debug_last_rx_byte;
extern volatile uint8_t k230_debug_parser_state;
extern volatile uint8_t k230_debug_rx_pin_level;
extern volatile uint8_t k230_debug_rx_fifo_empty;
extern volatile uint8_t k230_debug_last_iidx;
extern volatile uint8_t k230_debug_loopback_enabled;
extern volatile k230_result_t k230_debug_last_result;

/* 在SYSCFG_DL_init()之后调用，启用UART2 RX中断与软件接收缓冲区。 */
void k230_init(void);

/*
 * 向K230发送视觉任务命令。
 *
 * TASK1调用：
 *     k230_set_task(K230_TASK_INITIAL_ROOM, 0);
 *
 * TASK2查找6号房调用：
 *     k230_set_task(K230_TASK_FIND_TARGET, 6);
 *
 * 返回true表示参数合法且命令已经写入UART；参数组合非法时返回false。
 */
bool k230_set_task(k230_task_t task, uint8_t target_room);

/*
 * 非阻塞读取并解析K230串口数据。
 *
 * 参数：
 *     result：用于保存解析结果的结构体地址，不能传入NULL。
 *
 * 返回值：
 *     true ：成功收到并解析一帧AA ROOM SIDE FOUND TASK 55，result已更新。
 *     false：当前UART缓冲区中还没有完整有效帧，result保持不变。
 *
 * 这个函数不会等待K230发送数据，因此可以放在主循环中反复调用，
 * 不会因为串口暂时没有数据而阻塞循迹或电机控制。
 */
bool k230_get_result(k230_result_t *result);

/*
 * 兼容旧代码的简化接口，只读取房号，不提供左右信息。
 *
 * 返回值：
 *     1~8：成功收到且FOUND=1的房号。
 *     0  ：当前没有收到完整有效帧。
 *
 * 新代码如果需要判断左右，应使用k230_get_result()。
 */
uint8_t k230_get_room(void);

#endif
