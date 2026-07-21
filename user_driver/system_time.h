#ifndef SYSTEM_TIME_H
#define SYSTEM_TIME_H

#include <stdint.h>

/*
 * PID定时器的中断周期：
 * 每1ms中断一次填1；
 * 每10ms中断一次填10。
 */
#define SYSTEM_TIME_TICK_MS  10U

/* 更新系统时间，在定时器中断中调用 */
void SystemTime_Tick(void);

/* 获取系统运行时间，单位：毫秒 */
uint32_t SystemTime_GetMs(void);

/* 判断指定延时是否结束 */
uint8_t SystemTime_IsOver(uint32_t start_time, uint32_t delay_ms);

#endif