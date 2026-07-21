#include "system_time.h"

/* 系统运行时间，单位：毫秒 */
static volatile uint32_t system_time_ms = 0;

/* 每次定时器中断调用一次 */
void SystemTime_Tick(void)
{
    system_time_ms += SYSTEM_TIME_TICK_MS;
}

/* 获取系统运行时间 */
uint32_t SystemTime_GetMs(void)
{
    return system_time_ms;
}

/* 判断延时是否结束 */
uint8_t SystemTime_IsOver(uint32_t start_time, uint32_t delay_ms)
{
    if ((SystemTime_GetMs() - start_time) >= delay_ms)
    {
        return 1;
    }

    return 0;
}