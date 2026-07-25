#ifndef HUIDU_H
#define HUIDU_H
#include <stdint.h>

// 接线
// 灰度模块
// VCC       5V(根据说明书确定具体是多少电压，不能接错了)
// GND       GND
// L4       GPIOA.28
// L3       GPIOA.0
// L2       GPIOA.31
// L1       GPIOA.1
// R1       GPIOB.4
// R2       GPIOA.8
// R3       GPIOB.5
// R4       GPIOA.9


void huidu_get_value();// 获取灰度模块的值，存储在huidu_value数组中
uint8_t adjust_motor(void);// 调整电机速度，使小车沿着黑线行驶
uint8_t huidu_get_left_black_count(void);// 最近一次采样中左侧L4~L1的黑线数量
uint8_t huidu_get_right_black_count(void);// 最近一次采样中右侧R1~R4的黑线数量
void huidu_set_tracking_enabled(uint8_t enabled);// 允许/屏蔽灰度对电机轮速的修改
extern volatile uint8_t huidu_tracking_enabled;// CCS Watch调试量：1启用，0屏蔽
uint8_t finish_line_detected(void);
void finish_detector_reset(void);

#endif
