#ifndef MOTOR_H
#define MOTOR_H

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 364
// 轮胎直径 mm
#define MOTOR_WHEEL_D 65
//motor1 左   1正2反
//motor2 右
// G3507      TB6612
// PA17 <--> STBY
// PA24 <--> AIN1
// PA25 <--> AIN2
// PA26 <--> PWMA
// PB24 <--> BIN1
// PB25 <--> BIN2
// PA27 <--> PWMB
// GND <--> GND
//编码器
//PA22 <--> AA
//PB20 <--> AB
//PA15 <--> BA
//PA14 <--> BB
// 所有的GND都需要连接在一起

#include "ti_msp_dl_config.h"

void motor_init(uint8_t motor_id);// 初始化电机
void motor_set_duty(uint8_t motor_id, uint32_t duty);// 设置电机占空比
void motor_set_direction(uint8_t motor_id, uint8_t direction);// 设置电机转向
void car_stop(void);// 停止小车
void car_turn_around(void);// 小车掉头
void car_turn_left_90(void);// 小车左转90度
void car_turn_right_90(void);// 小车右转90度



#endif // MOTOR_H
