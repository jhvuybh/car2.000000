#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

/*
 * 按键引脚统一使用 SysConfig 生成的宏，避免手写引脚与
 * empty.syscfg 中的配置不一致：
 * KEY1=PB1，KEY2=PB0，KEY3=PA7，KEY4=PB10。
 * 按键外设已有硬件 RC 消抖，软件只进行低电平读取。
 */
#define KEY1_PORT      KEY_KEY1_PORT
#define KEY1_PIN_PIN   KEY_KEY1_PIN

#define KEY2_PORT      KEY_KEY2_PORT
#define KEY2_PIN_PIN   KEY_KEY2_PIN

#define KEY3_PORT      KEY_KEY3_PORT
#define KEY3_PIN_PIN   KEY_KEY3_PIN

#define KEY4_PORT      KEY_KEY4_PORT
#define KEY4_PIN_PIN   KEY_KEY4_PIN

uint8_t medicine_is_loaded(void);
uint8_t key1_is_pressed(void);
uint8_t key2_is_pressed(void);
uint8_t key3_is_pressed(void);
uint8_t key4_is_pressed(void);

#endif
