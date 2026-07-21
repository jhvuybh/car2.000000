#ifndef LED_H
#define LED_H
#include <stdint.h>

void LED_Init(void);
void LED_ON(uint8_t led_id);
void LED_OFF(uint8_t led_id);

#endif /* LED_H */