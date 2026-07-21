#include"LED.h"
#include "ti_msp_dl_config.h"
//led1= PA.13   red
//led2= PA.12   blue

void LED_Init(void)//初始化led关闭
{
    DL_GPIO_clearPins(
        LED_PORT,
        LED_RED_PIN | LED_BLUEE_PIN
    );
}


void LED_ON(uint8_t led_id)
{
    if(led_id == 1){
        
        DL_GPIO_setPins(LED_PORT, LED_RED_PIN);
    }
    else if(led_id == 2){
        
        DL_GPIO_setPins(LED_PORT, LED_BLUEE_PIN);
    }
}

void LED_OFF(uint8_t led_id)
{
    if(led_id == 1){
        
        DL_GPIO_clearPins(LED_PORT, LED_RED_PIN);
    }
    else if(led_id == 2){
        
        DL_GPIO_clearPins(LED_PORT, LED_BLUEE_PIN);
    }
}
