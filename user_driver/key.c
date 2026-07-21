#include "key.h"
#include "delay.h"
extern int status;

// uint8_t get_key_state(uint32_t key) {
//     uint32_t low_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
//     if((low_bits & key) != 0) return 1;
//     else return 0;
// }



/* 读取微动开关原始状态 */
static uint8_t medicine_read_raw(void)
{
    uint32_t pin_state;

    pin_state = DL_GPIO_readPins(
        KEY_HUOWU_PORT,
        KEY_HUOWU_PIN
    );

    /*
     * 内部上拉：
     * 低电平表示开关压下。
     */
    if ((pin_state & KEY_HUOWU_PIN) == 0)
    {
        return 1;
    }

    return 0;
}


/* 消抖后的货物检测 */
uint8_t medicine_is_loaded(void)
{
    static uint8_t stable_state = 0;

    uint8_t first_state;
    uint8_t second_state;

    first_state = medicine_read_raw();

    delay_ms(20);

    second_state = medicine_read_raw();

    /*
     * 两次读取相同，才更新稳定状态。
     * 不相同说明仍在抖动，保持上一次结果。
     */
    if (first_state == second_state)
    {
        stable_state = second_state;
    }

    return stable_state;
}




uint32_t counter_1_A = 0;// 计数器1A
uint32_t counter_2_A = 0;// 计数器2A

void GROUP1_IRQHandler()// GPIOA中断服务函数,电机编码器
{
    switch (DL_GPIO_getPendingInterrupt(GPIOA))
    {
    case DC_MOTOR_AA_IIDX :
        /* code */
        counter_1_A ++;
        break;
    case DC_MOTOR_BA_IIDX:
        /* code */
        counter_2_A ++;
        break;
    default:
        break;
    }

}
