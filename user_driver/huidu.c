#include "huidu.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
uint8_t huidu_value[] = {0, 0, 0, 0, 0 ,0,0,0,};

uint8_t get_gpio_state(GPIO_Regs *gpio_port, uint32_t gpio) {
    uint32_t high_bits = DL_GPIO_readPins(gpio_port, gpio); 
    if((high_bits & gpio) != 0) return 1;
    else return 0;
}

void huidu_get_value()
{
    huidu_value[0] = get_gpio_state(HUIDU_L4_PORT, HUIDU_L4_PIN);
    huidu_value[1] = get_gpio_state(HUIDU_L3_PORT, HUIDU_L3_PIN);
    huidu_value[2] = get_gpio_state(HUIDU_L2_PORT, HUIDU_L2_PIN);
    huidu_value[3] = get_gpio_state(HUIDU_L1_PORT, HUIDU_L1_PIN);
    huidu_value[4] = get_gpio_state(HUIDU_R1_PORT, HUIDU_R1_PIN);
    huidu_value[5] = get_gpio_state(HUIDU_R2_PORT, HUIDU_R2_PIN);
    huidu_value[6] = get_gpio_state(HUIDU_R3_PORT, HUIDU_R3_PIN);
    huidu_value[7] = get_gpio_state(HUIDU_R4_PORT, HUIDU_R4_PIN);
}
extern float target_speed_1;// 电机目标速度 mm/s
extern float target_speed_2;// 电机目标速度 mm/s
 
float target_speed_5[] = {125, 175, 200, 400, 500};// 5档速度，单位 mm/s    
//识别到线置0,没识别到线置1
// 限制目标速度范围
static float limit_target_speed(float speed)
{
    if (speed > target_speed_5[4]) {
        speed = target_speed_5[4];
    } 

    if (speed < 0) {
        speed = 0;
    }

    return speed;
}





uint8_t adjust_motor(void)    // 调整电机速度，使小车沿着黑线行驶,返回值为识别到线灯数量
{
    static const int8_t weight[8] = {
        -7, -5, -3, -1, 1, 3, 5, 7
    };

    static float last_error = 0;// 上一次黑线位置误差，用于丢线时判断黑线在左侧还是右侧

    int16_t weighted_sum = 0;// 黑线位置加权和
    uint8_t black_count = 0;// 检测到黑线的灰度模块数量
    uint8_t i;

    float error;
    float correction;

    float base_speed = 350.0f;   // 基础速度，单位 mm/s
    float trace_kp = 25.0f;     // 黑线偏离中心时的比例系数
    float center_kp = 8.0f;    // 黑线接近中心时的比例系数

    /* 每次调用都重新读取8路灰度 */
    huidu_get_value();

    for (i = 0; i < 8; i++)
    {
        /* 当前代码规定：0表示黑线，1表示白色 */
        if (huidu_value[i] == 0)
        {
            weighted_sum += weight[i];
            black_count++;
        }
    }

    /*
     * 8路全部检测到黑色。
     * 可能是终点线或者十字路口，先交给主函数处理。
     */
    if (black_count == 8)
    {
        target_speed_1 = 0;
        target_speed_2 = 0;
        return 8;
    }

    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    /* 完全丢线 */
    if (black_count == 0)
    {
        if (last_error < -0.5f)
        {
            /* 上一次黑线在左侧，继续向左寻找 */
            target_speed_1 = target_speed_5[0];
            target_speed_2 = target_speed_5[3];
        }
        else if (last_error > 0.5f)
        {
            /* 上一次黑线在右侧，继续向右寻找 */
            target_speed_1 = target_speed_5[3];
            target_speed_2 = target_speed_5[0];
        }
        else
        {
            /* 开机时就没有检测到黑线 */
            target_speed_1 = 0;
            target_speed_2 = 0;
        }

        return 0;
    }

    /* 计算黑线位置 */
    error = (float)weighted_sum / black_count;// 计算黑线位置误差，负数表示黑线在左侧，正数表示黑线在右侧
    last_error = error;                      // 保存当前误差，用于丢线时判断黑线在左侧还是右侧 

    /* 中央附近使用较小修正，防止左右反复摆动 */
    if (error >= -1.0f && error <= 1.0f)
    {
        correction = center_kp * error;
    }
    else
    {
        correction = trace_kp * error;
    }     

    /*
     * 黑线在左侧：左轮减速，右轮加速。
     * 黑线在右侧：左轮加速，右轮减速。
     */
    target_speed_1 =
        limit_target_speed(base_speed + correction);

    target_speed_2 =
        limit_target_speed(base_speed - correction);

    return black_count;
}