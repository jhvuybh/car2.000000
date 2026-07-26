#include "huidu.h"
#include "ti_msp_dl_config.h"
uint8_t huidu_value[] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t huidu_tracking_enabled = 1U;

void huidu_set_tracking_enabled(uint8_t enabled)
{
    huidu_tracking_enabled = (enabled != 0U) ? 1U : 0U;
}

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

/*
 * 返回最近一次灰度采样中左/右半边的黑线数量。
 * adjust_motor()会先调用huidu_get_value()，因此主状态机应在调用
 * adjust_motor()之后读取这两个结果。当前传感器约定0表示黑、1表示白。
 */
uint8_t huidu_get_left_black_count(void)
{
    uint8_t black_count = 0U;
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        if (huidu_value[i] == 0U)
        {
            black_count++;
        }
    }

    return black_count;
}

uint8_t huidu_get_right_black_count(void)
{
    uint8_t black_count = 0U;
    uint8_t i;

    for (i = 4U; i < 8U; i++)
    {
        if (huidu_value[i] == 0U)
        {
            black_count++;
        }
    }

    return black_count;
}

extern float target_speed_1;// 电机目标速度 mm/s
extern float target_speed_2;// 电机目标速度 mm/s
 
float target_speed_5[] = {225, 375, 600, 1200, 1500};// 5档速度，单位 mm/s    

/*
 * 外层循迹PD参数，可在CCS Watch中在线观察和微调。
 * 直道保持较高速度；黑线偏离中心越远，基础速度越低。
 */
volatile float trace_kp = 28.0f;
volatile float trace_kd = 20.0f;
volatile float trace_straight_speed = 650.0f;
volatile float trace_min_speed = 400.0f;
volatile float trace_slowdown = 35.0f;

/* CCS Watch调试量 */
volatile float trace_error = 0.0f;
volatile float trace_correction = 0.0f;
volatile float trace_base_speed = 0.0f;


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
    float error_change;
    float abs_error;
    float correction;
    float base_speed;

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
     * 转弯、掉头及出弯保护期间仍更新灰度值和黑线数量供状态机观察，
     * 但不允许灰度循迹覆盖固定动作已经设置好的方向和目标速度。
     */
    if (huidu_tracking_enabled == 0U)
    {
        return black_count;
    }

    /*
     * 8路全部检测到黑色。
     * 可能是终点线或者十字路口，先交给主函数处理。
     */
    if (black_count == 8)
    {
        return 8;
    }


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

        return 0;
    }

    /* 计算黑线位置 */
    error = (float)weighted_sum / black_count;// 计算黑线位置误差，负数表示黑线在左侧，正数表示黑线在右侧
    error_change = error - last_error;

    /*
     * 连续PD循迹：
     * P项根据当前位置纠偏，D项抑制越过中心后的左右摆动。
     * 不再在中心附近切换增益，避免修正量突然跳变。
     */
    correction = trace_kp * error + trace_kd * error_change;

    /* 直道高速、弯道降速，兼顾20秒完赛和弯道稳定性。 */
    abs_error = (error >= 0.0f) ? error : -error;
    base_speed = trace_straight_speed - trace_slowdown * abs_error;
    if (base_speed < trace_min_speed)
    {
        base_speed = trace_min_speed;
    }

    /* 保存调试量，并在本次计算完成后更新历史误差。 */
    trace_error = error;
    trace_correction = correction;
    trace_base_speed = base_speed;
    last_error = error;

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






/*
 * 黑白方块终点检测
 * 当前工程规定：灰度值0表示黑色，1表示白色。
 */
static uint8_t finish_stable_count = 0;

/* 统计一个8位数据中有多少个1 */
static uint8_t count_bits(uint8_t value)
{
    uint8_t count = 0;

    while (value != 0)
    {
        count += value & 1U;
        value >>= 1;
    }

    return count;
}

/* 将8路灰度传感器转换为位图，1表示黑色 */
static uint8_t get_black_mask(void)
{
    uint8_t mask = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (huidu_value[i] == 0)
        {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

/*
 * 判断当前图案是否像黑白相间的终点线。
 *
 * 普通循迹线通常是连续的1～3个黑色探头，
 * 黑白方块终点会产生多个黑白跳变。
 */
static uint8_t finish_pattern_raw(void)
{
    uint8_t mask;
    uint8_t black_count;
    uint8_t transition_count = 0;
    uint8_t i;

    mask = get_black_mask();
    black_count = count_bits(mask);

    /* 终点应有多个黑块，但不能是8路全黑路口 */
    if ((black_count < 3) || (black_count > 6))
    {
        return 0;
    }

    /* 统计相邻探头之间的黑白变化次数 */
    for (i = 0; i < 7; i++)
    {
        uint8_t bit1 = (mask >> i) & 1U;
        uint8_t bit2 = (mask >> (i + 1U)) & 1U;

        if (bit1 != bit2)
        {
            transition_count++;
        }
    }

    /* 黑白相间图案通常会有较多跳变 */
    if (transition_count >= 4)
    {
        return 1;
    }

    return 0;
}

/*
 * 连续3次识别到终点图案才确认，
 * 防止普通轨迹偶然产生相似数据。
 *
 * 调用本函数前必须先调用adjust_motor()，
 * 因为adjust_motor()会更新huidu_value[]。
 */
uint8_t finish_line_detected(void)
{
    if (finish_pattern_raw())
    {
        if (finish_stable_count < 3)
        {
            finish_stable_count++;
        }
    }
    else
    {
        finish_stable_count = 0;
    }

    if (finish_stable_count >= 3)
    {
        finish_stable_count = 0;
        return 1;
    }

    return 0;
}

void finish_detector_reset(void)
{
    finish_stable_count = 0;
}
