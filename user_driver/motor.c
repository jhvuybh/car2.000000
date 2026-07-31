#include "motor.h"
#include "system_time.h"

#define PWM_MAX_DUTY      8000// 最大占空比/10000

//转弯参数
#define CROSS_FORWARD_SPEED   300.0f    // 检测到十字路口后，继续向前行驶的速度
#define TURN_90_SPEED         300.0f   // 90度转弯速度
#define CROSS_CENTER_TIME_MS  800U //继续行驶时间
#define TURN_90_TIME_MS       870U //转弯时间

//设置掉头参数
#define TURN_AROUND_TIME_MS    2010U //掉头时间
#define TURN_AROUND_SPEED      300.0f //掉头速度




 

// 初始化电机
void motor_init(uint8_t motor_id)
{
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);
    if(motor_id == 1){
        // DL_Timer_startCounter(PWMAB_INST);
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(MOTOR_PWMAB_INST, 0, GPIO_MOTOR_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(MOTOR_PWMAB_INST, 0, GPIO_MOTOR_PWMAB_C1_IDX);
    }
    DL_Timer_startCounter(MOTOR_PWMAB_INST);
    DL_Timer_startCounter(PID_INST);
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    NVIC_EnableIRQ(PID_INST_INT_IRQN);
}

// 限幅函数
int limit_duty(int duty)
{
    if(duty > PWM_MAX_DUTY){
        duty = PWM_MAX_DUTY;
    }
    if(duty < 0)
    {
        duty = 0;
    }
    return duty;
}
// 设置电机占空比
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    duty = limit_duty(duty);
    if(motor_id == 1){
        DL_Timer_setCaptureCompareValue(MOTOR_PWMAB_INST, duty, GPIO_MOTOR_PWMAB_C0_IDX);
    }
    else if(motor_id == 2){
        DL_Timer_setCaptureCompareValue(MOTOR_PWMAB_INST, duty, GPIO_MOTOR_PWMAB_C1_IDX);
    }
}

// direction: 0 停止，1 正转，2 反转
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}




void car_forward()       //直行
{
    extern float target_speed_1;
    extern float target_speed_2;
    motor_set_direction( 1, 1);
    motor_set_direction( 2, 1);
    target_speed_1 = 300.0f;
    target_speed_2 = 300.0f;
}





void car_stop(void)// 停止小车
{
    extern float target_speed_1;
    extern float target_speed_2;
    extern int PWM_1_duty;
    extern int PWM_2_duty;  
    extern float current_error_1;
    extern float last_error_1;
    extern float current_error_2;
    extern float last_error_2;

    target_speed_1 = 0;
    target_speed_2 = 0;

    PWM_1_duty = 0;
    PWM_2_duty = 0;

    current_error_1 = 0;
    last_error_1 = 0;
    current_error_2 = 0;
    last_error_2 = 0;

    motor_set_duty(1, 0);
    motor_set_duty(2, 0);

    motor_set_direction(1, 0);
    motor_set_direction(2, 0);
}



void car_turn_around(void)// 小车掉头
{
    uint32_t start_time;
    extern float target_speed_1;
    extern float target_speed_2;
    /*
     * 先停车，再改变方向，避免电机突然反转。
     */
    car_stop();// 停车

    /*
     * 左轮前进，右轮后退，原地旋转。
     */
    motor_set_direction(1, 1);
    motor_set_direction(2, 2);

    target_speed_1 = TURN_AROUND_SPEED;
    target_speed_2 = TURN_AROUND_SPEED;
    // motor_wait_ms(TURN_AROUND_TIME_MS);
    start_time = SystemTime_GetMs();

    /*
     * 固定时间旋转。
     * SystemTime_Tick()由10ms定时器中断更新。
     */
    while (SystemTime_IsOver(
               start_time,
               TURN_AROUND_TIME_MS) == 0)
    {
        /* 等待掉头时间结束 */
    }

    /* 掉头时间结束，立即停车 */
    car_stop();// 停车

    /* 恢复两轮前进方向 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);
}



static void motor_wait_ms(uint32_t wait_ms)  // 等待指定毫秒数，同时保持PID和系统时间继续运行
{
    uint32_t start_time = SystemTime_GetMs();

    while (SystemTime_IsOver(start_time, wait_ms) == 0)
    {
        /* PID和系统时间继续在中断中运行 */
    }
}




void car_turn_left_90(void)  // 小车左转90度
{
    extern float target_speed_1;
    extern float target_speed_2;

    /* 先向前走到十字路口中心 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    target_speed_1 = CROSS_FORWARD_SPEED;
    target_speed_2 = CROSS_FORWARD_SPEED;

    motor_wait_ms(CROSS_CENTER_TIME_MS);
    car_stop();

    /* 左轮后退，右轮前进，原地左转 */
    motor_set_direction(1, 2);
    motor_set_direction(2, 1);

    target_speed_1 = TURN_90_SPEED;
    target_speed_2 = TURN_90_SPEED;

    motor_wait_ms(TURN_90_TIME_MS);
    car_stop();

    /* 恢复前进方向 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);
}

void car_turn_right_90(void)         // 小车右转90度
{
    extern float target_speed_1;
    extern float target_speed_2;

    /* 先向前走到十字路口中心 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);

    target_speed_1 = CROSS_FORWARD_SPEED;
    target_speed_2 = CROSS_FORWARD_SPEED;

    motor_wait_ms(CROSS_CENTER_TIME_MS);
    car_stop();

    /* 左轮前进，右轮后退，原地右转 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 2);

    target_speed_1 = TURN_90_SPEED;
    target_speed_2 = TURN_90_SPEED;

    motor_wait_ms(TURN_90_TIME_MS);
    car_stop();

    /* 恢复前进方向 */
    motor_set_direction(1, 1);
    motor_set_direction(2, 1);
}



extern uint32_t counter_1_A; 
float speed_1 = 0;

extern uint32_t counter_2_A;
float speed_2 = 0;

/*
 * 编码器测速低通滤波系数（后续可调范围0.15~0.40）：
 *   数值越小，电机越平顺，但速度反馈延迟越大；
 *   数值越大，响应越快，但低速脉冲量化造成的“一冲一冲”更明显。
 * 当前PID周期为10ms、编码器每周期脉冲较少，建议先使用0.25。
 * 这里只滤波速度反馈，里程仍累计原始脉冲，不影响停车距离精度。
 */
volatile float motor_speed_filter_alpha = 0.20f;   //响应速度慢调高,顿挫调低
volatile float motor_raw_speed_1 = 0.0f; /* CCS Watch：左轮未滤波速度 */
volatile float motor_raw_speed_2 = 0.0f; /* CCS Watch：右轮未滤波速度 */

/*
 * 左右轮编码器脉冲总和。
 * PID中断每次读取瞬时脉冲前先累加到这里；除以2后就是两轮平均脉冲数。
 * 返程只在直行段读取该里程，因此原地转弯结束后必须由主状态机清零。
 */
static volatile uint32_t odometer_pulse_sum = 0U;
static volatile uint8_t odometer_enabled = 0U;

void motor_odometer_reset(void)
{
    odometer_pulse_sum = 0U;
    /* 同时丢弃距离清零前、尚未来得及被PID中断读取的零散脉冲。 */
    counter_1_A = 0U;
    counter_2_A = 0U;
}

void motor_odometer_set_enabled(uint8_t enabled)
{
    odometer_enabled = (enabled != 0U) ? 1U : 0U;
}

float motor_odometer_get_mm(void)
{
    uint32_t pulse_sum_snapshot = odometer_pulse_sum;

    return ((float)pulse_sum_snapshot * PI * MOTOR_WHEEL_D) /
           (2.0f * MOTOR_BIANMAQI);
}

// 计算电机速度
void calculate_speed(uint8_t motor_id)
{
    float alpha = motor_speed_filter_alpha;

    /* 防止调试时输入非法系数。 */
    if (alpha < 0.01f) {
        alpha = 0.01f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    if (motor_id == 1) {
        /* 保存本周期左轮脉冲，随后瞬时测速计数器会清零。 */
        if (odometer_enabled != 0U) {
            odometer_pulse_sum += counter_1_A;
        }
        motor_raw_speed_1 = (float)counter_1_A / MOTOR_BIANMAQI *
                            PI * MOTOR_WHEEL_D * 100.0f;
        speed_1 += alpha * (motor_raw_speed_1 - speed_1);
        counter_1_A = 0; // 计算完速度后清零计数器
    }
    if (motor_id == 2) {
        /* 保存本周期右轮脉冲；里程函数最终取左右轮平均值。 */
        if (odometer_enabled != 0U) {
            odometer_pulse_sum += counter_2_A;
        }
        motor_raw_speed_2 = (float)counter_2_A / MOTOR_BIANMAQI *
                            PI * MOTOR_WHEEL_D * 100.0f;
        speed_2 += alpha * (motor_raw_speed_2 - speed_2);
        counter_2_A = 0; // 计算完速度后清零计数器
    }
}


//增量式pid控制
float kp = 16.0f;          // 比例系数
float ki = 0.04f;         // 积分系数
float kd = 5.0f;        // 微分系数

int PWM_1_duty = 0;
float target_speed_1 = 0;         // 目标速度 mm/s
float last_error_1 = 0;          // 上一次误差
float last_last_error_1 = 0.0f; // 上上次误差
float current_error_1 = 0;     // 当前误差


int PWM_2_duty = 0;
float target_speed_2 = 0; // 目标速度 mm/s
float last_error_2 = 0;
float last_last_error_2 = 0.0f;
float current_error_2 = 0;
// 电机PID调节函数

void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;
    if (motor_id == 1) {
        error = target_speed_1 - speed_1;// 计算速度误差
        current_error_1 = error;//左电机误差
        PWM_1_duty += (int)(kp * (current_error_1 - last_error_1) + ki *(current_error_1) + kd*(current_error_1 - 2*(last_error_1) + last_last_error_1));// 增量式PID公式
        last_last_error_1 = last_error_1;
        last_error_1 = current_error_1;
        PWM_1_duty = limit_duty(PWM_1_duty);
        motor_set_duty(motor_id, PWM_1_duty);
    }
    if (motor_id == 2) {
        error = target_speed_2 - speed_2;
        current_error_2 = error;//右电机误差
        PWM_2_duty += (int)(kp * (current_error_2 - last_error_2) + ki *(current_error_2) + kd*(current_error_2 - 2*(last_error_2) + last_last_error_2));// 增量式PID公式
        last_last_error_2 = last_error_2;
        last_error_2 = current_error_2;
        PWM_2_duty = limit_duty(PWM_2_duty);
        motor_set_duty(motor_id, PWM_2_duty);
    }
}

void PID_INST_IRQHandler()// PID定时器中断服务函数
{
    switch (DL_Timer_getPendingInterrupt(PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        SystemTime_Tick();// 更新系统时间
        calculate_speed(1);//`计算电机1速度
        DC_MOTOR_PID(1);// 调整电机1速度
        calculate_speed(2);// 计算电机2速度
        DC_MOTOR_PID(2);// 调整电机2速度
        break;
    
    default:
        break;
    }
}



