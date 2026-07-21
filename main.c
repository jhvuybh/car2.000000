
#include "ti_msp_dl_config.h"
#include<stdio.h>
#include<stdint.h>
#include "motor.h"
#include "huidu.h"
#include "uart.h"
#include "delay.h"
#include "key.h"
#include "system_time.h"
#include "LED.h"
#include "k230.h"



extern float target_speed_1;// 左电机目标速度 mm/s
extern float target_speed_2;// 右电机目标速度 mm/s


/* 小车任务状态 */
// typedef enum
// {
//     STATE_WAIT_ROOM,       // 等待K230发送房号
//     STATE_WAIT_LOAD,       // 等待装药
//     STATE_OUTWARD,         // 前往病房
//     STATE_WAIT_UNLOAD,     // 等待卸药
//     STATE_TURN_BACK,       // 掉头
//     STATE_RETURNING,       // 返回药房
//     STATE_FINISHED         // 任务完成
// } MissionState;



int main(void)
{
    // MissionState mission_state = STATE_WAIT_ROOM;

    // uint8_t target_room = 0;// 目标房号
    // uint8_t received_room;// K230发送的房号
    uint8_t cross_locked = 0; /* 防止同一个路口重复计数 */
    uint8_t cross_count = 0;  /* 已经过的十字路口数量 */
    uint8_t black_count = 0;// 检测到黑线的灰度模块数量
    SYSCFG_DL_init();
    motor_init(1);// 初始化电机和pid
    motor_init(2);
    LED_Init();



    while (1)
    {
        black_count = adjust_motor();

        /* 第一次进入十字路口 */
        if ((black_count >= 6) && (cross_locked == 0))
        {
            cross_locked = 1;

            car_turn_around();
        }

        /*
         * 离开十字路口后重新允许检测。
         * 正常细线一般只有1～2路检测到黑色。
         */
        if ((black_count <= 2) && (cross_locked == 1))
        {
            cross_locked = 0;
        }

        delay_ms(2);
    }
}
//      while (1)
//      {
//          switch (mission_state)
//          {
//              /*
//               * 等待K230识别目标房号
//               */
//              case STATE_WAIT_ROOM:
//                  car_stop();
//                  received_room = k230_get_room();
//                  if (received_room >= 1 &&
//                      received_room <= 8)
//                  {
//                      target_room = received_room;
//                      mission_state = STATE_WAIT_LOAD;
//                  }
//                  break;
//              /*
//               * 已经识别房号，等待装药
//               */
//              case STATE_WAIT_LOAD:
//                  car_stop();
//                  if (medicine_is_loaded() == 1)
//                  {
                
//                      route_reset();
//                      motor_set_direction(1, 1);
//                      motor_set_direction(2, 1);
//                      mission_state = STATE_OUTWARD;
//                  }
//                  break;
//              /*
//               * 按照固定路线前往病房
//               */
//              case STATE_OUTWARD:
//                  if (route_update(target_room, 0) == 1)
//                  {
//                      car_stop();
//                      /* 到达病房，停车并点亮红灯 */
             
//                      mission_state = STATE_WAIT_UNLOAD;
//                  }
//                  break;
//              /*
//               * 到达病房，等待人工卸药
//               */
//              case STATE_WAIT_UNLOAD:
//                  car_stop();
//                  if (medicine_is_loaded() == 0)
//                  {
             
//                      mission_state = STATE_TURN_BACK;
//                  }
//                  break;
//              /*
//               * 卸药完成，小车掉头
//               */
//              case STATE_TURN_BACK:
//                  car_turn_around();
//                  route_reset();
//                  mission_state = STATE_RETURNING;
//                  break;
//              /*
//               * 按照反向路线返回药房
//               */
//              case STATE_RETURNING:
//                  if (route_update(target_room, 1) == 1)
//                  {
//                      car_stop();
//                      /* 返回药房，点亮绿灯 */
           
//                      mission_state = STATE_FINISHED;
//                  }
//                  break;
//              /*
//               * 任务完成，保持停车
//               */
//              case STATE_FINISHED:
//                  car_stop();
//                  /*
//                   * 比赛下一次测试可以按复位键，
//                   * 不需要在这里自动开始下一次任务。
//                   */
//                  break;
//              /*
//               * 异常状态，立即停车
//               */
//              default:
//                  car_stop();
//                  mission_state = STATE_WAIT_ROOM;
//                  break;
//          }
//          /*
//           * 主循环不需要很长的延时，
//           * 否则可能漏掉路口。
//           */
//          delay_ms(1);
//      }
// }