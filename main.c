#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "LED.h"
#include "delay.h"
#include "huidu.h"
#include "k230.h"
#include "key.h"
#include "motor.h"
#include "system_time.h"

/*
 * 基本要求路线说明：
 *
 * 路口1：近端病房。1号固定在左侧，2号固定在右侧；其他房号直行。
 * 路口2：中部病房。接近路口时K230找到目标则缓存LEFT/RIGHT；找不到则直行。
 * 路口3：接近远端主路口时，K230提前识别目标所在的左/右走廊。
 * 路口4：接近远端走廊末端时，K230提前识别具体病房方向。
 * 到达路口后只读取提前缓存的结果并立即执行，不在路口停车等待视觉识别。
 *
 * 去程把每个路口动作依次保存在route[]中；卸药并掉头后，返程倒序读取
 * route[]，并把LEFT/RIGHT互换。因此返程不再进行数字识别。
 */

/* 赛道最多经过4个路口，因此最多记录4个去程动作。 */
#define ROUTE_MAX_ACTIONS               4U
/* 8个灰度探头全部检测到黑色时，认为小车到达十字路口。 */
#define CROSS_BLACK_COUNT               8U
/* 离开路口后只剩不超过2个探头压线，此时允许识别下一个路口。 */
#define CROSS_CLEAR_BLACK_COUNT         2U
/* 病房和药房门口是黑色虚线：3~7路黑为终点；8路全黑专用于路口。 */
#define FINISH_MIN_BLACK_COUNT           3U
/* 出发后先驶离起点虚线；到时且回到正常窄线后才允许识别第一个路口。 */
#define START_GUARD_TIME_MS             1000U
/* 行驶过程中周期性重发视觉任务命令，防止K230漏收一次命令。 */
#define VISION_COMMAND_RETRY_MS         500U
/*
 * 最远端返程前两个直角弯不一定出现8路全黑，因此改用左右半边灰度图案判断：
 * 预期转向一侧至少3路黑线，另一侧最多1路黑线，并连续满足3次才确认到弯。
 */
/* 预期转向一侧4个探头中，至少需要检测到3个黑色。 */
#define RETURN_CORNER_SIDE_MIN_BLACK    3U
/* 非转向一侧最多允许探头检测到黑色，防止把宽线误认为直角弯。 */
#define RETURN_CORNER_OTHER_MAX_BLACK   3U
/* 上述左右图案必须连续出现3次，过滤单次抖动和瞬时干扰。 */
#define RETURN_CORNER_STABLE_SAMPLES    3U

#define NEAR_CROSS_INDEX                1U
#define MIDDLE_CROSS_INDEX              2U
#define REMOTE_MAIN_CROSS_INDEX         3U
#define REMOTE_ROOM_CROSS_INDEX         4U

#define LED_GREEN_ID                    1U
/* 当前小车没有红灯：使用2号蓝灯表示已到达病房并正在等待卸药。 */
#define LED_BLUE_ID                     2U

typedef enum
{
    MISSION_IDENTIFY_ROOM,   /* 在药房读取工作人员手持的目标房号 */
    MISSION_WAIT_LOAD,       /* 已得到房号，停车等待药品装入 */
    MISSION_OUTWARD_TRACE,   /* 去程循迹，同时提前缓存K230给出的路口方向 */
    MISSION_WAIT_UNLOAD,     /* 到达病房，亮蓝灯并等待药品被取走 */
    MISSION_RETURN_TRACE,    /* 按去程记录的反向动作返回药房 */
    MISSION_FINISHED,        /* 已回到药房，停车并保持绿灯点亮 */
    MISSION_FAULT            /* 到路口前未识别方向或路线异常，停车等待复位 */
} MissionState;

/* 路线表中的一个动作。直行也必须记录，否则返程路口顺序会错位。 */
typedef enum
{
    ROUTE_STRAIGHT,          /* 穿过当前路口继续直行 */
    ROUTE_LEFT,              /* 在当前路口左转90度 */
    ROUTE_RIGHT              /* 在当前路口右转90度 */
} RouteAction;

/*
 * 去程路线栈示例：route = [直行, 直行, 左转, 右转]。
 * 返程从数组尾部向前读取，并把左右互换，执行顺序为左转、右转、直行、直行。
 */
static RouteAction route[ROUTE_MAX_ACTIONS];
/* route[]中已经保存的有效动作数量。 */
static uint8_t route_length = 0U;
/* 返程当前还剩多少个路口动作没有执行，同时也是倒序读取下标。 */
static uint8_t return_route_index = 0U;

/*
 * 在药房识别到的本次目标房号，合法范围为1~8。
 * 放在文件作用域并声明为volatile，方便程序停在UART/定时器中断时仍可直接在
 * CCS Watch中观察；它仍只由main状态机在收到完整有效的K230结果帧后更新。
 */
volatile uint8_t target_room = 0U;

/* 将一次去程动作保存到路线数组；数组已满时返回false。 */
static bool route_push(RouteAction action)
{
    if (route_length >= ROUTE_MAX_ACTIONS)
    {
        return false;
    }

    route[route_length] = action;
    route_length++;
    return true;
}

/*
 * 计算返程动作：
 * 去程左转的路口，反方向到达时需要右转；去程右转则需要左转；直行不变。
 */
static RouteAction route_reverse_action(RouteAction action)
{
    if (action == ROUTE_LEFT)
    {
        return ROUTE_RIGHT;
    }

    if (action == ROUTE_RIGHT)
    {
        return ROUTE_LEFT;
    }

    return ROUTE_STRAIGHT;
}

/* 调用底层电机接口执行一个路口动作。 */
static void route_execute_action(RouteAction action)
{
    /* 固定路口动作开始后，灰度只能读取，不能再修改左右轮速度。 */
    huidu_set_tracking_enabled(0U);

    if (action == ROUTE_LEFT)
    {
        car_turn_left_90();
    }
    else if (action == ROUTE_RIGHT)
    {
        car_turn_right_90();
    }

    /* 直行无需额外动作；转向函数结束后也必须重新给出前进速度。 */
    car_forward();
    /* 固定动作结束后立即恢复灰度循迹，由cross_locked防止重复识别路口。 */
    huidu_set_tracking_enabled(1U);
}

/* 把K230给出的画面左/右位置转换成小车的左/右转动作。 */
static bool route_action_from_side(k230_side_t side, RouteAction *action)
{
    if (action == NULL)
    {
        return false;
    }

    if (side == K230_SIDE_LEFT)
    {
        *action = ROUTE_LEFT;
        return true;
    }

    if (side == K230_SIDE_RIGHT)
    {
        *action = ROUTE_RIGHT;
        return true;
    }

    /* 赛道分支只有左、右两个方向，CENTER不能作为转向依据。 */
    return false;
}

/* 清掉路口停车前积压的旧视觉帧，避免下一路口使用上一路口的方向。 */
static void k230_discard_pending_results(void)
{
    k230_result_t discarded_result;

    while (k230_get_result(&discarded_result))
    {
        /* 只负责清空FIFO中的完整旧帧。 */
    }
}

/*
 * 行驶中持续读取K230结果，并锁存最近一次稳定的LEFT/RIGHT。
 *
 * 一旦识别成功，即使数字随后被车身遮挡或暂时离开画面，也保留该方向，
 * 直到小车通过当前路口。每通过一个路口，主函数会主动清除这个缓存，
 * 因此不会把上一个路口的方向误用于下一个路口。
 */
static void k230_latch_route_side(
    uint8_t target_room,
    k230_side_t *cached_side,
    uint8_t *cached_valid)
{
    k230_result_t result;

    while (k230_get_result(&result))
    {
        if (result.found &&
            result.task == K230_TASK_FIND_TARGET &&
            result.room == target_room &&
            (result.side == K230_SIDE_LEFT ||
             result.side == K230_SIDE_RIGHT))
        {
            *cached_side = result.side;
            *cached_valid = 1U;
        }
    }
}

int main(void)
{
    /* mission_state决定主循环当前执行哪个比赛阶段。 */
    MissionState mission_state = MISSION_IDENTIFY_ROOM;
    /* 保存最近一次从K230串口收到的完整识别结果。 */
    k230_result_t vision_result;
    /* 临时保存当前路口即将执行的动作。 */
    RouteAction action;

    /* 去程经过的路口编号：1近端、2中部、3远端主路口、4远端末端。 */
    uint8_t outward_cross_index = 0U;
    /* 当前有多少个灰度探头检测到黑色。 */
    uint8_t black_count = 0U;
    /* 路口锁：进入路口置1，完全离开路口后清0，防止重复计数。 */
    uint8_t cross_locked = 0U;
    /* 起点虚线保护：置1时只循迹，不把起点标记计为路口。 */
    uint8_t start_guard_active = 0U;
    /* 置1表示已经转入具体病房，之后检测到黑白门线即可停车。 */
    uint8_t heading_to_room = 0U;
    /* 最远端返程直角弯需要连续多次满足方向图案后才确认为路口。 */
    uint8_t return_corner_stable_count = 0U;
    /* 接近路口时提前锁存的目标房号方向，以及该缓存是否有效。 */
    k230_side_t cached_route_side = K230_SIDE_CENTER;
    uint8_t cached_route_side_valid = 0U;

    /* 以下变量分别记录起点保护和视觉命令重发的起始时刻。 */
    uint32_t start_guard_start = 0U;
    uint32_t vision_command_time = 0U;

    /* 初始化SysConfig生成的外设、电机PID和指示灯。 */
    SYSCFG_DL_init();
    k230_init();
    motor_init(1U);
    motor_init(2U);
    LED_Init();
    car_stop();

    /*
     * TASK1只识别画面中央的房号，用于工作人员在药房手持纸张输入任务。
     * 识别成功前小车始终保持停止。
     */
    (void)k230_set_task(K230_TASK_INITIAL_ROOM, 0U);
    vision_command_time = SystemTime_GetMs();

    while (1)
    {
        switch (mission_state)
        {
            case MISSION_IDENTIFY_ROOM:
            {
                /* 阶段1：识别工作人员给出的目标房号。 */
                car_stop();

                if (k230_get_result(&vision_result) &&
                    vision_result.found &&
                    vision_result.task == K230_TASK_INITIAL_ROOM &&
                    vision_result.room >= 1U &&
                    vision_result.room <= 8U)
                {
                    target_room = vision_result.room;

                    /* 出发后K230只寻找本次任务的目标房号。 */
                    (void)k230_set_task(
                        K230_TASK_FIND_TARGET,
                        target_room
                    );
                    vision_command_time = SystemTime_GetMs();
                    mission_state = MISSION_WAIT_LOAD;
                }

                /* K230可能比主控启动慢，未识别前周期性重发TASK1。 */
                if (SystemTime_IsOver(
                        vision_command_time,
                        VISION_COMMAND_RETRY_MS))
                {
                    (void)k230_set_task(K230_TASK_INITIAL_ROOM, 0U);
                    vision_command_time = SystemTime_GetMs();
                }
                break;
            }

            case MISSION_WAIT_LOAD:
            {
                /* 阶段2：房号已确定，等待微动开关确认药品装入。 */
                car_stop();

                /* 等待期间重发TASK2，保证K230已进入寻找目标模式。 */
                if (SystemTime_IsOver(
                        vision_command_time,
                        VISION_COMMAND_RETRY_MS))
                {
                    (void)k230_set_task(
                        K230_TASK_FIND_TARGET,
                        target_room
                    );
                    vision_command_time = SystemTime_GetMs();
                }

                /* 临时测试：跳过PB13货物微动开关，识别到房号后直接出发。 */
                if (medicine_is_loaded())
                {
                    /* 新任务出发前清空上一阶段可能残留的路线和检测状态。 */
                    route_length = 0U;
                    return_route_index = 0U;
                    outward_cross_index = 0U;
                    cross_locked = 0U;
                    huidu_set_tracking_enabled(1U);
                    start_guard_active = 1U;
                    start_guard_start = SystemTime_GetMs();
                    heading_to_room = 0U;
                    cached_route_side = K230_SIDE_CENTER;
                    cached_route_side_valid = 0U;

                    k230_discard_pending_results();

                    car_forward();
                    mission_state = MISSION_OUTWARD_TRACE;
                }
                break;
            }

            case MISSION_OUTWARD_TRACE:
            {
                /* 阶段3：去程循迹，同时在到达路口前持续接收目标左右位置。 */
                black_count = adjust_motor();

                /*
                 * 数字在路口前方可见时就锁存方向；即使到路口时被车身挡住，
                 * 后续FOUND=0也不会清除已经锁存的有效结果。
                 */
                k230_latch_route_side(
                    target_room,
                    &cached_route_side,
                    &cached_route_side_valid
                );

                if (SystemTime_IsOver(
                        vision_command_time,
                        VISION_COMMAND_RETRY_MS))
                {
                    (void)k230_set_task(
                        K230_TASK_FIND_TARGET,
                        target_room
                    );
                    vision_command_time = SystemTime_GetMs();
                }

                /*
                 * 起点是虚线，可能短暂出现8路全黑。出发后的保护时间内只循迹；
                 * 时间到后还必须看到正常窄线（不超过2路黑）才开放路口判断。
                 */
                if (start_guard_active)
                {
                    if (SystemTime_IsOver(
                            start_guard_start,
                            START_GUARD_TIME_MS) &&
                        black_count <= CROSS_CLEAR_BLACK_COUNT)
                    {
                        start_guard_active = 0U;
                        cross_locked = 0U;
                    }
                    break;
                }

                /*
                 * 只有最后一次转入具体病房，并且已经离开转弯路口后，
                 * 才把“至少3路灰度检测到黑色”的虚线判断为病房终点。
                 */
                if (heading_to_room &&
                    !cross_locked &&
                    black_count >= FINISH_MIN_BLACK_COUNT &&
                    black_count < CROSS_BLACK_COUNT)
                {
                    car_stop();
                    /* 赛题要求亮红灯；本车没有红灯，因此用蓝灯代替。 */
                    LED_ON(LED_BLUE_ID);
                    mission_state = MISSION_WAIT_UNLOAD;
                    break;
                }

                if (black_count == CROSS_BLACK_COUNT && !cross_locked)
                {
                    /*
                     * 第一次看到该路口：锁定并编号。这里只读取行驶中已经
                     * 缓存的视觉结果，不在路口等待K230重新识别。
                     */
                    cross_locked = 1U;
                    outward_cross_index++;
                    /* 转弯函数需要从停止状态开始；这里不是停车等待视觉。 */
                    car_stop();

                    if (outward_cross_index == NEAR_CROSS_INDEX)
                    {
                        /* 1、2号病房位置固定，近端路口不需要调用K230。 */
                        if (target_room == 1U)
                        {
                            action = ROUTE_LEFT;
                            heading_to_room = 1U;
                        }
                        else if (target_room == 2U)
                        {
                            action = ROUTE_RIGHT;
                            heading_to_room = 1U;
                        }
                        else
                        {
                            action = ROUTE_STRAIGHT;
                        }

                        if (!route_push(action))
                        {
                            mission_state = MISSION_FAULT;
                            break;
                        }

                        route_execute_action(action);
                        /* 通过一个路口后清空缓存，下一路口必须取得新结果。 */
                        cached_route_side = K230_SIDE_CENTER;
                        cached_route_side_valid = 0U;
                        k230_discard_pending_results();
                    }
                    else if (outward_cross_index == MIDDLE_CROSS_INDEX)
                    {
                        /*
                         * 中部路口：提前看见目标就按缓存方向转弯；没有看见
                         * 说明目标在远端，直接穿过该路口。
                         */
                        if (cached_route_side_valid &&
                            route_action_from_side(cached_route_side, &action))
                        {
                            heading_to_room = 1U;
                        }
                        else
                        {
                            action = ROUTE_STRAIGHT;
                        }

                        if (!route_push(action))
                        {
                            mission_state = MISSION_FAULT;
                            break;
                        }

                        route_execute_action(action);
                        cached_route_side = K230_SIDE_CENTER;
                        cached_route_side_valid = 0U;
                        k230_discard_pending_results();
                    }
                    else if (outward_cross_index == REMOTE_MAIN_CROSS_INDEX ||
                             outward_cross_index == REMOTE_ROOM_CROSS_INDEX)
                    {
                        /*
                         * 远端两个路口都必须在到达前得到方向。若缓存无效，
                         * 继续行驶一定会走错，因此立即进入故障停车状态。
                         */
                        if (!cached_route_side_valid ||
                            !route_action_from_side(cached_route_side, &action))
                        {
                            mission_state = MISSION_FAULT;
                            break;
                        }

                        if (!route_push(action))
                        {
                            mission_state = MISSION_FAULT;
                            break;
                        }

                        /* 远端第4个路口转弯后，下一条虚线就是目标病房。 */
                        if (outward_cross_index == REMOTE_ROOM_CROSS_INDEX)
                        {
                            heading_to_room = 1U;
                        }

                        route_execute_action(action);
                        cached_route_side = K230_SIDE_CENTER;
                        cached_route_side_valid = 0U;
                        k230_discard_pending_results();
                    }
                    else
                    {
                        /* 超过赛道最大路口数仍未到达病房，立即停车。 */
                        mission_state = MISSION_FAULT;
                    }
                }
                else if (black_count <= CROSS_CLEAR_BLACK_COUNT)
                {
                    cross_locked = 0U;
                }
                break;
            }

            case MISSION_WAIT_UNLOAD:
            {
                /* 阶段4：病房停车。微动开关松开表示药品已经被人工取走。 */
                car_stop();

                if (!medicine_is_loaded())
                {
                    /* 药品被取走，熄灭到达病房时点亮的蓝灯。 */
                    LED_OFF(LED_BLUE_ID);

                    huidu_set_tracking_enabled(0U);
                    car_turn_around();
                    /* 返程从route[]最后一个动作开始读取。 */
                    return_route_index = route_length;
                    return_corner_stable_count = 0U;
                    cross_locked = 1U;

                    car_forward();
                    huidu_set_tracking_enabled(1U);
                    mission_state = MISSION_RETURN_TRACE;
                }
                break;
            }

            case MISSION_RETURN_TRACE:
            {
                uint8_t return_route_detected;
                uint8_t left_black_count;
                uint8_t right_black_count;
                RouteAction expected_return_action;

                /* 阶段5：返程只使用路线栈和灰度循迹，不再读取K230结果。 */
                black_count = adjust_motor();

                /*
                 * 所有去程路口都已反向通过、路口保护已经结束后，
                 * 至少3路灰度检测到黑色即认为到达药房门口虚线。
                 */
                if (return_route_index == 0U &&
                    !cross_locked &&
                    black_count >= FINISH_MIN_BLACK_COUNT &&
                    black_count < CROSS_BLACK_COUNT)
                {
                    car_stop();
                    LED_ON(LED_GREEN_ID);
                    mission_state = MISSION_FINISHED;
                    break;
                }

                /* 普通十字路口仍沿用8路全黑，避免降低全局路口阈值。 */
                return_route_detected =
                    (black_count == CROSS_BLACK_COUNT) ? 1U : 0U;

                /*
                 * 最远端路线有4个去程动作。返程索引4和3分别对应最远端
                 * 病房弯和主路弯，这两个位置是直角线，不保证8路同时压黑。
                 * 根据路线栈中的预期反向动作，只检查应转向一侧的4个探头；
                 * 另一侧最多允许1路黑，并连续确认，减少普通宽弯误触发。
                 */
                if (!return_route_detected &&
                    route_length == REMOTE_ROOM_CROSS_INDEX &&
                    return_route_index >= REMOTE_MAIN_CROSS_INDEX)
                {
                    expected_return_action = route_reverse_action(
                        route[return_route_index - 1U]
                    );
                    left_black_count = huidu_get_left_black_count();
                    right_black_count = huidu_get_right_black_count();

                    if ((expected_return_action == ROUTE_LEFT &&
                         /* 预期左转：左侧至少3黑，右侧最多1黑。 */
                         left_black_count >= RETURN_CORNER_SIDE_MIN_BLACK &&
                         right_black_count <= RETURN_CORNER_OTHER_MAX_BLACK) ||
                        (expected_return_action == ROUTE_RIGHT &&
                         /* 预期右转：右侧至少3黑，左侧最多1黑。 */
                         right_black_count >= RETURN_CORNER_SIDE_MIN_BLACK &&
                         left_black_count <= RETURN_CORNER_OTHER_MAX_BLACK))
                    {
                        if (return_corner_stable_count <
                            RETURN_CORNER_STABLE_SAMPLES)
                        {
                            return_corner_stable_count++;
                        }
                    }
                    else
                    {
                        return_corner_stable_count = 0U;
                    }

                    if (return_corner_stable_count >=
                        RETURN_CORNER_STABLE_SAMPLES)
                    {
                        return_route_detected = 1U;
                    }
                }
                else
                {
                    return_corner_stable_count = 0U;
                }

                if (return_route_detected && !cross_locked)
                {
                    /* 到达返程路口后，从路线尾部弹出一个动作并执行其反动作。 */
                    cross_locked = 1U;
                    return_corner_stable_count = 0U;
                    car_stop();

                    if (return_route_index == 0U)
                    {
                        mission_state = MISSION_FAULT;
                        break;
                    }

                    return_route_index--;
                    action = route_reverse_action(route[return_route_index]);
                    route_execute_action(action);

                }
                else if (black_count <= CROSS_CLEAR_BLACK_COUNT)
                {
                    cross_locked = 0U;
                }
                break;
            }

            case MISSION_FINISHED:
            {
                /* 阶段6：保持停车和绿灯，等待下一次人工复位。 */
                car_stop();
                break;
            }

            case MISSION_FAULT:
            default:
            {
                /* 两灯同时亮表示视觉超时、路线越界或状态异常。 */
                car_stop();
                LED_ON(LED_GREEN_ID);
                LED_ON(LED_BLUE_ID);
                mission_state = MISSION_FAULT;
                break;
            }
        }

        /* 小延时降低主循环占用；循迹和串口仍能保持足够高的刷新频率。 */
        delay_ms(2U);
    }
}
