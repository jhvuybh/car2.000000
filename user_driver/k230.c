#include "k230.h"
/* SysConfig生成的外设定义，提供K230_INST以及DriverLib UART接口。 */
#include "ti_msp_dl_config.h"

/* MSPM0 -> K230任务命令帧：A5 TASK TARGET 5A。 */
#define K230_COMMAND_HEADER 0xA5
#define K230_COMMAND_TAIL   0x5A
#define K230_TX_FIFO_WAIT_LIMIT 100000U

/* K230 -> MSPM0识别结果帧：AA ROOM SIDE FOUND TASK 55。 */
#define K230_RESULT_HEADER  0xAA
#define K230_RESULT_TAIL    0x55

#define K230_RX_BUFFER_SIZE 64U
#define K230_RX_BUFFER_MASK (K230_RX_BUFFER_SIZE - 1U)

/* 置1时启用MSPM0 UART2内部回环并自动发送一帧1号房结果，仅用于定位故障。 */
#define K230_INTERNAL_LOOPBACK_TEST 0U

static volatile uint8_t k230_rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint8_t k230_rx_write_index = 0U;
static volatile uint8_t k230_rx_read_index = 0U;

/* 全局诊断量：即使CPU停在其他中断中，也能从CCS Watch直接观察。 */
volatile uint32_t k230_debug_rx_byte_count = 0U;
volatile uint32_t k230_debug_build_id = 0U;
volatile uint32_t k230_debug_poll_count = 0U;
volatile uint32_t k230_debug_init_count = 0U;
volatile uint32_t k230_debug_irq_count = 0U;
volatile uint32_t k230_debug_valid_frame_count = 0U;
volatile uint32_t k230_debug_rejected_frame_count = 0U;
volatile uint8_t k230_debug_last_rx_byte = 0U;
volatile uint8_t k230_debug_parser_state = 0U;
volatile uint8_t k230_debug_rx_pin_level = 0U;
volatile uint8_t k230_debug_rx_fifo_empty = 1U;
volatile uint8_t k230_debug_last_iidx = 0U;
volatile uint8_t k230_debug_loopback_enabled = 0U;
volatile k230_result_t k230_debug_last_result = {
    0U,
    K230_SIDE_CENTER,
    false,
    K230_TASK_INITIAL_ROOM
};

static void k230_capture_rx_fifo(void)
{
    while (DL_UART_isRXFIFOEmpty(K230_INST) == false)
    {
        uint8_t receive_data = DL_UART_receiveData(K230_INST);
        uint8_t next_write =
            (uint8_t)((k230_rx_write_index + 1U) & K230_RX_BUFFER_MASK);

        if (next_write == k230_rx_read_index)
        {
            /* 满缓冲区丢弃最旧字节，保留新数据并等待下一个0xAA重同步。 */
            k230_rx_read_index =
                (uint8_t)((k230_rx_read_index + 1U) & K230_RX_BUFFER_MASK);
            k230_debug_rejected_frame_count++;
        }

        k230_rx_buffer[k230_rx_write_index] = receive_data;
        k230_rx_write_index = next_write;
        k230_debug_rx_byte_count++;
        k230_debug_last_rx_byte = receive_data;
    }
}

void k230_init(void)
{
    static const uint8_t loopback_frame[6] = {
        K230_RESULT_HEADER,
        1U,
        K230_SIDE_CENTER,
        1U,
        K230_TASK_INITIAL_ROOM,
        K230_RESULT_TAIL
    };
    uint8_t i;

    k230_rx_write_index = 0U;
    k230_rx_read_index = 0U;
    k230_debug_build_id = 1610U;
    k230_debug_init_count++;

    DL_UART_Main_enableFIFOs(K230_INST);
    DL_UART_Main_setRXFIFOThreshold(
        K230_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY
    );
    DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);

    if (K230_INTERNAL_LOOPBACK_TEST != 0U)
    {
        k230_debug_loopback_enabled = 1U;

        for (i = 0U; i < sizeof(loopback_frame); i++)
        {
            while (DL_UART_isTXFIFOFull(K230_INST) != false)
            {
                /* 仅发送6字节自测帧，等待FIFO空位。 */
            }
            DL_UART_transmitData(K230_INST, loopback_frame[i]);
        }
    }
}

/* ISR只搬运原始字节；完整帧校验仍由k230_get_result()在主循环完成。 */
void K230_INST_IRQHandler(void)
{
    DL_UART_IIDX pending_interrupt =
        DL_UART_Main_getPendingInterrupt(K230_INST);

    k230_debug_irq_count++;
    k230_debug_last_iidx = (uint8_t)pending_interrupt;

    switch (pending_interrupt)
    {
        case DL_UART_MAIN_IIDX_RX:
            k230_capture_rx_fifo();
            break;

        default:
            break;
    }
}

/*
 * 向K230发送视觉任务命令。
 *
 * task决定K230采用哪种检测框筛选方式：
 *
 * K230_TASK_INITIAL_ROOM：
 *     初次识别目标房号，只接受画面中央区域中的数字，target_room必须为0。
 *
 * K230_TASK_FIND_TARGET：
 *     小车行驶到后续观察点时，只寻找target_room指定的房号，并判断左/中/右，
 *     target_room必须在1~8范围内。
 */
bool k230_set_task(k230_task_t task, uint8_t target_room)
{
    uint8_t command_frame[4];
    uint8_t i;

    /* TASK1不需要目标数字，因此只接受target_room=0。 */
    if (task == K230_TASK_INITIAL_ROOM)
    {
        if (target_room != 0)
        {
            return false;
        }
    }
    /* TASK2必须明确告诉K230需要寻找1~8中的哪一个房号。 */
    else if (task == K230_TASK_FIND_TARGET)
    {
        if (target_room < 1 || target_room > 8)
        {
            return false;
        }
    }
    /* 其他任务号未定义，拒绝发送。 */
    else
    {
        return false;
    }

    /* 按固定顺序组成4字节二进制命令帧。 */
    command_frame[0] = K230_COMMAND_HEADER;
    command_frame[1] = (uint8_t)task;
    command_frame[2] = target_room;
    command_frame[3] = K230_COMMAND_TAIL;

    /*
     * 这里只等待TX FIFO出现空位，不调用DL_UART_transmitDataBlocking()。
     * DriverLib的阻塞版本在写入每个字节后还会等待整个UART BUSY清零；若BUSY
     * 因接收活动或异常状态持续为1，主程序会永远卡住，无法进入接收主循环。
     */
    for (i = 0; i < sizeof(command_frame); i++)
    {
        uint32_t wait_count = K230_TX_FIFO_WAIT_LIMIT;

        while (DL_UART_isTXFIFOFull(K230_INST) != false)
        {
            if (--wait_count == 0U)
            {
                /* UART异常时返回失败，不能让整个任务状态机永久卡死。 */
                return false;
            }
        }
        DL_UART_transmitData(K230_INST, command_frame[i]);
    }

    return true;
}

/*
 * 从K230对应的UART接收FIFO中读取数据，并解析6字节识别结果帧：
 *
 *     AA ROOM SIDE FOUND TASK 55
 *
 * 状态机流程：
 *
 *     状态0：等待帧头0xAA
 *       ↓
 *     状态1：读取房号0~8
 *       ↓
 *     状态2：读取方向0~2
 *       ↓
 *     状态3：读取成功标志0或1
 *       ↓
 *     状态4：读取任务号1或任务号2
 *       ↓
 *     状态5：检查帧尾0x55
 *       ↓
 *     完整有效后写入result并返回true
 *
 * 函数中的解析变量使用static保存，因为一帧数据可能分多次进入UART FIFO。
 * 本次调用即使只收到半帧，下次调用仍会从上一次状态继续解析。
 */
bool k230_get_result(k230_result_t *result)
{
    /* 当前解析状态，初始为0，即等待结果帧头0xAA。 */
    static uint8_t parser_state = 0;
    /* 以下变量临时保存一帧中已经通过范围检查的各字段。 */
    static uint8_t received_room = 0;
    static uint8_t received_side = 0;
    static uint8_t received_found = 0;
    static uint8_t received_task = 0;

    k230_debug_poll_count++;

    /*
     * 中断未触发时仍直接搬运硬件FIFO，避免把接收正确性完全依赖在NVIC上。
     * 临时屏蔽UART2 IRQ可防止主循环和ISR同时读取同一个FIFO。
     */
    NVIC_DisableIRQ(K230_INST_INT_IRQN);
    k230_capture_rx_fifo();
    NVIC_EnableIRQ(K230_INST_INT_IRQN);

    k230_debug_rx_pin_level =
        ((DL_GPIO_readPins(GPIO_K230_RX_PORT, GPIO_K230_RX_PIN) &
          GPIO_K230_RX_PIN) != 0U) ? 1U : 0U;
    k230_debug_rx_fifo_empty =
        (k230_rx_read_index == k230_rx_write_index) ? 1U : 0U;

    /* 软件缓冲区为空时立即退出，所以本函数不会阻塞循迹和电机控制。 */
    while (k230_rx_read_index != k230_rx_write_index)
    {
        uint8_t receive_data = k230_rx_buffer[k230_rx_read_index];
        k230_rx_read_index =
            (uint8_t)((k230_rx_read_index + 1U) & K230_RX_BUFFER_MASK);

        k230_debug_rx_fifo_empty = 0U;

        switch (parser_state)
        {
            /* 状态0：寻找固定结果帧头0xAA，其他噪声字节全部丢弃。 */
            case 0:
                if (receive_data == K230_RESULT_HEADER)
                {
                    parser_state = 1;
                }
                break;

            /* 状态1：读取ROOM。无稳定结果时为0，识别成功时为1~8。 */
            case 1:
                if (receive_data <= 8)
                {
                    received_room = receive_data;
                    parser_state = 2;
                }
                else
                {
                    k230_debug_rejected_frame_count++;
                    /* 错误字节若恰好为新帧头，则立即开始解析下一帧。 */
                    parser_state = (receive_data == K230_RESULT_HEADER) ? 1 : 0;
                }
                break;

            /* 状态2：读取SIDE，合法值为0中间、1左边、2右边。 */
            case 2:
                if (receive_data <= K230_SIDE_RIGHT)
                {
                    received_side = receive_data;
                    parser_state = 3;
                }
                else
                {
                    k230_debug_rejected_frame_count++;
                    parser_state = (receive_data == K230_RESULT_HEADER) ? 1 : 0;
                }
                break;

            /* 状态3：读取FOUND，0表示无稳定结果，1表示识别成功。 */
            case 3:
                if (receive_data <= 1)
                {
                    received_found = receive_data;
                    parser_state = 4;
                }
                else
                {
                    k230_debug_rejected_frame_count++;
                    parser_state = (receive_data == K230_RESULT_HEADER) ? 1 : 0;
                }
                break;

            /* 状态4：读取任务号，只允许TASK1或TASK2。 */
            case 4:
                if (receive_data == K230_TASK_INITIAL_ROOM ||
                    receive_data == K230_TASK_FIND_TARGET)
                {
                    received_task = receive_data;
                    parser_state = 5;
                }
                else
                {
                    k230_debug_rejected_frame_count++;
                    parser_state = (receive_data == K230_RESULT_HEADER) ? 1 : 0;
                }
                break;

            /* 状态5：检查固定帧尾，并验证ROOM与FOUND组合是否合理。 */
            case 5:
                /* 检查完帧尾后默认重新等待下一帧。 */
                parser_state = 0;

                if (receive_data == K230_RESULT_TAIL &&
                    ((received_found == 0U && received_room == 0U) ||
                     (received_found == 1U &&
                      received_room >= 1U && received_room <= 8U)) &&
                    result != NULL)
                {
                    /* 只有完整帧全部校验通过后才覆盖调用者保存的旧结果。 */
                    result->room = received_room;
                    result->side = (k230_side_t)received_side;
                    result->found = (received_found != 0);
                    result->task = (k230_task_t)received_task;

                    k230_debug_last_result.room = received_room;
                    k230_debug_last_result.side = (k230_side_t)received_side;
                    k230_debug_last_result.found = (received_found != 0);
                    k230_debug_last_result.task = (k230_task_t)received_task;
                    k230_debug_valid_frame_count++;
                    k230_debug_parser_state = parser_state;
                    return true;
                }

                k230_debug_rejected_frame_count++;

                /* 错误帧尾若恰好是0xAA，保留它作为下一帧的帧头。 */
                if (receive_data == K230_RESULT_HEADER)
                {
                    parser_state = 1;
                }
                break;

            /* 状态变量异常时恢复到最安全的等待帧头状态。 */
            default:
                k230_debug_rejected_frame_count++;
                parser_state = 0;
                break;
        }

        k230_debug_parser_state = parser_state;
    }

    k230_debug_rx_fifo_empty =
        (k230_rx_read_index == k230_rx_write_index) ? 1U : 0U;

    /* 当前FIFO中没有组成一帧完整有效结果。 */
    return false;
}

/*
 * 兼容旧代码的简化接口，只在K230稳定识别成功时返回房号。
 * 新程序需要房号左右位置、FOUND和TASK时，应直接使用k230_get_result()。
 */
uint8_t k230_get_room(void)
{
    k230_result_t result;

    if (k230_get_result(&result) && result.found)
    {
        return result.room;
    }

    /* 有效房号是1~8，因此0可安全表示“没有新的稳定识别结果”。 */
    return 0;
}
