#include "zdt_stepper.h"

#include "system_time.h"
#include "ti_msp_dl_config.h"

/* Emm_V5协议固定帧尾/校验字节。 */
#define ZDT_CHECK_BYTE          0x6BU
/* UART异常时最多等待这么多次，防止电机断线导致主程序永久卡死。 */
#define ZDT_TX_WAIT_LIMIT       100000U
/* 环形缓冲区长度必须是2的整数次幂。 */
#define ZDT_RX_BUFFER_SIZE      64U
#define ZDT_RX_BUFFER_MASK      (ZDT_RX_BUFFER_SIZE - 1U)

#define ZDT_FUNC_ENABLE         0xF3U
#define ZDT_FUNC_POSITION       0xFDU
#define ZDT_FUNC_QUICK_CONFIG   0xF1U
#define ZDT_FUNC_QUICK_MOVE     0xFCU
#define ZDT_FUNC_STOP           0xFEU
#define ZDT_FUNC_HOME           0x9AU
#define ZDT_FUNC_HOME_ABORT     0x9CU
#define ZDT_FUNC_SET_ZERO       0x0AU
#define ZDT_FUNC_READ_SPEED     0x35U
#define ZDT_FUNC_READ_POSITION  0x36U
#define ZDT_FUNC_READ_HOME      0x3BU

static volatile uint8_t zdt_rx_buffer[ZDT_RX_BUFFER_SIZE];
static volatile uint8_t zdt_rx_write_index = 0U;
static volatile uint8_t zdt_rx_read_index = 0U;
static volatile uint32_t zdt_rx_overflow_count = 0U;

static zdt_stepper_feedback_t zdt_feedback;

static uint8_t zdt_parse_frame[8];
static uint8_t zdt_parse_length = 0U;
static uint8_t zdt_expected_length = 0U;

static bool zdt_send(const uint8_t *data, uint8_t length)
{
    uint8_t index;

    for (index = 0U; index < length; index++) {
        uint32_t wait_count = ZDT_TX_WAIT_LIMIT;

        /* 等待上一字节发送完成，同时保留超时退出能力。 */
        while (DL_UART_isBusy(STEPPER_INST) != false) {
            if (--wait_count == 0U) {
                return false;
            }
        }

        DL_UART_Main_transmitData(STEPPER_INST, data[index]);
    }

    return true;
}

static uint8_t zdt_direction_from_signed(int32_t value)
{
    /* 将上层统一的正负位置转换为驱动器协议的CW/CCW方向。 */
    if (value >= 0) {
        return ZDT_STEPPER_POSITIVE_DIR;
    }
    return (ZDT_STEPPER_POSITIVE_DIR == 0U) ? 1U : 0U;
}

static uint32_t zdt_magnitude_from_signed(int32_t value)
{
    /* 先扩展为int64_t，避免INT32_MIN直接取负造成溢出。 */
    int64_t wide_value = value;
    if (wide_value < 0) {
        wide_value = -wide_value;
    }
    return (uint32_t)wide_value;
}

static bool zdt_send_position(
    int32_t signed_pulses,
    uint16_t speed_rpm,
    uint8_t acceleration,
    uint8_t position_mode
)
{
    uint32_t pulses = zdt_magnitude_from_signed(signed_pulses);
    uint8_t command[13];

    if (speed_rpm > 5000U || position_mode > 2U) {
        return false;
    }

    command[0] = ZDT_STEPPER_ADDRESS;
    command[1] = ZDT_FUNC_POSITION;
    command[2] = zdt_direction_from_signed(signed_pulses);
    command[3] = (uint8_t)(speed_rpm >> 8);
    command[4] = (uint8_t)speed_rpm;
    command[5] = acceleration;
    command[6] = (uint8_t)(pulses >> 24);
    command[7] = (uint8_t)(pulses >> 16);
    command[8] = (uint8_t)(pulses >> 8);
    command[9] = (uint8_t)pulses;
    /* 1=相对回零点的绝对位置，2=相对当前实时位置。 */
    command[10] = position_mode;
    command[11] = 0U; /* no multi-motor synchronized start */
    command[12] = ZDT_CHECK_BYTE;

    return zdt_send(command, sizeof(command));
}

static void zdt_parse_complete_frame(void)
{
    uint8_t function = zdt_parse_frame[1];

    zdt_feedback.last_function = function;
    zdt_feedback.last_response_ms = SystemTime_GetMs();

    if (function == ZDT_FUNC_READ_POSITION) {
        /* 回复格式：地址 36 符号 位置4字节 6B，共8字节。 */
        uint32_t magnitude =
            ((uint32_t)zdt_parse_frame[3] << 24) |
            ((uint32_t)zdt_parse_frame[4] << 16) |
            ((uint32_t)zdt_parse_frame[5] << 8) |
            ((uint32_t)zdt_parse_frame[6]);
        int64_t signed_position = (int64_t)magnitude;

        if (zdt_parse_frame[2] != 0U) {
            signed_position = -signed_position;
        }
        if (signed_position > INT32_MAX) {
            signed_position = INT32_MAX;
        } else if (signed_position < INT32_MIN) {
            signed_position = INT32_MIN;
        }

        zdt_feedback.position_units = (int32_t)signed_position;
        zdt_feedback.position_valid = true;
    } else if (function == ZDT_FUNC_READ_SPEED) {
        /* 回复格式：地址 35 符号 转速2字节 6B，共6字节。 */
        uint16_t magnitude =
            ((uint16_t)zdt_parse_frame[3] << 8) |
            ((uint16_t)zdt_parse_frame[4]);
        int32_t signed_speed = magnitude;

        if (zdt_parse_frame[2] != 0U) {
            signed_speed = -signed_speed;
        }
        if (signed_speed > INT16_MAX) {
            signed_speed = INT16_MAX;
        } else if (signed_speed < INT16_MIN) {
            signed_speed = INT16_MIN;
        }

        zdt_feedback.speed_rpm = (int16_t)signed_speed;
        zdt_feedback.speed_valid = true;
    } else if (function == ZDT_FUNC_READ_HOME) {
        /* 回复格式：地址 3B 回零状态 6B，共4字节。 */
        zdt_feedback.homing_status = zdt_parse_frame[2];
        zdt_feedback.homing_status_valid = true;
    } else {
        /* Control-command replies are address, function, status, 0x6B. */
        zdt_feedback.last_status = zdt_parse_frame[2];
    }
}

static void zdt_feed_parser(uint8_t byte)
{
    /* 从任意字节流位置恢复同步：先查找当前电机地址作为帧头。 */
    if (zdt_parse_length == 0U) {
        if (byte == ZDT_STEPPER_ADDRESS) {
            zdt_parse_frame[0] = byte;
            zdt_parse_length = 1U;
        }
        return;
    }

    if (zdt_parse_length == 1U) {
        zdt_parse_frame[1] = byte;
        zdt_parse_length = 2U;

        if (byte == ZDT_FUNC_READ_POSITION) {
            zdt_expected_length = 8U;
        } else if (byte == ZDT_FUNC_READ_SPEED) {
            zdt_expected_length = 6U;
        } else {
            zdt_expected_length = 4U;
        }
        return;
    }

    if (zdt_parse_length < sizeof(zdt_parse_frame)) {
        zdt_parse_frame[zdt_parse_length++] = byte;
    } else {
        zdt_parse_length = 0U;
        zdt_expected_length = 0U;
        return;
    }

    if (zdt_parse_length == zdt_expected_length) {
        if (zdt_parse_frame[zdt_expected_length - 1U] == ZDT_CHECK_BYTE) {
            zdt_parse_complete_frame();
        }
        zdt_parse_length = 0U;
        zdt_expected_length = 0U;
    }
}

void zdt_stepper_init(void)
{
    zdt_rx_write_index = 0U;
    zdt_rx_read_index = 0U;
    zdt_rx_overflow_count = 0U;
    zdt_parse_length = 0U;
    zdt_expected_length = 0U;

    zdt_feedback.position_units = 0;
    zdt_feedback.speed_rpm = 0;
    zdt_feedback.homing_status = 0U;
    zdt_feedback.last_function = 0U;
    zdt_feedback.last_status = 0U;
    zdt_feedback.last_response_ms = 0U;
    zdt_feedback.position_valid = false;
    zdt_feedback.speed_valid = false;
    zdt_feedback.homing_status_valid = false;

    DL_UART_Main_enableInterrupt(STEPPER_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(STEPPER_INST_INT_IRQN);
    NVIC_EnableIRQ(STEPPER_INST_INT_IRQN);
}

void STEPPER_INST_IRQHandler(void)
{
    DL_UART_IIDX pending = DL_UART_Main_getPendingInterrupt(STEPPER_INST);

    if (pending == DL_UART_MAIN_IIDX_RX) {
        uint8_t byte = (uint8_t)DL_UART_Main_receiveData(STEPPER_INST);
        uint8_t next_write =
            (uint8_t)((zdt_rx_write_index + 1U) & ZDT_RX_BUFFER_MASK);

        if (next_write == zdt_rx_read_index) {
            /* 缓冲区满时丢弃最旧数据，ISR不能在这里阻塞。 */
            zdt_rx_read_index =
                (uint8_t)((zdt_rx_read_index + 1U) & ZDT_RX_BUFFER_MASK);
            zdt_rx_overflow_count++;
        }

        zdt_rx_buffer[zdt_rx_write_index] = byte;
        zdt_rx_write_index = next_write;
    }
}

bool zdt_stepper_enable(bool enable)
{
    uint8_t command[6] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_ENABLE,
        0xABU,
        enable ? 1U : 0U,
        0U,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_stop(void)
{
    uint8_t command[5] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_STOP,
        0x98U,
        0U,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_set_current_position_zero(void)
{
    uint8_t command[4] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_SET_ZERO,
        0x6DU,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_move_relative(
    int32_t pulses,
    uint16_t speed_rpm,
    uint8_t acceleration
)
{
    return zdt_send_position(pulses, speed_rpm, acceleration, 2U);
}

bool zdt_stepper_move_absolute(
    int32_t target_pulses,
    uint16_t speed_rpm,
    uint8_t acceleration
)
{
    return zdt_send_position(target_pulses, speed_rpm, acceleration, 1U);
}

bool zdt_stepper_configure_quick_position(
    uint16_t speed_rpm,
    uint8_t acceleration,
    bool absolute_mode
)
{
    uint8_t command[8];

    if (speed_rpm > 5000U) {
        return false;
    }

    command[0] = ZDT_STEPPER_ADDRESS;
    command[1] = ZDT_FUNC_QUICK_CONFIG;
    command[2] = (uint8_t)(speed_rpm >> 8);
    command[3] = (uint8_t)speed_rpm;
    command[4] = acceleration;
    /* 1=绝对目标位置，2=相对当前实时位置。 */
    command[5] = absolute_mode ? 1U : 2U;
    command[6] = 0U;
    command[7] = ZDT_CHECK_BYTE;

    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_quick_move(int32_t signed_pulses)
{
    int32_t protocol_pulses = signed_pulses;
    uint32_t raw;

    /* 快速位置协议没有独立方向字节：正数使用驱动器协议的CW方向，负数使用
     * CCW方向。根据ZDT_STEPPER_POSITIVE_DIR翻转符号，使上层始终保持
     * “正命令=滑轨上升”的统一机械坐标。平衡控制行程只有数百脉冲，不会
     * 出现INT32_MIN取负溢出。 */
    if (ZDT_STEPPER_POSITIVE_DIR != 0U) {
        protocol_pulses = -protocol_pulses;
    }
    raw = (uint32_t)protocol_pulses;
    uint8_t command[7] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_QUICK_MOVE,
        (uint8_t)(raw >> 24),
        (uint8_t)(raw >> 16),
        (uint8_t)(raw >> 8),
        (uint8_t)raw,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_start_homing(uint8_t mode)
{
    uint8_t command[5] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_HOME,
        mode,
        0U,
        ZDT_CHECK_BYTE
    };

    if (mode > 3U) {
        return false;
    }
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_abort_homing(void)
{
    uint8_t command[4] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_HOME_ABORT,
        0x48U,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_request_position(void)
{
    uint8_t command[3] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_READ_POSITION,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_request_speed(void)
{
    uint8_t command[3] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_READ_SPEED,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

bool zdt_stepper_request_homing_status(void)
{
    uint8_t command[3] = {
        ZDT_STEPPER_ADDRESS,
        ZDT_FUNC_READ_HOME,
        ZDT_CHECK_BYTE
    };
    return zdt_send(command, sizeof(command));
}

void zdt_stepper_poll(void)
{
    /* 主循环解析串口；避免在UART中断里执行较长的数据处理。 */
    while (zdt_rx_read_index != zdt_rx_write_index) {
        uint8_t byte = zdt_rx_buffer[zdt_rx_read_index];
        zdt_rx_read_index =
            (uint8_t)((zdt_rx_read_index + 1U) & ZDT_RX_BUFFER_MASK);
        zdt_feed_parser(byte);
    }
}

bool zdt_stepper_get_feedback(zdt_stepper_feedback_t *feedback)
{
    if (feedback == 0) {
        return false;
    }
    *feedback = zdt_feedback;
    return true;
}
