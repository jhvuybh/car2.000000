#include "hwt101.h"

#include "delay.h"
#include "system_time.h"
#include "ti_msp_dl_config.h"

/* WIT标准协议：每帧固定11字节。 */
#define HWT101_FRAME_HEADER       0x55U
#define HWT101_FRAME_LENGTH       11U
#define HWT101_TYPE_ACCEL         0x51U
#define HWT101_TYPE_GYRO          0x52U
#define HWT101_TYPE_ANGLE         0x53U
#define HWT101_TYPE_MAG           0x54U

/* WIT标准5字节寄存器写命令：FF AA REG DATA_L DATA_H。 */
#define HWT101_REG_SAVE           0x00U
#define HWT101_REG_OUTPUT_CONTENT 0x02U
#define HWT101_REG_UNLOCK         0x69U
#define HWT101_UNLOCK_LOW         0x88U
#define HWT101_UNLOCK_HIGH        0xB5U
#define HWT101_OUTPUT_ANGLE_ONLY  0x0008U

#define HWT101_UART_ERROR_MASK \
    (DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR | \
     DL_UART_MAIN_INTERRUPT_BREAK_ERROR | \
     DL_UART_MAIN_INTERRUPT_PARITY_ERROR | \
     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR | \
     DL_UART_MAIN_INTERRUPT_NOISE_ERROR)

/* 环形缓冲区长度必须是2的整数次幂。 */
#define HWT101_RX_BUFFER_SIZE     1024U
#define HWT101_RX_BUFFER_MASK     (HWT101_RX_BUFFER_SIZE - 1U)

static volatile uint8_t hwt101_rx_buffer[HWT101_RX_BUFFER_SIZE];
/* 1024字节环形缓存需要16位下标。较大的缓存可容纳OLED刷新期间到达的
 * HWT101数据，避免原128字节缓存被高速串口迅速写满。 */
static volatile uint16_t hwt101_rx_write_index = 0U;
static volatile uint16_t hwt101_rx_read_index = 0U;

volatile uint32_t hwt101_debug_rx_byte_count = 0U;
volatile uint32_t hwt101_debug_valid_frame_count = 0U;
volatile uint32_t hwt101_debug_rejected_frame_count = 0U;
/* UART硬件错误统计：电机运行时该值增长通常表示供电或线路干扰。 */
volatile uint32_t hwt101_debug_uart_error_count = 0U;
volatile uint32_t hwt101_debug_buffer_overflow_count = 0U;
volatile uint8_t hwt101_debug_parser_state = 0U;
volatile uint8_t hwt101_debug_last_rx_byte = 0U;
volatile uint8_t hwt101_debug_last_data_type = 0U;
volatile hwt101_angles_t hwt101_debug_last_angles = {0.0f, 0.0f, 0.0f};
volatile hwt101_gyro_t hwt101_debug_last_gyro = {0.0f, 0.0f, 0.0f};

static hwt101_angles_t latest_angles = {0.0f, 0.0f, 0.0f};
static hwt101_gyro_t latest_gyro = {0.0f, 0.0f, 0.0f};
static hwt101_accel_t latest_accel = {0.0f, 0.0f, 0.0f};
static hwt101_mag_t latest_mag = {0.0f, 0.0f, 0.0f};

static uint8_t angles_new = 0U;
static uint8_t gyro_new = 0U;
static uint8_t accel_new = 0U;
static uint8_t mag_new = 0U;
static uint8_t angle_valid = 0U;
static uint32_t last_angle_ms = 0U;

static uint8_t tracker_initialized = 0U;
static float tracker_last_yaw = 0.0f;
static float tracker_delta = 0.0f;
static float turn_target = 0.0f;
static uint8_t turn_target_active = 0U;

static int16_t hwt101_i16(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static uint8_t hwt101_checksum(uint8_t type, const uint8_t data[8])
{
    uint16_t sum = HWT101_FRAME_HEADER + type;
    uint8_t index;

    for (index = 0U; index < 8U; index++) {
        sum += data[index];
    }
    return (uint8_t)sum;
}

#if HWT101_CONFIGURE_ANGLE_ONLY_ON_BOOT != 0U
static void hwt101_write_register(uint8_t reg, uint16_t value)
{
    const uint8_t command[5] = {
        0xFFU,
        0xAAU,
        reg,
        (uint8_t)(value & 0x00FFU),
        (uint8_t)((value >> 8) & 0x00FFU)
    };
    uint8_t index;

    for (index = 0U; index < sizeof(command); index++) {
        DL_UART_Main_transmitDataBlocking(HWT101_INST, command[index]);
    }
}

static void hwt101_configure_angle_only(void)
{
    /* 解锁配置寄存器，然后只打开角度输出位(bit3)。这里不发送SAVE命令，
     * 避免主控每次开机都擦写HWT101内部Flash；下次开机由主控再次配置。 */
    hwt101_write_register(
        HWT101_REG_UNLOCK,
        ((uint16_t)HWT101_UNLOCK_HIGH << 8) | HWT101_UNLOCK_LOW
    );
    delay_ms(20U);
    hwt101_write_register(HWT101_REG_OUTPUT_CONTENT, HWT101_OUTPUT_ANGLE_ONLY);
    delay_ms(20U);
}
#endif

static float hwt101_wrap_difference(float current, float previous)
{
    float difference = current - previous;

    if (difference > 180.0f) {
        difference -= 360.0f;
    } else if (difference < -180.0f) {
        difference += 360.0f;
    }
    return difference;
}

static void hwt101_tracker_update(float yaw)
{
    if (tracker_initialized == 0U) {
        tracker_last_yaw = yaw;
        tracker_initialized = 1U;
        return;
    }

    tracker_delta +=
        hwt101_wrap_difference(yaw, tracker_last_yaw) *
        HWT101_YAW_DIRECTION_SIGN;
    tracker_last_yaw = yaw;
}

static void hwt101_accept_frame(uint8_t type, const uint8_t data[8])
{
    int16_t value_x = hwt101_i16(data[0], data[1]);
    int16_t value_y = hwt101_i16(data[2], data[3]);
    int16_t value_z = hwt101_i16(data[4], data[5]);

    hwt101_debug_last_data_type = type;

    if (type == HWT101_TYPE_ANGLE) {
        latest_angles.roll = (float)value_x * 180.0f / 32768.0f;
        latest_angles.pitch = (float)value_y * 180.0f / 32768.0f;
        latest_angles.yaw = (float)value_z * 180.0f / 32768.0f;
        angles_new = 1U;
        angle_valid = 1U;
        last_angle_ms = SystemTime_GetMs();
        hwt101_tracker_update(latest_angles.yaw);
        hwt101_debug_last_angles = latest_angles;
    } else if (type == HWT101_TYPE_GYRO) {
        latest_gyro.wx = (float)value_x * 2000.0f / 32768.0f;
        latest_gyro.wy = (float)value_y * 2000.0f / 32768.0f;
        latest_gyro.wz = (float)value_z * 2000.0f / 32768.0f;
        gyro_new = 1U;
        hwt101_debug_last_gyro = latest_gyro;
    } else if (type == HWT101_TYPE_ACCEL) {
        latest_accel.ax = (float)value_x * 16.0f / 32768.0f;
        latest_accel.ay = (float)value_y * 16.0f / 32768.0f;
        latest_accel.az = (float)value_z * 16.0f / 32768.0f;
        accel_new = 1U;
    } else if (type == HWT101_TYPE_MAG) {
        latest_mag.hx = (float)value_x;
        latest_mag.hy = (float)value_y;
        latest_mag.hz = (float)value_z;
        mag_new = 1U;
    }
}

static void hwt101_capture_fifo(void)
{
    while (DL_UART_isRXFIFOEmpty(HWT101_INST) == false) {
        uint8_t byte = DL_UART_receiveData(HWT101_INST);
        uint16_t next_write =
            (uint16_t)((hwt101_rx_write_index + 1U) & HWT101_RX_BUFFER_MASK);

        if (next_write == hwt101_rx_read_index) {
            hwt101_rx_read_index =
                (uint16_t)((hwt101_rx_read_index + 1U) & HWT101_RX_BUFFER_MASK);
            hwt101_debug_rejected_frame_count++;
            hwt101_debug_buffer_overflow_count++;
        }

        hwt101_rx_buffer[hwt101_rx_write_index] = byte;
        hwt101_rx_write_index = next_write;
        hwt101_debug_rx_byte_count++;
        hwt101_debug_last_rx_byte = byte;
    }
}

void hwt101_init(void)
{
    hwt101_rx_write_index = 0U;
    hwt101_rx_read_index = 0U;
    angles_new = 0U;
    gyro_new = 0U;
    accel_new = 0U;
    mag_new = 0U;
    angle_valid = 0U;
    tracker_initialized = 0U;
    tracker_delta = 0.0f;
    turn_target_active = 0U;

    DL_UART_Main_enableFIFOs(HWT101_INST);
    DL_UART_Main_setRXFIFOThreshold(
        HWT101_INST,
        DL_UART_RX_FIFO_LEVEL_ONE_ENTRY
    );
    DL_UART_Main_enableInterrupt(HWT101_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(HWT101_INST_INT_IRQN);
    NVIC_EnableIRQ(HWT101_INST_INT_IRQN);

#if HWT101_CONFIGURE_ANGLE_ONLY_ON_BOOT != 0U
    hwt101_configure_angle_only();
#endif
}

void HWT101_INST_IRQHandler(void)
{
    DL_UART_IIDX pending = DL_UART_Main_getPendingInterrupt(HWT101_INST);

    if (pending == DL_UART_MAIN_IIDX_RX) {
        hwt101_capture_fifo();
    }
}

void hwt101_poll(void)
{
    static uint8_t state = 0U;
    static uint8_t type = 0U;
    static uint8_t data[8];
    static uint8_t data_index = 0U;
    uint32_t uart_errors;

    /* 错误中断不启用，改由主循环读取并清除。这样即使电机噪声使错误位
     * 连续出现，也不会一直占用CPU并饿死PID计时器中断。 */
    uart_errors = DL_UART_Main_getRawInterruptStatus(
        HWT101_INST,
        HWT101_UART_ERROR_MASK
    );
    if (uart_errors != 0U) {
        hwt101_debug_uart_error_count++;
        DL_UART_Main_clearInterruptStatus(HWT101_INST, uart_errors);
    }

    /* Also drain UART1 from the main loop.  This keeps reception working even
     * if an RX interrupt is delayed while OLED or motor code is executing. */
    NVIC_DisableIRQ(HWT101_INST_INT_IRQN);
    hwt101_capture_fifo();
    NVIC_EnableIRQ(HWT101_INST_INT_IRQN);

    while (hwt101_rx_read_index != hwt101_rx_write_index) {
        uint8_t byte = hwt101_rx_buffer[hwt101_rx_read_index];
        hwt101_rx_read_index =
            (uint16_t)((hwt101_rx_read_index + 1U) & HWT101_RX_BUFFER_MASK);
        hwt101_debug_parser_state = state;

        if (state == 0U) {
            if (byte == HWT101_FRAME_HEADER) {
                state = 1U;
            }
        } else if (state == 1U) {
            if (byte >= HWT101_TYPE_ACCEL && byte <= HWT101_TYPE_MAG) {
                type = byte;
                data_index = 0U;
                state = 2U;
            } else {
                hwt101_debug_rejected_frame_count++;
                state = (byte == HWT101_FRAME_HEADER) ? 1U : 0U;
            }
        } else if (state == 2U) {
            data[data_index++] = byte;
            if (data_index >= 8U) {
                state = 3U;
            }
        } else {
            if (byte == hwt101_checksum(type, data)) {
                hwt101_debug_valid_frame_count++;
                hwt101_accept_frame(type, data);
            } else {
                hwt101_debug_rejected_frame_count++;
            }
            state = (byte == HWT101_FRAME_HEADER) ? 1U : 0U;
        }
    }
}

uint8_t hwt101_get_angles(hwt101_angles_t *angles)
{
    if (angles == 0 || angles_new == 0U) {
        return 0U;
    }
    *angles = latest_angles;
    angles_new = 0U;
    return 1U;
}

uint8_t hwt101_get_gyro(hwt101_gyro_t *gyro)
{
    if (gyro == 0 || gyro_new == 0U) {
        return 0U;
    }
    *gyro = latest_gyro;
    gyro_new = 0U;
    return 1U;
}

uint8_t hwt101_get_accel(hwt101_accel_t *accel)
{
    if (accel == 0 || accel_new == 0U) {
        return 0U;
    }
    *accel = latest_accel;
    accel_new = 0U;
    return 1U;
}

uint8_t hwt101_get_mag(hwt101_mag_t *mag)
{
    if (mag == 0 || mag_new == 0U) {
        return 0U;
    }
    *mag = latest_mag;
    mag_new = 0U;
    return 1U;
}

float hwt101_get_yaw(void) { return latest_angles.yaw; }
float hwt101_get_pitch(void) { return latest_angles.pitch; }
float hwt101_get_roll(void) { return latest_angles.roll; }

uint8_t hwt101_has_valid_angle(void)
{
    return angle_valid;
}

uint8_t hwt101_is_online(void)
{
    if (angle_valid == 0U) {
        return 0U;
    }
    return SystemTime_IsOver(last_angle_ms, HWT101_DATA_TIMEOUT_MS) ? 0U : 1U;
}

void hwt101_yaw_tracker_reset(void)
{
    tracker_delta = 0.0f;
    turn_target_active = 0U;

    if (angle_valid != 0U) {
        tracker_last_yaw = latest_angles.yaw;
        tracker_initialized = 1U;
    } else {
        tracker_initialized = 0U;
    }
}

float hwt101_yaw_tracker_get_delta(void)
{
    return tracker_delta;
}

void hwt101_set_turn_target(float target_degrees)
{
    turn_target = target_degrees;
    turn_target_active = 1U;
}

uint8_t hwt101_yaw_tracker_target_reached(void)
{
    float stop_angle;

    if (turn_target_active == 0U) {
        return 0U;
    }

    if (turn_target > 0.0f) {
        stop_angle = turn_target - HWT101_STOP_ADVANCE_DEG;
        if (tracker_delta >= stop_angle) {
            turn_target_active = 0U;
            return 1U;
        }
    } else {
        stop_angle = turn_target + HWT101_STOP_ADVANCE_DEG;
        if (tracker_delta <= stop_angle) {
            turn_target_active = 0U;
            return 1U;
        }
    }
    return 0U;
}
