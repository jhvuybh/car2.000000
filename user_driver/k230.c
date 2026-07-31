#include "k230.h"

#include "system_time.h"
#include "ti_msp_dl_config.h"

#define K230_FRAME_HEADER          0xAAU
#define K230_FRAME_TAIL            0x55U
#define K230_FRAME_LENGTH          8U
#define K230_RX_BUFFER_SIZE        64U
#define K230_RX_BUFFER_MASK        (K230_RX_BUFFER_SIZE - 1U)

static volatile uint8_t rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint8_t rx_write_index = 0U;
static volatile uint8_t rx_read_index = 0U;

volatile uint32_t k230_debug_rx_byte_count = 0U;
volatile uint32_t k230_debug_valid_frame_count = 0U;
volatile uint32_t k230_debug_rejected_frame_count = 0U;
volatile uint8_t k230_debug_last_rx_byte = 0U;
volatile uint8_t k230_debug_parser_index = 0U;
volatile k230_ball_result_t k230_debug_last_result = {
    0U, false, 0U, 0U, 0U
};

static uint8_t parse_frame[K230_FRAME_LENGTH];
static uint8_t parse_index = 0U;
static uint32_t last_frame_ms = 0U;
static bool received_any_frame = false;

static void capture_uart_fifo(void)
{
    while (DL_UART_isRXFIFOEmpty(K230_INST) == false) {
        uint8_t byte = DL_UART_receiveData(K230_INST);
        uint8_t next = (uint8_t)((rx_write_index + 1U) & K230_RX_BUFFER_MASK);

        if (next == rx_read_index) {
            rx_read_index = (uint8_t)((rx_read_index + 1U) & K230_RX_BUFFER_MASK);
            k230_debug_rejected_frame_count++;
        }

        rx_buffer[rx_write_index] = byte;
        rx_write_index = next;
        k230_debug_rx_byte_count++;
        k230_debug_last_rx_byte = byte;
    }
}

static bool validate_and_decode(k230_ball_result_t *result)
{
    uint8_t checksum;
    uint16_t position;
    bool valid;

    checksum = (uint8_t)(
        parse_frame[1] + parse_frame[2] + parse_frame[3] +
        parse_frame[4] + parse_frame[5]
    );
    valid = (parse_frame[2] != 0U);
    position = (uint16_t)(((uint16_t)parse_frame[3] << 8) | parse_frame[4]);

    if (parse_frame[0] != K230_FRAME_HEADER ||
        parse_frame[7] != K230_FRAME_TAIL ||
        parse_frame[6] != checksum ||
        parse_frame[2] > 1U ||
        parse_frame[5] > 100U ||
        (valid && position > K230_BALL_POSITION_MAX) ||
        (!valid && position != 0xFFFFU)) {
        return false;
    }

    result->sequence = parse_frame[1];
    result->valid = valid;
    result->position = valid ? position : 0U;
    result->confidence = parse_frame[5];
    result->received_ms = SystemTime_GetMs();

    last_frame_ms = result->received_ms;
    received_any_frame = true;
    k230_debug_last_result = *result;
    k230_debug_valid_frame_count++;
    return true;
}

void k230_init(void)
{
    rx_write_index = 0U;
    rx_read_index = 0U;
    parse_index = 0U;
    received_any_frame = false;

    DL_UART_Main_enableFIFOs(K230_INST);
    DL_UART_Main_setRXFIFOThreshold(K230_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(K230_INST_INT_IRQN);
    NVIC_EnableIRQ(K230_INST_INT_IRQN);
}

void K230_INST_IRQHandler(void)
{
    DL_UART_IIDX pending = DL_UART_Main_getPendingInterrupt(K230_INST);

    if (pending == DL_UART_MAIN_IIDX_RX) {
        capture_uart_fifo();
    }
}

bool k230_get_ball_result(k230_ball_result_t *result)
{
    bool decoded = false;

    if (result == NULL) {
        return false;
    }

    /* Also drain the FIFO here, so reception remains robust during debugging. */
    NVIC_DisableIRQ(K230_INST_INT_IRQN);
    capture_uart_fifo();
    NVIC_EnableIRQ(K230_INST_INT_IRQN);

    while (rx_read_index != rx_write_index) {
        uint8_t byte = rx_buffer[rx_read_index];
        rx_read_index = (uint8_t)((rx_read_index + 1U) & K230_RX_BUFFER_MASK);

        if (parse_index == 0U) {
            if (byte == K230_FRAME_HEADER) {
                parse_frame[0] = byte;
                parse_index = 1U;
            }
            continue;
        }

        parse_frame[parse_index++] = byte;
        k230_debug_parser_index = parse_index;

        if (parse_index >= K230_FRAME_LENGTH) {
            if (validate_and_decode(result)) {
                decoded = true;
            } else {
                k230_debug_rejected_frame_count++;
            }

            /* A bad tail may itself be the next header. */
            if (!decoded && byte == K230_FRAME_HEADER) {
                parse_frame[0] = byte;
                parse_index = 1U;
            } else {
                parse_index = 0U;
            }
            k230_debug_parser_index = parse_index;

            if (decoded) {
                return true;
            }
        }
    }

    return false;
}

bool k230_is_online(void)
{
    if (!received_any_frame) {
        return false;
    }
    return !SystemTime_IsOver(last_frame_ms, K230_BALL_TIMEOUT_MS);
}
