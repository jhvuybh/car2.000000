#ifndef K230_H
#define K230_H

#include <stdbool.h>
#include <stdint.h>

/*
 * K230 steel-ball position receiver
 * ---------------------------------
 * Wiring:
 *   K230 UART2_TX (GPIO11) -> MSPM0 PB16 / K230 UART RX
 *   K230 GND               -> MSPM0 GND
 *
 * K230 sends one 8-byte binary frame after every inference:
 *   AA SEQ VALID POS_H POS_L CONF CHECKSUM 55
 *
 * POS is 0..1000 from one end of the 25 cm tube to the other end.
 * When VALID is zero, POS must be 0xFFFF.  CONF is 0..100.
 */

#define K230_BALL_POSITION_MIN       0U
#define K230_BALL_POSITION_MAX       1000U
#define K230_BALL_POSITION_CENTER    500U
#define K230_BALL_POSITION_PER_CM    30U // 距离精度
#define K230_BALL_TIMEOUT_MS         250U

typedef struct {
    uint8_t sequence;
    bool valid;
    uint16_t position;
    uint8_t confidence;
    uint32_t received_ms;
} k230_ball_result_t;

/* Call once after SYSCFG_DL_init(). */
void k230_init(void);

/*
 * Non-blocking parser.  Returns true only when a new complete valid frame is
 * copied to result.  A frame with VALID=0 is still a valid communication frame.
 */
bool k230_get_ball_result(k230_ball_result_t *result);

/* True while complete frames are arriving within K230_BALL_TIMEOUT_MS. */
bool k230_is_online(void);

/* CCS Watch diagnostics. */
extern volatile uint32_t k230_debug_rx_byte_count;
extern volatile uint32_t k230_debug_valid_frame_count;
extern volatile uint32_t k230_debug_rejected_frame_count;
extern volatile uint8_t k230_debug_last_rx_byte;
extern volatile uint8_t k230_debug_parser_index;
extern volatile k230_ball_result_t k230_debug_last_result;

#endif
