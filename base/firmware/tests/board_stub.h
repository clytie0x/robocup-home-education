/* Controls for the laptop stand-in of board.h. */
#ifndef BOARD_STUB_H
#define BOARD_STUB_H

#include <stdint.h>

extern uint32_t stub_ms;          /* what board_millis() returns */
extern int32_t stub_counts[2];    /* what board_read_counts() returns, left and right */
extern float stub_pwm[2];         /* the latest board_set_pwm() */
extern char stub_tx[1024];        /* everything board_uart_write() sent since stub_clear_tx() */
extern int stub_tx_length;

void stub_reset(void);
void stub_clear_tx(void);
void stub_receive(const char *bytes); /* queue bytes for board_uart_read() */
void stub_receive_error(void);        /* queue one byte with a parity error */

#endif
