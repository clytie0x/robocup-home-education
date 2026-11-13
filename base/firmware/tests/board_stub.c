/* board.h on the laptop: time, counts and received bytes come from the test; PWM and sent
 * bytes are recorded for it. */
#include "board_stub.h"

#include <string.h>

#include "../board.h"

uint32_t stub_ms;
int32_t stub_counts[2];
float stub_pwm[2];
char stub_tx[1024];
int stub_tx_length;

static int rx[1024];
static int rx_head, rx_tail;

void stub_reset(void)
{
    stub_ms = 0;
    stub_counts[0] = stub_counts[1] = 0;
    stub_pwm[0] = stub_pwm[1] = 0.0f;
    rx_head = rx_tail = 0;
    stub_clear_tx();
}

void stub_clear_tx(void)
{
    stub_tx_length = 0;
    stub_tx[0] = '\0';
}

static void push(int value)
{
    rx[rx_tail] = value;
    rx_tail = (rx_tail + 1) % 1024;
}

void stub_receive(const char *bytes)
{
    for (; *bytes; bytes++) {
        push((uint8_t)*bytes);
    }
}

void stub_receive_error(void)
{
    push(BOARD_UART_ERROR);
}

void board_init(void) {}

uint32_t board_millis(void)
{
    return stub_ms;
}

void board_wait_for_tick(void) {}

void board_read_counts(int32_t *left, int32_t *right)
{
    *left = stub_counts[0];
    *right = stub_counts[1];
}

void board_set_pwm(float left_pct, float right_pct)
{
    stub_pwm[0] = left_pct;
    stub_pwm[1] = right_pct;
}

int board_uart_read(void)
{
    if (rx_head == rx_tail) {
        return BOARD_UART_EMPTY;
    }
    int value = rx[rx_head];
    rx_head = (rx_head + 1) % 1024;
    return value;
}

void board_uart_write(const char *data, int length)
{
    if (stub_tx_length + length < (int)sizeof stub_tx) {
        memcpy(stub_tx + stub_tx_length, data, (size_t)length);
        stub_tx_length += length;
        stub_tx[stub_tx_length] = '\0';
    }
}

void board_watchdog_kick(void) {}
