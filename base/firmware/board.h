/* The only interface between the firmware and the hardware.
 *
 * board.c (written once the board is identified) implements these for the real STM32;
 * tests/board_stub.c implements them on the laptop. Nothing else may touch registers,
 * so protocol, control, odom, safety and app all run in the laptop tests. */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#define BOARD_UART_EMPTY (-1) /* no byte waiting */
#define BOARD_UART_ERROR (-2) /* a byte arrived with a parity or framing error (8E1 link) */

/* Clocks, PWM, encoder timers, UART (115200 baud, 8 data bits, even parity, 1 stop bit),
 * independent watchdog (100 ms). Motors start at zero PWM. */
void board_init(void);

/* Milliseconds since power-up. Wraps after 49 days; callers subtract, never compare. */
uint32_t board_millis(void);

/* Returns once per control tick (CONTROL_HZ), from a hardware timer, so the loop period
 * does not depend on how long the work took. */
void board_wait_for_tick(void);

/* Running encoder counts of each wheel, as wired (signs fixed later by config.h).
 * Hardware timers in encoder mode count without the CPU; board.c extends their 16 bits
 * to 32 bits, so no count is lost to interrupt load. */
void board_read_counts(int32_t *left, int32_t *right);

/* Signed PWM duty in percent, -100 to +100, as wired (signs fixed by config.h).
 * At 0 the motor brakes if brake_at_zero is 1 (L298N enable high, both inputs low) and
 * coasts if it is 0 (enable low). */
void board_set_pwm(float left_pct, float right_pct, int brake_at_zero);

/* Next received byte (0 to 255), BOARD_UART_EMPTY or BOARD_UART_ERROR.
 * The receive side needs a buffer of at least 64 bytes (interrupt or DMA ring): the loop
 * reads it every 5 ms and 115200 baud brings up to 58 bytes in that time. */
int board_uart_read(void);

/* Queue bytes for sending. Must not block the control loop. */
void board_uart_write(const char *data, int length);

/* Reload the independent watchdog. If the loop stops, the MCU resets with motors at zero. */
void board_watchdog_kick(void);

/* U#: let the queued reply leave (at most 50 ms), stop the motors, leave a mark in a
 * backup register and restart. stage1 sees the mark and starts the ST ROM bootloader,
 * which waits for stm32flash on the same cable. Does not return on the board. */
void board_enter_bootloader(void);

/* The application has run long enough to count as healthy: forget earlier watchdog or
 * fault resets (stage1 sends the start after three in a row to the ROM bootloader). */
void board_boot_ok(void);

#endif
