/* Entry point on the STM32: set up, then one control tick every 5 ms forever. */
#include "app.h"
#include "board.h"
#include "config.h"

/* After this long in the loop the start counts as healthy (boot.h, BOOT_MAX_CRASHES). */
#define HEALTHY_TICKS (2 * CONTROL_HZ)

int main(void)
{
    static app a; /* static: kept off the small MCU stack */
    int ticks = 0;
    board_init();
    app_init(&a);
    for (;;) {
        if (ticks < HEALTHY_TICKS && ++ticks == HEALTHY_TICKS) {
            board_boot_ok();
        }
        board_wait_for_tick();
        app_poll_uart(&a);
        app_tick(&a);
        /* Kicked only after a full tick: if the loop hangs, the watchdog resets the MCU,
         * and after the reset the motors stay at zero until a new F or B. */
        board_watchdog_kick();
    }
}
