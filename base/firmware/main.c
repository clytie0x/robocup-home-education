/* Entry point on the STM32: set up, then one control tick every 5 ms forever. */
#include "app.h"
#include "board.h"

int main(void)
{
    static app a; /* static: kept off the small MCU stack */
    board_init();
    app_init(&a);
    for (;;) {
        board_wait_for_tick();
        app_poll_uart(&a);
        app_tick(&a);
        /* Kicked only after a full tick: if the loop hangs, the watchdog resets the MCU,
         * and after the reset the motors stay at zero until a new F or B. */
        board_watchdog_kick();
    }
}
