/* Reset and interrupt vectors for the STM32F103 (RM0008 10.1.2, table 63). */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);

/* Any unexpected interrupt or fault stops here. The watchdog then resets the MCU, and
 * after a reset the motors stay off until a new F or B. */
static void Default_Handler(void)
{
    for (;;) {
    }
}

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++; /* initialised variables: from flash to RAM */
    }
    for (uint32_t *dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;      /* the rest of the variables start at zero */
    }
    main();
    Default_Handler();
}

#define VECTORS (16 + 43) /* core exceptions + medium-density peripheral interrupts */
#define IRQ(n) (16 + (n))

/* Entry 0 is the initial stack pointer; the rest are handler addresses. */
__attribute__((section(".isr_vector"), used))
void (*const vectors[VECTORS])(void) = {
    [0] = (void (*)(void))(&_estack),
    [1] = Reset_Handler,
    [2 ... 14] = Default_Handler,
    [15] = SysTick_Handler,
    [IRQ(0) ... IRQ(36)] = Default_Handler,
    [IRQ(37)] = USART1_IRQHandler,
    [IRQ(38) ... VECTORS - 1] = Default_Handler,
};
