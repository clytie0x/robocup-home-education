/* Reset and interrupt vectors of the application (RM0008 10.1.2, table 63). The
 * application sits at APP_BASE, above stage1 (boot.h); stage1 jumps here. */
#include <stdint.h>

#include "boot.h"
#include "stm32f103.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void SysTick_Handler(void);
void USART1_IRQHandler(void);
void board_panic(void);

/* Any fault or unexpected interrupt: motors off and restart (board.c). */
static void Default_Handler(void)
{
    board_panic();
}

#define VECTORS (16 + 43) /* core exceptions + medium-density peripheral interrupts */
#define IRQ(n) (16 + (n))

void Reset_Handler(void);

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

/* stage1 checks this header and the CRC-32 after the image before it starts the
 * application. tools/image.py writes the length and appends the CRC after linking. */
typedef struct {
    uint32_t magic;
    uint32_t length; /* bytes of the image, without the CRC that follows it */
} app_header;

__attribute__((section(".app_header"), used))
const app_header header = {APP_MAGIC, 0u};

void Reset_Handler(void)
{
    /* Interrupts use this table, not the one at address 0 (stage1's, or the ROM
     * bootloader's after its Go command). */
    SCB_VTOR = (uint32_t)vectors;
    uint32_t *src = &_sidata;
    for (uint32_t *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++; /* initialised variables: from flash to RAM */
    }
    for (uint32_t *dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;      /* the rest of the variables start at zero */
    }
    __asm volatile("cpsie i" ::: "memory"); /* stage1 starts us with interrupts masked */
    main();
    Default_Handler();
}
