/* stage1: the first 4 KiB of flash. Written once with the BOOT0 jumper, never rewritten
 * by an update, so the chip can always be reached from the Pi without the jumper.
 *
 * On every start it decides (boot.c) between the application at APP_BASE and the ST ROM
 * bootloader, which waits for stm32flash on the robot's cable (USART1, 8E1):
 *   - U# left BOOT_MAGIC in backup register 1         -> ROM bootloader
 *   - three watchdog or fault restarts in a row         -> ROM bootloader
 *   - the application's header or CRC-32 is wrong       -> ROM bootloader
 *   - otherwise                                          -> application
 *
 * No variables in RAM (nothing copies .data here), no interrupts, no clock change: it
 * runs on the 8 MHz reset clock and leaves the chip as reset left it. */
#include <stdint.h>

#include "boot.h"
#include "stm32f103.h"

extern uint32_t _estack;
void stage1_start(void);

/* A fault inside stage1 stops here. stage1 only reads registers and flash. */
static void hang(void)
{
    for (;;) {
    }
}

/* Only the core exceptions: stage1 enables no interrupt. */
__attribute__((section(".isr_vector"), used))
void (*const vectors[16])(void) = {
    [0] = (void (*)(void))(&_estack),
    [1] = stage1_start,
    [2 ... 15] = hang,
};

/* L298N inputs and enables low (board.c, same pins), so the motors cannot move on
 * floating pins while stage1 checks the CRC or while the ROM bootloader waits. */
static void motors_off(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    GPIOB->BSRR = 0xFu << (12 + 16);                          /* PB12..PB15 low */
    GPIOB->CRH = (GPIOB->CRH & ~0xFFFF0000u) | 0x22220000u;   /* PB12..PB15 outputs */
    GPIOA->BSRR = 0x3u << (6 + 16);                           /* PA6, PA7 low */
    GPIOA->CRL = (GPIOA->CRL & ~0xFF000000u) | 0x22000000u;   /* PA6, PA7 outputs */
}

static unsigned reset_flags(void)
{
    uint32_t csr = RCC_CSR;
    RCC_CSR |= RCC_CSR_RMVF; /* clear them, so the next start sees only its own cause */
    unsigned flags = 0;
    if (csr & RCC_CSR_PORRSTF) {
        flags |= RESET_POWER;
    }
    if (csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) {
        flags |= RESET_WATCHDOG;
    }
    if (csr & RCC_CSR_SFTRSTF) {
        flags |= RESET_SOFTWARE;
    }
    if (csr & RCC_CSR_PINRSTF) {
        flags |= RESET_PIN;
    }
    return flags;
}

/* Start the program whose vector table is at base: its stack pointer, then its reset
 * handler. */
static void jump(uint32_t base)
{
    uint32_t sp = ((const volatile uint32_t *)base)[0];
    uint32_t pc = ((const volatile uint32_t *)base)[1];
    __asm volatile("msr msp, %0\n\tbx %1" : : "r"(sp), "r"(pc) : "memory");
    hang();
}

void stage1_start(void)
{
    __asm volatile("cpsid i" ::: "memory");
    motors_off();
    RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    unsigned flags = reset_flags();
    unsigned magic = BKP_DR1 & 0xFFFFu;
    unsigned crashes = BKP_DR2 & 0xFFFFu;
    int app_ok = magic != BOOT_MAGIC && boot_app_valid((const uint8_t *)APP_BASE, APP_BASE, APP_SPACE);
    int where = boot_decide(flags, magic, &crashes, app_ok);

    PWR_CR |= PWR_CR_DBP;
    BKP_DR1 = 0; /* the mark is used once: the next reset starts the application again */
    BKP_DR2 = crashes;
    PWR_CR &= ~PWR_CR_DBP;
    RCC_APB1ENR &= ~(RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN);

    if (where == BOOT_ROM) {
        /* As if BOOT0 were 1: then address 0 shows system memory, so point the vector
         * table there, and leave interrupts unmasked as after a reset. */
        SCB_VTOR = ROM_BOOTLOADER;
        __asm volatile("cpsie i" ::: "memory");
        jump(ROM_BOOTLOADER);
    }
    SCB_VTOR = APP_BASE;
    jump(APP_BASE); /* the application unmasks interrupts after setting up */
}
