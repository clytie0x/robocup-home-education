/* board.h for the robot's Blue Pill (STM32F103C8T6, 8 MHz crystal) on the carrier board.
 *
 * Pin map, read from the old firmware (base/firmware/README.md, "Pin map"):
 *   USART1  PA9 TX, PA10 RX           CH340E to the Pi, 115200 baud 8E1
 *   right   PWM PA6 = TIM3_CH1, PB13 high = forward, PB12 high = backward
 *   left    PWM PA7 = TIM3_CH2, PB15 high = forward, PB14 high = backward
 *   right   encoder PB6/PB7 = TIM4 CH1/CH2 (counts up going forward)
 *   left    encoder PA0/PA1 = TIM2 CH1/CH2 (counts down going forward)
 *   PC13    Blue Pill LED (low = on), blinks once a second while the loop runs
 * No pin remap is needed: all of these are the timers' and USART's default pins. */
#include "board.h"

#include "boot.h"
#include "config.h"
#include "stm32f103.h"

#define CPU_HZ 72000000u
#define TICK_MS (1000u / CONTROL_HZ)
#define PWM_TIMER_HZ 36000000u /* TIM3 clock 72 MHz, prescaler 2 */

static volatile uint32_t millis;

/* ---- GPIO ---- */

static void pin_mode(gpio_regs *port, int pin, uint32_t cfg)
{
    volatile uint32_t *cr = pin < 8 ? &port->CRL : &port->CRH;
    int shift = (pin % 8) * 4;
    *cr = (*cr & ~(0xFu << shift)) | (cfg << shift);
}

static void pin_write(gpio_regs *port, int pin, int high)
{
    port->BSRR = high ? (1u << pin) : (1u << (pin + 16));
}

/* First thing after reset: the L298N inputs and enables are driven low, so the motors
 * cannot move on floating pins while the clocks start. */
static void motors_off_pins(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    for (int pin = 12; pin <= 15; pin++) {
        pin_write(GPIOB, pin, 0);
        pin_mode(GPIOB, pin, PIN_OUTPUT_2MHZ);
    }
    pin_write(GPIOA, 6, 0);
    pin_write(GPIOA, 7, 0);
    pin_mode(GPIOA, 6, PIN_OUTPUT_2MHZ);
    pin_mode(GPIOA, 7, PIN_OUTPUT_2MHZ);
}

/* ---- Restarts ---- */

static void system_reset(void)
{
    __asm volatile("dsb" ::: "memory");
    SCB_AIRCR = SCB_AIRCR_SYSRESETREQ;
    __asm volatile("dsb" ::: "memory");
    for (;;) {
    }
}

/* Leave the mark for stage1 and restart: the next start runs the ST ROM bootloader. The
 * backup register survives the reset; the watchdog does not (a system reset stops it),
 * so it cannot fire while the bootloader waits. */
static void restart_into_rom_bootloader(void)
{
    __asm volatile("cpsid i" ::: "memory");
    motors_off_pins();
    RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR_CR |= PWR_CR_DBP;
    BKP_DR1 = BOOT_MAGIC;
    system_reset();
}

/* Any fault or unexpected interrupt (startup.c): motors off and restart at once. stage1
 * counts these restarts; three in a row before the loop ran 2 s, and it starts the ROM
 * bootloader instead, so a broken update can always be replaced from the Pi. */
void board_panic(void)
{
    motors_off_pins();
    system_reset();
}

/* ---- Clock: 8 MHz crystal x 9 = 72 MHz; APB1 36 MHz (its timers still get 72 MHz) ---- */

static void clock_init(void)
{
    /* Start from the internal 8 MHz clock whatever ran before: after the ROM bootloader's
     * Go command its PLL may still be running, and the PLL cannot be changed while on. */
    RCC_CR |= RCC_CR_HSION;
    while (!(RCC_CR & RCC_CR_HSIRDY)) {
    }
    RCC_CFGR &= ~3u;                     /* system clock = HSI */
    while (((RCC_CFGR >> 2) & 3u) != 0u) {
    }
    RCC_CR &= ~RCC_CR_PLLON;
    while (RCC_CR & RCC_CR_PLLRDY) {
    }
    RCC_CR |= RCC_CR_HSEON;
    for (uint32_t wait = 0; !(RCC_CR & RCC_CR_HSERDY); wait++) {
        /* About 10 cycles a turn at 8 MHz: 0.25 s, far above a crystal's few ms and well
         * inside stage1's 3 s watchdog (2 s at the fastest LSI). */
        if (wait > 200000u) {
            /* No crystal: the loop cannot keep time. Wait in the ROM bootloader (it runs on
             * the internal clock), motors off, where the Pi can still reach the chip. */
            restart_into_rom_bootloader();
        }
    }
    FLASH_ACR = 0x10u | 0x2u;            /* prefetch on, 2 wait states for 72 MHz */
    RCC_CFGR = (7u << 18) | (1u << 16)   /* PLL x9 from HSE */
             | (4u << 8);                /* APB1 = HCLK / 2 */
    RCC_CR |= RCC_CR_PLLON;
    while (!(RCC_CR & RCC_CR_PLLRDY)) {
    }
    RCC_CFGR |= 2u;                      /* system clock = PLL */
    while (((RCC_CFGR >> 2) & 3u) != 2u) {
    }
}

static void systick_init(void)
{
    SYST_RVR = CPU_HZ / 1000u - 1u;
    SYST_CVR = 0;
    SYST_CSR = 7u; /* CPU clock, interrupt on, enabled */
}

void SysTick_Handler(void)
{
    millis++;
}

/* ---- Motor PWM on TIM3 ---- */

static void pwm_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = CPU_HZ / PWM_TIMER_HZ - 1u;
    TIM3->ARR = PWM_TIMER_HZ / PWM_HZ - 1u;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCMR1 = (6u << 4) | (1u << 3)     /* CH1 PWM mode 1, preload */
                | (6u << 12) | (1u << 11);  /* CH2 the same */
    TIM3->CCER = (1u << 0) | (1u << 4);     /* CH1 and CH2 outputs on */
    TIM3->CR1 = (1u << 7);                  /* ARR preload */
    TIM3->EGR = 1u;                         /* load PSC and ARR now */
    TIM3->CR1 |= 1u;
    /* Hand PA6 and PA7 to the timer only now, with the duty already at zero. */
    pin_mode(GPIOA, 6, PIN_ALT_OUTPUT_2MHZ);
    pin_mode(GPIOA, 7, PIN_ALT_OUTPUT_2MHZ);
}

static uint32_t duty(float pct)
{
    float a = pct < 0.0f ? -pct : pct;
    if (a > 100.0f) {
        a = 100.0f;
    }
    return (uint32_t)(a * (float)(TIM3->ARR + 1u) / 100.0f);
}

/* One L298N channel: two direction pins and its PWM compare register. At zero, both
 * inputs low with the enable fully on brakes (the low-side switches short the motor);
 * with the enable off it coasts, as the old firmware did. */
static void drive(int forward_pin, int backward_pin, volatile uint32_t *ccr, float pct, int brake)
{
    pin_write(GPIOB, forward_pin, pct > 0.0f);
    pin_write(GPIOB, backward_pin, pct < 0.0f);
    *ccr = (pct == 0.0f && brake) ? TIM3->ARR + 1u : duty(pct); /* above ARR: always high */
}

void board_set_pwm(float left_pct, float right_pct, int brake_at_zero)
{
    drive(15, 14, &TIM3->CCR2, left_pct, brake_at_zero);
    drive(13, 12, &TIM3->CCR1, right_pct, brake_at_zero);
}

/* ---- Encoders: TIM2 (left) and TIM4 (right) in encoder mode, both edges of both channels ---- */

static void encoder_init(tim_regs *t)
{
    t->SMCR = 3u;                                         /* encoder mode 3: count on TI1 and TI2 */
    t->CCMR1 = (1u << 0) | (3u << 4) | (1u << 8) | (3u << 12); /* TI1, TI2 inputs, filter N=8 */
    t->CCER = 0;                                          /* no inversion */
    t->ARR = 0xFFFFu;
    t->CNT = 0;
    t->CR1 = 1u;
}

static void encoders_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
    pin_mode(GPIOA, 0, PIN_INPUT_FLOATING); /* floating, as the old firmware had them */
    pin_mode(GPIOA, 1, PIN_INPUT_FLOATING);
    pin_mode(GPIOB, 6, PIN_INPUT_FLOATING);
    pin_mode(GPIOB, 7, PIN_INPUT_FLOATING);
    encoder_init(TIM2);
    encoder_init(TIM4);
}

/* The timers count in 16 bits. Read every 5 ms, the change since the last read is far
 * below 32768 counts, so the signed 16-bit difference is always the true change. */
static int32_t extend(tim_regs *t, uint16_t *last, int32_t *total)
{
    uint16_t now = (uint16_t)t->CNT;
    *total += (int16_t)(uint16_t)(now - *last);
    *last = now;
    return *total;
}

void board_read_counts(int32_t *left, int32_t *right)
{
    static uint16_t last_left, last_right;
    static int32_t total_left, total_right;
    *left = extend(TIM2, &last_left, &total_left);
    *right = extend(TIM4, &last_right, &total_right);
}

/* ---- USART1: interrupt-driven rings, so the control loop never waits on the line ---- */

#define RX_SIZE 128u
#define TX_SIZE 256u
static volatile int16_t rx_ring[RX_SIZE];
static volatile uint32_t rx_head, rx_tail;
static volatile uint8_t tx_ring[TX_SIZE];
static volatile uint32_t tx_head, tx_tail;

static void uart_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    pin_mode(GPIOA, 9, PIN_ALT_OUTPUT_2MHZ);
    pin_write(GPIOA, 10, 1);
    pin_mode(GPIOA, 10, PIN_INPUT_PULL); /* pulled up: an unplugged CH340 reads idle, not noise */
    USART1->BRR = CPU_HZ / 115200u;      /* 625 */
    USART1->CR2 = 0;                     /* 1 stop bit; also undoes the ROM bootloader's setup */
    USART1->CR3 = 0;
    /* 9-bit frame = 8 data bits + even parity, then 1 stop bit: 8E1 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE |
                  USART_CR1_RXNEIE;
    NVIC_ISER1 = 1u << (USART1_IRQ - 32);
}

static void rx_push(int16_t value)
{
    uint32_t next = (rx_head + 1u) % RX_SIZE;
    if (next != rx_tail) { /* full: drop, the parser then sees a broken message */
        rx_ring[rx_head] = value;
        rx_head = next;
    }
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        uint32_t data = USART1->DR; /* reading SR then DR clears the error flags too */
        int bad = (sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0;
        rx_push(bad ? (int16_t)BOARD_UART_ERROR : (int16_t)(data & 0xFFu));
    }
    if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (tx_tail != tx_head) {
            USART1->DR = tx_ring[tx_tail];
            tx_tail = (tx_tail + 1u) % TX_SIZE;
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

int board_uart_read(void)
{
    if (rx_tail == rx_head) {
        return BOARD_UART_EMPTY;
    }
    int value = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1u) % RX_SIZE;
    return value;
}

void board_uart_write(const char *data, int length)
{
    for (int i = 0; i < length; i++) {
        uint32_t next = (tx_head + 1u) % TX_SIZE;
        if (next == tx_tail) {
            break; /* full: drop the rest rather than stall the control loop */
        }
        tx_ring[tx_head] = (uint8_t)data[i];
        tx_head = next;
    }
    USART1->CR1 |= USART_CR1_TXEIE;
}

/* ---- Watchdog, tick, LED ---- */

static void watchdog_init(void)
{
    IWDG_KR = 0x5555u; /* unlock PR and RLR */
    IWDG_PR = 1u;      /* LSI (about 40 kHz) / 8 */
    IWDG_RLR = 500u;   /* 500 / 5 kHz = 100 ms (67 to 133 ms over the LSI's spread) */
    /* stage1 already runs it at 3 s: wait (about 0.1 ms) until the new values are in, so
     * the reload below already uses 100 ms. */
    for (uint32_t wait = 0; (IWDG_SR & 3u) && wait < 100000u; wait++) {
    }
    IWDG_KR = 0xAAAAu;
    IWDG_KR = 0xCCCCu; /* start; it cannot be stopped until the next reset */
}

void board_watchdog_kick(void)
{
    IWDG_KR = 0xAAAAu;
}

void board_enter_bootloader(void)
{
    /* The Uok# reply is still in the ring: let it reach the Pi first (about 0.5 ms). */
    uint32_t start = millis;
    while ((tx_tail != tx_head || !(USART1->SR & USART_SR_TC)) && millis - start < 50u) {
        board_watchdog_kick();
    }
    restart_into_rom_bootloader();
}

void board_boot_ok(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR_CR |= PWR_CR_DBP;
    BKP_DR2 = 0; /* stage1's count of restarts in a row */
    PWR_CR &= ~PWR_CR_DBP;
}

uint32_t board_millis(void)
{
    return millis;
}

void board_wait_for_tick(void)
{
    static uint32_t next;
    if ((int32_t)(millis - next) > (int32_t)(10u * TICK_MS)) {
        next = millis; /* far behind (should not happen): restart the schedule, no burst */
    }
    while ((int32_t)(millis - next) < 0) {
    }
    next += TICK_MS;
    /* Heartbeat: PC13 LED toggles every 0.5 s while the loop runs. */
    pin_write(GPIOC, 13, (millis / 500u) % 2u);
}

void board_init(void)
{
    motors_off_pins();
    clock_init();
    systick_init();
    pin_mode(GPIOC, 13, PIN_OUTPUT_2MHZ);
    pwm_init();
    encoders_init();
    uart_init();
#ifdef HANG_TEST
    /* Test image only (build/app-hang.bin, FLASHING.md section 3): hang before the
     * application's own watchdog starts, to prove stage1's 3 s watchdog and crash count. */
    for (;;) {
    }
#endif
    watchdog_init();
}
