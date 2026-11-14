/* The few STM32F103 registers board.c uses, with the reference manual sections
 * (RM0008). Written out here instead of pulling in the vendor headers, so the build needs
 * only arm-none-eabi-gcc. */
#ifndef STM32F103_H
#define STM32F103_H

#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* RCC, RM0008 7.3 */
#define RCC_BASE 0x40021000u
#define RCC_CR REG(RCC_BASE + 0x00)
#define RCC_CFGR REG(RCC_BASE + 0x04)
#define RCC_APB2ENR REG(RCC_BASE + 0x18)
#define RCC_APB1ENR REG(RCC_BASE + 0x1C)
#define RCC_CSR REG(RCC_BASE + 0x24)
#define RCC_CR_HSION (1u << 0)
#define RCC_CR_HSIRDY (1u << 1)
#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_PLLON (1u << 24)
#define RCC_CR_PLLRDY (1u << 25)
#define RCC_APB2ENR_IOPAEN (1u << 2)
#define RCC_APB2ENR_IOPBEN (1u << 3)
#define RCC_APB2ENR_IOPCEN (1u << 4)
#define RCC_APB2ENR_USART1EN (1u << 14)
#define RCC_APB1ENR_TIM2EN (1u << 0)
#define RCC_APB1ENR_TIM3EN (1u << 1)
#define RCC_APB1ENR_TIM4EN (1u << 2)
#define RCC_APB1ENR_BKPEN (1u << 27)
#define RCC_APB1ENR_PWREN (1u << 28)
#define RCC_CSR_RMVF (1u << 24)
#define RCC_CSR_PINRSTF (1u << 26)
#define RCC_CSR_PORRSTF (1u << 27)
#define RCC_CSR_SFTRSTF (1u << 28)
#define RCC_CSR_IWDGRSTF (1u << 29)
#define RCC_CSR_WWDGRSTF (1u << 30)

/* Power control and backup registers, RM0008 5.4 and 6.4. The backup registers keep their
 * value across a system reset (and across power-off while VBAT is fed). */
#define PWR_CR REG(0x40007000u)
#define PWR_CR_DBP (1u << 8) /* allow writes to the backup domain */
#define BKP_DR1 REG(0x40006C04u)
#define BKP_DR2 REG(0x40006C08u)

/* Flash access control, RM0008 3.3.3 */
#define FLASH_ACR REG(0x40022000u)

/* GPIO, RM0008 9.2 */
typedef struct {
    volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} gpio_regs;
#define GPIOA ((gpio_regs *)0x40010800u)
#define GPIOB ((gpio_regs *)0x40010C00u)
#define GPIOC ((gpio_regs *)0x40011000u)
/* 4-bit pin configurations (CNF << 2 | MODE) */
#define PIN_INPUT_FLOATING 0x4u
#define PIN_INPUT_PULL 0x8u      /* pull-up or pull-down, chosen by ODR */
#define PIN_OUTPUT_2MHZ 0x2u     /* push-pull */
#define PIN_ALT_OUTPUT_2MHZ 0xAu /* alternate function push-pull */

/* General-purpose timers TIM2 to TIM4, RM0008 15.4 */
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR,
        CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR;
} tim_regs;
#define TIM2 ((tim_regs *)0x40000000u)
#define TIM3 ((tim_regs *)0x40000400u)
#define TIM4 ((tim_regs *)0x40000800u)

/* USART, RM0008 27.6 */
typedef struct {
    volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} usart_regs;
#define USART1 ((usart_regs *)0x40013800u)
#define USART_SR_PE (1u << 0)
#define USART_SR_FE (1u << 1)
#define USART_SR_NE (1u << 2)
#define USART_SR_ORE (1u << 3)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TC (1u << 6)
#define USART_SR_TXE (1u << 7)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TXEIE (1u << 7)
#define USART_CR1_PCE (1u << 10)
#define USART_CR1_M (1u << 12)
#define USART_CR1_UE (1u << 13)

/* Independent watchdog, RM0008 19.4 */
#define IWDG_KR REG(0x40003000u)
#define IWDG_PR REG(0x40003004u)
#define IWDG_RLR REG(0x40003008u)
#define IWDG_SR REG(0x4000300Cu) /* PVU, RVU: an update of PR or RLR is under way */

/* Cortex-M3 core: SysTick and NVIC (PM0056 4.5 and 4.3) */
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define SYST_CSR_COUNTFLAG (1u << 16) /* set at each wrap, cleared by reading CSR */
#define NVIC_ISER1 REG(0xE000E104u)
/* System control block, PM0056 4.4 */
#define SCB_VTOR REG(0xE000ED08u)
#define SCB_AIRCR REG(0xE000ED0Cu)
#define SCB_AIRCR_SYSRESETREQ 0x05FA0004u /* key + request a system reset */
#define USART1_IRQ 37

#endif
