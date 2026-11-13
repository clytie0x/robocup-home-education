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

/* Cortex-M3 core: SysTick and NVIC (PM0056 4.5 and 4.3) */
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define NVIC_ISER1 REG(0xE000E104u)
#define USART1_IRQ 37

#endif
