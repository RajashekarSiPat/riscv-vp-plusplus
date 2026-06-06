#pragma once

#include <cstdint>

namespace stm32f1 {

/*
 * The FE310-style PLIC occupies the architectural STM32 peripheral window at
 * 0x40000000. Keep every RM0008 peripheral offset intact under this alias.
 */
static constexpr uint64_t PERIPH_ALIAS = 0x01000000ull;

static constexpr uint64_t AFIO = 0x40010000ull + PERIPH_ALIAS;
static constexpr uint64_t EXTI = 0x40010400ull + PERIPH_ALIAS;
static constexpr uint64_t GPIOA = 0x40010800ull + PERIPH_ALIAS;
static constexpr uint64_t GPIOB = 0x40010C00ull + PERIPH_ALIAS;
static constexpr uint64_t GPIOC = 0x40011000ull + PERIPH_ALIAS;
static constexpr uint64_t GPIOD = 0x40011400ull + PERIPH_ALIAS;
static constexpr uint64_t GPIOE = 0x40011800ull + PERIPH_ALIAS;
static constexpr uint64_t DMA1 = 0x40020000ull + PERIPH_ALIAS;
static constexpr uint64_t CRC = 0x40023000ull + PERIPH_ALIAS;
static constexpr uint64_t FLASH = 0x40022000ull + PERIPH_ALIAS;
static constexpr uint64_t ADC1 = 0x40012400ull + PERIPH_ALIAS;
static constexpr uint64_t ADC2 = 0x40012800ull + PERIPH_ALIAS;
static constexpr uint64_t ADC3 = 0x40013C00ull + PERIPH_ALIAS;
static constexpr uint64_t DAC = 0x40007400ull + PERIPH_ALIAS;
static constexpr uint64_t CAN1 = 0x40006400ull + PERIPH_ALIAS;
static constexpr uint64_t SDIO = 0x40018000ull + PERIPH_ALIAS;
static constexpr uint64_t RTC = 0x40002800ull + PERIPH_ALIAS;
static constexpr uint64_t BKP = 0x40006C00ull + PERIPH_ALIAS;
static constexpr uint64_t PWR = 0x40007000ull + PERIPH_ALIAS;
static constexpr uint64_t WWDG = 0x40002C00ull + PERIPH_ALIAS;
static constexpr uint64_t IWDG = 0x40003000ull + PERIPH_ALIAS;
static constexpr uint64_t TIM2 = 0x40000000ull + PERIPH_ALIAS;
static constexpr uint64_t TIM3 = 0x40000400ull + PERIPH_ALIAS;
static constexpr uint64_t TIM4 = 0x40000800ull + PERIPH_ALIAS;
static constexpr uint64_t TIM5 = 0x40000C00ull + PERIPH_ALIAS;
static constexpr uint64_t TIM1 = 0x40012C00ull + PERIPH_ALIAS;
static constexpr uint64_t SPI2 = 0x40003800ull + PERIPH_ALIAS;
static constexpr uint64_t SPI1 = 0x40013000ull + PERIPH_ALIAS;
static constexpr uint64_t USART2 = 0x40004400ull + PERIPH_ALIAS;
static constexpr uint64_t USART1 = 0x40013800ull + PERIPH_ALIAS;
static constexpr uint64_t I2C1 = 0x40005400ull + PERIPH_ALIAS;
static constexpr uint64_t I2C2 = 0x40005800ull + PERIPH_ALIAS;
static constexpr uint64_t RCC = 0x40021000ull + PERIPH_ALIAS;

static constexpr uint32_t RCC_APB2_AFIO = 1u << 0;
static constexpr uint32_t RCC_APB2_GPIOA = 1u << 2;
static constexpr uint32_t RCC_APB2_GPIOB = 1u << 3;
static constexpr uint32_t RCC_APB2_GPIOC = 1u << 4;
static constexpr uint32_t RCC_APB2_GPIOD = 1u << 5;
static constexpr uint32_t RCC_APB2_GPIOE = 1u << 6;
static constexpr uint32_t RCC_APB2_ADC1 = 1u << 9;
static constexpr uint32_t RCC_APB2_ADC2 = 1u << 10;

static constexpr uint32_t RCC_AHB_DMA1 = 1u << 0;
static constexpr uint32_t RCC_AHB_DMA2 = 1u << 1;
static constexpr uint32_t RCC_AHB_CRC = 1u << 6;
static constexpr uint32_t RCC_AHB_FSMC = 1u << 8;
static constexpr uint32_t RCC_AHB_SDIO = 1u << 10;
static constexpr uint32_t RCC_AHB_OTGFS = 1u << 12;

static constexpr uint32_t RCC_APB1_TIM2 = 1u << 0;
static constexpr uint32_t RCC_APB1_TIM3 = 1u << 1;
static constexpr uint32_t RCC_APB1_TIM4 = 1u << 2;
static constexpr uint32_t RCC_APB1_TIM5 = 1u << 3;
static constexpr uint32_t RCC_APB1_WWDG = 1u << 11;
static constexpr uint32_t RCC_APB2_TIM1 = 1u << 11;
static constexpr uint32_t RCC_APB1_CAN1 = 1u << 25;
static constexpr uint32_t RCC_APB1_DAC = 1u << 29;

static constexpr uint32_t DMA1_CH1_IRQ = 7u;
static constexpr uint32_t DMA1_CH2_IRQ = 8u;
static constexpr uint32_t DMA1_CH3_IRQ = 9u;
static constexpr uint32_t DMA1_CH4_IRQ = 10u;
static constexpr uint32_t DMA1_CH5_IRQ = 11u;
static constexpr uint32_t DMA1_CH6_IRQ = 12u;
static constexpr uint32_t DMA1_CH7_IRQ = 13u;

static constexpr uint32_t RCC_APB1_I2C1 = 1u << 21;
static constexpr uint32_t RCC_APB1_I2C2 = 1u << 22;
static constexpr uint32_t RCC_APB1_SPI2 = 1u << 14;
static constexpr uint32_t RCC_APB1_USART2 = 1u << 17;
static constexpr uint32_t RCC_APB1_BKP = 1u << 27;
static constexpr uint32_t RCC_APB1_PWR = 1u << 28;
static constexpr uint32_t RCC_APB2_SPI1 = 1u << 12;
static constexpr uint32_t RCC_APB2_USART1 = 1u << 14;
static constexpr uint32_t RCC_BDCR_RTCEN = 1u << 15;
static constexpr uint32_t RCC_BDCR_BDRST = 1u << 16;

static constexpr uint32_t IRQ_RTC = 16u;
static constexpr uint32_t IRQ_FLASH = 23u;
static constexpr uint32_t IRQ_ADC1_2 = 24u;
static constexpr uint32_t IRQ_TIM1_UP = 17u;
static constexpr uint32_t IRQ_TIM2_UP = 18u;
static constexpr uint32_t IRQ_TIM3_UP = 19u;
static constexpr uint32_t IRQ_TIM4_UP = 20u;
static constexpr uint32_t IRQ_TIM5_UP = 21u;
static constexpr uint32_t IRQ_WWDG = 22u;
static constexpr uint32_t IRQ_CAN1_TX = 25u;
static constexpr uint32_t IRQ_CAN1_RX0 = 26u;
static constexpr uint32_t IRQ_CAN1_RX1 = 27u;
static constexpr uint32_t IRQ_CAN1_SCE = 28u;
static constexpr uint32_t IRQ_SDIO = 29u;

}  // namespace stm32f1
