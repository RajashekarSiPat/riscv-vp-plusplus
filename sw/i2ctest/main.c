/* main.c - RV32 bare-metal STM32-style peripheral verification suite for VP++
 *
 * The firmware exercises the STM32F1 peripheral models in the riscv-vp-plusplus
 * platform. I2C0 acts as master, I2C1 acts as the peer slave, and every directed
 * scenario validates the interrupts observed during the transaction.
 */

#ifndef TEST_MASK
#define TEST_MASK 0x1FFFFFFu
#endif
volatile unsigned int g_test_mask __attribute__((section(".test_cfg"))) = TEST_MASK;

#include <stdint.h>

#define MMIO32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define MMIO64(a) (*(volatile uint64_t *)(uintptr_t)(a))

#define UC_TBUF MMIO32(0x09004004UL)
#define EXITER MMIO32(0x09010000UL)

#define I2C0_BASE 0x41005400UL
#define I2C1_BASE 0x41005800UL
#define FLASH_BASE 0x41022000UL
#define ADC1_BASE 0x41012400UL
#define ADC2_BASE 0x41012800UL
#define DAC_BASE 0x41007400UL
#define CAN1_BASE 0x41006400UL
#define SDIO_BASE 0x41018000UL
#define USB_BASE 0x41005C00UL
#define OTG_BASE 0x50000000UL
#define ETH_BASE 0x41028000UL
#define AFIO_BASE 0x41010000UL
#define EXTI_BASE 0x41010400UL
#define GPIOA_BASE 0x41010800UL
#define GPIOB_BASE 0x41010C00UL
#define RCC_BASE 0x41021000UL

#define AFIO_EXTICR1 MMIO32(AFIO_BASE + 0x08UL)

#define EXTI_IMR MMIO32(EXTI_BASE + 0x00UL)
#define EXTI_RTSR MMIO32(EXTI_BASE + 0x08UL)
#define EXTI_FTSR MMIO32(EXTI_BASE + 0x0CUL)
#define EXTI_PR MMIO32(EXTI_BASE + 0x14UL)

#define GPIOA_CRL MMIO32(GPIOA_BASE + 0x00UL)
#define GPIOA_IDR MMIO32(GPIOA_BASE + 0x08UL)
#define GPIOA_ODR MMIO32(GPIOA_BASE + 0x0CUL)
#define GPIOA_BSRR MMIO32(GPIOA_BASE + 0x10UL)
#define GPIOA_BRR MMIO32(GPIOA_BASE + 0x14UL)

#define GPIOB_CRL MMIO32(GPIOB_BASE + 0x00UL)
#define GPIOB_IDR MMIO32(GPIOB_BASE + 0x08UL)
#define GPIOB_ODR MMIO32(GPIOB_BASE + 0x0CUL)
#define GPIOB_BSRR MMIO32(GPIOB_BASE + 0x10UL)
#define GPIOB_BRR MMIO32(GPIOB_BASE + 0x14UL)

#define RCC_CR MMIO32(RCC_BASE + 0x00UL)
#define RCC_CFGR MMIO32(RCC_BASE + 0x04UL)
#define RCC_CIR MMIO32(RCC_BASE + 0x08UL)
#define RCC_APB2RSTR MMIO32(RCC_BASE + 0x0CUL)
#define RCC_APB1RSTR MMIO32(RCC_BASE + 0x10UL)
#define RCC_AHBENR MMIO32(RCC_BASE + 0x14UL)
#define RCC_APB2ENR MMIO32(RCC_BASE + 0x18UL)
#define RCC_APB1ENR MMIO32(RCC_BASE + 0x1CUL)
#define RCC_AHBRSTR MMIO32(RCC_BASE + 0x28UL)
#define RCC_BDCR MMIO32(RCC_BASE + 0x20UL)
#define RCC_CSR MMIO32(RCC_BASE + 0x24UL)

#define RCC_CR_HSION (1u << 0)
#define RCC_CR_HSIRDY (1u << 1)
#define RCC_CR_HSEON (1u << 16)
#define RCC_CR_HSERDY (1u << 17)
#define RCC_CR_PLLON (1u << 24)
#define RCC_CR_PLLRDY (1u << 25)
#define RCC_APB1_I2C1 (1u << 21)
#define RCC_APB1_I2C2 (1u << 22)
#define RCC_APB1_SPI2 (1u << 14)
#define RCC_APB1_USART2 (1u << 17)
#define RCC_APB1_BKP (1u << 27)
#define RCC_APB1_PWR (1u << 28)
#define RCC_APB1_CAN1 (1u << 25)
#define RCC_APB1_DAC (1u << 29)
#define RCC_APB1_WWDG (1u << 11)
#define RCC_APB1_TIM2 (1u << 0)
#define RCC_APB1_TIM3 (1u << 1)
#define RCC_APB1_TIM4 (1u << 2)
#define RCC_APB1_TIM5 (1u << 3)
#define RCC_AHB_DMA1 (1u << 0)
#define RCC_AHB_CRC (1u << 6)
#define RCC_AHB_FSMC (1u << 8)
#define RCC_AHB_SDIO (1u << 10)
#define RCC_AHB_OTGFS (1u << 12)
#define RCC_AHB_ETHMAC (1u << 14)
#define RCC_AHB_ETHMACTX (1u << 15)
#define RCC_AHB_ETHMACRX (1u << 16)
#define RCC_APB1_USB (1u << 23)
#define RCC_APB2_AFIO (1u << 0)
#define RCC_APB2_GPIOA (1u << 2)
#define RCC_APB2_GPIOB (1u << 3)
#define RCC_APB2_ADC1 (1u << 9)
#define RCC_APB2_ADC2 (1u << 10)
#define RCC_APB2_SPI1 (1u << 12)
#define RCC_APB2_USART1 (1u << 14)
#define RCC_APB2_TIM1 (1u << 11)
#define RCC_BDCR_RTCEN (1u << 15)
#define RCC_BDCR_BDRST (1u << 16)

#define PWR_BASE 0x41007000UL
#define BKP_BASE 0x41006C00UL
#define RTC_BASE 0x41002800UL
#define PWR_CR MMIO32(PWR_BASE + 0x00UL)
#define PWR_CSR MMIO32(PWR_BASE + 0x04UL)
#define PWR_CR_DBP (1u << 8)

#define BKP_DR1 MMIO32(BKP_BASE + 0x04UL)
#define BKP_DR10 MMIO32(BKP_BASE + 0x28UL)
#define BKP_RTCCR MMIO32(BKP_BASE + 0x2CUL)
#define BKP_CR MMIO32(BKP_BASE + 0x30UL)
#define BKP_CSR MMIO32(BKP_BASE + 0x34UL)
#define BKP_DR11 MMIO32(BKP_BASE + 0x38UL)
#define BKP_DR42 MMIO32(BKP_BASE + 0xB4UL)

#define RTC_CRL MMIO32(RTC_BASE + 0x00UL)
#define RTC_CRH MMIO32(RTC_BASE + 0x04UL)
#define RTC_PRLH MMIO32(RTC_BASE + 0x08UL)
#define RTC_PRLL MMIO32(RTC_BASE + 0x0CUL)
#define RTC_DIVH MMIO32(RTC_BASE + 0x10UL)
#define RTC_DIVL MMIO32(RTC_BASE + 0x14UL)
#define RTC_CNTH MMIO32(RTC_BASE + 0x18UL)
#define RTC_CNTL MMIO32(RTC_BASE + 0x1CUL)
#define RTC_ALRH MMIO32(RTC_BASE + 0x20UL)
#define RTC_ALRL MMIO32(RTC_BASE + 0x24UL)

#define RTC_CRL_SECF (1u << 0)
#define RTC_CRL_ALRF (1u << 1)
#define RTC_CRH_SECIE (1u << 0)
#define RTC_CRH_ALRIE (1u << 1)

#define TIM1_BASE 0x41012C00UL
#define TIM2_BASE 0x41000000UL
#define TIM1_CR1 MMIO32(TIM1_BASE + 0x00UL)
#define TIM1_DIER MMIO32(TIM1_BASE + 0x0CUL)
#define TIM1_SR MMIO32(TIM1_BASE + 0x10UL)
#define TIM1_EGR MMIO32(TIM1_BASE + 0x14UL)
#define TIM1_CNT MMIO32(TIM1_BASE + 0x24UL)
#define TIM1_PSC MMIO32(TIM1_BASE + 0x28UL)
#define TIM1_ARR MMIO32(TIM1_BASE + 0x2CUL)

#define TIM2_CR1 MMIO32(TIM2_BASE + 0x00UL)
#define TIM2_DIER MMIO32(TIM2_BASE + 0x0CUL)
#define TIM2_SR MMIO32(TIM2_BASE + 0x10UL)
#define TIM2_EGR MMIO32(TIM2_BASE + 0x14UL)
#define TIM2_CNT MMIO32(TIM2_BASE + 0x24UL)
#define TIM2_PSC MMIO32(TIM2_BASE + 0x28UL)
#define TIM2_ARR MMIO32(TIM2_BASE + 0x2CUL)

#define TIM_CR1_CEN (1u << 0)
#define TIM_DIER_UIE (1u << 0)
#define TIM_SR_UIF (1u << 0)
#define TIM_EGR_UG (1u << 0)

#define FLASH_ACR MMIO32(FLASH_BASE + 0x00UL)
#define FLASH_KEYR MMIO32(FLASH_BASE + 0x04UL)
#define FLASH_OPTKEYR MMIO32(FLASH_BASE + 0x08UL)
#define FLASH_SR MMIO32(FLASH_BASE + 0x0CUL)
#define FLASH_CR MMIO32(FLASH_BASE + 0x10UL)
#define FLASH_AR MMIO32(FLASH_BASE + 0x14UL)
#define FLASH_OBR MMIO32(FLASH_BASE + 0x1CUL)
#define FLASH_WRPR MMIO32(FLASH_BASE + 0x20UL)

#define FLASH_SR_EOP (1u << 0)
#define FLASH_SR_PGERR (1u << 2)
#define FLASH_SR_WRPRTERR (1u << 4)
#define FLASH_SR_BSY (1u << 5)
#define FLASH_CR_PG (1u << 0)
#define FLASH_CR_PER (1u << 1)
#define FLASH_CR_MER (1u << 2)
#define FLASH_CR_OPTPG (1u << 4)
#define FLASH_CR_OPTER (1u << 5)
#define FLASH_CR_STRT (1u << 6)
#define FLASH_CR_LOCK (1u << 7)
#define FLASH_CR_OPTWRE (1u << 9)
#define FLASH_CR_OBL_LAUNCH (1u << 13)

#define ADC_SR MMIO32(ADC1_BASE + 0x00UL)
#define ADC_CR1 MMIO32(ADC1_BASE + 0x04UL)
#define ADC_CR2 MMIO32(ADC1_BASE + 0x08UL)
#define ADC_SMPR1 MMIO32(ADC1_BASE + 0x0CUL)
#define ADC_SMPR2 MMIO32(ADC1_BASE + 0x10UL)
#define ADC_SQR1 MMIO32(ADC1_BASE + 0x2CUL)
#define ADC_SQR2 MMIO32(ADC1_BASE + 0x30UL)
#define ADC_SQR3 MMIO32(ADC1_BASE + 0x34UL)
#define ADC_DR MMIO32(ADC1_BASE + 0x4CUL)
#define ADC2_SR MMIO32(ADC2_BASE + 0x00UL)
#define ADC2_CR1 MMIO32(ADC2_BASE + 0x04UL)
#define ADC2_CR2 MMIO32(ADC2_BASE + 0x08UL)
#define ADC2_SMPR1 MMIO32(ADC2_BASE + 0x0CUL)
#define ADC2_SMPR2 MMIO32(ADC2_BASE + 0x10UL)
#define ADC2_SQR1 MMIO32(ADC2_BASE + 0x2CUL)
#define ADC2_SQR2 MMIO32(ADC2_BASE + 0x30UL)
#define ADC2_SQR3 MMIO32(ADC2_BASE + 0x34UL)
#define ADC2_DR MMIO32(ADC2_BASE + 0x4CUL)
#define ADC_SR_EOC (1u << 1)
#define ADC_SR_STRT (1u << 4)
#define ADC_CR1_EOCIE (1u << 5)
#define ADC_CR2_ADON (1u << 0)
#define ADC_CR2_SWSTART (1u << 22)

#define DAC_CR MMIO32(DAC_BASE + 0x00UL)
#define DAC_SWTRIGR MMIO32(DAC_BASE + 0x04UL)
#define DAC_DHR12R1 MMIO32(DAC_BASE + 0x08UL)
#define DAC_DHR12L1 MMIO32(DAC_BASE + 0x0CUL)
#define DAC_DHR8R1 MMIO32(DAC_BASE + 0x10UL)
#define DAC_DHR12R2 MMIO32(DAC_BASE + 0x14UL)
#define DAC_DHR12L2 MMIO32(DAC_BASE + 0x18UL)
#define DAC_DHR8R2 MMIO32(DAC_BASE + 0x1CUL)
#define DAC_DHR12RD MMIO32(DAC_BASE + 0x20UL)
#define DAC_DHR12LD MMIO32(DAC_BASE + 0x24UL)
#define DAC_DHR8RD MMIO32(DAC_BASE + 0x28UL)
#define DAC_DOR1 MMIO32(DAC_BASE + 0x2CUL)
#define DAC_DOR2 MMIO32(DAC_BASE + 0x30UL)
#define DAC_CR_EN1 (1u << 0)
#define DAC_CR_BOFF1 (1u << 1)
#define DAC_CR_TEN1 (1u << 2)
#define DAC_CR_TSEL1_MASK (0x7u << 3)
#define DAC_CR_WAVE1_MASK (0x3u << 6)
#define DAC_CR_MAMP1_MASK (0xFu << 8)
#define DAC_CR_EN2 (1u << 16)
#define DAC_CR_BOFF2 (1u << 17)
#define DAC_CR_TEN2 (1u << 18)
#define DAC_CR_TSEL2_MASK (0x7u << 19)
#define DAC_CR_WAVE2_MASK (0x3u << 22)
#define DAC_CR_MAMP2_MASK (0xFu << 24)
#define DAC_CR_RW_MASK (DAC_CR_EN1 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_MASK | \
                        DAC_CR_WAVE1_MASK | DAC_CR_MAMP1_MASK | DAC_CR_EN2 | DAC_CR_BOFF2 | \
                        DAC_CR_TEN2 | DAC_CR_TSEL2_MASK | DAC_CR_WAVE2_MASK | DAC_CR_MAMP2_MASK)
#define DAC_SWTRIGR_SWTRIG1 (1u << 0)
#define DAC_SWTRIGR_SWTRIG2 (1u << 1)

#define CAN1_MCR MMIO32(CAN1_BASE + 0x00UL)
#define CAN1_MSR MMIO32(CAN1_BASE + 0x04UL)
#define CAN1_TSR MMIO32(CAN1_BASE + 0x08UL)
#define CAN1_RF0R MMIO32(CAN1_BASE + 0x0CUL)
#define CAN1_RF1R MMIO32(CAN1_BASE + 0x10UL)
#define CAN1_IER MMIO32(CAN1_BASE + 0x14UL)
#define CAN1_ESR MMIO32(CAN1_BASE + 0x18UL)
#define CAN1_BTR MMIO32(CAN1_BASE + 0x1CUL)
#define CAN1_TI0R MMIO32(CAN1_BASE + 0x180UL)
#define CAN1_TDT0R MMIO32(CAN1_BASE + 0x184UL)
#define CAN1_TDL0R MMIO32(CAN1_BASE + 0x188UL)
#define CAN1_TDH0R MMIO32(CAN1_BASE + 0x18CUL)
#define CAN1_RI0R MMIO32(CAN1_BASE + 0x1B0UL)
#define CAN1_RDT0R MMIO32(CAN1_BASE + 0x1B4UL)
#define CAN1_RDL0R MMIO32(CAN1_BASE + 0x1B8UL)
#define CAN1_RDH0R MMIO32(CAN1_BASE + 0x1BCUL)
#define CAN_MCR_INRQ (1u << 0)
#define CAN_MCR_SLEEP (1u << 1)
#define CAN_MCR_TXFP (1u << 2)
#define CAN_MCR_RFLM (1u << 3)
#define CAN_MCR_NART (1u << 4)
#define CAN_MCR_AWUM (1u << 5)
#define CAN_MCR_ABOM (1u << 6)
#define CAN_MCR_TTCM (1u << 7)
#define CAN_MCR_RW_MASK (CAN_MCR_INRQ | CAN_MCR_SLEEP | CAN_MCR_TXFP | CAN_MCR_RFLM | \
                         CAN_MCR_NART | CAN_MCR_AWUM | CAN_MCR_ABOM | CAN_MCR_TTCM)
#define CAN_TSR_RQCP0 (1u << 0)
#define CAN_TSR_TXOK0 (1u << 1)
#define CAN_TSR_TME0 (1u << 26)
#define CAN_TSR_TME1 (1u << 27)
#define CAN_TSR_TME2 (1u << 28)
#define CAN_IER_TMEIE (1u << 0)
#define CAN_IER_FMPIE0 (1u << 1)
#define CAN_IER_FFIE0 (1u << 2)
#define CAN_IER_FOVIE0 (1u << 3)
#define CAN_IER_ERRIE (1u << 15)
#define CAN_IER_WKUIE (1u << 16)
#define CAN_IER_BOFIE (1u << 17)
#define CAN_IER_EPVIE (1u << 18)
#define CAN_IER_EWGIE (1u << 19)
#define CAN_TIR_TXRQ (1u << 0)

#define SDIO_POWER MMIO32(SDIO_BASE + 0x00UL)
#define SDIO_CLKCR MMIO32(SDIO_BASE + 0x04UL)
#define SDIO_ARG MMIO32(SDIO_BASE + 0x08UL)
#define SDIO_CMD MMIO32(SDIO_BASE + 0x0CUL)
#define SDIO_RESPCMD MMIO32(SDIO_BASE + 0x10UL)
#define SDIO_RESP1 MMIO32(SDIO_BASE + 0x14UL)
#define SDIO_RESP2 MMIO32(SDIO_BASE + 0x18UL)
#define SDIO_RESP3 MMIO32(SDIO_BASE + 0x1CUL)
#define SDIO_RESP4 MMIO32(SDIO_BASE + 0x20UL)
#define SDIO_DTIMER MMIO32(SDIO_BASE + 0x24UL)
#define SDIO_DLEN MMIO32(SDIO_BASE + 0x28UL)
#define SDIO_DCTRL MMIO32(SDIO_BASE + 0x2CUL)
#define SDIO_DCOUNT MMIO32(SDIO_BASE + 0x30UL)
#define SDIO_STA MMIO32(SDIO_BASE + 0x34UL)
#define SDIO_ICR MMIO32(SDIO_BASE + 0x38UL)
#define SDIO_MASK MMIO32(SDIO_BASE + 0x3CUL)
#define SDIO_FIFOCNT MMIO32(SDIO_BASE + 0x48UL)
#define SDIO_FIFO MMIO32(SDIO_BASE + 0x80UL)
#define SDIO_POWER_PWRCTRL_MASK 0x3u
#define SDIO_CLKCR_CLKDIV_MASK 0xFFu
#define SDIO_CLKCR_CLKEN (1u << 8)
#define SDIO_CLKCR_PWRSAV (1u << 9)
#define SDIO_CLKCR_BYPASS (1u << 10)
#define SDIO_CLKCR_WIDBUS_MASK (0x3u << 11)
#define SDIO_CLKCR_HWFC_EN (1u << 14)
#define SDIO_CMD_CMDINDEX_MASK 0x3Fu
#define SDIO_CMD_WAITRESP_MASK (0x3u << 6)
#define SDIO_CMD_CPSMEN (1u << 10)
#define SDIO_CMD_ENCMDCOMPL (1u << 12)
#define SDIO_CMD_NIEN (1u << 13)
#define SDIO_CMD_ATACMD (1u << 14)
#define SDIO_DCTRL_DTEN (1u << 0)
#define SDIO_DCTRL_DTDIR (1u << 1)
#define SDIO_DCTRL_DTMODE (1u << 2)
#define SDIO_DCTRL_DMAEN (1u << 3)
#define SDIO_DCTRL_DBLOCKSIZE_MASK (0xFu << 4)
#define SDIO_STA_CMDREND (1u << 6)
#define SDIO_STA_CMDSENT (1u << 7)
#define SDIO_STA_DATAEND (1u << 8)
#define SDIO_STA_TXACT (1u << 12)
#define SDIO_STA_RXACT (1u << 13)
#define SDIO_STA_TXFIFOHE (1u << 14)
#define SDIO_STA_RXFIFOHF (1u << 15)
#define SDIO_MASK_CMDRENDIE (1u << 6)
#define SDIO_MASK_CMDSENTIE (1u << 7)
#define SDIO_MASK_DATAENDIE (1u << 8)

#define FSMC_BASE 0xA0000000UL
#define FSMC_BCR1 MMIO32(FSMC_BASE + 0x00UL)
#define FSMC_BTR1 MMIO32(FSMC_BASE + 0x04UL)
#define FSMC_BCR2 MMIO32(FSMC_BASE + 0x08UL)
#define FSMC_BTR2 MMIO32(FSMC_BASE + 0x0CUL)
#define FSMC_BCR3 MMIO32(FSMC_BASE + 0x10UL)
#define FSMC_BTR3 MMIO32(FSMC_BASE + 0x14UL)
#define FSMC_BCR4 MMIO32(FSMC_BASE + 0x18UL)
#define FSMC_BTR4 MMIO32(FSMC_BASE + 0x1CUL)
#define FSMC_BCR_MBKEN (1u << 0)
#define FSMC_BCR_MUXEN (1u << 1)
#define FSMC_BCR_MTYP_MASK (0x3u << 2)
#define FSMC_BCR_MWID_MASK (0x3u << 4)
#define FSMC_BCR_FACCEN (1u << 6)
#define FSMC_BCR_BURSTEN (1u << 8)
#define FSMC_BCR_WAITPOL (1u << 9)
#define FSMC_BCR_WRAPMOD (1u << 10)
#define FSMC_BCR_WAITCFG (1u << 11)
#define FSMC_BCR_WREN (1u << 12)
#define FSMC_BCR_WAITEN (1u << 13)
#define FSMC_BCR_EXTMOD (1u << 14)
#define FSMC_BCR_ASYNCWAIT (1u << 15)
#define FSMC_BCR_CBURSTRW (1u << 19)
#define FSMC_BCR_CCLKEN (1u << 20)
#define FSMC_BCR_RW_MASK (FSMC_BCR_MBKEN | FSMC_BCR_MUXEN | FSMC_BCR_MTYP_MASK | FSMC_BCR_MWID_MASK | \
                          FSMC_BCR_FACCEN | FSMC_BCR_BURSTEN | FSMC_BCR_WAITPOL | FSMC_BCR_WRAPMOD | \
                          FSMC_BCR_WAITCFG | FSMC_BCR_WREN | FSMC_BCR_WAITEN | FSMC_BCR_EXTMOD | \
                          FSMC_BCR_ASYNCWAIT | FSMC_BCR_CBURSTRW | FSMC_BCR_CCLKEN)
#define FSMC_BTR_RW_MASK 0x0FFFFFFFu

#define USB_EP0R MMIO32(USB_BASE + 0x00UL)
#define USB_EP1R MMIO32(USB_BASE + 0x04UL)
#define USB_EP2R MMIO32(USB_BASE + 0x08UL)
#define USB_EP3R MMIO32(USB_BASE + 0x0CUL)
#define USB_CNTR MMIO32(USB_BASE + 0x40UL)
#define USB_ISTR MMIO32(USB_BASE + 0x44UL)
#define USB_FNR MMIO32(USB_BASE + 0x48UL)
#define USB_DADDR MMIO32(USB_BASE + 0x4CUL)
#define USB_BTABLE MMIO32(USB_BASE + 0x50UL)
#define USB_CNTR_RESETM (1u << 10)
#define USB_CNTR_CTRM (1u << 15)
#define USB_ISTR_RESET (1u << 10)
#define USB_ISTR_CTR (1u << 15)

#define OTG_GOTGCTL MMIO32(OTG_BASE + 0x00UL)
#define OTG_GOTGINT MMIO32(OTG_BASE + 0x04UL)
#define OTG_GAHBCFG MMIO32(OTG_BASE + 0x08UL)
#define OTG_GUSBCFG MMIO32(OTG_BASE + 0x0CUL)
#define OTG_GRSTCTL MMIO32(OTG_BASE + 0x10UL)
#define OTG_GINTSTS MMIO32(OTG_BASE + 0x14UL)
#define OTG_GINTMSK MMIO32(OTG_BASE + 0x18UL)
#define OTG_GRXFSIZ MMIO32(OTG_BASE + 0x24UL)
#define OTG_GNPTXFSIZ MMIO32(OTG_BASE + 0x28UL)
#define OTG_GAHBCFG_GINT (1u << 0)
#define OTG_GUSBCFG_FDMOD (1u << 30)
#define OTG_GRSTCTL_CSRST (1u << 0)
#define OTG_GINTSTS_USBRST (1u << 12)
#define OTG_GINTSTS_ENUMDNE (1u << 13)

#define ETH_STATUS MMIO32(ETH_BASE + 0x00UL)
#define ETH_RECV_SIZE MMIO32(ETH_BASE + 0x04UL)
#define ETH_RECV_DST MMIO32(ETH_BASE + 0x08UL)
#define ETH_SEND_SRC MMIO32(ETH_BASE + 0x0CUL)
#define ETH_SEND_SIZE MMIO32(ETH_BASE + 0x10UL)
#define ETH_MAC_HIGH MMIO32(ETH_BASE + 0x14UL)
#define ETH_MAC_LOW MMIO32(ETH_BASE + 0x18UL)
#define ETH_MACCR MMIO32(ETH_BASE + 0x100UL)
#define ETH_MACFFR MMIO32(ETH_BASE + 0x104UL)
#define ETH_MACMIIAR MMIO32(ETH_BASE + 0x110UL)
#define ETH_MACMIIDR MMIO32(ETH_BASE + 0x114UL)
#define ETH_MACFCR MMIO32(ETH_BASE + 0x118UL)
#define ETH_MACVLANTR MMIO32(ETH_BASE + 0x11CUL)
#define ETH_DMASR MMIO32(ETH_BASE + 0x200UL)
#define ETH_DMAIER MMIO32(ETH_BASE + 0x204UL)
#define ETH_DMAOMR MMIO32(ETH_BASE + 0x208UL)
#define ETH_DMABMR MMIO32(ETH_BASE + 0x20CUL)
#define ETH_STATUS_RECV (1u << 0)
#define ETH_STATUS_SEND (1u << 1)
#define ETH_DMASR_TI (1u << 0)
#define ETH_DMASR_RI (1u << 6)
#define ETH_DMASR_NIS (1u << 16)
#define ETH_DMAIER_TIE (1u << 0)
#define ETH_DMAIER_RIE (1u << 6)
#define ETH_DMAIER_NISE (1u << 16)
#define ETH_DMABMR_SR (1u << 0)

#define I2C0_CR1 MMIO32(I2C0_BASE + 0x00UL)
#define I2C0_CR2 MMIO32(I2C0_BASE + 0x04UL)
#define I2C0_OAR1 MMIO32(I2C0_BASE + 0x08UL)
#define I2C0_OAR2 MMIO32(I2C0_BASE + 0x0CUL)
#define I2C0_DR MMIO32(I2C0_BASE + 0x10UL)
#define I2C0_SR1 MMIO32(I2C0_BASE + 0x14UL)
#define I2C0_SR2 MMIO32(I2C0_BASE + 0x18UL)
#define I2C0_CCR MMIO32(I2C0_BASE + 0x1CUL)
#define I2C0_TRISE MMIO32(I2C0_BASE + 0x20UL)

#define I2C1_CR1 MMIO32(I2C1_BASE + 0x00UL)
#define I2C1_CR2 MMIO32(I2C1_BASE + 0x04UL)
#define I2C1_OAR1 MMIO32(I2C1_BASE + 0x08UL)
#define I2C1_OAR2 MMIO32(I2C1_BASE + 0x0CUL)
#define I2C1_DR MMIO32(I2C1_BASE + 0x10UL)
#define I2C1_SR1 MMIO32(I2C1_BASE + 0x14UL)
#define I2C1_SR2 MMIO32(I2C1_BASE + 0x18UL)
#define I2C1_CCR MMIO32(I2C1_BASE + 0x1CUL)
#define I2C1_TRISE MMIO32(I2C1_BASE + 0x20UL)

#define CR1_PE (1u << 0)
#define CR1_ENGC (1u << 6)
#define CR1_START (1u << 8)
#define CR1_STOP (1u << 9)
#define CR1_ACK (1u << 10)
#define CR1_SWRST (1u << 15)
#define CR1_RW_MASK (CR1_PE | CR1_ACK | CR1_ENGC)

#define CR2_FREQ_MASK (0x3Fu)
#define CR2_ITERREN (1u << 8)
#define CR2_ITEVTEN (1u << 9)
#define CR2_ITBUFEN (1u << 10)
#define CR2_DMAEN (1u << 11)
#define CR2_LAST (1u << 12)
#define CR2_RW_MASK (CR2_FREQ_MASK | CR2_ITERREN | CR2_ITEVTEN | CR2_ITBUFEN | CR2_DMAEN | CR2_LAST)

#define SR1_SB (1u << 0)
#define SR1_ADDR (1u << 1)
#define SR1_BTF (1u << 2)
#define SR1_ADD10 (1u << 3)
#define SR1_STOPF (1u << 4)
#define SR1_RxNE (1u << 6)
#define SR1_TxE (1u << 7)
#define SR1_BERR (1u << 8)
#define SR1_ARLO (1u << 9)
#define SR1_AF (1u << 10)
#define SR1_OVR (1u << 11)
#define SR1_PECERR (1u << 12)
#define SR1_TIMEOUT (1u << 14)
#define SR1_SMBALERT (1u << 15)
#define SR1_ERR_MASK (SR1_BERR | SR1_ARLO | SR1_AF | SR1_OVR | SR1_PECERR | SR1_TIMEOUT | SR1_SMBALERT)

#define SR2_MSL (1u << 0)
#define SR2_BUSY (1u << 1)
#define SR2_TRA (1u << 2)
#define SR2_GENCALL (1u << 4)
#define SR2_SMBDEFAULT (1u << 5)
#define SR2_SMBHOST (1u << 6)
#define SR2_DUALF (1u << 7)

#define OAR1_ADDMODE (1u << 15)
#define OAR1_RW_MASK (OAR1_ADDMODE | 0x03FFu)
#define OAR2_ENDUAL (1u << 0)
#define OAR2_RW_MASK 0x00FFu

#define PLIC_BASE 0x40000000UL
#define PLIC_PRIO(n) MMIO32(PLIC_BASE + (n) * 4UL)
#define PLIC_PENDING0 MMIO32(PLIC_BASE + 0x1000UL)
#define PLIC_ENABLE_HART0(n) MMIO32(PLIC_BASE + 0x2000UL + (((n) / 32u) * 4UL))
#define PLIC_THRESHOLD_HART0 MMIO32(PLIC_BASE + 0x200000UL)
#define PLIC_CLAIM_HART0 MMIO32(PLIC_BASE + 0x200004UL)

#define IRQ_FLASH 23u
#define IRQ_I2C0_EV 1u
#define IRQ_I2C0_ER 2u
#define IRQ_I2C1_EV 3u
#define IRQ_I2C1_ER 4u
#define IRQ_EXTI1 6u
#define IRQ_DMA1_CH1 7u
#define IRQ_ADC1_2 24u
#define IRQ_CAN1_TX 25u
#define IRQ_CAN1_RX0 26u
#define IRQ_CAN1_RX1 27u
#define IRQ_CAN1_SCE 28u
#define IRQ_SDIO 29u
#define IRQ_USB 30u
#define IRQ_ETH 31u
#define IRQ_RTC 16u
#define IRQ_TIM1_UP 17u
#define IRQ_TIM2_UP 18u
#define IRQ_WWDG 22u

#define WWDG_BASE 0x41002C00UL
#define WWDG_CR MMIO32(WWDG_BASE + 0x00UL)
#define WWDG_CFR MMIO32(WWDG_BASE + 0x04UL)
#define WWDG_SR MMIO32(WWDG_BASE + 0x08UL)
#define WWDG_CR_T_MASK 0x7Fu
#define WWDG_CR_WDGA (1u << 7)
#define WWDG_CFR_W_MASK 0x7Fu
#define WWDG_CFR_WDGTB_MASK (0x3u << 7)
#define WWDG_CFR_EWI (1u << 9)
#define WWDG_SR_EWIF (1u << 0)

#define IWDG_BASE 0x41003000UL
#define IWDG_KR MMIO32(IWDG_BASE + 0x00UL)
#define IWDG_PR MMIO32(IWDG_BASE + 0x04UL)
#define IWDG_RLR MMIO32(IWDG_BASE + 0x08UL)
#define IWDG_SR MMIO32(IWDG_BASE + 0x0CUL)
#define IWDG_KR_START 0xCCCCu
#define IWDG_KR_RELOAD 0xAAAAu
#define IWDG_KR_UNLOCK 0x5555u
#define IWDG_PR_MASK 0x7u
#define IWDG_RLR_MASK 0x0FFFu

#define DMA1_BASE 0x41020000UL
#define DMA1_ISR MMIO32(DMA1_BASE + 0x00UL)
#define DMA1_IFCR MMIO32(DMA1_BASE + 0x04UL)
#define DMA1_CCR1 MMIO32(DMA1_BASE + 0x08UL)
#define DMA1_CNDTR1 MMIO32(DMA1_BASE + 0x0CUL)
#define DMA1_CPAR1 MMIO32(DMA1_BASE + 0x10UL)
#define DMA1_CMAR1 MMIO32(DMA1_BASE + 0x14UL)

#define CRC_BASE 0x41023000UL
#define CRC_DR MMIO32(CRC_BASE + 0x00UL)
#define CRC_IDR MMIO32(CRC_BASE + 0x04UL)
#define CRC_CR MMIO32(CRC_BASE + 0x08UL)

#define SPI1_BASE 0x41013000UL
#define SPI2_BASE 0x41003800UL
#define SPI1_CR1 MMIO32(SPI1_BASE + 0x00UL)
#define SPI1_CR2 MMIO32(SPI1_BASE + 0x04UL)
#define SPI1_SR MMIO32(SPI1_BASE + 0x08UL)
#define SPI1_DR MMIO32(SPI1_BASE + 0x0CUL)
#define SPI1_CRCPR MMIO32(SPI1_BASE + 0x10UL)
#define SPI1_RXCRCR MMIO32(SPI1_BASE + 0x14UL)
#define SPI1_TXCRCR MMIO32(SPI1_BASE + 0x18UL)
#define SPI1_I2SCFGR MMIO32(SPI1_BASE + 0x1CUL)
#define SPI1_I2SPR MMIO32(SPI1_BASE + 0x20UL)
#define SPI2_CR1 MMIO32(SPI2_BASE + 0x00UL)
#define SPI2_CR2 MMIO32(SPI2_BASE + 0x04UL)
#define SPI2_SR MMIO32(SPI2_BASE + 0x08UL)
#define SPI2_DR MMIO32(SPI2_BASE + 0x0CUL)
#define SPI2_CRCPR MMIO32(SPI2_BASE + 0x10UL)
#define SPI2_RXCRCR MMIO32(SPI2_BASE + 0x14UL)
#define SPI2_TXCRCR MMIO32(SPI2_BASE + 0x18UL)
#define SPI2_I2SCFGR MMIO32(SPI2_BASE + 0x1CUL)
#define SPI2_I2SPR MMIO32(SPI2_BASE + 0x20UL)

#define USART1_BASE 0x41013800UL
#define USART2_BASE 0x41004400UL
#define USART1_SR MMIO32(USART1_BASE + 0x00UL)
#define USART1_DR MMIO32(USART1_BASE + 0x04UL)
#define USART1_BRR MMIO32(USART1_BASE + 0x08UL)
#define USART1_CR1 MMIO32(USART1_BASE + 0x0CUL)
#define USART1_CR2 MMIO32(USART1_BASE + 0x10UL)
#define USART1_CR3 MMIO32(USART1_BASE + 0x14UL)
#define USART1_GTPR MMIO32(USART1_BASE + 0x18UL)
#define USART2_SR MMIO32(USART2_BASE + 0x00UL)
#define USART2_DR MMIO32(USART2_BASE + 0x04UL)
#define USART2_BRR MMIO32(USART2_BASE + 0x08UL)
#define USART2_CR1 MMIO32(USART2_BASE + 0x0CUL)
#define USART2_CR2 MMIO32(USART2_BASE + 0x10UL)
#define USART2_CR3 MMIO32(USART2_BASE + 0x14UL)
#define USART2_GTPR MMIO32(USART2_BASE + 0x18UL)

#define DMA_ISR_GIF1 (1u << 0)
#define DMA_ISR_TCIF1 (1u << 1)
#define DMA_ISR_HTIF1 (1u << 2)
#define DMA_ISR_TEIF1 (1u << 3)

#define DMA_IFCR_CGIF1 (1u << 0)
#define DMA_IFCR_CTCIF1 (1u << 1)
#define DMA_IFCR_CHTIF1 (1u << 2)
#define DMA_IFCR_CTEIF1 (1u << 3)

#define DMA_CCR_EN (1u << 0)
#define DMA_CCR_TCIE (1u << 1)
#define DMA_CCR_TEIE (1u << 3)
#define DMA_CCR_DIR (1u << 4)
#define DMA_CCR_CIRC (1u << 5)
#define DMA_CCR_PINC (1u << 6)
#define DMA_CCR_MINC (1u << 7)
#define DMA_CCR_PSIZE_32 (2u << 8)
#define DMA_CCR_MSIZE_32 (2u << 10)
#define DMA_CCR_MEM2MEM (1u << 14)

#define USART_SR_PE (1u << 0)
#define USART_SR_FE (1u << 1)
#define USART_SR_NE (1u << 2)
#define USART_SR_ORE (1u << 3)
#define USART_SR_IDLE (1u << 4)
#define USART_SR_RXNE (1u << 5)
#define USART_SR_TC (1u << 6)
#define USART_SR_TXE (1u << 7)

#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_IDLEIE (1u << 4)
#define USART_CR1_RXNEIE (1u << 5)
#define USART_CR1_TCIE (1u << 6)
#define USART_CR1_TXEIE (1u << 7)
#define USART_CR1_PEIE (1u << 8)
#define USART_CR1_UE (1u << 13)
#define USART_CR1_RW_MASK (USART_CR1_RE | USART_CR1_TE | USART_CR1_IDLEIE | USART_CR1_RXNEIE | \
                           USART_CR1_TCIE | USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_UE)
#define USART_CR2_RW_MASK 0xFFFFu
#define USART_CR3_RW_MASK 0x04FFu
#define USART_GTPR_RW_MASK 0xFFFFu

#define IRQ_USART1 14u
#define IRQ_USART2 15u

#define SPI_SR_RXNE (1u << 0)
#define SPI_SR_TXE (1u << 1)
#define SPI_SR_OVR (1u << 6)
#define SPI_CR1_CPHA (1u << 0)
#define SPI_CR1_CPOL (1u << 1)
#define SPI_CR1_MSTR (1u << 2)
#define SPI_CR1_BR_MASK (0x7u << 3)
#define SPI_CR1_SPE (1u << 6)
#define SPI_CR1_LSBFIRST (1u << 7)
#define SPI_CR1_SSI (1u << 8)
#define SPI_CR1_SSM (1u << 9)
#define SPI_CR1_RXONLY (1u << 10)
#define SPI_CR1_DFF (1u << 11)
#define SPI_CR1_CRCNEXT (1u << 12)
#define SPI_CR1_CRCEN (1u << 13)
#define SPI_CR1_BIDIOE (1u << 14)
#define SPI_CR1_BIDIMODE (1u << 15)
#define SPI_CR1_RW_MASK (SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | SPI_CR1_BR_MASK | SPI_CR1_SPE | \
                         SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_RXONLY | SPI_CR1_DFF | \
                         SPI_CR1_CRCNEXT | SPI_CR1_CRCEN | SPI_CR1_BIDIOE | SPI_CR1_BIDIMODE)
#define SPI_CR2_SSOE (1u << 2)
#define SPI_CR2_ERRIE (1u << 5)
#define SPI_CR2_RXNEIE (1u << 6)
#define SPI_CR2_TXEIE (1u << 7)
#define SPI_CR2_RW_MASK (SPI_CR2_SSOE | SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE)
#define SPI_IRQ1 5u
#define SPI_IRQ2 8u

#define GPIO_CRL_OUT_2MHZ_PP 0x2u
#define GPIO_CRL_IN_FLOATING 0x4u

#define I2C0_SLAVE_ADDR 0x50u
#define I2C1_SLAVE_ADDR 0x51u

#define LOG_SIZE 256u

typedef struct {
	unsigned int irq_id;
	unsigned int sr1;
	unsigned int sr2;
} I2cIrqEvent;

volatile I2cIrqEvent g_log[LOG_SIZE];
volatile unsigned int g_log_count = 0u;
static volatile uint32_t g_dma_src_words[2];
static volatile uint32_t g_dma_dst_words[2];
static volatile uint32_t g_eth_src_words[4];
static volatile uint32_t g_eth_dst_words[4];
static int g_pass = 0;
static int g_fail = 0;

static void put_char(char c) { UC_TBUF = (unsigned int)c; }
static void put_str(const char *s) { while (*s) put_char(*s++); }

static void put_hex(unsigned int v)
{
	static const char hex[] = "0123456789ABCDEF";
	int i;
	put_char('0');
	put_char('x');
	for (i = 28; i >= 0; i -= 4)
		put_char(hex[(v >> i) & 0xFu]);
}

static void put_dec(unsigned int v)
{
	char buf[12];
	int i = 0;
	if (v == 0u) {
		put_char('0');
		return;
	}
	while (v > 0u) {
		buf[i++] = (char)('0' + (v % 10u));
		v /= 10u;
	}
	while (--i >= 0) {
		put_char(buf[i]);
	}
}

static void pass_test(const char *name)
{
	++g_pass;
	put_str("[PASS] ");
	put_str(name);
	put_str("\r\n");
}

static void fail_test(const char *name, const char *why)
{
	++g_fail;
	put_str("[FAIL] ");
	put_str(name);
	put_str(" - ");
	put_str(why);
	put_str("\r\n");
}

static int expect_true(const char *name, int ok, const char *why)
{
	if (!ok) {
		fail_test(name, why);
	}
	return ok;
}

static int expect_eq(const char *name, unsigned int got, unsigned int expected)
{
	if (got == expected) {
		return 1;
	}
	put_str("[FAIL] ");
	put_str(name);
	put_str(" - got ");
	put_hex(got);
	put_str(", expected ");
	put_hex(expected);
	put_str("\r\n");
	++g_fail;
	return 0;
}

static int expect_mask(const char *name, unsigned int got, unsigned int mask, unsigned int expected)
{
	return expect_eq(name, got & mask, expected);
}

static int i2c_wait_n(unsigned int target)
{
	unsigned int timeout = 20000000u;
	while (g_log_count < target && --timeout > 0u) {
		__asm__ volatile("wfi");
		__asm__ volatile("" ::: "memory");
	}
	return (int)(g_log_count >= target);
}

static int i2c_wait_sr1(unsigned int mask)
{
	unsigned int timeout = 20000000u;
	while (--timeout > 0u) {
		if ((I2C0_SR1 & mask) == mask) {
			return 1;
		}
		__asm__ volatile("" ::: "memory");
	}
	return 0;
}

static int i2c1_wait_sr1(unsigned int mask)
{
	unsigned int timeout = 20000000u;
	while (--timeout > 0u) {
		if ((I2C1_SR1 & mask) == mask) {
			return 1;
		}
		__asm__ volatile("" ::: "memory");
	}
	return 0;
}

static int i2c_find(unsigned int from, unsigned int irq_id, unsigned int sr1_mask)
{
	for (unsigned int i = from; i < g_log_count; ++i) {
		if (g_log[i].irq_id == irq_id && (g_log[i].sr1 & sr1_mask)) {
			return (int)i;
		}
	}
	return -1;
}

static int i2c_count(unsigned int from, unsigned int irq_id, unsigned int sr1_mask)
{
	int count = 0;
	for (unsigned int i = from; i < g_log_count; ++i) {
		if (g_log[i].irq_id == irq_id && (sr1_mask == 0u || (g_log[i].sr1 & sr1_mask))) {
			++count;
		}
	}
	return count;
}

static void i2c_clear_log(void)
{
	for (unsigned int i = 0u; i < LOG_SIZE; ++i) {
		g_log[i].irq_id = 0u;
		g_log[i].sr1 = 0u;
		g_log[i].sr2 = 0u;
	}
	__asm__ volatile("fence" ::: "memory");
	g_log_count = 0u;
}

typedef struct {
	unsigned int from;
} I2cMonitor;

static void monitor_begin(I2cMonitor *m)
{
	m->from = g_log_count;
}

static int monitor_seen_ev(const I2cMonitor *m, unsigned int mask)
{
	return i2c_find(m->from, IRQ_I2C0_EV, mask) >= 0;
}

static int monitor_seen_er(const I2cMonitor *m, unsigned int mask)
{
	return i2c_find(m->from, IRQ_I2C0_ER, mask) >= 0;
}

static int monitor_ev_count(const I2cMonitor *m, unsigned int mask)
{
	return i2c_count(m->from, IRQ_I2C0_EV, mask);
}

static int monitor_er_count(const I2cMonitor *m, unsigned int mask)
{
	return i2c_count(m->from, IRQ_I2C0_ER, mask);
}

static int sb_expect_tx_sequence(const char *name, const I2cMonitor *m)
{
	int ok = 1;
	ok &= expect_true(name, monitor_seen_ev(m, SR1_SB), "missing SB event");
	ok &= expect_true(name, monitor_seen_ev(m, SR1_ADDR), "missing ADDR event");
	ok &= expect_true(name, monitor_seen_ev(m, SR1_TxE), "missing TxE event");
	ok &= expect_true(name, monitor_seen_ev(m, SR1_BTF), "missing BTF event");
	return ok;
}

static int sb_expect_rx_sequence(const char *name, const I2cMonitor *m)
{
	int ok = 1;
	ok &= expect_true(name, monitor_seen_ev(m, SR1_SB), "missing SB event");
	ok &= expect_true(name, monitor_seen_ev(m, SR1_ADDR), "missing ADDR event");
	ok &= expect_true(name, monitor_seen_ev(m, SR1_RxNE), "missing RxNE event");
	return ok;
}

static int sb_expect_error(const char *name, const I2cMonitor *m, unsigned int mask)
{
	return expect_true(name, monitor_seen_er(m, mask), "missing expected ER interrupt");
}

static int sb_expect_no_error(const char *name, const I2cMonitor *m)
{
	return expect_eq(name, (unsigned int)monitor_er_count(m, 0u), 0u);
}

static int sb_expect_sr2_bus(const char *name, unsigned int sr2, unsigned int expected)
{
	return expect_mask(name, sr2, SR2_MSL | SR2_BUSY | SR2_TRA, expected);
}

static void setup_trap_handler(void);
static void setup_plic(void);
static void enable_irq(void);
static void i2c_init(void);
static void gpio_exti_init(void);
static void usart_init(void);
static void spi_init(void);
static void test_usb_fs(void);
static void test_otg_fs(void);
static void test_eth(void);
static void i2c_stop(void);
static void i2c_start(void);
static void i2c_write_addr(unsigned int addr, unsigned int read);
static void i2c_write_byte(unsigned int data);
static unsigned int i2c_read_byte(void);
static int i2c_rx_byte(unsigned int *data);
static void i2c_recover(void);
static void i2c1_config_7bit(unsigned int addr, unsigned int cr1_extra);
static void i2c1_config_10bit(unsigned int addr10);
static void i2c_write_addr10_header(unsigned int addr10, unsigned int read);
static int i2c0_addr7_write(unsigned int addr, unsigned int data);
static int i2c0_addr7_read(unsigned int addr, unsigned int source, unsigned int *data);
static int i2c0_write_then_repeated_read(unsigned int addr, unsigned int write_data, unsigned int source, unsigned int *data);
static int i2c0_read_then_repeated_write(unsigned int addr, unsigned int source, unsigned int write_data, unsigned int *data);
static int i2c0_read_two_bytes(unsigned int addr, unsigned int first_source, unsigned int second_source, unsigned int *first, unsigned int *second);
static int i2c0_addr10_write(unsigned int addr10, unsigned int data);
static int i2c0_addr10_read(unsigned int addr10, unsigned int source, unsigned int *data);
static void i2c_finish(void);
static int start_addr_write(unsigned int addr);
static void test_fsmc(void);

extern void trap_entry(void);
void trap_handler(void);

static void setup_trap_handler(void)
{
	uintptr_t addr = (uintptr_t)(void *)trap_entry;
	__asm__ volatile("csrw mtvec, %0" :: "r"(addr));
}

static void setup_plic(void)
{
	PLIC_PRIO(IRQ_FLASH) = 1u;
	PLIC_PRIO(IRQ_I2C0_EV) = 1u;
	PLIC_PRIO(IRQ_I2C0_ER) = 1u;
	PLIC_PRIO(IRQ_I2C1_EV) = 1u;
	PLIC_PRIO(IRQ_I2C1_ER) = 1u;
	PLIC_PRIO(IRQ_EXTI1) = 1u;
	PLIC_PRIO(IRQ_DMA1_CH1) = 1u;
	PLIC_PRIO(SPI_IRQ1) = 1u;
	PLIC_PRIO(SPI_IRQ2) = 1u;
	PLIC_PRIO(IRQ_USART1) = 1u;
	PLIC_PRIO(IRQ_USART2) = 1u;
	PLIC_PRIO(IRQ_ADC1_2) = 1u;
	PLIC_PRIO(IRQ_CAN1_TX) = 1u;
	PLIC_PRIO(IRQ_CAN1_RX0) = 1u;
	PLIC_PRIO(IRQ_CAN1_RX1) = 1u;
	PLIC_PRIO(IRQ_CAN1_SCE) = 1u;
	PLIC_PRIO(IRQ_SDIO) = 1u;
	PLIC_PRIO(IRQ_USB) = 1u;
	PLIC_PRIO(IRQ_ETH) = 1u;
	PLIC_PRIO(IRQ_RTC) = 1u;
	PLIC_PRIO(IRQ_TIM1_UP) = 1u;
	PLIC_PRIO(IRQ_TIM2_UP) = 1u;
	PLIC_PRIO(IRQ_WWDG) = 1u;
	PLIC_ENABLE_HART0(IRQ_I2C0_EV) = (1u << IRQ_I2C0_EV) | (1u << IRQ_I2C0_ER) | (1u << IRQ_I2C1_EV) |
	                                  (1u << IRQ_I2C1_ER) | (1u << IRQ_EXTI1) | (1u << IRQ_DMA1_CH1) |
	                                  (1u << SPI_IRQ1) | (1u << SPI_IRQ2) | (1u << IRQ_USART1) |
	                                  (1u << IRQ_USART2) | (1u << IRQ_RTC) | (1u << IRQ_TIM1_UP) |
	                                  (1u << IRQ_TIM2_UP) | (1u << IRQ_WWDG) | (1u << IRQ_FLASH) |
	                                  (1u << IRQ_ADC1_2) | (1u << IRQ_CAN1_TX) | (1u << IRQ_CAN1_RX0) |
	                                  (1u << IRQ_CAN1_RX1) | (1u << IRQ_CAN1_SCE) | (1u << IRQ_SDIO) |
	                                  (1u << IRQ_USB) | (1u << IRQ_ETH);
	PLIC_THRESHOLD_HART0 = 0u;
}

static void enable_irq(void)
{
	__asm__ volatile("csrs mie, %0" :: "r"(1u << 11));
	__asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));
}

static void i2c_init(void)
{
	RCC_APB1ENR |= RCC_APB1_I2C1 | RCC_APB1_I2C2;
	I2C0_CR1 = CR1_SWRST;
	I2C0_CR1 = 0u;
	I2C0_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C0_CCR = 4000u;
	I2C0_TRISE = 37u;
	I2C0_CR1 = CR1_PE;

	I2C1_CR1 = CR1_SWRST;
	I2C1_CR1 = 0u;
	I2C1_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C1_OAR1 = I2C0_SLAVE_ADDR;
	I2C1_OAR2 = 0u;
	I2C1_CCR = 4000u;
	I2C1_TRISE = 37u;
	I2C1_CR1 = CR1_PE | CR1_ACK;
}

static void gpio_exti_init(void)
{
	RCC_APB2ENR |= RCC_APB2_AFIO | RCC_APB2_GPIOA | RCC_APB2_GPIOB;

	AFIO_EXTICR1 = (AFIO_EXTICR1 & ~(0xFu << 4u)) | (1u << 4u);
	EXTI_IMR |= 1u << 1u;
	EXTI_RTSR |= 1u << 1u;
	EXTI_FTSR &= ~(1u << 1u);
	EXTI_PR = 1u << 1u;

	GPIOA_CRL = (GPIOA_CRL & ~0xFu) | GPIO_CRL_OUT_2MHZ_PP;
	GPIOB_CRL = (GPIOB_CRL & ~(0xFu << 4u)) | (GPIO_CRL_IN_FLOATING << 4u);
	GPIOA_BRR = 1u << 0u;
	GPIOB_BRR = 1u << 1u;
}

static void test_rcc(void)
{
	put_str("\r\n--- RCC Clock and Reset Test ---\r\n");

	if (!expect_eq("RCC-001.CR", RCC_CR, 0x00000083u)) return;
	if (!expect_eq("RCC-001.CFGR", RCC_CFGR, 0u)) return;
	if (!expect_eq("RCC-001.AHBENR", RCC_AHBENR, 0x00000014u)) return;
	if (!expect_eq("RCC-001.APB1ENR", RCC_APB1ENR, 0u)) return;
	if (!expect_eq("RCC-001.CSR", RCC_CSR, 0x0C000000u)) return;
	pass_test("RCC-001: RM0008 reset values");

	RCC_CR = 0x00000083u | RCC_CR_HSEON | RCC_CR_PLLON;
	if (!expect_mask("RCC-002.CR", RCC_CR,
	                 RCC_CR_HSION | RCC_CR_HSIRDY | RCC_CR_HSEON | RCC_CR_HSERDY |
	                     RCC_CR_PLLON | RCC_CR_PLLRDY,
	                 RCC_CR_HSION | RCC_CR_HSIRDY | RCC_CR_HSEON | RCC_CR_HSERDY |
	                     RCC_CR_PLLON | RCC_CR_PLLRDY))
		return;
	RCC_CFGR = 1u;
	if (!expect_eq("RCC-002.CFGR", RCC_CFGR & 0xFu, 0x5u)) return;
	pass_test("RCC-002: oscillator ready and system-clock status");

	RCC_APB1ENR = 0xFFFFFFFFu;
	RCC_APB2ENR = 0xFFFFFFFFu;
	if (!expect_eq("RCC-003.APB1", RCC_APB1ENR, 0x3AFEC9FFu)) return;
	if (!expect_eq("RCC-003.APB2", RCC_APB2ENR, 0x0038FFFDu)) return;
	RCC_CSR = 1u << 24;
	if (!expect_eq("RCC-003.CSR_FLAGS", RCC_CSR & 0xFC000000u, 0u)) return;
	pass_test("RCC-003: writable masks and reset-flag clear");

	RCC_APB1ENR = 0u;
	RCC_APB2ENR = 0u;
	RCC_CFGR = 0u;
	RCC_CR = 0x00000083u;
}

static void test_gpio_exti(void)
{
	put_str("\r\n--- GPIO and EXTI Loopback Test ---\r\n");
	I2cMonitor mon;

	gpio_exti_init();
	i2c_clear_log();
	monitor_begin(&mon);
	GPIOA_BSRR = 1u << 0u;
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("GPIO-001", "EXTI timeout");
		return;
	}
	if (!expect_eq("GPIO-001.IRQ", g_log[mon.from].irq_id, IRQ_EXTI1)) return;
	if (!expect_eq("GPIO-001.PR", g_log[mon.from].sr1 & (1u << 1u), 1u << 1u)) return;
	if (!expect_eq("GPIO-001.CLEAR", EXTI_PR & (1u << 1u), 0u)) return;
	if (!expect_eq("GPIO-001.IDR", GPIOB_IDR & (1u << 1u), 1u << 1u)) return;
	pass_test("GPIO-001: A0 drives B1 and raises EXTI1");

	EXTI_FTSR |= 1u << 1u;
	monitor_begin(&mon);
	GPIOA_BRR = 1u << 0u;
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("GPIO-002", "falling edge EXTI timeout");
		return;
	}
	if (!expect_eq("GPIO-002.IRQ", g_log[mon.from].irq_id, IRQ_EXTI1)) return;
	if (!expect_eq("GPIO-002.IDR", GPIOB_IDR & (1u << 1u), 0u)) return;
	pass_test("GPIO-002: falling edge propagates through loopback");
}

static void usart_init(void)
{
	RCC_APB2ENR |= RCC_APB2_USART1;
	RCC_APB1ENR |= RCC_APB1_USART2;

	USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_TCIE |
	             USART_CR1_TXEIE;
	USART1_BRR = 0x1A1u;
	USART1_CR2 = 0u;
	USART1_CR3 = 0u;
	USART1_GTPR = 0u;

	USART2_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_TCIE |
	             USART_CR1_TXEIE;
	USART2_BRR = 0x1A1u;
	USART2_CR2 = 0u;
	USART2_CR3 = 0u;
	USART2_GTPR = 0u;
}

static void spi_init(void)
{
	RCC_APB2ENR |= RCC_APB2_SPI1;
	RCC_APB1ENR |= RCC_APB1_SPI2;

	SPI1_CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
	SPI1_CR2 = SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
	SPI1_CRCPR = 7u;
	SPI1_RXCRCR = 0u;
	SPI1_TXCRCR = 0u;
	SPI1_I2SCFGR = 0u;
	SPI1_I2SPR = 0u;

	SPI2_CR1 = SPI_CR1_SSM | SPI_CR1_SSI;
	SPI2_CR2 = SPI_CR2_RXNEIE | SPI_CR2_TXEIE;
	SPI2_CRCPR = 7u;
	SPI2_RXCRCR = 0u;
	SPI2_TXCRCR = 0u;
	SPI2_I2SCFGR = 0u;
	SPI2_I2SPR = 0u;
}

static uint32_t crc_reference_update(uint32_t crc, uint8_t byte)
{
	crc ^= (uint32_t)byte << 24u;
	for (unsigned i = 0u; i < 8u; ++i) {
		if ((crc & 0x80000000u) != 0u) {
			crc = (crc << 1u) ^ 0x04C11DB7u;
		} else {
			crc <<= 1u;
		}
	}
	return crc;
}

static uint32_t crc_reference_word(uint32_t crc, uint32_t word)
{
	const uint8_t *bytes = (const uint8_t *)&word;
	for (unsigned i = 0u; i < 4u; ++i) {
		crc = crc_reference_update(crc, bytes[i]);
	}
	return crc;
}

static void test_spi(void)
{
	put_str("\r\n--- SPI Register and Loopback Test ---\r\n");
	unsigned int from;

	spi_init();
	if (!expect_eq("SPI-001.SR", SPI1_SR, SPI_SR_TXE)) return;
	if (!expect_eq("SPI-001.CR1", SPI1_CR1, SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE)) return;
	pass_test("SPI-001: initialized register defaults");

	SPI1_CR1 = 0xFFFFFFFFu;
	SPI1_CR2 = 0xFFFFFFFFu;
	SPI1_CRCPR = 0xFFFFFFFFu;
	SPI1_I2SCFGR = 0xFFFFFFFFu;
	SPI1_I2SPR = 0xFFFFFFFFu;
	if (!expect_eq("SPI-002.CR1", SPI1_CR1, SPI_CR1_RW_MASK)) return;
	if (!expect_eq("SPI-002.CR2", SPI1_CR2, SPI_CR2_RW_MASK)) return;
	if (!expect_eq("SPI-002.CRCPR", SPI1_CRCPR, 0xFFFFu)) return;
	if (!expect_eq("SPI-002.I2SCFGR", SPI1_I2SCFGR, 0x0FFFu)) return;
	if (!expect_eq("SPI-002.I2SPR", SPI1_I2SPR, 0x03FFu)) return;
	pass_test("SPI-002: writable masks");

	RCC_APB2ENR &= ~RCC_APB2_SPI1;
	if (!expect_eq("SPI-003.GATED", SPI1_SR, 0u)) return;
	SPI1_CR1 = 0u;
	RCC_APB2ENR |= RCC_APB2_SPI1;
	if (!expect_eq("SPI-003.RETAIN", SPI1_CR1, SPI_CR1_RW_MASK)) return;
	RCC_APB2RSTR = RCC_APB2_SPI1;
	RCC_APB2RSTR = 0u;
	if (!expect_eq("SPI-003.RESET", SPI1_CR1, 0u)) return;
	if (!expect_eq("SPI-003.RESET_SR", SPI1_SR, SPI_SR_TXE)) return;
	pass_test("SPI-003: APB gate and reset");

	spi_init();
	i2c_clear_log();
	from = g_log_count;
	SPI2_DR = 0x5Au;
	SPI2_CR1 |= SPI_CR1_SPE;
	SPI1_DR = 0xA5u;
	if (!i2c_wait_n(from + 2u)) {
		fail_test("SPI-004", "loopback timeout");
		return;
	}
	int tx_idx = i2c_find(from, SPI_IRQ1, SPI_SR_RXNE | SPI_SR_TXE);
	int rx_idx = i2c_find(from, SPI_IRQ2, SPI_SR_RXNE);
	if (!expect_true("SPI-004", tx_idx >= 0, "master interrupt missing")) return;
	if (!expect_true("SPI-004", rx_idx >= 0, "slave interrupt missing")) return;
	if (!expect_eq("SPI-004.MASTER_RX", g_log[tx_idx].sr2 & 0xFFu, 0x5Au)) return;
	if (!expect_eq("SPI-004.SLAVE_RX", g_log[rx_idx].sr2 & 0xFFu, 0xA5u)) return;
	pass_test("SPI-004: byte loopback with interrupts");
}

static void test_usart(void)
{
	put_str("\r\n--- USART Register and Loopback Test ---\r\n");
	unsigned int from;

	usart_init();
	if (!expect_eq("USART-001.SR", USART1_SR, USART_SR_TXE | USART_SR_TC)) return;
	if (!expect_eq("USART-001.CR1", USART1_CR1,
	               USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_TCIE |
	                   USART_CR1_TXEIE))
		return;
	if (!expect_eq("USART-001.BRR", USART1_BRR, 0x1A1u)) return;
	pass_test("USART-001: initialized register defaults");

	USART1_CR1 = 0xFFFFFFFFu;
	USART1_CR2 = 0xFFFFFFFFu;
	USART1_CR3 = 0xFFFFFFFFu;
	USART1_GTPR = 0xFFFFFFFFu;
	if (!expect_eq("USART-002.CR1", USART1_CR1, USART_CR1_RW_MASK)) return;
	if (!expect_eq("USART-002.CR2", USART1_CR2, USART_CR2_RW_MASK)) return;
	if (!expect_eq("USART-002.CR3", USART1_CR3, USART_CR3_RW_MASK)) return;
	if (!expect_eq("USART-002.GTPR", USART1_GTPR, USART_GTPR_RW_MASK)) return;
	pass_test("USART-002: writable masks");

	RCC_APB2ENR &= ~RCC_APB2_USART1;
	if (!expect_eq("USART-003.GATED", USART1_SR, 0u)) return;
	USART1_CR1 = 0u;
	RCC_APB2ENR |= RCC_APB2_USART1;
	if (!expect_eq("USART-003.RETAIN", USART1_CR1, USART_CR1_RW_MASK)) return;
	RCC_APB2RSTR = RCC_APB2_USART1;
	RCC_APB2RSTR = 0u;
	if (!expect_eq("USART-003.RESET", USART1_CR1, 0u)) return;
	if (!expect_eq("USART-003.RESET_SR", USART1_SR, USART_SR_TXE | USART_SR_TC)) return;
	pass_test("USART-003: APB gate and reset");

	usart_init();
	i2c_clear_log();
	from = g_log_count;
	USART1_DR = 0xA5u;
	if (!i2c_wait_n(from + 2u)) {
		fail_test("USART-004", "loopback timeout");
		return;
	}
	int tx_idx = i2c_find(from, IRQ_USART1, USART_SR_TXE | USART_SR_TC);
	int rx_idx = i2c_find(from, IRQ_USART2, USART_SR_RXNE);
	if (!expect_true("USART-004", tx_idx >= 0, "sender interrupt missing")) return;
	if (!expect_true("USART-004", rx_idx >= 0, "receiver interrupt missing")) return;
	if (!expect_eq("USART-004.DATA", g_log[rx_idx].sr2 & 0xFFu, 0xA5u)) return;
	if (!expect_eq("USART-004.SR", USART2_SR & USART_SR_RXNE, 0u)) return;
	pass_test("USART-004: byte loopback with interrupts");
}

static void test_dma_crc(void)
{
	put_str("\r\n--- DMA and CRC Test ---\r\n");
	I2cMonitor mon;
	(void)mon;
	uintptr_t dma_src = (uintptr_t)&g_dma_src_words[0];
	uintptr_t dma_dst = (uintptr_t)&g_dma_dst_words[0];

	RCC_AHBENR |= RCC_AHB_DMA1 | RCC_AHB_CRC;
	i2c_clear_log();
	DMA1_IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;
	DMA1_CPAR1 = (uint32_t)dma_src;
	DMA1_CMAR1 = (uint32_t)dma_dst;
	DMA1_CNDTR1 = 2u;
	g_dma_src_words[0] = 0x11223344u;
	g_dma_src_words[1] = 0x55667788u;
	g_dma_dst_words[0] = 0u;
	g_dma_dst_words[1] = 0u;

	DMA1_CCR1 = DMA_CCR_MEM2MEM | DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_MSIZE_32 |
	            DMA_CCR_PSIZE_32 | DMA_CCR_TCIE | DMA_CCR_EN;
	if (!i2c_wait_n(1u)) {
		fail_test("DMA-001", "DMA completion interrupt timeout");
		return;
	}
	if (!expect_eq("DMA-001.IRQ", g_log[0].irq_id, IRQ_DMA1_CH1)) return;
	if (!expect_true("DMA-001", (g_log[0].sr1 & DMA_ISR_TCIF1) != 0u, "TCIF missing in interrupt log")) return;
	if (!expect_eq("DMA-001.DST0", g_dma_dst_words[0], 0x11223344u)) return;
	if (!expect_eq("DMA-001.DST1", g_dma_dst_words[1], 0x55667788u)) return;
	pass_test("DMA-001: DMA1 channel 1 mem2mem copy and interrupt");

	CRC_CR = 1u;
	if (!expect_eq("CRC-001.RESET", CRC_DR, 0xFFFFFFFFu)) return;
	CRC_IDR = 0xA5u;
	if (!expect_eq("CRC-001.IDR", CRC_IDR & 0xFFu, 0xA5u)) return;
	uint32_t expected = 0xFFFFFFFFu;
	expected = crc_reference_word(expected, 0x01020304u);
	expected = crc_reference_word(expected, 0xA0B0C0D0u);
	CRC_DR = 0x01020304u;
	CRC_DR = 0xA0B0C0D0u;
	if (!expect_eq("CRC-001.VALUE", CRC_DR, expected)) return;
	pass_test("CRC-001: reset, IDR, and CRC accumulation");
}

static void test_backup_domain(void)
{
	put_str("\r\n--- Backup Domain and RTC Test ---\r\n");

	if (!expect_eq("BD-001.PWR", PWR_CR, 0u)) return;
	if (!expect_eq("BD-001.BKP", BKP_DR1, 0u)) return;
	if (!expect_eq("BD-001.RTC", RTC_CRL, 0u)) return;
	pass_test("BD-001: reset values");

	RCC_APB1ENR |= RCC_APB1_BKP | RCC_APB1_PWR;
	BKP_DR1 = 0x11223344u;
	BKP_DR10 = 0x55667788u;
	BKP_DR11 = 0xAABBCCDDu;
	BKP_DR42 = 0x10203040u;
	if (!expect_eq("BD-002.GATED", BKP_DR1, 0u)) return;
	if (!expect_eq("BD-002.GATED10", BKP_DR10, 0u)) return;
	if (!expect_eq("BD-002.GATED11", BKP_DR11, 0u)) return;
	pass_test("BD-002: backup registers ignore writes while DBP is clear");

	PWR_CR |= PWR_CR_DBP;
	BKP_DR1 = 0x11223344u;
	BKP_DR10 = 0x55667788u;
	BKP_DR11 = 0xAABBCCDDu;
	BKP_DR42 = 0x10203040u;
	BKP_RTCCR = 0x0000005Au;
	BKP_CR = 0x00000003u;
	BKP_CSR = 0x00000007u;
	if (!expect_eq("BD-003.DR1", BKP_DR1, 0x11223344u)) return;
	if (!expect_eq("BD-003.DR10", BKP_DR10, 0x55667788u)) return;
	if (!expect_eq("BD-003.DR11", BKP_DR11, 0xAABBCCDDu)) return;
	if (!expect_eq("BD-003.DR42", BKP_DR42, 0x10203040u)) return;
	if (!expect_eq("BD-003.RTCCR", BKP_RTCCR, 0x0000005Au)) return;
	if (!expect_eq("BD-003.CR", BKP_CR, 0x00000003u)) return;
	if (!expect_eq("BD-003.CSR", BKP_CSR, 0x00000007u)) return;
	pass_test("BD-003: backup registers are writable when DBP is set");

	PWR_CR &= ~PWR_CR_DBP;
	BKP_DR1 = 0xDEADBEEFu;
	if (!expect_eq("BD-004.IGNORE", BKP_DR1, 0x11223344u)) return;
	if (!expect_eq("BD-004.IGNORE42", BKP_DR42, 0x10203040u)) return;
	pass_test("BD-004: writes are blocked again after DBP clear");

	PWR_CR |= PWR_CR_DBP;
	RTC_CRL = 0u;
	RTC_CRH = RTC_CRH_ALRIE;
	RTC_PRLH = 0u;
	RTC_PRLL = 0u;
	RTC_DIVH = 0u;
	RTC_DIVL = 0u;
	RTC_CNTH = 0u;
	RTC_CNTL = 0u;
	RTC_ALRH = 0u;
	RTC_ALRL = 3u;
	if (!expect_eq("BD-005.ALRL", RTC_ALRL, 3u)) return;
	if (!expect_eq("BD-005.CRH", RTC_CRH, RTC_CRH_ALRIE)) return;
	RCC_BDCR = RCC_BDCR_RTCEN;
	unsigned int cnt0 = RTC_CNTL;
	unsigned int cnt1 = RTC_CNTL;
	if (!expect_true("BD-005.TICK", cnt1 != cnt0, "RTC counter did not advance")) return;
	for (unsigned int i = 0u; i < 8u; ++i) {
		(void)RTC_CNTL;
	}
	if (!expect_true("BD-005.FLAG", (RTC_CRL & RTC_CRL_ALRF) != 0u, "alarm flag missing")) return;
	RTC_CRL = RTC_CRL_ALRF | RTC_CRL_SECF;
	if (!expect_eq("BD-005.CLEAR", RTC_CRL & (RTC_CRL_SECF | RTC_CRL_ALRF), 0u)) return;
	pass_test("BD-005: RTC alarm flag and counter progression");

	RCC_BDCR = RCC_BDCR_BDRST;
	RCC_BDCR = RCC_BDCR_RTCEN;
	if (!expect_eq("BD-006.DR1", BKP_DR1, 0u)) return;
	if (!expect_eq("BD-006.DR42", BKP_DR42, 0u)) return;
	if (!expect_eq("BD-006.RTCCR", BKP_RTCCR, 0u)) return;
	if (!expect_eq("BD-006.RTC_ALRL", RTC_ALRL, 0u)) return;
	pass_test("BD-006: backup reset clears the backup domain");
}

static void test_timers(void)
{
	put_str("\r\n--- TIM1/TIM2 Update Event Test ---\r\n");

	RCC_APB2ENR |= RCC_APB2_TIM1;
	RCC_APB1ENR |= RCC_APB1_TIM2 | RCC_APB1_TIM3 | RCC_APB1_TIM4 | RCC_APB1_TIM5;
	i2c_clear_log();

	TIM1_CR1 = 0u;
	TIM1_DIER = 0u;
	TIM1_SR = TIM_SR_UIF;
	TIM1_PSC = 0u;
	TIM1_ARR = 2u;
	TIM1_SR = TIM_SR_UIF;
	TIM1_CR1 = TIM_CR1_CEN;

	TIM2_CR1 = 0u;
	TIM2_DIER = 0u;
	TIM2_SR = TIM_SR_UIF;
	TIM2_PSC = 0u;
	TIM2_ARR = 3u;
	TIM2_SR = TIM_SR_UIF;
	TIM2_CR1 = TIM_CR1_CEN;

	if (!expect_eq("TIM-001.TIM1_CR1", TIM1_CR1, TIM_CR1_CEN)) return;
	if (!expect_eq("TIM-001.TIM1_ARR", TIM1_ARR, 2u)) return;
	if (!expect_eq("TIM-001.TIM2_CR1", TIM2_CR1, TIM_CR1_CEN)) return;
	if (!expect_eq("TIM-001.TIM2_ARR", TIM2_ARR, 3u)) return;
	for (unsigned int i = 0u; i < 16u; ++i) {
		(void)TIM1_CNT;
		(void)TIM2_CNT;
	}
	unsigned int tim1_cnt = TIM1_CNT;
	unsigned int tim2_cnt = TIM2_CNT;
	if (!expect_true("TIM-001", tim1_cnt != 0u || tim2_cnt != 0u, "timers did not advance")) return;
	if (!expect_true("TIM-001", (TIM1_SR & TIM_SR_UIF) != 0u || (TIM2_SR & TIM_SR_UIF) != 0u,
	                 "update flags missing"))
		return;
	pass_test("TIM-001: TIM1 and TIM2 update-event progression");

	TIM1_CR1 = TIM_CR1_CEN;
	RCC_APB2ENR &= ~RCC_APB2_TIM1;
	if (!expect_eq("TIM-002.GATED", TIM1_CR1, 0u)) return;
	RCC_APB2ENR |= RCC_APB2_TIM1;
	if (!expect_eq("TIM-002.RETAIN", TIM1_CR1, TIM_CR1_CEN)) return;
	RCC_APB2RSTR = RCC_APB2_TIM1;
	RCC_APB2RSTR = 0u;
	if (!expect_eq("TIM-002.RESET_CR1", TIM1_CR1, 0u)) return;
	if (!expect_eq("TIM-002.RESET_CNT", TIM1_CNT, 0u)) return;
	pass_test("TIM-002: TIM1 clock gate and reset");
}

static void test_watchdogs(void)
{
	put_str("\r\n--- WWDG/IWDG Test ---\r\n");

	RCC_APB1ENR |= RCC_APB1_WWDG;
	WWDG_CR = 0xFFFFFFFFu;
	WWDG_CFR = 0xFFFFFFFFu;
	if (!expect_eq("WDG-001.CR", WWDG_CR, WWDG_CR_T_MASK | WWDG_CR_WDGA)) return;
	if (!expect_eq("WDG-001.CFR", WWDG_CFR, WWDG_CFR_W_MASK | WWDG_CFR_WDGTB_MASK | WWDG_CFR_EWI))
		return;
	pass_test("WDG-001: WWDG masks");

	RCC_APB1ENR &= ~RCC_APB1_WWDG;
	if (!expect_eq("WDG-001.GATED", WWDG_CR, 0u)) return;
	WWDG_CR = 0u;
	RCC_APB1ENR |= RCC_APB1_WWDG;
	if (!expect_eq("WDG-001.RETAIN", WWDG_CR, WWDG_CR_T_MASK | WWDG_CR_WDGA)) return;
	RCC_APB1RSTR = RCC_APB1_WWDG;
	RCC_APB1RSTR = 0u;
	if (!expect_eq("WDG-001.RESET", WWDG_CR, WWDG_CR_T_MASK)) return;
	pass_test("WDG-001: WWDG gate and reset");

	WWDG_CR = WWDG_CR_WDGA | 0x41u;
	WWDG_CFR = WWDG_CFR_EWI | 0x20u;
	i2c_clear_log();
	for (unsigned int i = 0u; i < 4u; ++i) {
		(void)WWDG_CR;
		(void)WWDG_SR;
	}
	if (!expect_true("WDG-002", (WWDG_SR & WWDG_SR_EWIF) != 0u, "WWDG early wakeup flag missing"))
		return;
	pass_test("WDG-002: WWDG counter progression and early wakeup flag");

	IWDG_KR = IWDG_KR_UNLOCK;
	IWDG_PR = 0xFFFFFFFFu;
	IWDG_RLR = 0xFFFFFFFFu;
	if (!expect_eq("WDG-003.PR", IWDG_PR, IWDG_PR_MASK)) return;
	if (!expect_eq("WDG-003.RLR", IWDG_RLR, IWDG_RLR_MASK)) return;
	IWDG_KR = IWDG_KR_RELOAD;
	IWDG_KR = IWDG_KR_START;
	if (!expect_eq("WDG-003.KR", IWDG_KR, IWDG_KR_START)) return;
	pass_test("WDG-003: IWDG unlock and reload/start semantics");
}

static void test_flash(void)
{
	put_str("\r\n--- FLASH Interface Test ---\r\n");

	if (!expect_eq("FL-001.ACR", FLASH_ACR, 0x30u)) return;
	if (!expect_eq("FL-001.CR", FLASH_CR, FLASH_CR_LOCK)) return;
	pass_test("FL-001: reset values");

	FLASH_CR = FLASH_CR_PG;
	if (!expect_eq("FL-002.LOCKED", FLASH_CR, FLASH_CR_LOCK)) return;
	FLASH_KEYR = 0x45670123u;
	FLASH_KEYR = 0xCDEF89ABu;
	i2c_clear_log();
	FLASH_CR = FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_OBL_LAUNCH;
	if (!expect_eq("FL-002.CR", FLASH_CR, FLASH_CR_PG | FLASH_CR_PER | FLASH_CR_OBL_LAUNCH)) return;
	if (!i2c_wait_n(1u)) {
		fail_test("FL-002", "flash interrupt timeout");
		return;
	}
	if (!expect_eq("FL-002.IRQ", g_log[0].irq_id, IRQ_FLASH)) return;
	if (!expect_true("FL-002", (g_log[0].sr1 & FLASH_SR_EOP) != 0u, "flash EOP missing")) return;
	FLASH_ACR = 0xFFFFFFFFu;
	if (!expect_eq("FL-002.ACR", FLASH_ACR, 0x3Fu)) return;
	pass_test("FL-002: unlock and writable masks");

	i2c_clear_log();
	FLASH_SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
	if (!expect_eq("FL-003.SRCLR", FLASH_SR, 0u)) return;
	FLASH_CR = FLASH_CR_LOCK;
	if (!expect_eq("FL-003.LOCK", FLASH_CR, FLASH_CR_LOCK)) return;
	pass_test("FL-003: status clear and relock");
}

static void test_adc(void)
{
	put_str("\r\n--- ADC1/ADC2 Conversion Test ---\r\n");
	i2c_clear_log();

	if (!expect_eq("ADC-001.SR", ADC_SR, 0u)) return;
	if (!expect_eq("ADC-001.CR1", ADC_CR1, 0u)) return;
	if (!expect_eq("ADC-001.CR2", ADC_CR2, 0u)) return;
	pass_test("ADC-001: reset values");

	RCC_APB2ENR |= RCC_APB2_ADC1 | RCC_APB2_ADC2;
	ADC_CR1 = ADC_CR1_EOCIE;
	RCC_APB2ENR &= ~RCC_APB2_ADC1;
	if (!expect_eq("ADC-001.GATED", ADC_CR1, 0u)) return;
	ADC_CR1 = 0u;
	RCC_APB2ENR |= RCC_APB2_ADC1;
	if (!expect_eq("ADC-001.RETAIN", ADC_CR1, ADC_CR1_EOCIE)) return;
	RCC_APB2RSTR = RCC_APB2_ADC1;
	RCC_APB2RSTR = 0u;
	if (!expect_eq("ADC-001.RESET", ADC_CR1, 0u)) return;

	ADC_CR1 = ADC_CR1_EOCIE;
	ADC_SQR3 = 5u;
	ADC_CR2 = ADC_CR2_ADON;
	i2c_clear_log();
	ADC_CR2 = ADC_CR2_ADON | ADC_CR2_SWSTART;
	if (!i2c_wait_n(1u)) {
		fail_test("ADC-002", "ADC1 interrupt timeout");
		return;
	}
	if (!expect_eq("ADC-002.IRQ", g_log[0].irq_id, IRQ_ADC1_2)) return;
	if (!expect_true("ADC-002", (ADC_SR & ADC_SR_EOC) != 0u, "ADC1 EOC missing")) return;
	if (!expect_eq("ADC-002.DR", ADC_DR, 0x105u)) return;
	if (!expect_eq("ADC-002.CLEAR", ADC_SR & ADC_SR_EOC, 0u)) return;
	pass_test("ADC-002: ADC1 software conversion");

	ADC2_CR1 = ADC_CR1_EOCIE;
	ADC2_SQR3 = 9u;
	ADC2_CR2 = ADC_CR2_ADON;
	ADC2_CR2 = ADC_CR2_ADON | ADC_CR2_SWSTART;
	if (!i2c_wait_n(2u)) {
		fail_test("ADC-003", "ADC2 interrupt timeout");
		return;
	}
	if (!expect_eq("ADC-003.IRQ", g_log[1].irq_id, IRQ_ADC1_2)) return;
	if (!expect_true("ADC-003", (ADC2_SR & ADC_SR_EOC) != 0u, "ADC2 EOC missing")) return;
	if (!expect_eq("ADC-003.DR", ADC2_DR, 0x209u)) return;
	if (!expect_eq("ADC-003.CLEAR", ADC2_SR & ADC_SR_EOC, 0u)) return;
	pass_test("ADC-003: ADC2 software conversion");

	ADC_SR = ADC_SR_EOC;
	ADC2_SR = ADC_SR_EOC;
	if (!expect_eq("ADC-004.CLEAR1", ADC_SR & ADC_SR_EOC, 0u)) return;
	if (!expect_eq("ADC-004.CLEAR2", ADC2_SR & ADC_SR_EOC, 0u)) return;
	pass_test("ADC-004: EOC clear");
}

static void test_dac(void)
{
	put_str("\r\n--- DAC Output Test ---\r\n");

	if (!expect_eq("DAC-001.CR", DAC_CR, 0u)) return;
	if (!expect_eq("DAC-001.SWTRIGR", DAC_SWTRIGR, 0u)) return;
	if (!expect_eq("DAC-001.DOR1", DAC_DOR1, 0u)) return;
	if (!expect_eq("DAC-001.DOR2", DAC_DOR2, 0u)) return;
	pass_test("DAC-001: reset values");

	RCC_APB1ENR |= RCC_APB1_DAC;
	DAC_CR = 0xFFFFFFFFu;
	if (!expect_eq("DAC-002.CR", DAC_CR, DAC_CR_RW_MASK)) return;
	pass_test("DAC-002: writable mask");

	DAC_CR = DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_MASK | DAC_CR_EN2 | DAC_CR_TEN2 | DAC_CR_TSEL2_MASK;
	DAC_DHR12R1 = 0x1234u;
	DAC_DHR12R2 = 0x0ABCu;
	DAC_DHR8R1 = 0x5Au;
	DAC_DHR8R2 = 0xC3u;
	DAC_DHR12RD = 0x0FED0CBAu;
	DAC_DHR12LD = 0x89AB7654u;
	DAC_DHR8RD = 0x11223344u;
	DAC_SWTRIGR = DAC_SWTRIGR_SWTRIG1 | DAC_SWTRIGR_SWTRIG2;
	if (!expect_eq("DAC-003.DOR1", DAC_DOR1, 0x234u)) return;
	if (!expect_eq("DAC-003.DOR2", DAC_DOR2, 0xABCu)) return;
	if (!expect_eq("DAC-003.DHR12RD", DAC_DHR12RD, 0x0FED0CBAu)) return;
	if (!expect_eq("DAC-003.DHR12LD", DAC_DHR12LD, 0x89AB7654u)) return;
	if (!expect_eq("DAC-003.DHR8RD", DAC_DHR8RD, 0x11223344u)) return;
	pass_test("DAC-003: trigger updates dual outputs and preserves holding registers");

	RCC_APB1ENR &= ~RCC_APB1_DAC;
	if (!expect_eq("DAC-004.GATED", DAC_CR, 0u)) return;
	DAC_CR = DAC_CR_EN1;
	RCC_APB1ENR |= RCC_APB1_DAC;
	if (!expect_eq("DAC-004.RETAIN", DAC_CR,
	               DAC_CR_EN1 | DAC_CR_TEN1 | DAC_CR_TSEL1_MASK | DAC_CR_EN2 | DAC_CR_TEN2 |
	                   DAC_CR_TSEL2_MASK))
		return;
	RCC_APB1RSTR = RCC_APB1_DAC;
	RCC_APB1RSTR = 0u;
	if (!expect_eq("DAC-004.RESET", DAC_CR, 0u)) return;
	if (!expect_eq("DAC-004.RESET_DOR1", DAC_DOR1, 0u)) return;
	if (!expect_eq("DAC-004.RESET_DOR2", DAC_DOR2, 0u)) return;
	pass_test("DAC-004: APB gate and reset");
}

static void test_can(void)
{
	put_str("\r\n--- CAN1 Register and Loopback Test ---\r\n");
	unsigned int from;

	if (!expect_eq("CAN-001.MCR", CAN1_MCR, 0x00010002u)) return;
	if (!expect_eq("CAN-001.TSR", CAN1_TSR, 0x1C000000u)) return;
	if (!expect_eq("CAN-001.IER", CAN1_IER, 0u)) return;
	pass_test("CAN-001: reset values");

	CAN1_MCR = 0xFFFFFFFFu;
	if (!expect_eq("CAN-002.MCR", CAN1_MCR, CAN_MCR_RW_MASK)) return;
	pass_test("CAN-002: writable mask");

	RCC_APB1ENR |= RCC_APB1_CAN1;
	CAN1_IER = CAN_IER_TMEIE | CAN_IER_FMPIE0;
	i2c_clear_log();
	from = g_log_count;
	CAN1_TDT0R = 0x00000008u;
	CAN1_TDL0R = 0x11223344u;
	CAN1_TDH0R = 0x55667788u;
	CAN1_TI0R = 0x1ABCDE78u | CAN_TIR_TXRQ;
	if (!i2c_wait_n(from + 2u)) {
		fail_test("CAN-003", "interrupt timeout");
		return;
	}
	int tx_idx = i2c_find(from, IRQ_CAN1_TX, CAN_TSR_TXOK0);
	int rx_idx = i2c_find(from, IRQ_CAN1_RX0, 0x1u);
	if (!expect_true("CAN-003", tx_idx >= 0, "TX IRQ missing")) return;
	if (!expect_true("CAN-003", rx_idx >= 0, "RX0 IRQ missing")) return;
	if (!expect_true("CAN-003", (g_log[tx_idx].sr1 & CAN_TSR_TXOK0) != 0u, "TXOK missing")) return;
	if (!expect_eq("CAN-003.TX", g_log[tx_idx].sr2, 0x1ABCDE79u)) return;
	if (!expect_eq("CAN-003.RX", g_log[rx_idx].sr2, 0x1ABCDE79u)) return;
	if (!expect_eq("CAN-003.RI0R", CAN1_RI0R, 0x1ABCDE79u)) return;
	if (!expect_eq("CAN-003.RF0R", CAN1_RF0R & 0x3u, 0u)) return;
	if (!expect_eq("CAN-003.TSR", CAN1_TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0), 0u)) return;
	pass_test("CAN-003: TX request loopback and interrupt claims");

	RCC_APB1ENR &= ~RCC_APB1_CAN1;
	if (!expect_eq("CAN-004.GATED", CAN1_IER, 0u)) return;
	CAN1_IER = CAN_IER_TMEIE;
	RCC_APB1ENR |= RCC_APB1_CAN1;
	if (!expect_eq("CAN-004.RETAIN", CAN1_IER, CAN_IER_TMEIE | CAN_IER_FMPIE0)) return;
	RCC_APB1RSTR = RCC_APB1_CAN1;
	RCC_APB1RSTR = 0u;
	if (!expect_eq("CAN-004.RESET_MCR", CAN1_MCR, 0x00010002u)) return;
	if (!expect_eq("CAN-004.RESET_TSR", CAN1_TSR, 0x1C000000u)) return;
	if (!expect_eq("CAN-004.RESET_IER", CAN1_IER, 0u)) return;
	pass_test("CAN-004: APB gate and reset");
}

static void test_sdio(void)
{
	put_str("\r\n--- SDIO Register and FIFO Test ---\r\n");
	unsigned int from;

	if (!expect_eq("SDIO-001.POWER", SDIO_POWER, 0u)) return;
	if (!expect_eq("SDIO-001.CLKCR", SDIO_CLKCR, 0u)) return;
	if (!expect_eq("SDIO-001.STA", SDIO_STA, 0u)) return;
	pass_test("SDIO-001: reset values");

	RCC_AHBENR |= RCC_AHB_SDIO;
	SDIO_POWER = 0xFFFFFFFFu;
	SDIO_CLKCR = 0xFFFFFFFFu;
	if (!expect_eq("SDIO-002.POWER", SDIO_POWER, SDIO_POWER_PWRCTRL_MASK)) return;
	if (!expect_eq("SDIO-002.CLKCR", SDIO_CLKCR, 0x5FFFu)) return;
	pass_test("SDIO-002: writable mask");

	SDIO_MASK = SDIO_MASK_CMDRENDIE | SDIO_MASK_CMDSENTIE | SDIO_MASK_DATAENDIE;
	SDIO_ARG = 0x12345678u;
	i2c_clear_log();
	from = g_log_count;
	SDIO_CMD = SDIO_CMD_CPSMEN | SDIO_CMD_WAITRESP_MASK | 0x12u;
	if (!i2c_wait_n(from + 1u)) {
		fail_test("SDIO-003", "command interrupt timeout");
		return;
	}
	if (!expect_eq("SDIO-003.IRQ", g_log[0].irq_id, IRQ_SDIO)) return;
	if (!expect_eq("SDIO-003.RESPCMD", SDIO_RESPCMD, 0x12u)) return;
	if (!expect_eq("SDIO-003.RESP1", SDIO_RESP1, 0x1234576Au)) return;
	if (!expect_eq("SDIO-003.RESP2", SDIO_RESP2, ~0x12345678u)) return;
	if (!expect_true("SDIO-003", (SDIO_STA & SDIO_STA_CMDREND) != 0u, "CMDREND missing")) return;
	pass_test("SDIO-003: command completion and response registers");

	i2c_clear_log();
	SDIO_DCTRL = SDIO_DCTRL_DTEN;
	SDIO_FIFO = 0xA5A50001u;
	SDIO_FIFO = 0x5A5A0002u;
	if (!expect_eq("SDIO-004.COUNT", SDIO_FIFOCNT, 2u)) return;
	if (!expect_eq("SDIO-004.DCOUNT", SDIO_DCOUNT, 8u)) return;
	if (!expect_true("SDIO-004", (SDIO_STA & SDIO_STA_DATAEND) != 0u, "DATAEND missing")) return;
	if (!expect_eq("SDIO-004.READ0", SDIO_FIFO, 0xA5A50001u)) return;
	if (!expect_eq("SDIO-004.READ1", SDIO_FIFO, 0x5A5A0002u)) return;
	pass_test("SDIO-004: FIFO loopback and transfer accounting");

	SDIO_CLKCR = SDIO_CLKCR_CLKEN | 0x03u;
	RCC_AHBENR &= ~RCC_AHB_SDIO;
	if (!expect_eq("SDIO-005.GATED", SDIO_CLKCR, 0u)) return;
	SDIO_CLKCR = 0u;
	RCC_AHBENR |= RCC_AHB_SDIO;
	if (!expect_eq("SDIO-005.RETAIN", SDIO_CLKCR, SDIO_CLKCR_CLKEN | 0x03u)) return;
	RCC_AHBRSTR = RCC_AHB_SDIO;
	RCC_AHBRSTR = 0u;
	if (!expect_eq("SDIO-005.RESET_POWER", SDIO_POWER, 0u)) return;
	if (!expect_eq("SDIO-005.RESET_CLKCR", SDIO_CLKCR, 0u)) return;
	if (!expect_eq("SDIO-005.RESET_STA", SDIO_STA, 0u)) return;
	if (!expect_eq("SDIO-005.RESET_FIFO", SDIO_FIFOCNT, 0u)) return;
	pass_test("SDIO-005: AHB gate and reset");
}

static void test_fsmc(void)
{
	put_str("\r\n--- FSMC Register and External Bank Test ---\r\n");

	if (!expect_eq("FSMC-001.BCR1", FSMC_BCR1, 0u)) return;
	if (!expect_eq("FSMC-001.BTR1", FSMC_BTR1, 0u)) return;
	if (!expect_eq("FSMC-001.BCR2", FSMC_BCR2, 0u)) return;
	if (!expect_eq("FSMC-001.BTR2", FSMC_BTR2, 0u)) return;
	pass_test("FSMC-001: reset values");

	RCC_AHBENR |= RCC_AHB_FSMC;
	FSMC_BCR1 = 0xFFFFFFFFu;
	FSMC_BTR1 = 0xFFFFFFFFu;
	if (!expect_eq("FSMC-002.BCR1", FSMC_BCR1, FSMC_BCR_RW_MASK)) return;
	if (!expect_eq("FSMC-002.BTR1", FSMC_BTR1, FSMC_BTR_RW_MASK)) return;
	pass_test("FSMC-002: writable masks");

	FSMC_BCR1 = FSMC_BCR_MBKEN | FSMC_BCR_WREN;
	MMIO32(0x60000000UL) = 0x11223344u;
	MMIO32(0x60000004UL) = 0x55667788u;
	if (!expect_eq("FSMC-003.READ0", MMIO32(0x60000000UL), 0x11223344u)) return;
	if (!expect_eq("FSMC-003.READ1", MMIO32(0x60000004UL), 0x55667788u)) return;
	pass_test("FSMC-003: bank1 window loopback");

	RCC_AHBENR &= ~RCC_AHB_FSMC;
	if (!expect_eq("FSMC-004.GATED_BCR1", FSMC_BCR1, 0u)) return;
	if (!expect_eq("FSMC-004.GATED_BANK", MMIO32(0x60000000UL), 0u)) return;
	RCC_AHBENR |= RCC_AHB_FSMC;
	if (!expect_eq("FSMC-004.RETAIN_BCR1", FSMC_BCR1, FSMC_BCR_MBKEN | FSMC_BCR_WREN)) return;
	pass_test("FSMC-004: AHB gate preserves register state");

	RCC_AHBRSTR = RCC_AHB_FSMC;
	RCC_AHBRSTR = 0u;
	if (!expect_eq("FSMC-005.RESET_BCR1", FSMC_BCR1, 0u)) return;
	if (!expect_eq("FSMC-005.RESET_BTR1", FSMC_BTR1, 0u)) return;
	if (!expect_eq("FSMC-005.RESET_BANK", MMIO32(0x60000000UL), 0u)) return;
	pass_test("FSMC-005: AHB reset clears controller and disables bank");
}

static void test_usb_fs(void)
{
	put_str("\r\n--- USB Device FS Register Test ---\r\n");

	if (!expect_eq("USB-001.CNTR", USB_CNTR, 0u)) return;
	if (!expect_eq("USB-001.ISTR", USB_ISTR, 0u)) return;
	if (!expect_eq("USB-001.DADDR", USB_DADDR, 0u)) return;
	pass_test("USB-001: reset values");

	RCC_APB1ENR |= RCC_APB1_USB;
	USB_EP0R = 0xFFFFFFFFu;
	USB_BTABLE = 0xFFFFFFFFu;
	if (!expect_eq("USB-002.EP0R", USB_EP0R, 0xFFFFu)) return;
	if (!expect_eq("USB-002.BTABLE", USB_BTABLE, 0x1FFFu)) return;
	pass_test("USB-002: writable masks");

	i2c_clear_log();
	USB_CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM;
	if (!i2c_wait_n(1u)) {
		fail_test("USB-003", "interrupt timeout");
		return;
	}
	if (!expect_eq("USB-003.IRQ", g_log[0].irq_id, IRQ_USB)) return;
	if (!expect_true("USB-003", (USB_ISTR & (USB_ISTR_RESET | USB_ISTR_CTR)) != 0u, "status bits missing")) return;
	USB_ISTR = USB_ISTR_RESET | USB_ISTR_CTR;
	if (!expect_eq("USB-003.CLEAR", USB_ISTR & (USB_ISTR_RESET | USB_ISTR_CTR), 0u)) return;
	pass_test("USB-003: control-triggered interrupt and W0C status");

	RCC_APB1RSTR = RCC_APB1_USB;
	RCC_APB1RSTR = 0u;
	if (!expect_eq("USB-004.RESET", USB_CNTR, 0u)) return;
	pass_test("USB-004: APB reset");
}

static void test_otg_fs(void)
{
	put_str("\r\n--- OTG FS Register Test ---\r\n");

	if (!expect_eq("OTG-001.GAHBCFG", OTG_GAHBCFG, 0u)) return;
	if (!expect_eq("OTG-001.GINTSTS", OTG_GINTSTS, 0u)) return;
	pass_test("OTG-001: reset values");

	RCC_AHBENR |= RCC_AHB_OTGFS;
	OTG_GAHBCFG = OTG_GAHBCFG_GINT;
	OTG_GINTMSK = OTG_GINTSTS_USBRST | OTG_GINTSTS_ENUMDNE;
	i2c_clear_log();
	OTG_GRSTCTL = OTG_GRSTCTL_CSRST;
	if (!i2c_wait_n(1u)) {
		fail_test("OTG-002", "reset interrupt timeout");
		return;
	}
	if (!expect_eq("OTG-002.IRQ", g_log[0].irq_id, IRQ_USB)) return;
	if (!expect_true("OTG-002", (OTG_GINTSTS & OTG_GINTSTS_USBRST) != 0u, "USBRST missing")) return;
	OTG_GINTSTS = OTG_GINTSTS_USBRST;
	if (!expect_eq("OTG-002.CLEAR", OTG_GINTSTS & OTG_GINTSTS_USBRST, 0u)) return;
	pass_test("OTG-002: core reset event");

	i2c_clear_log();
	OTG_GUSBCFG = OTG_GUSBCFG_FDMOD;
	if (!i2c_wait_n(1u)) {
		fail_test("OTG-003", "enum interrupt timeout");
		return;
	}
	if (!expect_eq("OTG-003.IRQ", g_log[0].irq_id, IRQ_USB)) return;
	if (!expect_true("OTG-003", (OTG_GINTSTS & OTG_GINTSTS_ENUMDNE) != 0u, "ENUMDNE missing")) return;
	OTG_GINTSTS = OTG_GINTSTS_ENUMDNE;
	if (!expect_eq("OTG-003.CLEAR", OTG_GINTSTS & OTG_GINTSTS_ENUMDNE, 0u)) return;
	pass_test("OTG-003: force-device-mode event");
}

static void test_eth(void)
{
	put_str("\r\n--- Ethernet Mailbox and DMA Test ---\r\n");

	if (!expect_eq("ETH-001.STATUS", ETH_STATUS, 0u)) return;
	if (!expect_eq("ETH-001.MACCR", ETH_MACCR, 0u)) return;
	pass_test("ETH-001: reset values");

	RCC_AHBENR |= RCC_AHB_ETHMAC | RCC_AHB_ETHMACTX | RCC_AHB_ETHMACRX;
	ETH_DMAIER = ETH_DMAIER_TIE | ETH_DMAIER_RIE | ETH_DMAIER_NISE;
	ETH_MAC_HIGH = 0xAABBCCDDu;
	ETH_MAC_LOW = 0x11223344u;
	if (!expect_eq("ETH-002.MAC_HIGH", ETH_MAC_HIGH, 0xAABBCCDDu)) return;
	if (!expect_eq("ETH-002.MAC_LOW", ETH_MAC_LOW, 0x11223344u)) return;
	pass_test("ETH-002: writable address registers");

	for (unsigned i = 0u; i < 4u; ++i) {
		g_eth_src_words[i] = 0x11110000u + i;
		g_eth_dst_words[i] = 0u;
	}
	ETH_SEND_SRC = (uint32_t)(uintptr_t)&g_eth_src_words[0];
	ETH_RECV_DST = (uint32_t)(uintptr_t)&g_eth_dst_words[0];
	ETH_SEND_SIZE = sizeof(g_eth_src_words);
	i2c_clear_log();
	ETH_STATUS = ETH_STATUS_SEND;
	if (!i2c_wait_n(1u)) {
		fail_test("ETH-003", "send interrupt timeout");
		return;
	}
	if (!expect_eq("ETH-003.IRQ", g_log[0].irq_id, IRQ_ETH)) return;
	if (!expect_true("ETH-003", (ETH_DMASR & ETH_DMASR_TI) != 0u, "TX flag missing")) return;
	ETH_STATUS = ETH_STATUS_RECV;
	if (!i2c_wait_n(2u)) {
		fail_test("ETH-003", "receive interrupt timeout");
		return;
	}
	if (!expect_eq("ETH-003.IRQ2", g_log[1].irq_id, IRQ_ETH)) return;
	for (unsigned i = 0u; i < 4u; ++i) {
		if (!expect_eq("ETH-003.LOOP", g_eth_dst_words[i], 0x11110000u + i)) return;
	}
	ETH_DMASR = ETH_DMASR_TI | ETH_DMASR_RI | ETH_DMASR_NIS;
	if (!expect_eq("ETH-003.CLEAR", ETH_DMASR & (ETH_DMASR_TI | ETH_DMASR_RI | ETH_DMASR_NIS), 0u)) return;
	pass_test("ETH-003: send/receive loopback and DMA status");

	ETH_DMABMR = ETH_DMABMR_SR;
	if (!expect_eq("ETH-004.RESET_MACCR", ETH_MACCR, 0u)) return;
	if (!expect_eq("ETH-004.RESET_DMASR", ETH_DMASR, 0u)) return;
	pass_test("ETH-004: software reset");
}

static void i2c_stop(void)
{
	I2C0_CR1 |= CR1_STOP;
}

static void i2c_start(void)
{
	I2C0_CR1 |= CR1_START;
}

static void i2c_write_addr(unsigned int addr, unsigned int read)
{
	I2C0_DR = ((addr & 0x7Fu) << 1u) | (read ? 1u : 0u);
}

static void i2c_write_byte(unsigned int data)
{
	I2C0_DR = data;
}

static unsigned int i2c_read_byte(void)
{
	return I2C0_DR;
}

static int i2c_rx_byte(unsigned int *data)
{
	if (!i2c_wait_sr1(SR1_RxNE)) {
		return 0;
	}
	*data = I2C0_DR & 0xFFu;
	return 1;
}

static void i2c_recover(void)
{
	I2C0_CR1 |= CR1_STOP;
	i2c_init();
}

static void i2c1_config_7bit(unsigned int addr, unsigned int cr1_extra)
{
	I2C1_CR1 = CR1_SWRST;
	I2C1_CR1 = 0u;
	I2C1_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C1_OAR1 = addr & 0x7Fu;
	I2C1_OAR2 = 0u;
	I2C1_CCR = 4000u;
	I2C1_TRISE = 37u;
	I2C1_CR1 = CR1_PE | cr1_extra;
}

static void i2c1_config_10bit(unsigned int addr10)
{
	I2C1_CR1 = CR1_SWRST;
	I2C1_CR1 = 0u;
	I2C1_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C1_OAR1 = OAR1_ADDMODE | (addr10 & 0x03FFu);
	I2C1_OAR2 = 0u;
	I2C1_CCR = 4000u;
	I2C1_TRISE = 37u;
	I2C1_CR1 = CR1_PE | CR1_ACK;
}

static void i2c_write_addr10_header(unsigned int addr10, unsigned int read)
{
	I2C0_DR = 0xF0u | ((addr10 >> 7u) & 0x06u) | (read ? 1u : 0u);
}

static int i2c0_addr7_write(unsigned int addr, unsigned int data)
{
	unsigned int base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_n(base + 2u)) return 0;
	i2c_write_byte(data);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) return 0;
	if (!i2c1_wait_sr1(SR1_RxNE)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_addr7_read(unsigned int addr, unsigned int source, unsigned int *data)
{
	unsigned int base = g_log_count;
	I2C1_DR = source;
	I2C0_CR1 |= CR1_ACK;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_n(base + 2u)) return 0;
	if (!i2c_rx_byte(data)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_write_then_repeated_read(unsigned int addr, unsigned int write_data, unsigned int source, unsigned int *data)
{
	unsigned int base = g_log_count;
	I2C1_DR = source;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_n(base + 2u)) return 0;
	i2c_write_byte(write_data);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) return 0;
	if (!i2c1_wait_sr1(SR1_RxNE)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_n(base + 2u)) return 0;
	if (!i2c_rx_byte(data)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_read_then_repeated_write(unsigned int addr, unsigned int source, unsigned int write_data, unsigned int *data)
{
	unsigned int base = g_log_count;
	I2C1_DR = source;
	I2C0_CR1 |= CR1_ACK;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_n(base + 2u)) return 0;
	if (!i2c_rx_byte(data)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_n(base + 2u)) return 0;
	i2c_write_byte(write_data);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) return 0;
	if (!i2c1_wait_sr1(SR1_RxNE)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_read_two_bytes(unsigned int addr, unsigned int first_source, unsigned int second_source, unsigned int *first, unsigned int *second)
{
	unsigned int base = g_log_count;
	I2C1_DR = first_source;
	I2C0_CR1 |= CR1_ACK;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_n(base + 2u)) return 0;
	if (!i2c_rx_byte(first)) return 0;
	I2C1_DR = second_source;
	if (!i2c_rx_byte(second)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_addr10_write(unsigned int addr10, unsigned int data)
{
	unsigned int base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr10_header(addr10, 0u);
	if (!i2c_wait_sr1(SR1_ADD10)) return 0;
	i2c_write_byte(addr10 & 0xFFu);
	if (!i2c_wait_n(base + 2u)) return 0;
	i2c_write_byte(data);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) return 0;
	if (!i2c1_wait_sr1(SR1_RxNE)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_addr10_read(unsigned int addr10, unsigned int source, unsigned int *data)
{
	unsigned int base = g_log_count;
	I2C1_DR = source;
	I2C0_CR1 |= CR1_ACK;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr10_header(addr10, 0u);
	if (!i2c_wait_sr1(SR1_ADD10)) return 0;
	i2c_write_byte(addr10 & 0xFFu);
	if (!i2c_wait_n(base + 2u)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr10_header(addr10, 1u);
	if (!i2c_wait_n(base + 2u)) return 0;
	if (!i2c_rx_byte(data)) return 0;
	i2c_stop();
	return 1;
}

static int start_addr_write(unsigned int addr)
{
	unsigned int base = g_log_count;
	i2c_start();
	if (!i2c_wait_n(base + 1u)) return 0;
	i2c_write_addr(addr, 0u);
	return 1;
}

static void i2c_finish(void)
{
	if (g_fail == 0) {
		put_str("ALL TESTS PASSED\r\n");
	} else {
		put_str("SOME TESTS FAILED\r\n");
	}
	put_str("pass=");
	put_dec((unsigned int)g_pass);
	put_str(" fail=");
	put_dec((unsigned int)g_fail);
	put_str("\r\n");
	EXITER = 0u;
}

void trap_handler(void)
{
	uint32_t mcause;
	__asm__ volatile("csrr %0, mcause" : "=r"(mcause));
	if (mcause != 0x8000000Bu) {
		return;
	}

	uint32_t irq_id = PLIC_CLAIM_HART0;
	if (irq_id == 0u) {
		return;
	}

	if (irq_id == IRQ_I2C0_EV || irq_id == IRQ_I2C1_EV) {
		volatile uint32_t *sr1p = (irq_id == IRQ_I2C0_EV) ? &I2C0_SR1 : &I2C1_SR1;
		volatile uint32_t *sr2p = (irq_id == IRQ_I2C0_EV) ? &I2C0_SR2 : &I2C1_SR2;
		unsigned int sr1 = *sr1p;
		unsigned int sr2 = *sr2p;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr1;
			g_log[idx].sr2 = sr2;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_I2C0_ER || irq_id == IRQ_I2C1_ER) {
		volatile uint32_t *cr1p = (irq_id == IRQ_I2C0_ER) ? &I2C0_CR1 : &I2C1_CR1;
		volatile uint32_t *sr1p = (irq_id == IRQ_I2C0_ER) ? &I2C0_SR1 : &I2C1_SR1;
		volatile uint32_t *sr2p = (irq_id == IRQ_I2C0_ER) ? &I2C0_SR2 : &I2C1_SR2;
		unsigned int sr1 = *sr1p;
		unsigned int sr2 = *sr2p;
		*sr1p = 0u;
		*cr1p |= CR1_STOP;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr1;
			g_log[idx].sr2 = sr2;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_EXTI1) {
		unsigned int pr = EXTI_PR;
		EXTI_PR = pr;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = pr;
			g_log[idx].sr2 = 0u;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_DMA1_CH1) {
		unsigned int isr = DMA1_ISR;
		unsigned int ndtr = DMA1_CNDTR1;
		DMA1_IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = isr;
			g_log[idx].sr2 = ndtr;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == SPI_IRQ1 || irq_id == SPI_IRQ2) {
		volatile uint32_t *srp = (irq_id == SPI_IRQ1) ? &SPI1_SR : &SPI2_SR;
		volatile uint32_t *drp = (irq_id == SPI_IRQ1) ? &SPI1_DR : &SPI2_DR;
		unsigned int sr = *srp;
		unsigned int data = 0u;
		if ((sr & SPI_SR_RXNE) != 0u) {
			data = *drp;
		}
		if ((sr & SPI_SR_TXE) != 0u) {
			*srp &= ~SPI_SR_TXE;
		}
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr;
			g_log[idx].sr2 = data;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_USART1 || irq_id == IRQ_USART2) {
		volatile uint32_t *srp = (irq_id == IRQ_USART1) ? &USART1_SR : &USART2_SR;
		volatile uint32_t *drp = (irq_id == IRQ_USART1) ? &USART1_DR : &USART2_DR;
		unsigned int sr = *srp;
		unsigned int data = 0u;
		if ((sr & USART_SR_RXNE) != 0u) {
			data = *drp;
		}
		if ((sr & (USART_SR_TXE | USART_SR_TC)) != 0u) {
			*srp &= ~(USART_SR_TXE | USART_SR_TC);
		}
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr;
			g_log[idx].sr2 = data;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_RTC) {
		unsigned int crl = RTC_CRL;
		unsigned int cntl = RTC_CNTL;
		RTC_CRL = crl;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = crl;
			g_log[idx].sr2 = cntl;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_TIM1_UP || irq_id == IRQ_TIM2_UP) {
		volatile uint32_t *srp = (irq_id == IRQ_TIM1_UP) ? &TIM1_SR : &TIM2_SR;
		volatile uint32_t *cntp = (irq_id == IRQ_TIM1_UP) ? &TIM1_CNT : &TIM2_CNT;
		unsigned int sr = *srp;
		unsigned int cnt = *cntp;
		*srp = TIM_SR_UIF;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr;
			g_log[idx].sr2 = cnt;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_WWDG) {
		unsigned int sr = WWDG_SR;
		unsigned int cr = WWDG_CR;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr;
			g_log[idx].sr2 = cr;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_FLASH) {
		unsigned int sr = FLASH_SR;
		unsigned int cr = FLASH_CR;
		FLASH_SR = FLASH_SR_EOP;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr;
			g_log[idx].sr2 = cr;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_ADC1_2) {
		unsigned int sr1 = ADC_SR;
		unsigned int sr2 = ADC2_SR;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sr1;
			g_log[idx].sr2 = sr2;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_CAN1_TX) {
		unsigned int tsr = CAN1_TSR;
		unsigned int tir = CAN1_TI0R;
		CAN1_TSR = CAN_TSR_RQCP0 | CAN_TSR_TXOK0;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = tsr;
			g_log[idx].sr2 = tir;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_CAN1_RX0) {
		unsigned int rf0r = CAN1_RF0R;
		unsigned int rir = CAN1_RI0R;
		CAN1_RF0R = 0x21u;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = rf0r;
			g_log[idx].sr2 = rir;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_CAN1_RX1 || irq_id == IRQ_CAN1_SCE) {
		unsigned int esr = CAN1_ESR;
		unsigned int msr = CAN1_MSR;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = esr;
			g_log[idx].sr2 = msr;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_USB) {
		unsigned int status = USB_ISTR;
		unsigned int aux = USB_DADDR;
		USB_ISTR = status;
		unsigned int idx = g_log_count;
		if (status == 0u) {
			status = OTG_GINTSTS;
			aux = OTG_GINTMSK;
			OTG_GINTSTS = status;
		}
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = status;
			g_log[idx].sr2 = aux;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_ETH) {
		unsigned int dmasr = ETH_DMASR;
		unsigned int status = ETH_STATUS;
		ETH_DMASR = dmasr;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = dmasr;
			g_log[idx].sr2 = status;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	} else if (irq_id == IRQ_SDIO) {
		unsigned int sta = SDIO_STA;
		unsigned int count = SDIO_FIFOCNT;
		SDIO_ICR = sta;
		unsigned int idx = g_log_count;
		if (idx < LOG_SIZE) {
			g_log[idx].irq_id = irq_id;
			g_log[idx].sr1 = sta;
			g_log[idx].sr2 = count;
			__asm__ volatile("fence" ::: "memory");
			g_log_count = idx + 1u;
		}
	}

	PLIC_CLAIM_HART0 = irq_id;
}

static void test_register_model(void)
{
	put_str("\r\n--- I2C Register Model and Reset Test ---\r\n");
	I2cMonitor mon;

	if (!expect_eq("REG-001.CR1", I2C0_CR1, CR1_PE)) return;
	if (!expect_eq("REG-001.CR2", I2C0_CR2, 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN)) return;
	if (!expect_eq("REG-001.SR1", I2C0_SR1, 0u)) return;
	if (!expect_eq("REG-001.SR2", I2C0_SR2, 0u)) return;
	if (!expect_eq("REG-001.LOG", g_log_count, 0u)) return;
	pass_test("REG-001: initialized register defaults");

	I2C0_CR1 = CR1_RW_MASK | CR1_START | CR1_STOP | CR1_SWRST | (1u << 2) | (1u << 14);
	if (!expect_eq("REG-002.CR1", I2C0_CR1, 0u)) return;
	I2C0_CR1 = CR1_PE;
	I2C0_CR2 = 0xFFFFFFFFu;
	I2C0_OAR1 = 0xFFFFFFFFu;
	I2C0_OAR2 = 0xFFFFFFFFu;
	I2C0_CCR = 0xFFFFFFFFu;
	I2C0_TRISE = 0xFFFFFFFFu;
	if (!expect_eq("REG-002.CR2", I2C0_CR2, CR2_RW_MASK)) return;
	if (!expect_eq("REG-002.OAR1", I2C0_OAR1, OAR1_RW_MASK)) return;
	if (!expect_eq("REG-002.OAR2", I2C0_OAR2, OAR2_RW_MASK)) return;
	if (!expect_eq("REG-002.CCR", I2C0_CCR, 0xCFFFu)) return;
	if (!expect_eq("REG-002.TRISE", I2C0_TRISE, 0x3Fu)) return;
	pass_test("REG-002: writable and reserved-bit masks");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	I2C0_CR1 = 0u;
	i2c_start();
	I2C0_DR = 0x44u;
	if (!expect_eq("REG-003.SR1", I2C0_SR1, 0u)) return;
	if (!expect_eq("REG-003.SR2", I2C0_SR2, 0u)) return;
	if (!expect_eq("REG-003.LOG", g_log_count, 0u)) return;
	pass_test("REG-003: disabled peripheral ignores START and DR side effects");

	i2c_init();
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_sr1(SR1_SB)) {
		fail_test("REG-004", "SB missing before PE clear");
		return;
	}
	if (!expect_true("REG-004", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "SB IRQ missing before PE clear")) return;
	I2C0_CR1 = 0u;
	if (!expect_eq("REG-004.CR1", I2C0_CR1, 0u)) return;
	if (!expect_eq("REG-004.SR1", I2C0_SR1, 0u)) return;
	if (!expect_eq("REG-004.SR2", I2C0_SR2, 0u)) return;
	if (!expect_eq("REG-004.LOG", g_log_count, mon.from + 1u)) return;
	pass_test("REG-004: PE clear resets active transaction state");

	i2c_init();
	RCC_APB1ENR &= ~RCC_APB1_I2C1;
	if (!expect_eq("REG-005.GATED_READ", I2C0_CR1, 0u)) return;
	I2C0_CR1 = 0u;
	RCC_APB1ENR |= RCC_APB1_I2C1;
	if (!expect_eq("REG-005.RETAINED", I2C0_CR1, CR1_PE)) return;
	RCC_APB1RSTR = RCC_APB1_I2C1;
	RCC_APB1RSTR = 0u;
	if (!expect_eq("REG-005.RESET", I2C0_CR1, 0u)) return;
	pass_test("REG-005: RCC clock gate and APB reset");
}

static void test_irq_semantics(void)
{
	put_str("\r\n--- I2C Interrupt Semantics Test ---\r\n");
	I2cMonitor mon;
	monitor_begin(&mon);
	I2C0_CR2 = 36u | CR2_ITEVTEN | CR2_ITERREN;
	i2c_start();
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("IRQ-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(mon.from + 2u)) {
		fail_test("IRQ-001", "ADDR timeout");
		return;
	}
	i2c_write_byte(0x77u);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) {
		fail_test("IRQ-001", "TxE/BTF flag timeout");
		return;
	}
	if (!expect_true("IRQ-001", monitor_ev_count(&mon, SR1_SB) > 0, "SB event should remain visible")) return;
	if (!expect_true("IRQ-001", monitor_ev_count(&mon, SR1_ADDR) > 0, "ADDR event should remain visible")) return;
	if (!expect_true("IRQ-001", monitor_ev_count(&mon, SR1_BTF) > 0, "BTF event should remain visible")) return;
	pass_test("IRQ-001: ITBUFEN masks buffer-only IRQs while BTF remains visible");

	i2c_init();
	monitor_begin(&mon);
	I2C0_CR2 = 36u | CR2_ITBUFEN | CR2_ITERREN;
	i2c_start();
	if (!i2c_wait_sr1(SR1_SB)) {
		fail_test("IRQ-002", "SB flag timeout");
		return;
	}
	if (!expect_eq("IRQ-002.EV_IRQS", (unsigned int)i2c_count(mon.from, IRQ_I2C0_EV, 0u), 0u)) return;
	if (!expect_mask("IRQ-002.SR2", I2C0_SR2, SR2_MSL | SR2_BUSY, SR2_MSL | SR2_BUSY)) return;
	pass_test("IRQ-002: ITEVTEN gates event IRQ while flags still set");

	i2c_init();
	monitor_begin(&mon);
	I2C0_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN;
	i2c_start();
	if (!i2c_wait_sr1(SR1_SB)) {
		fail_test("IRQ-003", "SB timeout");
		return;
	}
	i2c_write_addr(0x7Eu, 0u);
	if (!i2c_wait_sr1(SR1_AF)) {
		fail_test("IRQ-003", "AF flag timeout");
		return;
	}
	if (!expect_true("IRQ-003", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "SB event missing")) return;
	if (!expect_eq("IRQ-003.ER_IRQ", (unsigned int)monitor_er_count(&mon, SR1_AF), 0u)) return;
	I2C0_SR1 = 0u;
	if (!expect_eq("IRQ-003.AF_CLEAR", I2C0_SR1 & SR1_AF, 0u)) return;
	pass_test("IRQ-003: ITERREN gates error IRQ and SR1 errors clear W0C");
}

static void test_core_transfer(void)
{
	put_str("\r\n--- I2C Core 7-bit Transfer Test ---\r\n");
	I2cMonitor mon;
	unsigned int rx = 0u;

	monitor_begin(&mon);
	if (!i2c0_addr7_write(I2C0_SLAVE_ADDR, 0x33u)) {
		fail_test("CORE-001", "write frame timeout");
		return;
	}
	if (!expect_eq("CORE-001.DATA", I2C1_DR & 0xFFu, 0x33u)) return;
	if (!sb_expect_tx_sequence("CORE-001", &mon)) return;
	if (!expect_true("CORE-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("CORE-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!sb_expect_no_error("CORE-001", &mon)) return;
	pass_test("CORE-001: 7-bit master write reaches peer slave");

	i2c_init();
	monitor_begin(&mon);
	if (!i2c0_addr7_read(I2C0_SLAVE_ADDR, 0x5Au, &rx)) {
		fail_test("CORE-002", "read frame timeout");
		return;
	}
	if (!expect_eq("CORE-002.DATA", rx, 0x5Au)) return;
	if (!sb_expect_rx_sequence("CORE-002", &mon)) return;
	if (!expect_true("CORE-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_TxE) >= 0, "slave TxE event missing")) return;
	if (!expect_true("CORE-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_BTF) >= 0, "slave BTF event missing")) return;
	if (!sb_expect_no_error("CORE-002", &mon)) return;
	pass_test("CORE-002: 7-bit master read receives peer data");

	i2c_init();
	monitor_begin(&mon);
	I2C0_CR2 = 36u | CR2_ITBUFEN | CR2_ITERREN;
	i2c_start();
	if (!i2c_wait_sr1(SR1_SB)) {
		fail_test("CORE-003", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_sr1(SR1_ADDR)) {
		fail_test("CORE-003", "ADDR timeout");
		return;
	}
	if (!expect_eq("CORE-003.EV_IRQS", (unsigned int)i2c_count(mon.from, IRQ_I2C0_EV, 0u), 0u)) return;
	if (!expect_true("CORE-003", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_mask("CORE-003.SR2", I2C0_SR2, SR2_MSL | SR2_BUSY | SR2_TRA, SR2_MSL | SR2_BUSY | SR2_TRA)) return;
	(void)I2C0_SR1;
	(void)I2C0_SR2;
	if (!expect_eq("CORE-003.ADDR", I2C0_SR1 & SR1_ADDR, 0u)) return;
	if (!sb_expect_no_error("CORE-003", &mon)) return;
	i2c_stop();
	pass_test("CORE-003: SR2 state and ADDR clear sequence");
}

static void test_addressing(void)
{
	put_str("\r\n--- I2C Addressing Mode Test ---\r\n");
	I2cMonitor mon;
	unsigned int rx = 0u;

	i2c_clear_log();
	monitor_begin(&mon);
	I2C1_OAR2 = (0x52u << 1u) | OAR2_ENDUAL;
	if (!i2c0_addr7_write(0x52u, 0x4Du)) {
		fail_test("ADDR-001", "OAR2 write timeout");
		return;
	}
	if (!expect_eq("ADDR-001.DATA", I2C1_DR & 0xFFu, 0x4Du)) return;
	if (!expect_true("ADDR-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ADDR-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ADDR-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!expect_true("ADDR-001", (I2C1_SR2 & SR2_DUALF) != 0u, "DUALF missing")) return;
	pass_test("ADDR-001: secondary 7-bit address write");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	I2C1_OAR2 = (0x52u << 1u) | OAR2_ENDUAL;
	if (!i2c0_addr7_read(0x52u, 0x71u, &rx)) {
		fail_test("ADDR-002", "OAR2 read timeout");
		return;
	}
	if (!expect_eq("ADDR-002.DATA", rx, 0x71u)) return;
	if (!expect_true("ADDR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ADDR-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ADDR-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_TxE) >= 0, "slave TxE event missing")) return;
	if (!expect_true("ADDR-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_BTF) >= 0, "slave BTF event missing")) return;
	if (!expect_true("ADDR-002", (I2C1_SR2 & SR2_DUALF) != 0u, "DUALF missing")) return;
	pass_test("ADDR-002: secondary 7-bit address read");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_10bit(0x2A5u);
	if (!i2c0_addr10_write(0x2A5u, 0xC3u)) {
		fail_test("ADDR-003", "10-bit write timeout");
		return;
	}
	if (!expect_eq("ADDR-003.DATA", I2C1_DR & 0xFFu, 0xC3u)) return;
	if (!expect_true("ADDR-003", i2c_find(0u, IRQ_I2C0_EV, SR1_ADD10) >= 0, "ADD10 event missing")) return;
	if (!expect_true("ADDR-003", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-003", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ADDR-003", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	pass_test("ADDR-003: 10-bit addressed write");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_10bit(0x2A5u);
	rx = 0u;
	if (!i2c0_addr10_read(0x2A5u, 0x9Eu, &rx)) {
		fail_test("ADDR-004", "10-bit read timeout");
		return;
	}
	if (!expect_eq("ADDR-004.DATA", rx, 0x9Eu)) return;
	if (!expect_true("ADDR-004", i2c_count(0u, IRQ_I2C0_EV, SR1_ADDR) >= 2, "repeated 10-bit ADDR events missing")) return;
	if (!expect_true("ADDR-004", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-004", i2c_count(mon.from, IRQ_I2C1_EV, SR1_TxE) > 0, "slave TxE event missing")) return;
	if (!expect_true("ADDR-004", i2c_count(mon.from, IRQ_I2C1_EV, SR1_BTF) > 0, "slave BTF event missing")) return;
	pass_test("ADDR-004: 10-bit addressed read with repeated START");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_7bit(I2C1_SLAVE_ADDR, CR1_ACK | CR1_ENGC);
	i2c_start();
	if (!i2c_wait_n(1u)) {
		fail_test("ADDR-005", "SB timeout");
		return;
	}
	i2c_write_addr(0u, 0u);
	if (!i2c_wait_n(2u)) {
		fail_test("ADDR-005", "ADDR timeout");
		return;
	}
	I2C0_DR = 0xA9u;
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) {
		fail_test("ADDR-005", "TxE timeout");
		return;
	}
	if (!expect_true("ADDR-005", i2c_find(0u, IRQ_I2C1_EV, SR1_RxNE) >= 0, "general-call RxNE event missing")) return;
	if (!expect_eq("ADDR-005.DATA", I2C1_DR & 0xFFu, 0xA9u)) return;
	if (!expect_true("ADDR-005", (I2C1_SR2 & SR2_GENCALL) != 0u, "GENCALL missing")) return;
	if (!expect_true("ADDR-005", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-005", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ADDR-005", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	i2c_stop();
	pass_test("ADDR-005: general-call address handling");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_10bit(0x2A5u);
	i2c_start();
	if (!i2c_wait_n(1u)) {
		fail_test("ADDR-006", "SB timeout");
		return;
	}
	i2c_write_addr10_header(0x1A5u, 0u);
	if (!i2c_wait_sr1(SR1_ADD10)) {
		fail_test("ADDR-006", "ADD10 timeout");
		return;
	}
	I2C0_DR = 0xA5u;
	if (!i2c_wait_n(2u)) {
		fail_test("ADDR-006", "AF timeout");
		return;
	}
	if (!expect_true("ADDR-006", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing for high-prefix mismatch")) return;
	if (!expect_eq("ADDR-006.SLAVE_RX", I2C1_SR1 & SR1_RxNE, 0u)) return;
	if (!expect_true("ADDR-006", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-006", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADD10) >= 0, "master ADD10 event missing")) return;
	pass_test("ADDR-006: 10-bit high-prefix mismatch rejects address");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_10bit(0x2A5u);
	i2c_start();
	if (!i2c_wait_n(1u)) {
		fail_test("ADDR-007", "SB timeout");
		return;
	}
	i2c_write_addr10_header(0x2A5u, 0u);
	if (!i2c_wait_sr1(SR1_ADD10)) {
		fail_test("ADDR-007", "ADD10 timeout");
		return;
	}
	I2C0_DR = 0xA4u;
	if (!i2c_wait_n(2u)) {
		fail_test("ADDR-007", "AF timeout");
		return;
	}
	if (!expect_true("ADDR-007", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing for low-byte mismatch")) return;
	if (!expect_eq("ADDR-007.SLAVE_RX", I2C1_SR1 & SR1_RxNE, 0u)) return;
	if (!expect_true("ADDR-007", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-007", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADD10) >= 0, "master ADD10 event missing")) return;
	pass_test("ADDR-007: 10-bit low-byte mismatch rejects address");
}

static void test_bus_protocol(void)
{
	put_str("\r\n--- I2C Bus Protocol Sequencing Test ---\r\n");
	unsigned int rx = 0u;
	I2cMonitor mon;

	i2c_clear_log();
	monitor_begin(&mon);
	if (!i2c0_addr7_write(I2C0_SLAVE_ADDR, 0xA6u)) {
		fail_test("BUS-001", "write phase timeout");
		return;
	}
	if (!expect_eq("BUS-001.SLAVE_RX", I2C1_DR & 0xFFu, 0xA6u)) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!expect_true("BUS-001", (I2C1_SR1 & SR1_STOPF) != 0u, "slave STOPF missing")) return;
	if (!expect_true("BUS-001", i2c_find(0u, IRQ_I2C1_EV, SR1_STOPF) >= 0, "STOPF IRQ missing")) return;
	i2c_init();
	monitor_begin(&mon);
	if (!i2c0_addr7_read(I2C0_SLAVE_ADDR, 0x6Cu, &rx)) {
		fail_test("BUS-001", "read phase timeout");
		return;
	}
	if (!expect_eq("BUS-001.MASTER_RX", rx, 0x6Cu)) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_TxE) >= 0, "slave TxE event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_BTF) >= 0, "slave BTF event missing")) return;
	if (!expect_true("BUS-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_STOPF) >= 0, "slave STOPF missing on read completion")) return;
	pass_test("BUS-001: write then read as separate bus frames with STOPF");
}

static void test_protocol_edges(void)
{
	put_str("\r\n--- I2C Protocol Edge Cases Test ---\r\n");
	I2cMonitor mon;
	unsigned int first = 0u;
	unsigned int second = 0u;

	monitor_begin(&mon);
	if (!i2c0_read_two_bytes(I2C0_SLAVE_ADDR, 0x10u, 0x20u, &first, &second)) {
		fail_test("PE-001", "two-byte read timeout");
		return;
	}
	if (!expect_eq("PE-001.FIRST", first, 0x10u)) return;
	if (!expect_eq("PE-001.SECOND", second, 0x20u)) return;
	if (!expect_true("PE-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("PE-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("PE-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("PE-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_BTF) >= 0, "slave BTF event missing")) return;
	if (!expect_true("PE-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_STOPF) >= 0, "slave STOPF event missing")) return;
	pass_test("PE-001: two-byte read depth");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(1u)) {
		fail_test("PE-002", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(2u)) {
		fail_test("PE-002", "ADDR timeout");
		return;
	}
	i2c_stop();
	if (!expect_true("PE-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("PE-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("PE-002", (I2C1_SR1 & SR1_STOPF) != 0u, "STOPF missing after address-only stop")) return;
	if (!expect_true("PE-002", i2c_find(0u, IRQ_I2C1_EV, SR1_STOPF) >= 0, "STOPF IRQ missing")) return;
	pass_test("PE-002: address-only STOPF");
}

static void test_error_handling(void)
{
	put_str("\r\n--- I2C Error Handling Test ---\r\n");
	I2cMonitor mon;

	i2c_clear_log();
	monitor_begin(&mon);
	if (!start_addr_write(0x7Eu)) {
		fail_test("ERR-001", "bad-address setup timeout");
		return;
	}
	if (!i2c_wait_n(2u)) {
		fail_test("ERR-001", "AF IRQ timeout");
		return;
	}
	if (!expect_true("ERR-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERR-001", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing")) return;
	pass_test("ERR-001: address NACK reports AF");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_7bit(I2C1_SLAVE_ADDR, 0u);
	if (!start_addr_write(I2C1_SLAVE_ADDR)) {
		fail_test("ERR-002", "address phase timeout");
		return;
	}
	if (!i2c_wait_n(2u)) {
		fail_test("ERR-002", "ADDR timeout");
		return;
	}
	I2C0_DR = 0xD1u;
	if (!i2c_wait_n(3u)) {
		fail_test("ERR-002", "data AF IRQ timeout");
		return;
	}
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ERR-002", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "master AF missing")) return;
	if (!expect_eq("ERR-002.SLAVE_DR", I2C1_DR & 0xFFu, 0u)) return;
	pass_test("ERR-002: data-phase NACK reports AF");
}

static void test_error_edgecases(void)
{
	put_str("\r\n--- I2C Error Edge Cases Test ---\r\n");
	I2cMonitor mon;

	i2c_clear_log();
	i2c1_config_7bit(I2C1_SLAVE_ADDR, CR1_ACK);
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("ERRX-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C1_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(mon.from + 2u)) {
		fail_test("ERRX-001", "ADDR timeout");
		return;
	}
	I2C0_DR = 0x11u;
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) {
		fail_test("ERRX-001", "first byte timeout");
		return;
	}
	if (!i2c1_wait_sr1(SR1_RxNE)) {
		fail_test("ERRX-001", "slave RxNE timeout");
		return;
	}
	I2C0_DR = 0x22u;
	if (!i2c_wait_n(mon.from + 5u)) {
		fail_test("ERRX-001", "overrun IRQ timeout");
		return;
	}
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(0u, IRQ_I2C1_ER, SR1_OVR) >= 0, "OVR IRQ missing")) return;
	if (!expect_true("ERRX-001", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "master AF missing")) return;
	pass_test("ERRX-001: slave overrun reports OVR and master AF");

	i2c_init();
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("ERRX-002", "SB timeout");
		return;
	}
	I2C1_CR1 |= CR1_START;
	if (!i2c_wait_n(mon.from + 2u)) {
		fail_test("ERRX-002", "ARLO IRQ timeout");
		return;
	}
	if (!expect_true("ERRX-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERRX-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_SB) >= 0, "peer master SB event missing")) return;
	if (!expect_true("ERRX-002", i2c_find(0u, IRQ_I2C0_ER, SR1_ARLO) >= 0, "ARLO IRQ missing")) return;
	if (!expect_true("ERRX-002", (I2C0_SR2 & SR2_MSL) == 0u, "master state not cleared after ARLO")) return;
	pass_test("ERRX-002: contending START reports arbitration loss");

	i2c_init();
	i2c_clear_log();
	i2c1_config_7bit(I2C1_SLAVE_ADDR, CR1_ACK);
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(1u)) {
		fail_test("ERRX-003", "SB timeout");
		return;
	}
	i2c_write_addr(I2C1_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(2u)) {
		fail_test("ERRX-003", "ADDR timeout");
		return;
	}
	i2c_start();
	if (!expect_true("ERRX-003", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERRX-003", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ERRX-003", i2c_find(0u, IRQ_I2C1_ER, SR1_BERR) >= 0, "BERR IRQ missing")) return;
	pass_test("ERRX-003: repeated START while slave-selected reports bus error");
}

static void test_robustness(void)
{
	put_str("\r\n--- I2C Robustness, Timing, and Data Test ---\r\n");
	I2cMonitor mon;
	static const unsigned int patterns[] = {0x00u, 0x01u, 0x55u, 0x80u, 0xAAu, 0xFFu};
	unsigned int i;

	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("ROB-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(mon.from + 2u)) {
		fail_test("ROB-001", "ADDR timeout");
		return;
	}
	for (i = 0u; i < sizeof(patterns) / sizeof(patterns[0]); ++i) {
		I2C0_DR = patterns[i] | 0xFFFFFF00u;
		if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) {
			fail_test("ROB-001", "TxE timeout");
			return;
		}
		if (!i2c1_wait_sr1(SR1_RxNE)) {
			fail_test("ROB-001", "slave RxNE timeout");
			return;
		}
		if (!expect_eq("ROB-001.DATA", I2C1_DR & 0xFFu, patterns[i])) return;
	}
	i2c_stop();
	if (!sb_expect_tx_sequence("ROB-001", &mon)) return;
	if (!expect_true("ROB-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ROB-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!sb_expect_no_error("ROB-001", &mon)) return;
	pass_test("ROB-001: byte data masking and representative patterns");

	i2c_init();
	i2c_clear_log();
	I2C0_CCR = 2u;
	if (!expect_eq("ROB-002.STD", I2C0_CCR, 2u)) return;
	I2C0_CCR = 0x8002u;
	if (!expect_eq("ROB-002.FAST", I2C0_CCR, 0x8002u)) return;
	I2C0_CCR = 0xC002u;
	if (!expect_eq("ROB-002.DUTY", I2C0_CCR, 0xC002u)) return;
	if (!expect_eq("ROB-002.LOG", g_log_count, 0u)) return;
	pass_test("ROB-002: CCR standard/fast/duty settings");

	i2c_init();
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_n(mon.from + 1u)) {
		fail_test("ROB-003", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_n(mon.from + 2u)) {
		fail_test("ROB-003", "ADDR timeout");
		return;
	}
	for (i = 0u; i < 32u; ++i) {
		I2C0_DR = i;
		if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) {
			fail_test("ROB-003", "stream TxE timeout");
			return;
		}
		if ((I2C1_SR1 & SR1_RxNE) != 0u) {
			(void)I2C1_DR;
		}
	}
	i2c_stop();
	if (!sb_expect_tx_sequence("ROB-003", &mon)) return;
	if (!expect_true("ROB-003", i2c_find(mon.from, IRQ_I2C1_EV, SR1_STOPF) >= 0, "slave STOPF event missing")) return;
	if (!expect_true("ROB-003", monitor_ev_count(&mon, SR1_TxE) > 0, "missing stream TxE IRQs")) return;
	if (!sb_expect_no_error("ROB-003", &mon)) return;
	pass_test("ROB-003: 32-byte transmit stream");

	i2c_init();
	i2c_clear_log();
	unsigned int checksum = 0u;
	for (i = 0u; i < 256u; ++i) {
		unsigned int cr2 = (i & CR2_FREQ_MASK) | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
		unsigned int ccr = (i & 0x0FFFu) + 1u;
		I2C0_CR2 = cr2;
		I2C0_CCR = ccr;
		checksum ^= I2C0_CR2 ^ I2C0_CCR;
		if ((I2C0_CR2 & CR2_RW_MASK) != cr2) {
			fail_test("ROB-004", "CR2 mismatch");
			return;
		}
		if (I2C0_CCR != ccr) {
			fail_test("ROB-004", "CCR mismatch");
			return;
		}
	}
	if (!expect_true("ROB-004", checksum != 0u, "checksum did not change")) return;
	if (!expect_eq("ROB-004.LOG", g_log_count, 0u)) return;
	pass_test("ROB-004: register access sweep");

	i2c_clear_log();
	for (i = 0u; i < 64u; ++i) {
		i2c_recover();
		if (!expect_eq("ROB-005.CR1", I2C0_CR1, CR1_PE)) return;
		if (!expect_eq("ROB-005.SR1", I2C0_SR1, 0u)) return;
		if (!expect_eq("ROB-005.SR2", I2C0_SR2, 0u)) return;
	}
	if (!expect_eq("ROB-005.LOG", g_log_count, 0u)) return;
	pass_test("ROB-005: repeated recover soak");
}

void isr_main(void)
{
	setup_trap_handler();
	setup_plic();
	enable_irq();

	if (g_test_mask & 0x001u) test_rcc();
	if (g_test_mask & 0x200u) test_gpio_exti();
	if (g_test_mask & 0x400u) test_dma_crc();
	if (g_test_mask & 0x2000u) test_backup_domain();
	if (g_test_mask & 0x4000u) test_timers();
	if (g_test_mask & 0x8000u) test_watchdogs();
	if (g_test_mask & 0x10000u) test_flash();
	if (g_test_mask & 0x20000u) test_adc();
	if (g_test_mask & 0x40000u) test_dac();
	if (g_test_mask & 0x80000u) test_can();
	if (g_test_mask & 0x100000u) test_sdio();
	if (g_test_mask & 0x200000u) test_fsmc();
	if (g_test_mask & 0x400000u) test_usb_fs();
	if (g_test_mask & 0x800000u) test_otg_fs();
	if (g_test_mask & 0x1000000u) test_eth();
	if (g_test_mask & 0x1000u) test_spi();
	if (g_test_mask & 0x800u) test_usart();
	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x001u) test_register_model();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x002u) test_irq_semantics();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x004u) test_core_transfer();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x008u) test_addressing();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x010u) test_bus_protocol();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x020u) test_protocol_edges();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x040u) test_error_handling();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x080u) test_error_edgecases();

	i2c_init();
	i2c_clear_log();
	if (g_test_mask & 0x100u) test_robustness();

	i2c_finish();
}

int main(void)
{
	isr_main();
	return 0;
}
