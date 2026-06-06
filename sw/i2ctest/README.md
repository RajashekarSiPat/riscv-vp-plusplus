# I2C verification

Bare-metal regression for the `i2ctest-vp` platform. The suite exercises the
STM32-style `i2c_stm32` controller pair in VP++, with I2C0 as master and I2C1
as the peer slave, and checks the interrupt trail for each directed scenario.
The platform also models RM0008 RCC clock/reset registers. I2C1 and I2C2 are
clocked and reset through `RCC_APB1ENR` and `RCC_APB1RSTR`.
The same platform now includes AFIO, EXTI, and GPIOA/B loopback wiring so the
STM32F1 external-interrupt path can be verified without manual interaction.
DMA1 channel 1 and the CRC block are also modeled on the STM32F1 AHB clock
domain.
USART1 and USART2 are modeled as a back-to-back pair with APB clock/reset
integration and interrupt-driven loopback coverage.
SPI1 and SPI2 are also modeled as a peer pair with a narrow full-duplex
register subset for directed verification.
The backup domain is also modeled with PWR, BKP, and RTC coverage, including
DBP gating, backup reset, and RTC alarm delivery through the PLIC.
TIM1 through TIM5 are modeled as a shared update-event timer family with
clock gating, reset, counter progression, and interrupt coverage.
WWDG and IWDG are modeled with deterministic register and countdown behavior
for directed verification.
DAC and CAN1 are also modeled in a minimal register-level form, including DAC
trigger/output behavior and CAN1 transmit loopback with interrupt delivery.

STM32 peripheral offsets are exposed through a `+0x01000000` alias because the
RISC-V PLIC occupies the original `0x40000000` peripheral window. For example,
RM0008 I2C1 at `0x40005400` is visible at `0x41005400`.

## Build

```bash
make
```

## Run

```bash
make sim
```

`TEST_MASK` selects the subset of tests to run:

| Bit | Test |
|-----|------|
| `0x01` | RCC and I2C register/clock/reset model |
| `0x02` | interrupt semantics |
| `0x04` | core transfer |
| `0x08` | addressing |
| `0x10` | bus protocol |
| `0x20` | protocol edges |
| `0x40` | error handling |
| `0x80` | error edge cases |
| `0x100` | robustness |
| `0x200` | GPIO / EXTI loopback |
| `0x400` | DMA1 channel 1 and CRC |
| `0x800` | USART1 / USART2 loopback |
| `0x1000` | SPI1 / SPI2 loopback |
| `0x2000` | PWR / BKP / RTC backup-domain regression |
| `0x4000` | TIM1 / TIM2 update-event regression |
| `0x8000` | WWDG / IWDG watchdog regression |
| `0x10000` | flash interface regression |
| `0x20000` | ADC1 / ADC2 conversion regression |
| `0x40000` | DAC register and trigger regression |
| `0x80000` | CAN1 register, loopback, and interrupt regression |
