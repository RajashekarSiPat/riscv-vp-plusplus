# STM32F1-compatible RISC-V SoC roadmap

## Goal

Provide RM0008-compatible STM32F1 peripherals around the existing RV32 CPU and
PLIC/CLINT execution environment. The target is peripheral software
compatibility and deterministic SystemC/TLM verification, not Cortex-M3 or NVIC
compatibility.

RM0008 describes several different products. The VP therefore uses a superset
profile based on STM32F103 high-density devices plus the F105/F107 connectivity
line. Each implemented peripheral must state which profile-specific features it
supports.

## Address and interrupt policy

- STM32 peripheral offsets remain identical to RM0008.
- The STM32 `0x40000000` peripheral window is exposed at `0x41000000` because
  the RISC-V PLIC occupies `0x40000000`.
- `stm32f1::PERIPH_ALIAS` is the only address translation constant.
- Peripheral interrupt conditions follow RM0008, but sources terminate at the
  RISC-V PLIC instead of the NVIC.
- DMA request behavior is modeled at the peripheral boundary and routed to an
  STM32F1-compatible DMA controller.

## Verification definition

A peripheral is complete only when it has:

1. Register reset values, access classes, reserved-bit masks, and documented
   read/write side effects.
2. RCC clock gating and peripheral reset integration.
3. Functional data-path timing and externally visible protocol behavior.
4. PLIC interrupt and DMA request coverage.
5. Bare-metal directed tests for normal, boundary, and error behavior.
6. A documented list of intentionally unsupported electrical or analog
   effects.

Tests should use two modeled peers or a protocol endpoint where practical, as
the I2C regression does, rather than validating register storage alone.

## Implementation order

| Phase | Blocks | Reason |
|---|---|---|
| 0 | Shared memory map, RCC, reset/clock contracts | Dependency for every peripheral |
| 1 | AFIO, GPIO, EXTI | Pin routing and external events |
| 2 | DMA1/DMA2, CRC | Shared data movement and a bounded reference model |
| 3 | STM32 USART/UART, SPI/I2S | Core serial peripherals and DMA integration |
| 4 | TIM1-TIM14, watchdogs, RTC/BKP | Common clocked state machines |
| 5 | ADC, DAC | Digital conversion behavior with injectable analog values |
| 6 | bxCAN, USB device, SDIO, FSMC | Larger protocol and storage controllers |
| 7 | OTG_FS, Ethernet MAC/DMA | Connectivity-line subsystems |
| 8 | Flash interface, option-byte behavior, power controller, signatures | System control completion |

## Current status

| Block | Status |
|---|---|
| RV32 CPU, PLIC, CLINT, RAM | Existing |
| I2C1/I2C2 | Functional model and directed regression |
| RCC | Register model, immediate oscillator readiness, clock-source status, reset flags, and APB clock/reset bindings |
| AFIO, GPIO, EXTI | Functional models with loopback verification |
| DMA1 channel 1, CRC | Functional AHB-clocked models with directed regression |
| USART1/USART2 | Functional register model and directed loopback regression |
| SPI1/SPI2 | Functional register model and directed loopback regression |
| PWR, BKP, RTC | Functional backup-domain model with DBP gating, backup reset, and alarm regression |
| TIM1-TIM5 | Shared update-event timer model with directed regression |
| WWDG, IWDG | Functional watchdog models with directed register/countdown regression |
| DAC | Minimal register-level model with trigger/output regression |
| CAN1 | Minimal register-level loopback model with interrupt regression |
| SDIO | Minimal register/FIFO model with command and transfer regression |
| FSMC | Minimal register and external-bank model with read/write regression |
| USART | Existing `usart2test` remains a custom test UART, not the RM0008 USART register model |
| Remaining RM0008 blocks | USB device/OTG FS and Ethernet MAC/DMA remain |

## Modeling boundaries

- Oscillators and PLLs become ready without analog startup delay for now. The
  ready flags and clock-source selection semantics are modeled.
- Analog inputs will use explicit SystemC/TLM injection APIs.
- USB and Ethernet require protocol-level endpoints; pin-level PHY signaling is
  outside the initial scope.
- Cortex-M3 system peripherals, NVIC exception numbering, and bit-band behavior
  are not provided by the RISC-V profile.
