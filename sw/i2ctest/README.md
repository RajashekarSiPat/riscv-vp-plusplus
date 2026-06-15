# I2C verification

Bare-metal regression for the `i2ctest-vp` platform. The suite exercises the
STM32-style `i2c_stm32` controller pair in VP++, with I2C0 as master and I2C1
as the peer slave, and checks the interrupt trail for each directed scenario. It
also timestamps interrupt entries through a small MMIO timer peripheral so the
spacing between interrupts can be asserted.

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
| `0x01` | register model |
| `0x02` | interrupt semantics |
| `0x04` | core transfer |
| `0x08` | addressing |
| `0x10` | bus protocol |
| `0x20` | protocol edges |
| `0x40` | error handling |
| `0x80` | error edge cases |
| `0x100` | robustness |
