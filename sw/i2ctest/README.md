# I2C verification

Bare-metal regression for the `i2ctest-vp` platform. The suite exercises the
existing `FU540_I2C` controller model against the built-in `DS1307` slave and
checks the interrupt raised for each command in the scenario.

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
| `0x01` | reset quiet |
| `0x02` | write then readback |
| `0x04` | repeated-start read |
| `0x08` | missing-slave NACK |
| `0x10` | interrupt re-arm |
