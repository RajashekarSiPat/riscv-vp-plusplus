RISCV_PREFIX ?= riscv32-unknown-elf-
CC  = $(RISCV_PREFIX)gcc
AS  = $(RISCV_PREFIX)gcc
LD  = $(RISCV_PREFIX)gcc

ARCH     = rv32imac_zicsr
ABI      = ilp32
TEST_MASK ?= 0x1F
CFLAGS   = -march=$(ARCH) -mabi=$(ABI) -O1 -Wall -DTEST_MASK=$(TEST_MASK)
ASFLAGS = -march=$(ARCH) -mabi=$(ABI)
LDFLAGS = -march=$(ARCH) -mabi=$(ABI) -nostartfiles -static -no-pie \
          -Wl,--no-relax -T link.ld

TARGET  = main.elf
OBJS    = bootstrap.o main.o

VP      ?= i2ctest-vp
VP_BIN  ?= ../../vp/build/bin/$(VP)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^
	$(RISCV_PREFIX)size $@

bootstrap.o: bootstrap.S
	$(AS) $(ASFLAGS) -c -o $@ $<

main.o: main.c
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(TARGET)
	PATH="$(HOME)/.local/bin:$(PATH)" $(VP_BIN) $(TARGET)

sim: $(TARGET)
	PATH="$(HOME)/.local/bin:$(PATH)" $(VP_BIN) $(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all run sim clean
