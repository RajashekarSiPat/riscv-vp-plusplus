/* main.c — RISC-V RV32 I2C verification suite for riscv-vp-plusplus
 *
 * The suite exercises the existing FU540 I2C controller model against the
 * built-in DS1307 slave. Every directed test checks the interrupt observed
 * for each command in the scenario, not just the data value.
 */

#ifndef TEST_MASK
#define TEST_MASK 0x1Fu
#endif
volatile unsigned int g_test_mask __attribute__((section(".test_cfg"))) = TEST_MASK;

#include <stdint.h>
#include <stdio.h>

#define MMIO32(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define I2C_BASE       0x10030000UL
#define I2C_PRER_LO    0x00
#define I2C_PRER_HI    0x04
#define I2C_CTR        0x08
#define I2C_TXR_RXR    0x0C
#define I2C_CR_SR      0x10

#define I2C_CTR_EN     (1u << 7)
#define I2C_CTR_IEN    (1u << 6)

#define I2C_CR_STA     (1u << 7)
#define I2C_CR_STO     (1u << 6)
#define I2C_CR_RD      (1u << 5)
#define I2C_CR_WR      (1u << 4)
#define I2C_CR_ACK     (1u << 3)
#define I2C_CR_IACK    (1u << 0)

#define I2C_SR_RXACK   (1u << 7)
#define I2C_SR_BUSY    (1u << 6)
#define I2C_SR_AL      (1u << 5)
#define I2C_SR_TIP     (1u << 1)
#define I2C_SR_IF      (1u << 0)

#define PLIC_BASE            0x40000000UL
#define PLIC_PRIO(n)         MMIO32(PLIC_BASE + (n) * 4UL)
#define PLIC_ENABLE_HART0(n) MMIO32(PLIC_BASE + 0x2000UL + (((n) / 32u) * 4UL))
#define PLIC_THRESHOLD_HART0 MMIO32(PLIC_BASE + 0x200000UL)
#define PLIC_CLAIM_HART0     MMIO32(PLIC_BASE + 0x200004UL)

#define IRQ_I2C 50u

typedef struct {
    uint32_t irq_id;
    uint32_t status;
} IrqEvent;

static volatile IrqEvent g_log[32];
static volatile uint32_t g_log_count;
static volatile uint32_t g_irq_count;
static int g_pass;
static int g_fail;

void trap_handler(void);
extern void trap_entry(void);

static void setup_trap_handler(void);
static void setup_plic(void);
static void enable_irq(void);
static int wait_irq_count(uint32_t target);
static uint32_t i2c_status(void);
static void i2c_set_txr(uint32_t value);
static void i2c_cmd(uint32_t value);
static void i2c_start_write(uint8_t addr);
static void i2c_start_read(uint8_t addr);
static void i2c_stop(void);
static void i2c_ack_irq(void);
static void i2c_write_byte(uint8_t value);
static uint8_t i2c_read_byte(void);
static void pass_test(const char *name);
static void fail_test(const char *name, const char *why);
static int  test_reset_quiet(void);
static int  test_write_then_readback(void);
static int  test_repeated_start_read(void);
static int  test_missing_slave_nack(void);
static int  test_irq_rearm(void);

void trap_handler(void)
{
    uint32_t mcause;
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));
    if (mcause != 0x8000000Bu) {
        return;
    }

    uint32_t irq_id = PLIC_CLAIM_HART0;
    if (irq_id != IRQ_I2C) {
        return;
    }

    uint32_t idx = g_log_count;
    if (idx < 32u) {
        g_log[idx].irq_id = irq_id;
        g_log[idx].status = i2c_status();
        __asm__ volatile("fence" ::: "memory");
        g_log_count = idx + 1u;
    }

    i2c_ack_irq();
    PLIC_CLAIM_HART0 = irq_id;
    g_irq_count++;
}

static void setup_trap_handler(void)
{
    uintptr_t addr = (uintptr_t)(void *)trap_entry;
    __asm__ volatile("csrw mtvec, %0" :: "r"(addr));
}

static void setup_plic(void)
{
    PLIC_PRIO(IRQ_I2C) = 1u;
    PLIC_ENABLE_HART0(IRQ_I2C) = (uint32_t)(1u << (IRQ_I2C % 32u));
    PLIC_THRESHOLD_HART0 = 0u;
}

static void enable_irq(void)
{
    __asm__ volatile("csrs mie, %0" :: "r"(1u << 11));
    __asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));
}

static int wait_irq_count(uint32_t target)
{
    for (uint32_t spins = 0; spins < 1000000u; ++spins) {
        if (g_irq_count >= target) {
            return 1;
        }
        __asm__ volatile("" ::: "memory");
    }
    return 0;
}

static uint32_t i2c_status(void)
{
    return MMIO32(I2C_BASE + I2C_CR_SR);
}

static void i2c_set_txr(uint32_t value)
{
    MMIO32(I2C_BASE + I2C_TXR_RXR) = value;
}

static void i2c_cmd(uint32_t value)
{
    MMIO32(I2C_BASE + I2C_CR_SR) = value;
}

static void i2c_start_write(uint8_t addr)
{
    i2c_set_txr(((uint32_t)addr << 1) | 0u);
    i2c_cmd(I2C_CR_STA | I2C_CR_WR);
}

static void i2c_start_read(uint8_t addr)
{
    i2c_set_txr(((uint32_t)addr << 1) | 1u);
    i2c_cmd(I2C_CR_STA | I2C_CR_RD);
}

static void i2c_stop(void)
{
    i2c_cmd(I2C_CR_STO);
}

static void i2c_ack_irq(void)
{
    i2c_cmd(I2C_CR_IACK);
}

static void i2c_write_byte(uint8_t value)
{
    i2c_set_txr(value);
    i2c_cmd(I2C_CR_WR);
}

static uint8_t i2c_read_byte(void)
{
    i2c_cmd(I2C_CR_RD);
    return (uint8_t)MMIO32(I2C_BASE + I2C_TXR_RXR);
}

static void pass_test(const char *name)
{
    printf("PASS %s\n", name);
    ++g_pass;
}

static void fail_test(const char *name, const char *why)
{
    printf("FAIL %s: %s\n", name, why);
    ++g_fail;
}

static int test_reset_quiet(void)
{
    const char *name = "reset_quiet";
    uint32_t base = g_irq_count;

    MMIO32(I2C_BASE + I2C_PRER_LO) = 0xFFu;
    MMIO32(I2C_BASE + I2C_PRER_HI) = 0xFFu;
    MMIO32(I2C_BASE + I2C_CTR) = I2C_CTR_EN | I2C_CTR_IEN;

    if (g_irq_count != base) {
        fail_test(name, "configuration generated an unexpected interrupt");
        return 0;
    }

    if ((i2c_status() & (I2C_SR_BUSY | I2C_SR_RXACK | I2C_SR_AL | I2C_SR_TIP | I2C_SR_IF)) != 0u) {
        fail_test(name, "status register not clean after reset");
        return 0;
    }

    pass_test(name);
    return 1;
}

static int test_write_then_readback(void)
{
    const char *name = "write_then_readback";
    uint32_t base = g_irq_count;
    uint32_t log_base = g_log_count;

    i2c_start_write(0x68u);
    if (!wait_irq_count(base + 1u)) {
        fail_test(name, "timeout waiting for start interrupt");
        return 0;
    }
    if (g_log[log_base + 0u].irq_id != IRQ_I2C || (g_log[log_base + 0u].status & (I2C_SR_BUSY | I2C_SR_RXACK)) != I2C_SR_BUSY) {
        fail_test(name, "unexpected status on START");
        return 0;
    }

    i2c_write_byte(0x00u);
    if (!wait_irq_count(base + 2u)) {
        fail_test(name, "timeout waiting for register-pointer write interrupt");
        return 0;
    }
    if (g_log[log_base + 1u].irq_id != IRQ_I2C || (g_log[log_base + 1u].status & (I2C_SR_BUSY | I2C_SR_RXACK)) != I2C_SR_BUSY) {
        fail_test(name, "unexpected status on register pointer write");
        return 0;
    }

    i2c_write_byte(0x55u);
    if (!wait_irq_count(base + 3u)) {
        fail_test(name, "timeout waiting for data write interrupt");
        return 0;
    }
    if (g_log[log_base + 2u].irq_id != IRQ_I2C || (g_log[log_base + 2u].status & (I2C_SR_BUSY | I2C_SR_RXACK)) != I2C_SR_BUSY) {
        fail_test(name, "unexpected status on data write");
        return 0;
    }

    i2c_stop();
    if (g_irq_count != base + 3u) {
        fail_test(name, "STOP generated an interrupt");
        return 0;
    }

    if ((i2c_status() & I2C_SR_BUSY) != 0u) {
        fail_test(name, "BUSY did not clear after STOP");
        return 0;
    }

    pass_test(name);
    return 1;
}

static int test_repeated_start_read(void)
{
    const char *name = "repeated_start_read";
    uint32_t base = g_irq_count;
    uint32_t log_base = g_log_count;

    i2c_start_write(0x68u);
    if (!wait_irq_count(base + 1u)) {
        fail_test(name, "timeout waiting for initial START");
        return 0;
    }
    i2c_write_byte(0x00u);
    if (!wait_irq_count(base + 2u)) {
        fail_test(name, "timeout waiting for pointer write");
        return 0;
    }
    i2c_start_read(0x68u);
    if (!wait_irq_count(base + 3u)) {
        fail_test(name, "timeout waiting for repeated START");
        return 0;
    }
    if ((g_log[log_base + 2u].status & (I2C_SR_BUSY | I2C_SR_RXACK)) != I2C_SR_BUSY) {
        fail_test(name, "unexpected status on repeated START");
        return 0;
    }

    uint8_t value = i2c_read_byte();
    if (!wait_irq_count(base + 4u)) {
        fail_test(name, "timeout waiting for read interrupt");
        return 0;
    }
    if (value != 0x55u) {
        fail_test(name, "read-back byte did not match the prior write");
        return 0;
    }
    if ((g_log[log_base + 3u].status & (I2C_SR_BUSY | I2C_SR_RXACK)) != I2C_SR_BUSY) {
        fail_test(name, "unexpected status on read");
        return 0;
    }

    i2c_stop();
    if (g_irq_count != base + 4u) {
        fail_test(name, "STOP generated an interrupt");
        return 0;
    }

    pass_test(name);
    return 1;
}

static int test_missing_slave_nack(void)
{
    const char *name = "missing_slave_nack";
    uint32_t base = g_irq_count;
    uint32_t log_base = g_log_count;

    i2c_start_write(0x69u);
    if (!wait_irq_count(base + 1u)) {
        fail_test(name, "timeout waiting for NACK interrupt");
        return 0;
    }
    if ((g_log[log_base + 0u].status & I2C_SR_RXACK) == 0u) {
        fail_test(name, "missing slave did not report RXACK");
        return 0;
    }
    if ((g_log[log_base + 0u].status & I2C_SR_BUSY) != 0u) {
        fail_test(name, "BUSY stayed set after failed START");
        return 0;
    }

    i2c_stop();
    if (g_irq_count != base + 1u) {
        fail_test(name, "STOP generated an interrupt");
        return 0;
    }

    pass_test(name);
    return 1;
}

static int test_irq_rearm(void)
{
    const char *name = "irq_rearm";
    uint32_t base = g_irq_count;
    uint32_t log_base = g_log_count;

    i2c_start_write(0x68u);
    if (!wait_irq_count(base + 1u)) {
        fail_test(name, "timeout waiting for first interrupt");
        return 0;
    }
    i2c_stop();

    i2c_start_write(0x68u);
    if (!wait_irq_count(base + 2u)) {
        fail_test(name, "timeout waiting for second interrupt");
        return 0;
    }

    if (g_log[log_base + 0u].irq_id != IRQ_I2C || g_log[log_base + 1u].irq_id != IRQ_I2C) {
        fail_test(name, "interrupt controller did not re-arm cleanly");
        return 0;
    }

    i2c_stop();
    pass_test(name);
    return 1;
}

int main(void)
{
    setup_trap_handler();
    setup_plic();
    enable_irq();

    printf("I2C verification mask: 0x%08x\n", g_test_mask);

    MMIO32(I2C_BASE + I2C_PRER_LO) = 0xFFu;
    MMIO32(I2C_BASE + I2C_PRER_HI) = 0xFFu;
    MMIO32(I2C_BASE + I2C_CTR) = I2C_CTR_EN | I2C_CTR_IEN;

    if (g_test_mask & 0x01u) test_reset_quiet();
    if (g_test_mask & 0x02u) test_write_then_readback();
    if (g_test_mask & 0x04u) test_repeated_start_read();
    if (g_test_mask & 0x08u) test_missing_slave_nack();
    if (g_test_mask & 0x10u) test_irq_rearm();

    printf("I2C summary: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
