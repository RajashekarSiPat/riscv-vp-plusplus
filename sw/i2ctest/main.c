/* main.c - RV32 bare-metal STM32-style I2C verification suite for VP++
 *
 * The firmware exercises the i2c_stm32 controller pair in the riscv-vp-plusplus
 * platform. I2C0 acts as master, I2C1 acts as the peer slave, and every directed
 * scenario validates the interrupts observed during the transaction.
 */

#ifndef TEST_MASK
#define TEST_MASK 0x1FFu
#endif
volatile unsigned int g_test_mask __attribute__((section(".test_cfg"))) = TEST_MASK;

#include <stdint.h>

#define MMIO32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define MMIO64(a) (*(volatile uint64_t *)(uintptr_t)(a))

#define UC_TBUF MMIO32(0x09004004UL)
#define EXITER MMIO32(0x09010000UL)
#define TIMER_CNT MMIO32(0x09005000UL)

#define I2C0_BASE 0x41005400UL
#define I2C1_BASE 0x41005800UL

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
#define PLIC_ENABLE_HART0(n) MMIO32(PLIC_BASE + 0x2000UL + (((n) / 32u) * 4UL))
#define PLIC_THRESHOLD_HART0 MMIO32(PLIC_BASE + 0x200000UL)
#define PLIC_CLAIM_HART0 MMIO32(PLIC_BASE + 0x200004UL)

#define IRQ_I2C0_EV 1u
#define IRQ_I2C0_ER 2u
#define IRQ_I2C1_EV 3u
#define IRQ_I2C1_ER 4u

#define I2C0_SLAVE_ADDR 0x50u
#define I2C1_SLAVE_ADDR 0x51u

#define LOG_SIZE 256u
#define I2C_SCL_PERIODS_PER_BYTE_FRAME 9u

typedef struct {
	unsigned int irq_id;
	unsigned int sr1;
	unsigned int sr2;
	unsigned int tick;
} I2cIrqEvent;

volatile I2cIrqEvent g_log[LOG_SIZE];
volatile unsigned int g_log_count = 0u;
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

static void timer_reset(void)
{
	TIMER_CNT = 0u;
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
		__asm__ volatile("wfi");
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
		__asm__ volatile("wfi");
		__asm__ volatile("" ::: "memory");
	}
	return 0;
}

static int i2c_find(unsigned int from, unsigned int irq_id, unsigned int sr1_mask);

typedef struct {
	unsigned int from;
} I2cMonitor;

static int i2c_wait_log(unsigned int from, unsigned int irq_id, unsigned int sr1_mask)
{
	unsigned int timeout = 20000000u;
	while (--timeout > 0u) {
		if (i2c_find(from, irq_id, sr1_mask) >= 0) {
			return 1;
		}
		__asm__ volatile("wfi");
		__asm__ volatile("" ::: "memory");
	}
	return 0;
}

static int expect_irq_gap_at_least(const char *name, unsigned int first_idx, unsigned int second_idx, unsigned int min_gap)
{
	unsigned int first_tick = g_log[first_idx].tick;
	unsigned int second_tick = g_log[second_idx].tick;
	unsigned int gap = second_tick - first_tick;

	if (gap >= min_gap) {
		return 1;
	}

	put_str("[FAIL] ");
	put_str(name);
	put_str(" - interrupt gap ");
	put_dec(gap);
	put_str(" SCL periods (first=");
	put_dec(first_tick);
	put_str(", second=");
	put_dec(second_tick);
	put_str("), expected at least ");
	put_dec(min_gap);
	put_str(" SCL periods\r\n");
	++g_fail;
	return 0;
}

static int expect_log_time_non_decreasing(const char *name, const I2cMonitor *m)
{
	for (unsigned int i = m->from + 1u; i < g_log_count; ++i) {
		if (g_log[i].tick < g_log[i - 1u].tick) {
			put_str("[FAIL] ");
			put_str(name);
			put_str(" - interrupt time moved backwards at log index ");
			put_dec(i);
			put_str(" (prev=");
			put_dec(g_log[i - 1u].tick);
			put_str(", curr=");
			put_dec(g_log[i].tick);
			put_str(")\r\n");
			++g_fail;
			return 0;
		}
	}
	return 1;
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
		g_log[i].tick = 0u;
	}
	__asm__ volatile("fence" ::: "memory");
	g_log_count = 0u;
}

static void monitor_begin(I2cMonitor *m)
{
	timer_reset();
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
	int sb = i2c_find(m->from, IRQ_I2C0_EV, SR1_SB);
	int addr = i2c_find(m->from, IRQ_I2C0_EV, SR1_ADDR);
	int txe = i2c_find(m->from, IRQ_I2C0_EV, SR1_TxE);
	int btf = i2c_find(m->from, IRQ_I2C0_EV, SR1_BTF);
	ok &= expect_true(name, sb >= 0, "missing SB event");
	ok &= expect_true(name, addr >= 0, "missing ADDR event");
	ok &= expect_true(name, txe >= 0, "missing TxE event");
	ok &= expect_true(name, btf >= 0, "missing BTF event");
	if (ok) {
		ok &= expect_true(name, sb < addr, "SB must precede ADDR");
		ok &= expect_irq_gap_at_least(name, (unsigned int)sb, (unsigned int)addr, I2C_SCL_PERIODS_PER_BYTE_FRAME);
		ok &= expect_true(name, addr < txe, "ADDR must precede TxE");
		ok &= expect_true(name, addr < btf, "ADDR must precede BTF");
		ok &= expect_irq_gap_at_least(name, (unsigned int)addr, (unsigned int)btf, I2C_SCL_PERIODS_PER_BYTE_FRAME);
	}
	return ok;
}

static int sb_expect_rx_sequence(const char *name, const I2cMonitor *m)
{
	int ok = 1;
	int sb = i2c_find(m->from, IRQ_I2C0_EV, SR1_SB);
	int addr = i2c_find(m->from, IRQ_I2C0_EV, SR1_ADDR);
	int rxne = i2c_find(m->from, IRQ_I2C0_EV, SR1_RxNE);
	ok &= expect_true(name, sb >= 0, "missing SB event");
	ok &= expect_true(name, addr >= 0, "missing ADDR event");
	ok &= expect_true(name, rxne >= 0, "missing RxNE event");
	if (ok) {
		ok &= expect_true(name, sb < addr, "SB must precede ADDR");
		ok &= expect_irq_gap_at_least(name, (unsigned int)sb, (unsigned int)addr, I2C_SCL_PERIODS_PER_BYTE_FRAME);
		ok &= expect_true(name, addr < rxne, "ADDR must precede RxNE");
		ok &= expect_irq_gap_at_least(name, (unsigned int)addr, (unsigned int)rxne, I2C_SCL_PERIODS_PER_BYTE_FRAME);
	}
	return ok;
}

static int sb_expect_af_sequence(const char *name, const I2cMonitor *m, unsigned int first_irq, unsigned int first_mask)
{
	int ok = 1;
	int first = i2c_find(m->from, first_irq, first_mask);
	int af = i2c_find(m->from, IRQ_I2C0_ER, SR1_AF);
	ok &= expect_true(name, first >= 0, "missing precondition event");
	ok &= expect_true(name, af >= 0, "missing AF event");
	if (ok) {
		ok &= expect_true(name, first < af, "precondition must precede AF");
		ok &= expect_irq_gap_at_least(name, (unsigned int)first, (unsigned int)af, I2C_SCL_PERIODS_PER_BYTE_FRAME);
	}
	return ok;
}

static int sb_expect_10bit_prefix_nack_sequence(const char *name, const I2cMonitor *m)
{
	int ok = 1;
	int sb = i2c_find(m->from, IRQ_I2C0_EV, SR1_SB);
	int add10 = i2c_find(m->from, IRQ_I2C0_EV, SR1_ADD10);
	int af = i2c_find(m->from, IRQ_I2C0_ER, SR1_AF);
	ok &= expect_true(name, sb >= 0, "missing SB event");
	ok &= expect_true(name, add10 < 0, "ADD10 must not be set after prefix NACK");
	ok &= expect_true(name, af >= 0, "missing AF event");
	if (ok) {
		ok &= expect_true(name, sb < af, "SB must precede AF");
		ok &= expect_irq_gap_at_least(name, (unsigned int)sb, (unsigned int)af, I2C_SCL_PERIODS_PER_BYTE_FRAME);
	}
	return ok;
}

static int sb_expect_10bit_af_sequence(const char *name, const I2cMonitor *m)
{
	int ok = 1;
	int sb = i2c_find(m->from, IRQ_I2C0_EV, SR1_SB);
	int add10 = i2c_find(m->from, IRQ_I2C0_EV, SR1_ADD10);
	int af = i2c_find(m->from, IRQ_I2C0_ER, SR1_AF);
	ok &= expect_true(name, sb >= 0, "missing SB event");
	ok &= expect_true(name, add10 >= 0, "missing ADD10 event");
	ok &= expect_true(name, af >= 0, "missing AF event");
	if (ok) {
		ok &= expect_true(name, sb < add10, "SB must precede ADD10");
		ok &= expect_true(name, add10 < af, "ADD10 must precede AF");
		ok &= expect_irq_gap_at_least(name, (unsigned int)add10, (unsigned int)af, I2C_SCL_PERIODS_PER_BYTE_FRAME);
	}
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

extern void trap_entry(void);
void trap_handler(void);

static void setup_trap_handler(void)
{
	uintptr_t addr = (uintptr_t)(void *)trap_entry;
	__asm__ volatile("csrw mtvec, %0" :: "r"(addr));
}

static void setup_plic(void)
{
	PLIC_PRIO(IRQ_I2C0_EV) = 1u;
	PLIC_PRIO(IRQ_I2C0_ER) = 1u;
	PLIC_PRIO(IRQ_I2C1_EV) = 1u;
	PLIC_PRIO(IRQ_I2C1_ER) = 1u;
	PLIC_ENABLE_HART0(IRQ_I2C0_EV) = (1u << IRQ_I2C0_EV) | (1u << IRQ_I2C0_ER) | (1u << IRQ_I2C1_EV) |
	                                  (1u << IRQ_I2C1_ER);
	PLIC_THRESHOLD_HART0 = 0u;
}

static void enable_irq(void)
{
	__asm__ volatile("csrs mie, %0" :: "r"(1u << 11));
	__asm__ volatile("csrs mstatus, %0" :: "r"(1u << 3));
}

static void i2c_init(void)
{
	I2C0_CR1 = CR1_SWRST;
	I2C0_CR1 = 0u;
	I2C0_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C0_CCR = 4000u;
	I2C0_TRISE = 37u;
	I2C0_CR1 = CR1_PE;

	I2C1_CR1 = CR1_SWRST;
	I2C1_CR1 = 0u;
	I2C1_CR2 = 36u | CR2_ITEVTEN | CR2_ITBUFEN | CR2_ITERREN;
	I2C1_OAR1 = I2C0_SLAVE_ADDR << 1u;
	I2C1_OAR2 = 0u;
	I2C1_CCR = 4000u;
	I2C1_TRISE = 37u;
	I2C1_CR1 = CR1_PE | CR1_ACK;
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
	I2C1_OAR1 = (addr & 0x7Fu) << 1u;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
	if (!i2c_rx_byte(data)) return 0;
	i2c_stop();
	return 1;
}

static int i2c0_write_then_repeated_read(unsigned int addr, unsigned int write_data, unsigned int source, unsigned int *data)
{
	unsigned int base = g_log_count;
	I2C1_DR = source;
	i2c_start();
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
	i2c_write_byte(write_data);
	if (!i2c_wait_sr1(SR1_TxE | SR1_BTF)) return 0;
	if (!i2c1_wait_sr1(SR1_RxNE)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
	if (!i2c_rx_byte(data)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 0u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr(addr, 1u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr10_header(addr10, 0u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADD10)) return 0;
	i2c_write_byte(addr10 & 0xFFu);
	if (!i2c_wait_log(base, IRQ_I2C1_EV, SR1_ADDR)) return 0;
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
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr10_header(addr10, 0u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADD10)) return 0;
	i2c_write_byte(addr10 & 0xFFu);
	if (!i2c_wait_log(base, IRQ_I2C1_EV, SR1_ADDR)) return 0;

	base = g_log_count;
	i2c_start();
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
	i2c_write_addr10_header(addr10, 1u);
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_ADDR)) return 0;
	if (!i2c_rx_byte(data)) return 0;
	i2c_stop();
	return 1;
}

static int start_addr_write(unsigned int addr)
{
	unsigned int base = g_log_count;
	i2c_start();
	if (!i2c_wait_log(base, IRQ_I2C0_EV, SR1_SB)) return 0;
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
			g_log[idx].tick = TIMER_CNT;
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
			g_log[idx].tick = TIMER_CNT;
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("REG-004", "SB IRQ log timeout");
		return;
	}
	if (!expect_true("REG-004", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "SB IRQ missing before PE clear")) return;
	I2C0_CR1 = 0u;
	if (!expect_eq("REG-004.CR1", I2C0_CR1, 0u)) return;
	if (!expect_eq("REG-004.SR1", I2C0_SR1, 0u)) return;
	if (!expect_eq("REG-004.SR2", I2C0_SR2, 0u)) return;
	if (!expect_eq("REG-004.LOG", g_log_count, mon.from + 1u)) return;
	pass_test("REG-004: PE clear resets active transaction state");
}

static void test_irq_semantics(void)
{
	put_str("\r\n--- I2C Interrupt Semantics Test ---\r\n");
	I2cMonitor mon;
	monitor_begin(&mon);
	I2C0_CR2 = 36u | CR2_ITEVTEN | CR2_ITERREN;
	i2c_start();
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("IRQ-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("IRQ-003", "SB IRQ log timeout");
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
	if (!expect_true("ADDR-004", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
		if (!expect_true("ADDR-004", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ADDR-004", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ADDR-004", i2c_count(mon.from, IRQ_I2C1_EV, SR1_TxE) > 0, "slave TxE event missing")) return;
	if (!expect_true("ADDR-004", i2c_count(mon.from, IRQ_I2C1_EV, SR1_BTF) > 0, "slave BTF event missing")) return;
	pass_test("ADDR-004: 10-bit addressed read with repeated START");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_7bit(I2C1_SLAVE_ADDR, CR1_ACK | CR1_ENGC);
	i2c_start();
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ADDR-005", "SB timeout");
		return;
	}
	i2c_write_addr(0u, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ADDR-006", "SB timeout");
		return;
	}
	i2c_write_addr10_header(0x1A5u, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_AF)) {
		fail_test("ADDR-006", "AF timeout");
		return;
	}
	if (!expect_true("ADDR-006", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing for high-prefix mismatch")) return;
	if (!expect_eq("ADDR-006.SLAVE_RX", I2C1_SR1 & SR1_RxNE, 0u)) return;
	if (!sb_expect_10bit_prefix_nack_sequence("ADDR-006", &mon)) return;
	pass_test("ADDR-006: 10-bit high-prefix mismatch rejects address");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_10bit(0x2A5u);
	i2c_start();
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ADDR-007", "SB timeout");
		return;
	}
	i2c_write_addr10_header(0x2A5u, 0u);
	if (!i2c_wait_sr1(SR1_ADD10)) {
		fail_test("ADDR-007", "ADD10 timeout");
		return;
	}
	I2C0_DR = 0xA4u;
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_AF)) {
		fail_test("ADDR-007", "AF timeout");
		return;
	}
	if (!expect_true("ADDR-007", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing for low-byte mismatch")) return;
	if (!expect_eq("ADDR-007.SLAVE_RX", I2C1_SR1 & SR1_RxNE, 0u)) return;
	if (!sb_expect_10bit_af_sequence("ADDR-007", &mon)) return;
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("PE-002", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
		fail_test("PE-002", "ADDR IRQ timeout");
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_AF)) {
		fail_test("ERR-001", "AF IRQ timeout");
		return;
	}
	if (!expect_true("ERR-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERR-001", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "AF missing")) return;
	if (!sb_expect_af_sequence("ERR-001", &mon, IRQ_I2C0_EV, SR1_SB)) return;
	pass_test("ERR-001: address NACK reports AF");

	i2c_init();
	i2c_clear_log();
	monitor_begin(&mon);
	i2c1_config_7bit(I2C1_SLAVE_ADDR, 0u);
	if (!start_addr_write(I2C1_SLAVE_ADDR)) {
		fail_test("ERR-002", "address phase timeout");
		return;
	}
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
		fail_test("ERR-002", "ADDR timeout");
		return;
	}
	I2C0_DR = 0xD1u;
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_AF)) {
		fail_test("ERR-002", "data AF IRQ timeout");
		return;
	}
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ERR-002", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ERR-002", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "master AF missing")) return;
	if (!sb_expect_af_sequence("ERR-002", &mon, IRQ_I2C0_EV, SR1_ADDR)) return;
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ERRX-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C1_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C1_ER, SR1_OVR)) {
		fail_test("ERRX-001", "overrun IRQ timeout");
		return;
	}
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_AF)) {
		fail_test("ERRX-001", "master AF timeout");
		return;
	}
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_SB) >= 0, "master SB event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR) >= 0, "master ADDR event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_ADDR) >= 0, "slave ADDR event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE) >= 0, "slave RxNE event missing")) return;
	if (!expect_true("ERRX-001", i2c_find(0u, IRQ_I2C1_ER, SR1_OVR) >= 0, "OVR IRQ missing")) return;
	if (!expect_true("ERRX-001", i2c_find(0u, IRQ_I2C0_ER, SR1_AF) >= 0, "master AF missing")) return;
	if (!expect_irq_gap_at_least("ERRX-001", (unsigned int)i2c_find(mon.from, IRQ_I2C0_EV, SR1_ADDR), (unsigned int)i2c_find(mon.from, IRQ_I2C0_ER, SR1_AF), I2C_SCL_PERIODS_PER_BYTE_FRAME)) return;
	if (!expect_irq_gap_at_least("ERRX-001", (unsigned int)i2c_find(mon.from, IRQ_I2C1_EV, SR1_RxNE), (unsigned int)i2c_find(0u, IRQ_I2C1_ER, SR1_OVR), I2C_SCL_PERIODS_PER_BYTE_FRAME)) return;
	pass_test("ERRX-001: slave overrun reports OVR and master AF");

	i2c_init();
	monitor_begin(&mon);
	i2c_start();
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ERRX-002", "SB timeout");
		return;
	}
	I2C1_CR1 |= CR1_START;
	if (!i2c_wait_log(mon.from, IRQ_I2C0_ER, SR1_ARLO)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ERRX-003", "SB timeout");
		return;
	}
	i2c_write_addr(I2C1_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ROB-001", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_SB)) {
		fail_test("ROB-003", "SB timeout");
		return;
	}
	i2c_write_addr(I2C0_SLAVE_ADDR, 0u);
	if (!i2c_wait_log(mon.from, IRQ_I2C0_EV, SR1_ADDR)) {
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
