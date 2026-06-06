#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <systemc>
#include <tlm_utils/simple_target_socket.h>
#include <utility>

class Stm32f1Rcc : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Rcc> socket{"target_socket"};

	explicit Stm32f1Rcc(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Rcc::b_transport);
		reset();
	}

	void bind_apb1(unsigned bit, std::function<void(bool)> clock, std::function<void()> reset_cb) {
		if (bit < m_apb1.size()) {
			m_apb1[bit] = {std::move(clock), std::move(reset_cb)};
			if (m_apb1[bit].clock) {
				m_apb1[bit].clock((m_apb1enr & (1u << bit)) != 0u);
			}
		}
	}

	void bind_apb2(unsigned bit, std::function<void(bool)> clock, std::function<void()> reset_cb) {
		if (bit < m_apb2.size()) {
			m_apb2[bit] = {std::move(clock), std::move(reset_cb)};
			if (m_apb2[bit].clock) {
				m_apb2[bit].clock((m_apb2enr & (1u << bit)) != 0u);
			}
		}
	}

	void bind_ahb(unsigned bit, std::function<void(bool)> clock, std::function<void()> reset_cb) {
		if (bit < m_ahb.size()) {
			m_ahb[bit] = {std::move(clock), std::move(reset_cb)};
			if (m_ahb[bit].clock) {
				m_ahb[bit].clock((m_ahbenr & (1u << bit)) != 0u);
			}
		}
	}

	void bind_bdcr(std::function<void(bool)> rtc_enable, std::function<void()> backup_reset) {
		m_bdcr_rtc_enable = std::move(rtc_enable);
		m_bdcr_backup_reset = std::move(backup_reset);
		if (m_bdcr_rtc_enable) {
			m_bdcr_rtc_enable((m_bdcr & (1u << 15)) != 0u);
		}
	}

	void reset() {
		m_cr = 0x00000083u;
		m_cfgr = 0u;
		m_cir = 0u;
		m_apb2rstr = 0u;
		m_apb1rstr = 0u;
		m_ahbenr = 0x00000014u;
		m_apb2enr = 0u;
		m_apb1enr = 0u;
		m_bdcr = 0u;
		m_csr = 0x0C000000u;
		m_ahbrstr = 0u;
		m_cfgr2 = 0u;
		notify_clocks(m_apb1, 0u);
		notify_clocks(m_apb2, 0u);
		notify_clocks(m_ahb, 0u);
		if (m_bdcr_rtc_enable) m_bdcr_rtc_enable(false);
	}

private:
	struct PeripheralBinding {
		std::function<void(bool)> clock;
		std::function<void()> reset;
	};

	static constexpr uint64_t OFF_CR = 0x00u;
	static constexpr uint64_t OFF_CFGR = 0x04u;
	static constexpr uint64_t OFF_CIR = 0x08u;
	static constexpr uint64_t OFF_APB2RSTR = 0x0Cu;
	static constexpr uint64_t OFF_APB1RSTR = 0x10u;
	static constexpr uint64_t OFF_AHBENR = 0x14u;
	static constexpr uint64_t OFF_APB2ENR = 0x18u;
	static constexpr uint64_t OFF_APB1ENR = 0x1Cu;
	static constexpr uint64_t OFF_BDCR = 0x20u;
	static constexpr uint64_t OFF_CSR = 0x24u;
	static constexpr uint64_t OFF_AHBRSTR = 0x28u;
	static constexpr uint64_t OFF_CFGR2 = 0x2Cu;

	static constexpr uint32_t CR_HSION = 1u << 0;
	static constexpr uint32_t CR_HSIRDY = 1u << 1;
	static constexpr uint32_t CR_HSEON = 1u << 16;
	static constexpr uint32_t CR_HSERDY = 1u << 17;
	static constexpr uint32_t CR_PLLON = 1u << 24;
	static constexpr uint32_t CR_PLLRDY = 1u << 25;
	static constexpr uint32_t CR_RW_MASK =
	    CR_HSION | (0x1Fu << 3) | CR_HSEON | (1u << 18) | (1u << 19) | CR_PLLON;
	static constexpr uint32_t CFGR_RW_MASK =
	    0x3u | (0xFu << 4) | (0x7u << 8) | (0x7u << 11) | (0x3u << 14) | (0x7Fu << 16) |
	    (0x7u << 24);
	static constexpr uint32_t APB2_MASK = 0x0038FFFDu;
	static constexpr uint32_t APB1_MASK = 0x3AFEC9FFu | (1u << 11);
	static constexpr uint32_t AHB_MASK = 0x00000557u;
	static constexpr uint32_t BDCR_RW_MASK = 0x00018305u;
	static constexpr uint32_t CSR_FLAGS = 0xFC000000u;

	std::array<PeripheralBinding, 32> m_apb1{};
	std::array<PeripheralBinding, 32> m_apb2{};
	std::array<PeripheralBinding, 32> m_ahb{};
	std::function<void(bool)> m_bdcr_rtc_enable;
	std::function<void()> m_bdcr_backup_reset;
	uint32_t m_cr = 0u;
	uint32_t m_cfgr = 0u;
	uint32_t m_cir = 0u;
	uint32_t m_apb2rstr = 0u;
	uint32_t m_apb1rstr = 0u;
	uint32_t m_ahbenr = 0u;
	uint32_t m_apb2enr = 0u;
	uint32_t m_apb1enr = 0u;
	uint32_t m_bdcr = 0u;
	uint32_t m_csr = 0u;
	uint32_t m_ahbrstr = 0u;
	uint32_t m_cfgr2 = 0u;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	static void notify_clocks(std::array<PeripheralBinding, 32> &bindings, uint32_t value) {
		for (unsigned bit = 0; bit < bindings.size(); ++bit) {
			if (bindings[bit].clock) {
				bindings[bit].clock((value & (1u << bit)) != 0u);
			}
		}
	}

	static void notify_resets(std::array<PeripheralBinding, 32> &bindings, uint32_t old_value,
	                          uint32_t new_value) {
		const uint32_t asserted = new_value & ~old_value;
		for (unsigned bit = 0; bit < bindings.size(); ++bit) {
			if ((asserted & (1u << bit)) != 0u && bindings[bit].reset) {
				bindings[bit].reset();
			}
		}
	}

	void write_cr(uint32_t value) {
		const uint32_t old_ready = m_cr & (CR_HSIRDY | CR_HSERDY | CR_PLLRDY);
		m_cr = (m_cr & ~CR_RW_MASK) | (value & CR_RW_MASK);
		m_cr = (m_cr & ~(CR_HSIRDY | CR_HSERDY | CR_PLLRDY)) |
		       ((m_cr & CR_HSION) ? CR_HSIRDY : 0u) | ((m_cr & CR_HSEON) ? CR_HSERDY : 0u) |
		       ((m_cr & CR_PLLON) ? CR_PLLRDY : 0u);
		const uint32_t new_ready = m_cr & (CR_HSIRDY | CR_HSERDY | CR_PLLRDY);
		if ((new_ready & CR_HSIRDY) != (old_ready & CR_HSIRDY)) m_cir |= 1u << 2;
		if ((new_ready & CR_HSERDY) != (old_ready & CR_HSERDY)) m_cir |= 1u << 3;
		if ((new_ready & CR_PLLRDY) != (old_ready & CR_PLLRDY)) m_cir |= 1u << 4;
		update_clock_status();
	}

	void update_clock_status() {
		uint32_t source = m_cfgr & 0x3u;
		if ((source == 1u && (m_cr & CR_HSERDY) == 0u) ||
		    (source == 2u && (m_cr & CR_PLLRDY) == 0u) || source == 3u) {
			source = 0u;
		}
		m_cfgr = (m_cfgr & ~(0x3u << 2)) | (source << 2);
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		switch (trans.get_address()) {
		case OFF_CR:
			if (write) write_cr(value);
			result = m_cr;
			break;
		case OFF_CFGR:
			if (write) {
				m_cfgr = (m_cfgr & ~CFGR_RW_MASK) | (value & CFGR_RW_MASK);
				update_clock_status();
			}
			result = m_cfgr;
			break;
		case OFF_CIR:
			if (write) {
				m_cir = (m_cir & ~0x00001F00u) | (value & 0x00001F00u);
				m_cir &= ~((value >> 16) & 0x9Fu);
			}
			result = m_cir;
			break;
		case OFF_APB2RSTR:
			if (write) {
				const uint32_t old = m_apb2rstr;
				m_apb2rstr = value & APB2_MASK;
				notify_resets(m_apb2, old, m_apb2rstr);
			}
			result = m_apb2rstr;
			break;
		case OFF_APB1RSTR:
			if (write) {
				const uint32_t old = m_apb1rstr;
				m_apb1rstr = value & APB1_MASK;
				notify_resets(m_apb1, old, m_apb1rstr);
			}
			result = m_apb1rstr;
			break;
		case OFF_AHBENR:
			if (write) {
				m_ahbenr = value & AHB_MASK;
				notify_clocks(m_ahb, m_ahbenr);
			}
			result = m_ahbenr;
			break;
		case OFF_APB2ENR:
			if (write) {
				m_apb2enr = value & APB2_MASK;
				notify_clocks(m_apb2, m_apb2enr);
			}
			result = m_apb2enr;
			break;
		case OFF_APB1ENR:
			if (write) {
				m_apb1enr = value & APB1_MASK;
				notify_clocks(m_apb1, m_apb1enr);
			}
			result = m_apb1enr;
			break;
		case OFF_BDCR:
			if (write) {
				const uint32_t old = m_bdcr;
				m_bdcr = value & BDCR_RW_MASK;
				if ((m_bdcr & 1u) != 0u) m_bdcr |= 1u << 1;
				if ((m_bdcr & (1u << 16)) != 0u) m_bdcr &= 0x00010000u;
				if (m_bdcr_rtc_enable) {
					const bool old_en = (old & (1u << 15)) != 0u;
					const bool new_en = (m_bdcr & (1u << 15)) != 0u;
					if (old_en != new_en) m_bdcr_rtc_enable(new_en);
				}
				if (((old ^ m_bdcr) & (1u << 16)) != 0u && (m_bdcr & (1u << 16)) != 0u && m_bdcr_backup_reset) {
					m_bdcr_backup_reset();
					m_bdcr &= ~(1u << 16);
				}
			}
			result = m_bdcr;
			break;
		case OFF_CSR:
			if (write) {
				if ((value & (1u << 24)) != 0u) m_csr &= ~CSR_FLAGS;
				m_csr = (m_csr & ~1u) | (value & 1u);
				if ((m_csr & 1u) != 0u) m_csr |= 1u << 1;
				else m_csr &= ~(1u << 1);
			}
			result = m_csr;
			break;
		case OFF_AHBRSTR:
			if (write) m_ahbrstr = value & 0x00005457u;
			result = m_ahbrstr;
			break;
		case OFF_CFGR2:
			if (write) m_cfgr2 = value & 0x00077FFFu;
			result = m_cfgr2;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
