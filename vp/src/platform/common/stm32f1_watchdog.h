#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Wwdg : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Wwdg> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Wwdg(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Wwdg::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_cr = 0x7Fu;
		m_cfr = 0u;
		m_sr = 0u;
		m_counter = 0x7Fu;
	}

private:
	static constexpr uint64_t OFF_CR = 0x00u;
	static constexpr uint64_t OFF_CFR = 0x04u;
	static constexpr uint64_t OFF_SR = 0x08u;

	static constexpr uint32_t CR_T_MASK = 0x7Fu;
	static constexpr uint32_t CR_WDGA = 1u << 7;
	static constexpr uint32_t CFR_W_MASK = 0x7Fu;
	static constexpr uint32_t CFR_WDGTB_MASK = 0x3u << 7;
	static constexpr uint32_t CFR_EWI = 1u << 9;
	static constexpr uint32_t SR_EWIF = 1u << 0;

	uint32_t m_cr = 0u;
	uint32_t m_cfr = 0u;
	uint32_t m_sr = 0u;
	uint32_t m_counter = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger_irq() {
		if (plic != nullptr && irq_id != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void tick() {
		if (!m_clock_enabled || (m_cr & CR_WDGA) == 0u) {
			return;
		}
		if (m_counter > 0x40u) {
			--m_counter;
		} else {
			m_sr |= SR_EWIF;
			if ((m_cfr & CFR_EWI) != 0u) {
				trigger_irq();
			}
		}
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		if (!m_clock_enabled) {
			if (!write) {
				write_word(trans, 0u);
			}
			trans.set_response_status(tlm::TLM_OK_RESPONSE);
			return;
		}

		tick();

		switch (trans.get_address()) {
		case OFF_CR:
			if (write) {
				m_cr = value & (CR_T_MASK | CR_WDGA);
				m_counter = m_cr & CR_T_MASK;
			}
			result = m_cr;
			break;
		case OFF_CFR:
			if (write) {
				m_cfr = value & (CFR_W_MASK | CFR_WDGTB_MASK | CFR_EWI);
			}
			result = m_cfr;
			break;
		case OFF_SR:
			if (write) {
				m_sr &= ~value;
			}
			result = m_sr;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};

class Stm32f1Iwdg : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Iwdg> socket{"target_socket"};

	explicit Stm32f1Iwdg(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Iwdg::b_transport);
		reset();
	}

	void peripheral_reset() { reset(); }

	void reset() {
		m_kr = 0x00000000u;
		m_pr = 0u;
		m_rlr = 0x0FFFu;
		m_sr = 0u;
		m_enabled = false;
		m_unlocked = false;
		m_counter = m_rlr & 0x0FFFu;
	}

private:
	static constexpr uint64_t OFF_KR = 0x00u;
	static constexpr uint64_t OFF_PR = 0x04u;
	static constexpr uint64_t OFF_RLR = 0x08u;
	static constexpr uint64_t OFF_SR = 0x0Cu;

	static constexpr uint32_t KR_START = 0xCCCCu;
	static constexpr uint32_t KR_RELOAD = 0xAAAAu;
	static constexpr uint32_t KR_UNLOCK = 0x5555u;
	static constexpr uint32_t PR_MASK = 0x7u;
	static constexpr uint32_t RLR_MASK = 0x0FFFu;

	uint32_t m_kr = 0u;
	uint32_t m_pr = 0u;
	uint32_t m_rlr = 0u;
	uint32_t m_sr = 0u;
	uint32_t m_counter = 0u;
	bool m_enabled = false;
	bool m_unlocked = false;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void tick() {
		if (!m_enabled || m_counter == 0u) {
			return;
		}
		--m_counter;
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		tick();

		switch (trans.get_address()) {
		case OFF_KR:
			if (write) {
				m_kr = value;
				if (value == KR_UNLOCK) {
					m_unlocked = true;
				} else if (value == KR_RELOAD) {
					m_counter = m_rlr & RLR_MASK;
				} else if (value == KR_START) {
					m_enabled = true;
					m_counter = m_rlr & RLR_MASK;
				}
			}
			result = m_kr;
			break;
		case OFF_PR:
			if (write && m_unlocked) {
				m_pr = value & PR_MASK;
			}
			result = m_pr;
			break;
		case OFF_RLR:
			if (write && m_unlocked) {
				m_rlr = value & RLR_MASK;
			}
			result = m_rlr;
			break;
		case OFF_SR:
			if (write) {
				m_sr &= ~value;
			}
			result = m_sr;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
