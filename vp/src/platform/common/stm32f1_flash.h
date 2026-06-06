#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Flash : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Flash> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Flash(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Flash::b_transport);
		reset();
	}

	void peripheral_reset() { reset(); }

	void reset() {
		m_acr = 0x00000030u;
		m_sr = 0u;
		m_cr = CR_LOCK;
		m_ar = 0u;
		m_obr = 0u;
		m_wrpr = 0xFFFFFFFFu;
		m_locked = true;
		m_opt_locked = true;
		m_key_stage = 0u;
		m_optkey_stage = 0u;
	}

private:
	static constexpr uint64_t OFF_ACR = 0x00u;
	static constexpr uint64_t OFF_KEYR = 0x04u;
	static constexpr uint64_t OFF_OPTKEYR = 0x08u;
	static constexpr uint64_t OFF_SR = 0x0Cu;
	static constexpr uint64_t OFF_CR = 0x10u;
	static constexpr uint64_t OFF_AR = 0x14u;
	static constexpr uint64_t OFF_OBR = 0x1Cu;
	static constexpr uint64_t OFF_WRPR = 0x20u;

	static constexpr uint32_t ACR_RW_MASK = 0x0000003Fu;
	static constexpr uint32_t SR_EOP = 1u << 0;
	static constexpr uint32_t SR_PGERR = 1u << 2;
	static constexpr uint32_t SR_WRPRTERR = 1u << 4;
	static constexpr uint32_t SR_BSY = 1u << 5;
	static constexpr uint32_t CR_PG = 1u << 0;
	static constexpr uint32_t CR_PER = 1u << 1;
	static constexpr uint32_t CR_MER = 1u << 2;
	static constexpr uint32_t CR_OPTPG = 1u << 4;
	static constexpr uint32_t CR_OPTER = 1u << 5;
	static constexpr uint32_t CR_STRT = 1u << 6;
	static constexpr uint32_t CR_LOCK = 1u << 7;
	static constexpr uint32_t CR_OPTWRE = 1u << 9;
	static constexpr uint32_t CR_OBL_LAUNCH = 1u << 13;
	static constexpr uint32_t CR_RW_MASK =
	    CR_PG | CR_PER | CR_MER | CR_OPTPG | CR_OPTER | CR_STRT | CR_LOCK | CR_OPTWRE | CR_OBL_LAUNCH;
	static constexpr uint32_t KEY1 = 0x45670123u;
	static constexpr uint32_t KEY2 = 0xCDEF89ABu;
	static constexpr uint32_t OPTKEY1 = 0x08192A3Bu;
	static constexpr uint32_t OPTKEY2 = 0x4C5D6E7Fu;

	uint32_t m_acr = 0u;
	uint32_t m_sr = 0u;
	uint32_t m_cr = 0u;
	uint32_t m_ar = 0u;
	uint32_t m_obr = 0u;
	uint32_t m_wrpr = 0u;
	bool m_locked = true;
	bool m_opt_locked = true;
	uint8_t m_key_stage = 0u;
	uint8_t m_optkey_stage = 0u;

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

	void complete_operation() {
		m_sr |= SR_EOP;
		trigger_irq();
	}

	void write_keyr(uint32_t value) {
		if (value == KEY1) {
			m_key_stage = 1u;
			return;
		}
		if (m_key_stage == 1u && value == KEY2) {
			m_locked = false;
			m_key_stage = 0u;
			m_cr &= ~CR_LOCK;
			return;
		}
		m_key_stage = 0u;
	}

	void write_optkeyr(uint32_t value) {
		if (value == OPTKEY1) {
			m_optkey_stage = 1u;
			return;
		}
		if (m_optkey_stage == 1u && value == OPTKEY2) {
			m_opt_locked = false;
			m_optkey_stage = 0u;
			m_cr |= CR_OPTWRE;
			return;
		}
		m_optkey_stage = 0u;
	}

	void write_cr(uint32_t value) {
		if (m_locked) {
			if ((value & CR_LOCK) != 0u) {
				m_cr = CR_LOCK;
			}
			return;
		}
		m_cr = value & CR_RW_MASK;
		if ((m_cr & CR_LOCK) != 0u) {
			m_locked = true;
		}
		complete_operation();
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
		case OFF_ACR:
			if (write) m_acr = value & ACR_RW_MASK;
			result = m_acr;
			break;
		case OFF_KEYR:
			if (write) write_keyr(value);
			result = 0u;
			break;
		case OFF_OPTKEYR:
			if (write) write_optkeyr(value);
			result = 0u;
			break;
		case OFF_SR:
			if (write) {
				m_sr &= ~value;
			}
			result = m_sr;
			break;
		case OFF_CR:
			if (write) write_cr(value);
			result = m_cr;
			break;
		case OFF_AR:
			if (write) m_ar = value;
			result = m_ar;
			break;
		case OFF_OBR:
			result = m_obr;
			break;
		case OFF_WRPR:
			if (write && (!m_opt_locked || (m_cr & CR_OPTWRE) != 0u)) m_wrpr = value;
			result = m_wrpr;
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
