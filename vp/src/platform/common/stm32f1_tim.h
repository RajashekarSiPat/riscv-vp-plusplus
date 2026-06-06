#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Tim : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Tim> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Tim(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Tim::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_cr1 = 0u;
		m_cr2 = 0u;
		m_smcr = 0u;
		m_dier = 0u;
		m_sr = 0u;
		m_egr = 0u;
		m_ccmr1 = 0u;
		m_ccmr2 = 0u;
		m_ccer = 0u;
		m_cnt = 0u;
		m_psc = 0u;
		m_arr = 0xFFFFu;
		m_ccr1 = 0u;
		m_ccr2 = 0u;
		m_ccr3 = 0u;
		m_ccr4 = 0u;
		m_dcr = 0u;
		m_dmar = 0u;
		m_psc_counter = 0u;
	}

private:
	static constexpr uint64_t OFF_CR1 = 0x00u;
	static constexpr uint64_t OFF_CR2 = 0x04u;
	static constexpr uint64_t OFF_SMCR = 0x08u;
	static constexpr uint64_t OFF_DIER = 0x0Cu;
	static constexpr uint64_t OFF_SR = 0x10u;
	static constexpr uint64_t OFF_EGR = 0x14u;
	static constexpr uint64_t OFF_CCMR1 = 0x18u;
	static constexpr uint64_t OFF_CCMR2 = 0x1Cu;
	static constexpr uint64_t OFF_CCER = 0x20u;
	static constexpr uint64_t OFF_CNT = 0x24u;
	static constexpr uint64_t OFF_PSC = 0x28u;
	static constexpr uint64_t OFF_ARR = 0x2Cu;
	static constexpr uint64_t OFF_CCR1 = 0x34u;
	static constexpr uint64_t OFF_CCR2 = 0x38u;
	static constexpr uint64_t OFF_CCR3 = 0x3Cu;
	static constexpr uint64_t OFF_CCR4 = 0x40u;
	static constexpr uint64_t OFF_DCR = 0x48u;
	static constexpr uint64_t OFF_DMAR = 0x4Cu;

	static constexpr uint32_t CR1_CEN = 1u << 0;
	static constexpr uint32_t CR1_UDIS = 1u << 1;
	static constexpr uint32_t CR1_URS = 1u << 2;
	static constexpr uint32_t CR1_OPM = 1u << 3;
	static constexpr uint32_t CR1_DIR = 1u << 4;
	static constexpr uint32_t CR1_CMS_MASK = 0x3u << 5;
	static constexpr uint32_t CR1_ARPE = 1u << 7;
	static constexpr uint32_t CR1_CKD_MASK = 0x3u << 8;
	static constexpr uint32_t CR1_RW_MASK = CR1_CEN | CR1_UDIS | CR1_URS | CR1_OPM | CR1_DIR |
	                                         CR1_CMS_MASK | CR1_ARPE | CR1_CKD_MASK;

	static constexpr uint32_t DIER_UIE = 1u << 0;
	static constexpr uint32_t DIER_RW_MASK = DIER_UIE;

	static constexpr uint32_t SR_UIF = 1u << 0;
	static constexpr uint32_t SR_RW_MASK = SR_UIF;

	static constexpr uint32_t EGR_UG = 1u << 0;
	static constexpr uint32_t EGR_RW_MASK = EGR_UG;

	static constexpr uint32_t PSC_RW_MASK = 0xFFFFu;
	static constexpr uint32_t ARR_RW_MASK = 0xFFFFFFFFu;

	uint32_t m_cr1 = 0u;
	uint32_t m_cr2 = 0u;
	uint32_t m_smcr = 0u;
	uint32_t m_dier = 0u;
	uint32_t m_sr = 0u;
	uint32_t m_egr = 0u;
	uint32_t m_ccmr1 = 0u;
	uint32_t m_ccmr2 = 0u;
	uint32_t m_ccer = 0u;
	uint32_t m_cnt = 0u;
	uint32_t m_psc = 0u;
	uint32_t m_arr = 0u;
	uint32_t m_ccr1 = 0u;
	uint32_t m_ccr2 = 0u;
	uint32_t m_ccr3 = 0u;
	uint32_t m_ccr4 = 0u;
	uint32_t m_dcr = 0u;
	uint32_t m_dmar = 0u;
	uint32_t m_psc_counter = 0u;
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

	void generate_update_event() {
		if ((m_cr1 & CR1_UDIS) != 0u) {
			return;
		}
		m_sr |= SR_UIF;
		if ((m_dier & DIER_UIE) != 0u) {
			trigger_irq();
		}
		if ((m_cr1 & CR1_OPM) != 0u) {
			m_cr1 &= ~CR1_CEN;
		}
	}

	void step_tick() {
		if (!m_clock_enabled || (m_cr1 & CR1_CEN) == 0u) {
			return;
		}
		if (++m_psc_counter <= m_psc) {
			return;
		}
		m_psc_counter = 0u;
		++m_cnt;
		if (m_cnt > m_arr) {
			m_cnt = 0u;
			generate_update_event();
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

		step_tick();

		switch (trans.get_address()) {
		case OFF_CR1:
			if (write) m_cr1 = value & CR1_RW_MASK;
			result = m_cr1;
			break;
		case OFF_CR2:
			if (write) m_cr2 = value;
			result = m_cr2;
			break;
		case OFF_SMCR:
			if (write) m_smcr = value;
			result = m_smcr;
			break;
		case OFF_DIER:
			if (write) m_dier = value & DIER_RW_MASK;
			result = m_dier;
			break;
		case OFF_SR:
			if (write) m_sr &= ~value;
			result = m_sr;
			break;
		case OFF_EGR:
			if (write) {
				m_egr = value & EGR_RW_MASK;
				if ((value & EGR_UG) != 0u) {
					m_psc_counter = 0u;
					m_cnt = 0u;
					generate_update_event();
				}
			}
			result = m_egr;
			break;
		case OFF_CCMR1:
			if (write) m_ccmr1 = value;
			result = m_ccmr1;
			break;
		case OFF_CCMR2:
			if (write) m_ccmr2 = value;
			result = m_ccmr2;
			break;
		case OFF_CCER:
			if (write) m_ccer = value;
			result = m_ccer;
			break;
		case OFF_CNT:
			if (write) m_cnt = value;
			result = m_cnt;
			break;
		case OFF_PSC:
			if (write) m_psc = value & PSC_RW_MASK;
			result = m_psc;
			break;
		case OFF_ARR:
			if (write) m_arr = value;
			result = m_arr;
			break;
		case OFF_CCR1:
			if (write) m_ccr1 = value;
			result = m_ccr1;
			break;
		case OFF_CCR2:
			if (write) m_ccr2 = value;
			result = m_ccr2;
			break;
		case OFF_CCR3:
			if (write) m_ccr3 = value;
			result = m_ccr3;
			break;
		case OFF_CCR4:
			if (write) m_ccr4 = value;
			result = m_ccr4;
			break;
		case OFF_DCR:
			if (write) m_dcr = value;
			result = m_dcr;
			break;
		case OFF_DMAR:
			if (write) m_dmar = value;
			result = m_dmar;
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
