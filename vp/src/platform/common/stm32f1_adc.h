#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Adc : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Adc> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;
	unsigned instance_index = 0u;

	explicit Stm32f1Adc(sc_core::sc_module_name name, unsigned index = 0u)
	    : sc_module(name), instance_index(index) {
		socket.register_b_transport(this, &Stm32f1Adc::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_sr = 0u;
		m_cr1 = 0u;
		m_cr2 = 0u;
		m_smpr1 = 0u;
		m_smpr2 = 0u;
		m_jofr1 = 0u;
		m_jofr2 = 0u;
		m_jofr3 = 0u;
		m_jofr4 = 0u;
		m_htr = 0u;
		m_ltr = 0u;
		m_sqr1 = 0u;
		m_sqr2 = 0u;
		m_sqr3 = 0u;
		m_jsqr = 0u;
		m_jdr1 = 0u;
		m_jdr2 = 0u;
		m_jdr3 = 0u;
		m_jdr4 = 0u;
		m_dr = 0u;
	}

private:
	static constexpr uint64_t OFF_SR = 0x00u;
	static constexpr uint64_t OFF_CR1 = 0x04u;
	static constexpr uint64_t OFF_CR2 = 0x08u;
	static constexpr uint64_t OFF_SMPR1 = 0x0Cu;
	static constexpr uint64_t OFF_SMPR2 = 0x10u;
	static constexpr uint64_t OFF_JOFR1 = 0x14u;
	static constexpr uint64_t OFF_JOFR2 = 0x18u;
	static constexpr uint64_t OFF_JOFR3 = 0x1Cu;
	static constexpr uint64_t OFF_JOFR4 = 0x20u;
	static constexpr uint64_t OFF_HTR = 0x24u;
	static constexpr uint64_t OFF_LTR = 0x28u;
	static constexpr uint64_t OFF_SQR1 = 0x2Cu;
	static constexpr uint64_t OFF_SQR2 = 0x30u;
	static constexpr uint64_t OFF_SQR3 = 0x34u;
	static constexpr uint64_t OFF_JSQR = 0x38u;
	static constexpr uint64_t OFF_JDR1 = 0x3Cu;
	static constexpr uint64_t OFF_JDR2 = 0x40u;
	static constexpr uint64_t OFF_JDR3 = 0x44u;
	static constexpr uint64_t OFF_JDR4 = 0x48u;
	static constexpr uint64_t OFF_DR = 0x4Cu;

	static constexpr uint32_t SR_AWD = 1u << 0;
	static constexpr uint32_t SR_EOC = 1u << 1;
	static constexpr uint32_t SR_JEOC = 1u << 2;
	static constexpr uint32_t SR_JSTRT = 1u << 3;
	static constexpr uint32_t SR_STRT = 1u << 4;
	static constexpr uint32_t CR1_AWDCH_MASK = 0x1Fu;
	static constexpr uint32_t CR1_EOCIE = 1u << 5;
	static constexpr uint32_t CR1_AWDIE = 1u << 6;
	static constexpr uint32_t CR1_JEOCIE = 1u << 7;
	static constexpr uint32_t CR1_SCAN = 1u << 8;
	static constexpr uint32_t CR1_AWDSGL = 1u << 9;
	static constexpr uint32_t CR1_JAUTO = 1u << 10;
	static constexpr uint32_t CR1_DISCEN = 1u << 11;
	static constexpr uint32_t CR1_JDISCEN = 1u << 12;
	static constexpr uint32_t CR1_DUALMOD_MASK = 0xFu << 16;
	static constexpr uint32_t CR1_RW_MASK = 0x00FFFFFFu;

	static constexpr uint32_t CR2_ADON = 1u << 0;
	static constexpr uint32_t CR2_CONT = 1u << 1;
	static constexpr uint32_t CR2_CAL = 1u << 2;
	static constexpr uint32_t CR2_RSTCAL = 1u << 3;
	static constexpr uint32_t CR2_DMA = 1u << 8;
	static constexpr uint32_t CR2_ALIGN = 1u << 11;
	static constexpr uint32_t CR2_JEXTTRIG = 1u << 15;
	static constexpr uint32_t CR2_EXTTRIG = 1u << 20;
	static constexpr uint32_t CR2_JSWSTART = 1u << 21;
	static constexpr uint32_t CR2_SWSTART = 1u << 22;
	static constexpr uint32_t CR2_TSVREFE = 1u << 23;
	static constexpr uint32_t CR2_RW_MASK = 0x00FFFFFFu;

	uint32_t m_sr = 0u;
	uint32_t m_cr1 = 0u;
	uint32_t m_cr2 = 0u;
	uint32_t m_smpr1 = 0u;
	uint32_t m_smpr2 = 0u;
	uint32_t m_jofr1 = 0u;
	uint32_t m_jofr2 = 0u;
	uint32_t m_jofr3 = 0u;
	uint32_t m_jofr4 = 0u;
	uint32_t m_htr = 0u;
	uint32_t m_ltr = 0u;
	uint32_t m_sqr1 = 0u;
	uint32_t m_sqr2 = 0u;
	uint32_t m_sqr3 = 0u;
	uint32_t m_jsqr = 0u;
	uint32_t m_jdr1 = 0u;
	uint32_t m_jdr2 = 0u;
	uint32_t m_jdr3 = 0u;
	uint32_t m_jdr4 = 0u;
	uint32_t m_dr = 0u;
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

	uint32_t conversion_value() const {
		const uint32_t channel = m_sqr3 & 0x1Fu;
		return (((instance_index + 1u) & 0x3u) << 8u) | (channel & 0xFFu);
	}

	void start_conversion() {
		m_sr |= SR_STRT;
		m_dr = conversion_value();
		m_sr |= SR_EOC;
		if ((m_cr1 & CR1_EOCIE) != 0u) {
			trigger_irq();
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

		switch (trans.get_address()) {
		case OFF_SR:
			if (write) {
				m_sr &= ~value;
			}
			result = m_sr;
			break;
		case OFF_CR1:
			if (write) {
				m_cr1 = value & CR1_RW_MASK;
			}
			result = m_cr1;
			break;
		case OFF_CR2: {
			if (write) {
				const bool was_on = (m_cr2 & CR2_ADON) != 0u;
				m_cr2 = value & CR2_RW_MASK;
				if (((m_cr2 & CR2_ADON) != 0u && was_on) || (m_cr2 & CR2_SWSTART) != 0u) {
					start_conversion();
				}
			}
			result = m_cr2;
			break;
		}
		case OFF_SMPR1:
			if (write) m_smpr1 = value;
			result = m_smpr1;
			break;
		case OFF_SMPR2:
			if (write) m_smpr2 = value;
			result = m_smpr2;
			break;
		case OFF_JOFR1:
			if (write) m_jofr1 = value;
			result = m_jofr1;
			break;
		case OFF_JOFR2:
			if (write) m_jofr2 = value;
			result = m_jofr2;
			break;
		case OFF_JOFR3:
			if (write) m_jofr3 = value;
			result = m_jofr3;
			break;
		case OFF_JOFR4:
			if (write) m_jofr4 = value;
			result = m_jofr4;
			break;
		case OFF_HTR:
			if (write) m_htr = value;
			result = m_htr;
			break;
		case OFF_LTR:
			if (write) m_ltr = value;
			result = m_ltr;
			break;
		case OFF_SQR1:
			if (write) m_sqr1 = value;
			result = m_sqr1;
			break;
		case OFF_SQR2:
			if (write) m_sqr2 = value;
			result = m_sqr2;
			break;
		case OFF_SQR3:
			if (write) m_sqr3 = value;
			result = m_sqr3;
			break;
		case OFF_JSQR:
			if (write) m_jsqr = value;
			result = m_jsqr;
			break;
		case OFF_JDR1:
			if (write) m_jdr1 = value;
			result = m_jdr1;
			break;
		case OFF_JDR2:
			if (write) m_jdr2 = value;
			result = m_jdr2;
			break;
		case OFF_JDR3:
			if (write) m_jdr3 = value;
			result = m_jdr3;
			break;
		case OFF_JDR4:
			if (write) m_jdr4 = value;
			result = m_jdr4;
			break;
		case OFF_DR:
			if (write) {
				m_dr = value & 0x0FFFu;
			} else {
				result = m_dr & 0x0FFFu;
				m_sr &= ~SR_EOC;
			}
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
