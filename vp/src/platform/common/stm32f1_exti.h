#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"
#include "stm32f1_afio.h"

class Stm32f1Exti : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Exti> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	std::array<uint32_t, 16> irq_ids{};
	Stm32f1Afio *afio = nullptr;

	explicit Stm32f1Exti(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Exti::b_transport);
		reset();
		for (unsigned i = 0; i < irq_ids.size(); ++i) {
			irq_ids[i] = 5u + i;
		}
	}

	void reset() {
		m_imr = 0u;
		m_emr = 0u;
		m_rtsr = 0u;
		m_ftsr = 0u;
		m_swier = 0u;
		m_pr = 0u;
		m_line_level.fill(false);
	}

	void signal_gpio(unsigned port, unsigned pin, bool level) {
		const unsigned line = pin;
		if (line >= 16u || afio == nullptr) {
			return;
		}
		if (afio->exti_port(line) != port) {
			return;
		}

		const bool prev = m_line_level[line];
		m_line_level[line] = level;
		if (!prev && level) {
			edge(line, m_rtsr);
		} else if (prev && !level) {
			edge(line, m_ftsr);
		}
	}

private:
	static constexpr uint64_t OFF_IMR = 0x00u;
	static constexpr uint64_t OFF_EMR = 0x04u;
	static constexpr uint64_t OFF_RTSR = 0x08u;
	static constexpr uint64_t OFF_FTSR = 0x0Cu;
	static constexpr uint64_t OFF_SWIER = 0x10u;
	static constexpr uint64_t OFF_PR = 0x14u;

	uint32_t m_imr = 0u;
	uint32_t m_emr = 0u;
	uint32_t m_rtsr = 0u;
	uint32_t m_ftsr = 0u;
	uint32_t m_swier = 0u;
	uint32_t m_pr = 0u;
	std::array<bool, 16> m_line_level{};

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger(unsigned line) {
		m_pr |= 1u << line;
		if (plic != nullptr) {
			plic->gateway_trigger_interrupt(irq_ids[line]);
		}
	}

	void edge(unsigned line, uint32_t edge_mask) {
		if (((edge_mask >> line) & 1u) == 0u) {
			return;
		}
		if ((m_imr >> line) & 1u) {
			trigger(line);
		}
		if ((m_emr >> line) & 1u) {
			m_swier |= 1u << line;
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

		switch (trans.get_address()) {
		case OFF_IMR:
			if (write) m_imr = value;
			result = m_imr;
			break;
		case OFF_EMR:
			if (write) m_emr = value;
			result = m_emr;
			break;
		case OFF_RTSR:
			if (write) m_rtsr = value;
			result = m_rtsr;
			break;
		case OFF_FTSR:
			if (write) m_ftsr = value;
			result = m_ftsr;
			break;
		case OFF_SWIER:
			if (write) {
				m_swier |= value;
				m_pr |= value;
				for (unsigned line = 0; line < 16u; ++line) {
					if ((value >> line) & 1u) {
						trigger(line);
					}
				}
			}
			result = m_swier;
			break;
		case OFF_PR:
			if (write) {
				m_pr &= ~value;
			}
			result = m_pr;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
