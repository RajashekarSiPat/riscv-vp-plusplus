#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

class Stm32f1Pwr : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Pwr> socket{"target_socket"};

	explicit Stm32f1Pwr(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Pwr::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_cr = 0u;
		m_csr = 0u;
	}

	bool dbp_enabled() const { return (m_cr & (1u << 8)) != 0u; }

private:
	static constexpr uint64_t OFF_CR = 0x00u;
	static constexpr uint64_t OFF_CSR = 0x04u;
	static constexpr uint32_t CR_RW_MASK = 0x00000300u;
	static constexpr uint32_t CSR_RW_MASK = 0x00000003u;

	uint32_t m_cr = 0u;
	uint32_t m_csr = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}
		if (!m_clock_enabled) {
			if (trans.get_command() == tlm::TLM_READ_COMMAND) {
				uint32_t zero = 0u;
				write_word(trans, zero);
			}
			trans.set_response_status(tlm::TLM_OK_RESPONSE);
			return;
		}
		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;
		switch (trans.get_address()) {
		case OFF_CR:
			if (write) m_cr = value & CR_RW_MASK;
			result = m_cr;
			break;
		case OFF_CSR:
			if (write) m_csr = value & CSR_RW_MASK;
			result = m_csr;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}
		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
