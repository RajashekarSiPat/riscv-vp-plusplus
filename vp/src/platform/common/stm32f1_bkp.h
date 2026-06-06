#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "stm32f1_pwr.h"

class Stm32f1Bkp : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Bkp> socket{"target_socket"};
	Stm32f1Pwr *pwr = nullptr;

	explicit Stm32f1Bkp(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Bkp::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_dr.fill(0u);
		m_rtccr = 0u;
		m_cr = 0u;
		m_csr = 0u;
	}

private:
	static constexpr uint64_t OFF_DR1 = 0x04u;
	static constexpr uint64_t OFF_RTCCR = 0x2Cu;
	static constexpr uint64_t OFF_CR = 0x30u;
	static constexpr uint64_t OFF_CSR = 0x34u;
	static constexpr uint64_t OFF_DR11 = 0x38u;
	static constexpr unsigned NUM_DR = 42u;

	std::array<uint32_t, NUM_DR> m_dr{};
	uint32_t m_rtccr = 0u;
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

	bool writable() const {
		return m_clock_enabled && pwr != nullptr && pwr->dbp_enabled();
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}

		if (!m_clock_enabled) {
			if (trans.get_command() == tlm::TLM_READ_COMMAND) {
				write_word(trans, 0u);
			}
			trans.set_response_status(tlm::TLM_OK_RESPONSE);
			return;
		}

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		const uint64_t addr = trans.get_address();
		if (addr >= OFF_DR1 && addr < OFF_DR1 + 10u * 4u) {
			const unsigned idx = static_cast<unsigned>((addr - OFF_DR1) / 4u);
			if (idx >= 10u) {
				trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
				return;
			}
			if (write && writable()) {
				m_dr[idx] = value;
			}
			result = m_dr[idx];
		} else if (addr == OFF_RTCCR) {
			if (write && writable()) {
				m_rtccr = value;
			}
			result = m_rtccr;
		} else if (addr == OFF_CR) {
			if (write && writable()) {
				m_cr = value;
			}
			result = m_cr;
		} else if (addr == OFF_CSR) {
			if (write && writable()) {
				m_csr = value;
			}
			result = m_csr;
		} else if (addr >= OFF_DR11 && addr < OFF_DR11 + (NUM_DR - 10u) * 4u) {
			const unsigned idx = 10u + static_cast<unsigned>((addr - OFF_DR11) / 4u);
			if (idx >= NUM_DR) {
				trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
				return;
			}
			if (write && writable()) {
				m_dr[idx] = value;
			}
			result = m_dr[idx];
		} else {
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
