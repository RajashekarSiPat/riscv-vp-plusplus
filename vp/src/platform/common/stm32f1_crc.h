#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

class Stm32f1Crc : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Crc> socket{"target_socket"};

	explicit Stm32f1Crc(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Crc::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_dr = 0xFFFFFFFFu;
		m_idr = 0u;
		m_cr = 0u;
	}

private:
	static constexpr uint64_t OFF_DR = 0x00u;
	static constexpr uint64_t OFF_IDR = 0x04u;
	static constexpr uint64_t OFF_CR = 0x08u;
	static constexpr uint32_t POLY = 0x04C11DB7u;

	uint32_t m_dr = 0u;
	uint32_t m_idr = 0u;
	uint32_t m_cr = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void feed_byte(uint8_t byte) {
		m_dr ^= static_cast<uint32_t>(byte) << 24;
		for (unsigned i = 0; i < 8u; ++i) {
			if ((m_dr & 0x80000000u) != 0u) {
				m_dr = (m_dr << 1) ^ POLY;
			} else {
				m_dr <<= 1;
			}
		}
	}

	void feed_word(uint32_t value) {
		const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
		for (unsigned i = 0; i < 4u; ++i) {
			feed_byte(bytes[i]);
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
		case OFF_DR:
			if (write) {
				feed_word(value);
			}
			result = m_dr;
			break;
		case OFF_IDR:
			if (write) m_idr = value & 0xFFu;
			result = m_idr;
			break;
		case OFF_CR:
			if (write && (value & 1u) != 0u) {
				reset();
			}
			result = 0u;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
