#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

class Stm32f1Dac : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Dac> socket{"target_socket"};

	explicit Stm32f1Dac(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Dac::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_cr = 0u;
		m_swtrigr = 0u;
		m_dhr12r1 = 0u;
		m_dhr12l1 = 0u;
		m_dhr8r1 = 0u;
		m_dhr12r2 = 0u;
		m_dhr12l2 = 0u;
		m_dhr8r2 = 0u;
		m_dhr12rd = 0u;
		m_dhr12ld = 0u;
		m_dhr8rd = 0u;
		m_dor1 = 0u;
		m_dor2 = 0u;
	}

private:
	static constexpr uint64_t OFF_CR = 0x00u;
	static constexpr uint64_t OFF_SWTRIGR = 0x04u;
	static constexpr uint64_t OFF_DHR12R1 = 0x08u;
	static constexpr uint64_t OFF_DHR12L1 = 0x0Cu;
	static constexpr uint64_t OFF_DHR8R1 = 0x10u;
	static constexpr uint64_t OFF_DHR12R2 = 0x14u;
	static constexpr uint64_t OFF_DHR12L2 = 0x18u;
	static constexpr uint64_t OFF_DHR8R2 = 0x1Cu;
	static constexpr uint64_t OFF_DHR12RD = 0x20u;
	static constexpr uint64_t OFF_DHR12LD = 0x24u;
	static constexpr uint64_t OFF_DHR8RD = 0x28u;
	static constexpr uint64_t OFF_DOR1 = 0x2Cu;
	static constexpr uint64_t OFF_DOR2 = 0x30u;

	static constexpr uint32_t CR_EN1 = 1u << 0;
	static constexpr uint32_t CR_BOFF1 = 1u << 1;
	static constexpr uint32_t CR_TEN1 = 1u << 2;
	static constexpr uint32_t CR_TSEL1_MASK = 0x7u << 3;
	static constexpr uint32_t CR_WAVE1_MASK = 0x3u << 6;
	static constexpr uint32_t CR_MAMP1_MASK = 0xFu << 8;
	static constexpr uint32_t CR_EN2 = 1u << 16;
	static constexpr uint32_t CR_BOFF2 = 1u << 17;
	static constexpr uint32_t CR_TEN2 = 1u << 18;
	static constexpr uint32_t CR_TSEL2_MASK = 0x7u << 19;
	static constexpr uint32_t CR_WAVE2_MASK = 0x3u << 22;
	static constexpr uint32_t CR_MAMP2_MASK = 0xFu << 24;
	static constexpr uint32_t CR_RW_MASK = CR_EN1 | CR_BOFF1 | CR_TEN1 | CR_TSEL1_MASK | CR_WAVE1_MASK |
	                                        CR_MAMP1_MASK | CR_EN2 | CR_BOFF2 | CR_TEN2 | CR_TSEL2_MASK |
	                                        CR_WAVE2_MASK | CR_MAMP2_MASK;

	uint32_t m_cr = 0u;
	uint32_t m_swtrigr = 0u;
	uint32_t m_dhr12r1 = 0u;
	uint32_t m_dhr12l1 = 0u;
	uint32_t m_dhr8r1 = 0u;
	uint32_t m_dhr12r2 = 0u;
	uint32_t m_dhr12l2 = 0u;
	uint32_t m_dhr8r2 = 0u;
	uint32_t m_dhr12rd = 0u;
	uint32_t m_dhr12ld = 0u;
	uint32_t m_dhr8rd = 0u;
	uint32_t m_dor1 = 0u;
	uint32_t m_dor2 = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void update_outputs(uint32_t mask) {
		if ((mask & 0x1u) != 0u) {
			m_dor1 = m_dhr12r1 & 0x0FFFu;
		}
		if ((mask & 0x2u) != 0u) {
			m_dor2 = m_dhr12r2 & 0x0FFFu;
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
		case OFF_CR:
			if (write) m_cr = value & CR_RW_MASK;
			result = m_cr;
			break;
		case OFF_SWTRIGR:
			if (write) {
				m_swtrigr = value & 0x3u;
				update_outputs(m_swtrigr);
			}
			result = m_swtrigr;
			break;
		case OFF_DHR12R1:
			if (write) m_dhr12r1 = value & 0x0FFFu;
			result = m_dhr12r1;
			break;
		case OFF_DHR12L1:
			if (write) m_dhr12l1 = value & 0x0FFFu;
			result = m_dhr12l1;
			break;
		case OFF_DHR8R1:
			if (write) m_dhr8r1 = value & 0x00FFu;
			result = m_dhr8r1;
			break;
		case OFF_DHR12R2:
			if (write) m_dhr12r2 = value & 0x0FFFu;
			result = m_dhr12r2;
			break;
		case OFF_DHR12L2:
			if (write) m_dhr12l2 = value & 0x0FFFu;
			result = m_dhr12l2;
			break;
		case OFF_DHR8R2:
			if (write) m_dhr8r2 = value & 0x00FFu;
			result = m_dhr8r2;
			break;
		case OFF_DHR12RD:
			if (write) m_dhr12rd = value;
			result = m_dhr12rd;
			break;
		case OFF_DHR12LD:
			if (write) m_dhr12ld = value;
			result = m_dhr12ld;
			break;
		case OFF_DHR8RD:
			if (write) m_dhr8rd = value;
			result = m_dhr8rd;
			break;
		case OFF_DOR1:
			result = m_dor1;
			break;
		case OFF_DOR2:
			result = m_dor2;
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
