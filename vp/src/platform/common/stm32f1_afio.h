#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

class Stm32f1Afio : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Afio> socket{"target_socket"};

	explicit Stm32f1Afio(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Afio::b_transport);
		reset();
	}

	void reset() {
		m_evcr = 0u;
		m_mapr = 0u;
		m_exticr.fill(0u);
		m_mapr2 = 0u;
	}

	uint32_t exti_port(unsigned line) const {
		if (line >= 16u) {
			return 0u;
		}
		const unsigned reg = line / 4u;
		const unsigned shift = (line % 4u) * 4u;
		return (m_exticr[reg] >> shift) & 0xFu;
	}

private:
	static constexpr uint64_t OFF_EVCR = 0x00u;
	static constexpr uint64_t OFF_MAPR = 0x04u;
	static constexpr uint64_t OFF_EXTICR1 = 0x08u;
	static constexpr uint64_t OFF_EXTICR2 = 0x0Cu;
	static constexpr uint64_t OFF_EXTICR3 = 0x10u;
	static constexpr uint64_t OFF_EXTICR4 = 0x14u;
	static constexpr uint64_t OFF_MAPR2 = 0x1Cu;

	static constexpr uint32_t MAPR_RW_MASK = 0x0F00FFFFu;
	static constexpr uint32_t MAPR2_RW_MASK = 0x00003FFFu;

	uint32_t m_evcr = 0u;
	uint32_t m_mapr = 0u;
	std::array<uint32_t, 4> m_exticr{};
	uint32_t m_mapr2 = 0u;

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

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		switch (trans.get_address()) {
		case OFF_EVCR:
			if (write) m_evcr = value & 0x0000007Fu;
			result = m_evcr;
			break;
		case OFF_MAPR:
			if (write) m_mapr = value & MAPR_RW_MASK;
			result = m_mapr;
			break;
		case OFF_EXTICR1:
			if (write) m_exticr[0] = value;
			result = m_exticr[0];
			break;
		case OFF_EXTICR2:
			if (write) m_exticr[1] = value;
			result = m_exticr[1];
			break;
		case OFF_EXTICR3:
			if (write) m_exticr[2] = value;
			result = m_exticr[2];
			break;
		case OFF_EXTICR4:
			if (write) m_exticr[3] = value;
			result = m_exticr[3];
			break;
		case OFF_MAPR2:
			if (write) m_mapr2 = value & MAPR2_RW_MASK;
			result = m_mapr2;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
