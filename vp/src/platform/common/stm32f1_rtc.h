#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"
#include "stm32f1_pwr.h"

class Stm32f1Rtc : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Rtc> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;
	Stm32f1Pwr *pwr = nullptr;

	explicit Stm32f1Rtc(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Rtc::b_transport);
		reset();
	}

	void set_enabled(bool enabled) {
		m_enabled = enabled;
		check_alarm();
	}

	void backup_reset() {
		reset();
	}

	void reset() {
		m_crh = 0u;
		m_crl = 0x0020u;
		m_prlh = 0u;
		m_prll = 0u;
		m_divh = 0u;
		m_divl = 0u;
		m_cnth = 0u;
		m_cntl = 0u;
		m_alrh = 0u;
		m_alrl = 0u;
		m_enabled = false;
		m_alarm_fired = false;
		m_counter = 0u;
		m_alarm = 0u;
	}

private:
	static constexpr uint64_t OFF_CRL = 0x00u;
	static constexpr uint64_t OFF_CRH = 0x04u;
	static constexpr uint64_t OFF_PRLH = 0x08u;
	static constexpr uint64_t OFF_PRLL = 0x0Cu;
	static constexpr uint64_t OFF_DIVH = 0x10u;
	static constexpr uint64_t OFF_DIVL = 0x14u;
	static constexpr uint64_t OFF_CNTH = 0x18u;
	static constexpr uint64_t OFF_CNTL = 0x1Cu;
	static constexpr uint64_t OFF_ALRH = 0x20u;
	static constexpr uint64_t OFF_ALRL = 0x24u;

	static constexpr uint32_t CRL_SECF = 1u << 0;
	static constexpr uint32_t CRL_ALRF = 1u << 1;
	static constexpr uint32_t CRL_RSF = 1u << 5;
	static constexpr uint32_t CRL_CNF = 1u << 4;
	static constexpr uint32_t CRH_SECIE = 1u << 0;
	static constexpr uint32_t CRH_ALRIE = 1u << 1;
	static constexpr uint32_t CRH_OVIE = 1u << 2;

	uint32_t m_crh = 0u;
	uint32_t m_crl = 0u;
	uint32_t m_prlh = 0u;
	uint32_t m_prll = 0u;
	uint32_t m_divh = 0u;
	uint32_t m_divl = 0u;
	uint32_t m_cnth = 0u;
	uint32_t m_cntl = 0u;
	uint32_t m_alrh = 0u;
	uint32_t m_alrl = 0u;
	bool m_enabled = false;
	bool m_alarm_fired = false;
	uint32_t m_counter = 0u;
	uint32_t m_alarm = 0u;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger_irq() {
		if (plic != nullptr) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void sync_counter_regs() {
		m_cnth = (m_counter >> 16u) & 0xFFFFu;
		m_cntl = m_counter & 0xFFFFu;
	}

	void sync_alarm() {
		m_alarm = ((m_alrh & 0xFFFFu) << 16u) | (m_alrl & 0xFFFFu);
		check_alarm();
	}

	void check_alarm() {
		if (m_enabled && !m_alarm_fired && m_alarm != 0u && m_counter >= m_alarm) {
			m_crl |= CRL_ALRF;
			m_alarm_fired = true;
			if ((m_crh & CRH_ALRIE) != 0u) {
				trigger_irq();
			}
		}
	}

	void tick() {
		if (!m_enabled) {
			return;
		}
		++m_counter;
		sync_counter_regs();
		m_crl |= CRL_SECF;
		if ((m_crh & CRH_SECIE) != 0u) {
			trigger_irq();
		}
		check_alarm();
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}
		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint32_t value = write ? read_word(trans) : 0u;
		uint32_t result = 0u;

		if (pwr != nullptr && !pwr->dbp_enabled()) {
			if (!write) {
				write_word(trans, 0u);
			}
			trans.set_response_status(tlm::TLM_OK_RESPONSE);
			return;
		}

		switch (trans.get_address()) {
		case OFF_CRL:
			if (write) {
				m_crl = value & 0x003Fu;
				if ((value & CRL_CNF) != 0u) {
					m_crl |= CRL_CNF;
				}
				if ((value & (CRL_SECF | CRL_ALRF)) != 0u) {
					m_crl &= ~(value & (CRL_SECF | CRL_ALRF));
				}
			}
			result = m_crl;
			break;
		case OFF_CRH:
			if (write) m_crh = value & 0x0007u;
			result = m_crh;
			break;
		case OFF_PRLH:
			if (write) m_prlh = value & 0xFFFFu;
			result = m_prlh;
			break;
		case OFF_PRLL:
			if (write) m_prll = value & 0xFFFFu;
			result = m_prll;
			break;
		case OFF_DIVH:
			result = m_divh;
			break;
		case OFF_DIVL:
			result = m_divl;
			break;
		case OFF_CNTH:
			if (write) {
				m_cnth = value & 0xFFFFu;
				m_counter = (m_cnth << 16u) | m_cntl;
			}
			result = m_cnth;
			break;
		case OFF_CNTL:
			if (write) {
				m_cntl = value & 0xFFFFu;
				m_counter = (m_cnth << 16u) | m_cntl;
			}
			result = m_cntl;
			break;
		case OFF_ALRH:
			if (write) {
				m_alrh = value & 0xFFFFu;
				sync_alarm();
				m_alarm_fired = false;
			}
			result = m_alrh;
			break;
		case OFF_ALRL:
			if (write) {
				m_alrl = value & 0xFFFFu;
				sync_alarm();
				m_alarm_fired = false;
			}
			result = m_alrl;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		if (!write && m_enabled && (trans.get_address() == OFF_CNTH || trans.get_address() == OFF_CNTL)) {
			tick();
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
