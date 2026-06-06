#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1UsbDeviceFs : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1UsbDeviceFs> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1UsbDeviceFs(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1UsbDeviceFs::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_ep.fill(0u);
		m_pma.fill(0u);
		m_cntr = 0u;
		m_istr = 0u;
		m_fnr = 0u;
		m_daddr = 0u;
		m_btable = 0u;
	}

private:
	static constexpr uint64_t OFF_EP0R = 0x00u;
	static constexpr uint64_t OFF_EP1R = 0x04u;
	static constexpr uint64_t OFF_EP2R = 0x08u;
	static constexpr uint64_t OFF_EP3R = 0x0Cu;
	static constexpr uint64_t OFF_EP4R = 0x10u;
	static constexpr uint64_t OFF_EP5R = 0x14u;
	static constexpr uint64_t OFF_EP6R = 0x18u;
	static constexpr uint64_t OFF_EP7R = 0x1Cu;
	static constexpr uint64_t OFF_CNTR = 0x40u;
	static constexpr uint64_t OFF_ISTR = 0x44u;
	static constexpr uint64_t OFF_FNR = 0x48u;
	static constexpr uint64_t OFF_DADDR = 0x4Cu;
	static constexpr uint64_t OFF_BTABLE = 0x50u;
	static constexpr uint64_t OFF_PMA_BASE = 0x400u;
	static constexpr uint64_t OFF_PMA_LIMIT = 0x800u;

	static constexpr uint32_t CNTR_RESETM = 1u << 10;
	static constexpr uint32_t CNTR_CTRM = 1u << 15;
	static constexpr uint32_t CNTR_RW_MASK = 0xFFFFu;
	static constexpr uint32_t ISTR_CTR = 1u << 15;
	static constexpr uint32_t ISTR_RESET = 1u << 10;
	static constexpr uint32_t ISTR_RW_MASK = ISTR_CTR | ISTR_RESET | 0x03FFu;
	static constexpr uint32_t DADDR_EF = 1u << 7;
	static constexpr uint32_t DADDR_RW_MASK = 0x7Fu | DADDR_EF;
	static constexpr uint32_t BTABLE_RW_MASK = 0x1FFFu;

	std::array<uint32_t, 8> m_ep{};
	std::array<uint8_t, 1024> m_pma{};
	uint32_t m_cntr = 0u;
	uint32_t m_istr = 0u;
	uint32_t m_fnr = 0u;
	uint32_t m_daddr = 0u;
	uint32_t m_btable = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger_irq(uint32_t bits) {
		if (plic != nullptr && irq_id != 0u && (bits & (m_cntr & (CNTR_RESETM | CNTR_CTRM))) != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void write_cntr(uint32_t value) {
		m_cntr = value & CNTR_RW_MASK;
		if ((value & (CNTR_RESETM | CNTR_CTRM)) != 0u) {
			m_istr |= ((value & CNTR_RESETM) != 0u ? ISTR_RESET : 0u) |
			          ((value & CNTR_CTRM) != 0u ? ISTR_CTR : 0u);
			trigger_irq(value);
		}
	}

	void write_istr(uint32_t value) {
		m_istr &= ~value;
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
		case OFF_EP0R:
		case OFF_EP1R:
		case OFF_EP2R:
		case OFF_EP3R:
		case OFF_EP4R:
		case OFF_EP5R:
		case OFF_EP6R:
		case OFF_EP7R: {
			const unsigned idx = static_cast<unsigned>(trans.get_address() / 4u);
			if (write) {
				m_ep[idx] = value & 0xFFFFu;
			}
			result = m_ep[idx];
			break;
		}
		case OFF_CNTR:
			if (write) write_cntr(value);
			result = m_cntr;
			break;
		case OFF_ISTR:
			if (write) write_istr(value);
			result = m_istr;
			break;
		case OFF_FNR:
			result = m_fnr;
			m_fnr = (m_fnr + 1u) & 0x07FFu;
			break;
		case OFF_DADDR:
			if (write) m_daddr = value & DADDR_RW_MASK;
			result = m_daddr;
			break;
		case OFF_BTABLE:
			if (write) m_btable = value & BTABLE_RW_MASK;
			result = m_btable;
			break;
		default:
			if (trans.get_address() >= OFF_PMA_BASE && trans.get_address() < OFF_PMA_LIMIT) {
				const uint64_t local = trans.get_address() - OFF_PMA_BASE;
				const uint32_t len = trans.get_data_length();
				if (local + len > m_pma.size()) {
					trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
					return;
				}
				if (write) {
					for (uint32_t i = 0u; i < len; ++i) {
						m_pma[local + i] = trans.get_data_ptr()[i];
					}
				} else {
					for (uint32_t i = 0u; i < len; ++i) {
						trans.get_data_ptr()[i] = m_pma[local + i];
					}
				}
				trans.set_response_status(tlm::TLM_OK_RESPONSE);
				return;
			}
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
