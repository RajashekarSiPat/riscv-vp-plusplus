#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1OtgFs : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1OtgFs> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1OtgFs(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1OtgFs::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_gotgctl = 0u;
		m_gotgint = 0u;
		m_gahbcfg = 0u;
		m_gusbcfg = 0u;
		m_grstctl = 0x80000000u;
		m_gintsts = 0u;
		m_gintmsk = 0u;
		m_grxfsiz = 0u;
		m_gnptxfsiz = 0u;
		m_device_mode = false;
		m_reset_count = 0u;
	}

private:
	static constexpr uint64_t OFF_GOTGCTL = 0x00u;
	static constexpr uint64_t OFF_GOTGINT = 0x04u;
	static constexpr uint64_t OFF_GAHBCFG = 0x08u;
	static constexpr uint64_t OFF_GUSBCFG = 0x0Cu;
	static constexpr uint64_t OFF_GRSTCTL = 0x10u;
	static constexpr uint64_t OFF_GINTSTS = 0x14u;
	static constexpr uint64_t OFF_GINTMSK = 0x18u;
	static constexpr uint64_t OFF_GRXFSIZ = 0x24u;
	static constexpr uint64_t OFF_GNPTXFSIZ = 0x28u;

	static constexpr uint32_t GAHBCFG_GINT = 1u << 0;
	static constexpr uint32_t GAHBCFG_RW_MASK = 0x000000FFu;
	static constexpr uint32_t GUSBCFG_FDMOD = 1u << 30;
	static constexpr uint32_t GUSBCFG_RW_MASK = 0x7FFFFFFFu;
	static constexpr uint32_t GRSTCTL_CSRST = 1u << 0;
	static constexpr uint32_t GINTSTS_USBRST = 1u << 12;
	static constexpr uint32_t GINTSTS_ENUMDNE = 1u << 13;
	static constexpr uint32_t GINTSTS_RW_MASK = GINTSTS_USBRST | GINTSTS_ENUMDNE | 0x00003FFFu;
	static constexpr uint32_t GINTMSK_RW_MASK = GINTSTS_RW_MASK;

	uint32_t m_gotgctl = 0u;
	uint32_t m_gotgint = 0u;
	uint32_t m_gahbcfg = 0u;
	uint32_t m_gusbcfg = 0u;
	uint32_t m_grstctl = 0u;
	uint32_t m_gintsts = 0u;
	uint32_t m_gintmsk = 0u;
	uint32_t m_grxfsiz = 0u;
	uint32_t m_gnptxfsiz = 0u;
	bool m_device_mode = false;
	uint32_t m_reset_count = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger(uint32_t bits) {
		if ((m_gahbcfg & GAHBCFG_GINT) != 0u && (m_gintmsk & bits) != 0u && plic != nullptr && irq_id != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void inject(uint32_t bits) {
		m_gintsts |= bits & GINTSTS_RW_MASK;
		trigger(bits);
	}

	void soft_reset() {
		m_gotgctl = 0u;
		m_gotgint = 0u;
		m_gusbcfg &= ~GUSBCFG_FDMOD;
		m_grstctl = 0x80000000u;
		m_gintsts = 0u;
		m_grxfsiz = 0u;
		m_gnptxfsiz = 0u;
		m_device_mode = false;
		++m_reset_count;
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
		case OFF_GOTGCTL:
			if (write) m_gotgctl = value;
			result = m_gotgctl;
			break;
		case OFF_GOTGINT:
			if (write) m_gotgint &= ~value;
			result = m_gotgint;
			break;
		case OFF_GAHBCFG:
			if (write) {
				m_gahbcfg = value & GAHBCFG_RW_MASK;
			}
			result = m_gahbcfg;
			break;
		case OFF_GUSBCFG:
			if (write) {
				m_gusbcfg = value & GUSBCFG_RW_MASK;
				if ((value & GUSBCFG_FDMOD) != 0u) {
					m_device_mode = true;
					inject(GINTSTS_ENUMDNE);
				} else {
					m_device_mode = false;
				}
			}
			result = m_gusbcfg;
			break;
		case OFF_GRSTCTL:
			if (write) {
				m_grstctl = value & ~GRSTCTL_CSRST;
				if ((value & GRSTCTL_CSRST) != 0u) {
					++m_reset_count;
					m_device_mode = false;
					inject(GINTSTS_USBRST);
				}
			}
			result = m_grstctl;
			break;
		case OFF_GINTSTS:
			if (write) {
				m_gintsts &= ~value;
			}
			result = m_gintsts;
			break;
		case OFF_GINTMSK:
			if (write) m_gintmsk = value & GINTMSK_RW_MASK;
			result = m_gintmsk;
			break;
		case OFF_GRXFSIZ:
			if (write) m_grxfsiz = value & 0xFFFFu;
			result = m_grxfsiz;
			break;
		case OFF_GNPTXFSIZ:
			if (write) m_gnptxfsiz = value;
			result = m_gnptxfsiz;
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
