#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Eth : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Eth> socket{"target_socket"};
	tlm_utils::simple_initiator_socket<Stm32f1Eth> mem_socket{"mem_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Eth(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Eth::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_status = 0u;
		m_receive_size = 0u;
		m_receive_dst = 0u;
		m_send_src = 0u;
		m_send_size = 0u;
		m_mac_high = 0u;
		m_mac_low = 0u;
		m_maccr = 0u;
		m_macffr = 0u;
		m_macmiiar = 0u;
		m_macmiidr = 0u;
		m_macfcr = 0u;
		m_macvlantr = 0u;
		m_dmaomr = 0u;
		m_dmabmr = 0u;
		m_dmasr = 0u;
		m_dmaier = 0u;
		m_has_frame = false;
		m_frame.fill(0u);
	}

private:
	static constexpr uint64_t OFF_STATUS = 0x00u;
	static constexpr uint64_t OFF_RECEIVE_SIZE = 0x04u;
	static constexpr uint64_t OFF_RECEIVE_DST = 0x08u;
	static constexpr uint64_t OFF_SEND_SRC = 0x0Cu;
	static constexpr uint64_t OFF_SEND_SIZE = 0x10u;
	static constexpr uint64_t OFF_MAC_HIGH = 0x14u;
	static constexpr uint64_t OFF_MAC_LOW = 0x18u;
	static constexpr uint64_t OFF_MACCR = 0x100u;
	static constexpr uint64_t OFF_MACFFR = 0x104u;
	static constexpr uint64_t OFF_MACMIIAR = 0x110u;
	static constexpr uint64_t OFF_MACMIIDR = 0x114u;
	static constexpr uint64_t OFF_MACFCR = 0x118u;
	static constexpr uint64_t OFF_MACVLANTR = 0x11Cu;
	static constexpr uint64_t OFF_DMASR = 0x200u;
	static constexpr uint64_t OFF_DMAIER = 0x204u;
	static constexpr uint64_t OFF_DMAOMR = 0x208u;
	static constexpr uint64_t OFF_DMABMR = 0x20Cu;

	static constexpr uint32_t STATUS_RECV = 1u;
	static constexpr uint32_t STATUS_SEND = 2u;
	static constexpr uint32_t DMABMR_SR = 1u << 0;
	static constexpr uint32_t DMAIER_NISE = 1u << 16;
	static constexpr uint32_t DMAIER_RIE = 1u << 6;
	static constexpr uint32_t DMAIER_TIE = 1u << 0;
	static constexpr uint32_t DMASR_NIS = 1u << 16;
	static constexpr uint32_t DMASR_RI = 1u << 6;
	static constexpr uint32_t DMASR_TI = 1u << 0;
	static constexpr uint32_t MACMIIAR_MB = 1u << 0;
	static constexpr uint32_t MACMIIAR_MIIWRITE = 1u << 1;

	static constexpr uint32_t FRAME_LIMIT = 1520u;

	std::array<uint8_t, FRAME_LIMIT> m_frame{};
	uint32_t m_status = 0u;
	uint32_t m_receive_size = 0u;
	uint32_t m_receive_dst = 0u;
	uint32_t m_send_src = 0u;
	uint32_t m_send_size = 0u;
	uint32_t m_mac_high = 0u;
	uint32_t m_mac_low = 0u;
	uint32_t m_maccr = 0u;
	uint32_t m_macffr = 0u;
	uint32_t m_macmiiar = 0u;
	uint32_t m_macmiidr = 0u;
	uint32_t m_macfcr = 0u;
	uint32_t m_macvlantr = 0u;
	uint32_t m_dmaomr = 0u;
	uint32_t m_dmabmr = 0u;
	uint32_t m_dmasr = 0u;
	uint32_t m_dmaier = 0u;
	bool m_has_frame = false;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	std::array<uint8_t, 6> current_mac() const {
		return {
			static_cast<uint8_t>(m_mac_low & 0xFFu),
			static_cast<uint8_t>((m_mac_low >> 8) & 0xFFu),
			static_cast<uint8_t>((m_mac_low >> 16) & 0xFFu),
			static_cast<uint8_t>((m_mac_low >> 24) & 0xFFu),
			static_cast<uint8_t>(m_mac_high & 0xFFu),
			static_cast<uint8_t>((m_mac_high >> 8) & 0xFFu),
		};
	}

	bool frame_matches_mac() const {
		if (m_receive_size < 6u) {
			return true;
		}
		const auto mac = current_mac();
		bool broadcast = true;
		bool match = true;
		for (unsigned i = 0u; i < 6u; ++i) {
			const uint8_t dst = m_frame[i];
			broadcast &= dst == 0xFFu;
			match &= dst == mac[i];
		}
		return broadcast || match;
	}

	uint32_t mem_read32(uint64_t addr) {
		uint32_t value = 0u;
		tlm::tlm_generic_payload trans;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		trans.set_command(tlm::TLM_READ_COMMAND);
		trans.set_address(addr);
		trans.set_data_ptr(reinterpret_cast<unsigned char *>(&value));
		trans.set_data_length(sizeof(value));
		trans.set_streaming_width(sizeof(value));
		trans.set_byte_enable_ptr(nullptr);
		trans.set_dmi_allowed(false);
		trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		mem_socket->b_transport(trans, delay);
		return value;
	}

	void mem_write32(uint64_t addr, uint32_t value) {
		tlm::tlm_generic_payload trans;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		trans.set_command(tlm::TLM_WRITE_COMMAND);
		trans.set_address(addr);
		trans.set_data_ptr(reinterpret_cast<unsigned char *>(&value));
		trans.set_data_length(sizeof(value));
		trans.set_streaming_width(sizeof(value));
		trans.set_byte_enable_ptr(nullptr);
		trans.set_dmi_allowed(false);
		trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		mem_socket->b_transport(trans, delay);
	}

	void trigger_irq(uint32_t bits) {
		if (plic != nullptr && irq_id != 0u && (m_dmaier & bits) != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void do_send() {
		const uint32_t len = std::min<uint32_t>(m_send_size, FRAME_LIMIT);
		for (uint32_t i = 0u; i < len; i += 4u) {
			const uint32_t word = mem_read32(static_cast<uint64_t>(m_send_src) + i);
			std::memcpy(&m_frame[i], &word, std::min<uint32_t>(4u, len - i));
		}
		m_receive_size = len;
		m_has_frame = true;
		m_status = STATUS_SEND;
		m_dmasr |= DMASR_TI | DMASR_NIS;
		trigger_irq(DMASR_TI | DMASR_NIS);
	}

	void do_receive() {
		if (!m_has_frame) {
			return;
		}
		if (!frame_matches_mac()) {
			m_has_frame = false;
			m_status = 0u;
			return;
		}
		const uint32_t len = m_receive_size;
		for (uint32_t i = 0u; i < len; i += 4u) {
			uint32_t word = 0u;
			std::memcpy(&word, &m_frame[i], std::min<uint32_t>(4u, len - i));
			mem_write32(static_cast<uint64_t>(m_receive_dst) + i, word);
		}
		m_has_frame = false;
		m_status = STATUS_RECV;
		m_dmasr |= DMASR_RI | DMASR_NIS;
		trigger_irq(DMASR_RI | DMASR_NIS);
	}

	void soft_reset() {
		m_status = 0u;
		m_receive_size = 0u;
		m_receive_dst = 0u;
		m_send_src = 0u;
		m_send_size = 0u;
		m_mac_high = 0u;
		m_mac_low = 0u;
		m_maccr = 0u;
		m_macffr = 0u;
		m_macmiiar = 0u;
		m_macmiidr = 0u;
		m_macfcr = 0u;
		m_macvlantr = 0u;
		m_dmaomr = 0u;
		m_dmasr = 0u;
		m_dmaier = 0u;
		m_has_frame = false;
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
		case OFF_STATUS:
			if (write) {
				m_status = value;
				if ((value & STATUS_SEND) != 0u) {
					do_send();
				} else if ((value & STATUS_RECV) != 0u) {
					do_receive();
				}
			}
			result = m_status;
			break;
		case OFF_RECEIVE_SIZE:
			if (write) m_receive_size = value;
			result = m_receive_size;
			break;
		case OFF_RECEIVE_DST:
			if (write) m_receive_dst = value;
			result = m_receive_dst;
			break;
		case OFF_SEND_SRC:
			if (write) m_send_src = value;
			result = m_send_src;
			break;
		case OFF_SEND_SIZE:
			if (write) m_send_size = value;
			result = m_send_size;
			break;
		case OFF_MAC_HIGH:
			if (write) m_mac_high = value;
			result = m_mac_high;
			break;
		case OFF_MAC_LOW:
			if (write) m_mac_low = value;
			result = m_mac_low;
			break;
		case OFF_MACCR:
			if (write) m_maccr = value;
			result = m_maccr;
			break;
		case OFF_MACFFR:
			if (write) m_macffr = value;
			result = m_macffr;
			break;
		case OFF_MACMIIAR:
			if (write) {
				m_macmiiar = value & 0xFFFFu;
				if ((value & MACMIIAR_MB) != 0u) {
					m_macmiidr ^= 0x00010001u;
					m_macmiiar &= ~MACMIIAR_MB;
					m_dmasr |= DMASR_NIS;
					trigger_irq(DMASR_NIS);
				}
			}
			result = m_macmiiar;
			break;
		case OFF_MACMIIDR:
			if (write) m_macmiidr = value & 0xFFFFu;
			result = m_macmiidr;
			break;
		case OFF_MACFCR:
			if (write) m_macfcr = value;
			result = m_macfcr;
			break;
		case OFF_MACVLANTR:
			if (write) m_macvlantr = value;
			result = m_macvlantr;
			break;
		case OFF_DMASR:
			if (write) {
				m_dmasr &= ~value;
			}
			result = m_dmasr;
			break;
		case OFF_DMAIER:
			if (write) m_dmaier = value;
			result = m_dmaier;
			break;
		case OFF_DMAOMR:
			if (write) m_dmaomr = value;
			result = m_dmaomr;
			break;
		case OFF_DMABMR:
			if (write) {
				m_dmabmr = value & ~DMABMR_SR;
				if ((value & DMABMR_SR) != 0u) {
					soft_reset();
				}
			}
			result = m_dmabmr;
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
