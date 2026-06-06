#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Usart : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Usart> socket{"target_socket"};
	sc_core::sc_fifo_out<uint8_t> tx_port;
	sc_core::sc_fifo_in<uint8_t> rx_port;

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Usart(sc_core::sc_module_name name)
	    : sc_module(name), tx_port("tx_port"), rx_port("rx_port") {
		socket.register_b_transport(this, &Stm32f1Usart::b_transport);
		SC_THREAD(rx_thread);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_sr = SR_TXE | SR_TC;
		m_dr = 0u;
		m_brr = 0u;
		m_cr1 = 0u;
		m_cr2 = 0u;
		m_cr3 = 0u;
		m_gtpr = 0u;
		m_rx_pending = false;
		m_rx_byte = 0u;
	}

private:
	static constexpr uint64_t OFF_SR = 0x00u;
	static constexpr uint64_t OFF_DR = 0x04u;
	static constexpr uint64_t OFF_BRR = 0x08u;
	static constexpr uint64_t OFF_CR1 = 0x0Cu;
	static constexpr uint64_t OFF_CR2 = 0x10u;
	static constexpr uint64_t OFF_CR3 = 0x14u;
	static constexpr uint64_t OFF_GTPR = 0x18u;

	static constexpr uint32_t SR_PE = 1u << 0;
	static constexpr uint32_t SR_FE = 1u << 1;
	static constexpr uint32_t SR_NE = 1u << 2;
	static constexpr uint32_t SR_ORE = 1u << 3;
	static constexpr uint32_t SR_IDLE = 1u << 4;
	static constexpr uint32_t SR_RXNE = 1u << 5;
	static constexpr uint32_t SR_TC = 1u << 6;
	static constexpr uint32_t SR_TXE = 1u << 7;
	static constexpr uint32_t SR_ALL = SR_PE | SR_FE | SR_NE | SR_ORE | SR_IDLE | SR_RXNE | SR_TC | SR_TXE;

	static constexpr uint32_t CR1_RE = 1u << 2;
	static constexpr uint32_t CR1_TE = 1u << 3;
	static constexpr uint32_t CR1_IDLEIE = 1u << 4;
	static constexpr uint32_t CR1_RXNEIE = 1u << 5;
	static constexpr uint32_t CR1_TCIE = 1u << 6;
	static constexpr uint32_t CR1_TXEIE = 1u << 7;
	static constexpr uint32_t CR1_PEIE = 1u << 8;
	static constexpr uint32_t CR1_UE = 1u << 13;
	static constexpr uint32_t CR1_RW_MASK = CR1_RE | CR1_TE | CR1_IDLEIE | CR1_RXNEIE | CR1_TCIE | CR1_TXEIE |
	                                        CR1_PEIE | CR1_UE;

	static constexpr uint32_t CR2_RW_MASK = 0xFFFFu;
	static constexpr uint32_t CR3_RW_MASK = 0x000004FFu;
	static constexpr uint32_t GTPR_RW_MASK = 0x0000FFFFu;

	uint32_t m_sr = 0u;
	uint32_t m_dr = 0u;
	uint32_t m_brr = 0u;
	uint32_t m_cr1 = 0u;
	uint32_t m_cr2 = 0u;
	uint32_t m_cr3 = 0u;
	uint32_t m_gtpr = 0u;
	bool m_clock_enabled = true;
	bool m_rx_pending = false;
	uint8_t m_rx_byte = 0u;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	bool irq_enabled(uint32_t mask) const {
		return (m_cr1 & CR1_UE) != 0u && (m_sr & mask) != 0u;
	}

	void trigger_irq() {
		if (plic != nullptr) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void sync_tx_flags() {
		m_sr |= SR_TXE | SR_TC;
		if ((m_cr1 & CR1_TXEIE) != 0u) {
			trigger_irq();
		}
		if ((m_cr1 & CR1_TCIE) != 0u) {
			trigger_irq();
		}
	}

	void receive_byte(uint8_t byte) {
		if (!m_clock_enabled || (m_cr1 & CR1_UE) == 0u || (m_cr1 & CR1_RE) == 0u) {
			return;
		}
		if ((m_sr & SR_RXNE) != 0u) {
			m_sr |= SR_ORE;
			if ((m_cr1 & CR1_PEIE) != 0u) {
				trigger_irq();
			}
			return;
		}
		m_rx_byte = byte;
		m_rx_pending = true;
		m_sr |= SR_RXNE;
		if ((m_cr1 & CR1_RXNEIE) != 0u) {
			trigger_irq();
		}
	}

	void rx_thread() {
		for (;;) {
			uint8_t byte = rx_port.read();
			receive_byte(byte);
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
		case OFF_SR:
			result = m_sr;
			break;
		case OFF_DR:
			if (write) {
				m_dr = value & 0xFFu;
				if ((m_cr1 & CR1_UE) != 0u && (m_cr1 & CR1_TE) != 0u) {
					if (!tx_port.nb_write(static_cast<uint8_t>(m_dr))) {
						m_sr |= SR_ORE;
					}
					sync_tx_flags();
				}
			} else {
				result = m_rx_pending ? m_rx_byte : (m_dr & 0xFFu);
				m_rx_pending = false;
				m_sr &= ~(SR_RXNE | SR_ORE);
			}
			break;
		case OFF_BRR:
			if (write) m_brr = value & GTPR_RW_MASK;
			result = m_brr;
			break;
		case OFF_CR1:
			if (write) m_cr1 = value & CR1_RW_MASK;
			result = m_cr1;
			break;
		case OFF_CR2:
			if (write) m_cr2 = value & CR2_RW_MASK;
			result = m_cr2;
			break;
		case OFF_CR3:
			if (write) m_cr3 = value & CR3_RW_MASK;
			result = m_cr3;
			break;
		case OFF_GTPR:
			if (write) m_gtpr = value & GTPR_RW_MASK;
			result = m_gtpr;
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
