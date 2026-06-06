#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Spi : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Spi> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;
	Stm32f1Spi *peer = nullptr;

	explicit Stm32f1Spi(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Spi::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void set_peer(Stm32f1Spi *other) { peer = other; }

	void reset() {
		m_cr1 = 0u;
		m_cr2 = 0u;
		m_sr = SR_TXE;
		m_dr = 0u;
		m_crcpoly = 0x0007u;
		m_rxcrcr = 0u;
		m_txcrcr = 0u;
		m_i2scfgr = 0u;
		m_i2spr = 0u;
		m_clock_enabled = true;
	}

private:
	static constexpr uint64_t OFF_CR1 = 0x00u;
	static constexpr uint64_t OFF_CR2 = 0x04u;
	static constexpr uint64_t OFF_SR = 0x08u;
	static constexpr uint64_t OFF_DR = 0x0Cu;
	static constexpr uint64_t OFF_CRCPR = 0x10u;
	static constexpr uint64_t OFF_RXCRCR = 0x14u;
	static constexpr uint64_t OFF_TXCRCR = 0x18u;
	static constexpr uint64_t OFF_I2SCFGR = 0x1Cu;
	static constexpr uint64_t OFF_I2SPR = 0x20u;

	static constexpr uint32_t SR_RXNE = 1u << 0;
	static constexpr uint32_t SR_TXE = 1u << 1;
	static constexpr uint32_t SR_OVR = 1u << 6;

	static constexpr uint32_t CR1_CPHA = 1u << 0;
	static constexpr uint32_t CR1_CPOL = 1u << 1;
	static constexpr uint32_t CR1_MSTR = 1u << 2;
	static constexpr uint32_t CR1_BR_MASK = 0x7u << 3;
	static constexpr uint32_t CR1_SPE = 1u << 6;
	static constexpr uint32_t CR1_LSBFIRST = 1u << 7;
	static constexpr uint32_t CR1_SSI = 1u << 8;
	static constexpr uint32_t CR1_SSM = 1u << 9;
	static constexpr uint32_t CR1_RXONLY = 1u << 10;
	static constexpr uint32_t CR1_DFF = 1u << 11;
	static constexpr uint32_t CR1_CRCNEXT = 1u << 12;
	static constexpr uint32_t CR1_CRCEN = 1u << 13;
	static constexpr uint32_t CR1_BIDIOE = 1u << 14;
	static constexpr uint32_t CR1_BIDIMODE = 1u << 15;
	static constexpr uint32_t CR1_RW_MASK = CR1_CPHA | CR1_CPOL | CR1_MSTR | CR1_BR_MASK | CR1_SPE |
	                                        CR1_LSBFIRST | CR1_SSI | CR1_SSM | CR1_RXONLY | CR1_DFF |
	                                        CR1_CRCNEXT | CR1_CRCEN | CR1_BIDIOE | CR1_BIDIMODE;

	static constexpr uint32_t CR2_SSOE = 1u << 2;
	static constexpr uint32_t CR2_ERRIE = 1u << 5;
	static constexpr uint32_t CR2_RXNEIE = 1u << 6;
	static constexpr uint32_t CR2_TXEIE = 1u << 7;
	static constexpr uint32_t CR2_RW_MASK = CR2_SSOE | CR2_ERRIE | CR2_RXNEIE | CR2_TXEIE;

	static constexpr uint32_t I2SCFGR_RW_MASK = 0x0FFFu;
	static constexpr uint32_t I2SPR_RW_MASK = 0x03FFu;
	static constexpr uint32_t DR_RW_MASK = 0xFFFFu;

	uint32_t m_cr1 = 0u;
	uint32_t m_cr2 = 0u;
	uint32_t m_sr = 0u;
	uint32_t m_dr = 0u;
	uint32_t m_crcpoly = 0u;
	uint32_t m_rxcrcr = 0u;
	uint32_t m_txcrcr = 0u;
	uint32_t m_i2scfgr = 0u;
	uint32_t m_i2spr = 0u;
	bool m_clock_enabled = true;

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

	void receive_from_peer(uint8_t byte) {
		if (!m_clock_enabled || (m_cr1 & CR1_SPE) == 0u) {
			return;
		}
		if ((m_sr & SR_RXNE) != 0u) {
			m_sr |= SR_OVR;
			if ((m_cr2 & CR2_ERRIE) != 0u) {
				trigger_irq();
			}
			return;
		}
		m_dr = (m_dr & ~DR_RW_MASK) | byte;
		m_sr |= SR_RXNE;
		if ((m_cr2 & CR2_RXNEIE) != 0u) {
			trigger_irq();
		}
	}

	void transmit(uint8_t byte) {
		const uint8_t peer_response = peer != nullptr ? static_cast<uint8_t>(peer->m_dr & DR_RW_MASK) : 0xFFu;
		if (peer != nullptr) {
			peer->receive_from_peer(byte);
		}
		m_dr = (m_dr & ~DR_RW_MASK) | peer_response;
		m_sr |= SR_RXNE | SR_TXE;
		if ((m_cr2 & CR2_RXNEIE) != 0u) {
			trigger_irq();
		}
		if ((m_cr2 & CR2_TXEIE) != 0u) {
			trigger_irq();
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
		case OFF_CR1:
			if (write) m_cr1 = value & CR1_RW_MASK;
			result = m_cr1;
			break;
		case OFF_CR2:
			if (write) m_cr2 = value & CR2_RW_MASK;
			result = m_cr2;
			break;
		case OFF_SR:
			result = m_sr;
			break;
		case OFF_DR:
			if (write) {
				m_dr = value & DR_RW_MASK;
				m_sr &= ~SR_TXE;
				if ((m_cr1 & CR1_SPE) != 0u) {
					transmit(static_cast<uint8_t>(m_dr & 0xFFu));
				}
			} else {
				result = m_dr;
				m_sr &= ~(SR_RXNE | SR_OVR);
			}
			break;
		case OFF_CRCPR:
			if (write) m_crcpoly = value & DR_RW_MASK;
			result = m_crcpoly;
			break;
		case OFF_RXCRCR:
			if (write) m_rxcrcr = value & DR_RW_MASK;
			result = m_rxcrcr;
			break;
		case OFF_TXCRCR:
			if (write) m_txcrcr = value & DR_RW_MASK;
			result = m_txcrcr;
			break;
		case OFF_I2SCFGR:
			if (write) m_i2scfgr = value & I2SCFGR_RW_MASK;
			result = m_i2scfgr;
			break;
		case OFF_I2SPR:
			if (write) m_i2spr = value & I2SPR_RW_MASK;
			result = m_i2spr;
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
