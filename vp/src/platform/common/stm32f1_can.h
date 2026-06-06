#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Can : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Can> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_tx = 0u;
	uint32_t irq_rx0 = 0u;
	uint32_t irq_rx1 = 0u;
	uint32_t irq_sce = 0u;

	explicit Stm32f1Can(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Can::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_mcr = 0x00010002u;
		m_msr = 0u;
		m_tsr = 0x1C000000u;
		m_rf0r = 0u;
		m_rf1r = 0u;
		m_ier = 0u;
		m_esr = 0u;
		m_btr = 0u;
		for (auto &m : m_tx) {
			m = {};
		}
		m_rx0 = {};
		m_rx_fifo.fill({});
		m_rx_head = 0u;
		m_rx_tail = 0u;
		m_rx_count = 0u;
	}

private:
	static constexpr uint64_t OFF_MCR = 0x00u;
	static constexpr uint64_t OFF_MSR = 0x04u;
	static constexpr uint64_t OFF_TSR = 0x08u;
	static constexpr uint64_t OFF_RF0R = 0x0Cu;
	static constexpr uint64_t OFF_RF1R = 0x10u;
	static constexpr uint64_t OFF_IER = 0x14u;
	static constexpr uint64_t OFF_ESR = 0x18u;
	static constexpr uint64_t OFF_BTR = 0x1Cu;
	static constexpr uint64_t OFF_TI0R = 0x180u;
	static constexpr uint64_t OFF_TDT0R = 0x184u;
	static constexpr uint64_t OFF_TDL0R = 0x188u;
	static constexpr uint64_t OFF_TDH0R = 0x18Cu;
	static constexpr uint64_t OFF_RI0R = 0x1B0u;
	static constexpr uint64_t OFF_RDT0R = 0x1B4u;
	static constexpr uint64_t OFF_RDL0R = 0x1B8u;
	static constexpr uint64_t OFF_RDH0R = 0x1BCu;

	static constexpr uint32_t MCR_INRQ = 1u << 0;
	static constexpr uint32_t MCR_SLEEP = 1u << 1;
	static constexpr uint32_t MCR_TXFP = 1u << 2;
	static constexpr uint32_t MCR_RFLM = 1u << 3;
	static constexpr uint32_t MCR_NART = 1u << 4;
	static constexpr uint32_t MCR_AWUM = 1u << 5;
	static constexpr uint32_t MCR_ABOM = 1u << 6;
	static constexpr uint32_t MCR_TTCM = 1u << 7;
	static constexpr uint32_t MCR_RW_MASK = MCR_INRQ | MCR_SLEEP | MCR_TXFP | MCR_RFLM | MCR_NART |
	                                        MCR_AWUM | MCR_ABOM | MCR_TTCM;

	static constexpr uint32_t TSR_RQCP0 = 1u << 0;
	static constexpr uint32_t TSR_TXOK0 = 1u << 1;
	static constexpr uint32_t TSR_TME0 = 1u << 26;
	static constexpr uint32_t TSR_TME1 = 1u << 27;
	static constexpr uint32_t TSR_TME2 = 1u << 28;
	static constexpr uint32_t TSR_RW_MASK = 0x1FFFFFFFu;

	static constexpr uint32_t RF0R_FMP0_MASK = 0x3u;
	static constexpr uint32_t RF0R_FULL0 = 1u << 3;
	static constexpr uint32_t RF0R_FOVR0 = 1u << 4;
	static constexpr uint32_t RF0R_RW_MASK = RF0R_FMP0_MASK | RF0R_FULL0 | RF0R_FOVR0;

	static constexpr uint32_t IER_TMEIE = 1u << 0;
	static constexpr uint32_t IER_FMPIE0 = 1u << 1;
	static constexpr uint32_t IER_FFIE0 = 1u << 2;
	static constexpr uint32_t IER_FOVIE0 = 1u << 3;
	static constexpr uint32_t IER_ERRIE = 1u << 15;
	static constexpr uint32_t IER_WKUIE = 1u << 16;
	static constexpr uint32_t IER_BOFIE = 1u << 17;
	static constexpr uint32_t IER_EPVIE = 1u << 18;
	static constexpr uint32_t IER_EWGIE = 1u << 19;

	struct TxMailbox {
		uint32_t tir = 0u;
		uint32_t tdt = 0u;
		uint32_t tdl = 0u;
		uint32_t tdh = 0u;
	};

	struct RxMailbox {
		uint32_t rir = 0u;
		uint32_t rdt = 0u;
		uint32_t rdl = 0u;
		uint32_t rdh = 0u;
	};

	TxMailbox m_tx[3]{};
	RxMailbox m_rx0{};
	std::array<RxMailbox, 3> m_rx_fifo{};
	unsigned m_rx_head = 0u;
	unsigned m_rx_tail = 0u;
	unsigned m_rx_count = 0u;
	uint32_t m_mcr = 0u;
	uint32_t m_msr = 0u;
	uint32_t m_tsr = 0u;
	uint32_t m_rf0r = 0u;
	uint32_t m_rf1r = 0u;
	uint32_t m_ier = 0u;
	uint32_t m_esr = 0u;
	uint32_t m_btr = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void trigger(uint32_t irq) {
		if (plic != nullptr && irq != 0u) {
			plic->gateway_trigger_interrupt(irq);
		}
	}

	void push_rx0_from_tx(const TxMailbox &tx) {
		RxMailbox rx{};
		rx.rir = tx.tir;
		rx.rdt = tx.tdt;
		rx.rdl = tx.tdl;
		rx.rdh = tx.tdh;
		if (m_rx_count < m_rx_fifo.size()) {
			m_rx_fifo[m_rx_tail] = rx;
			m_rx_tail = (m_rx_tail + 1u) % m_rx_fifo.size();
			++m_rx_count;
			m_rf0r = (m_rf0r & ~RF0R_FMP0_MASK) | (m_rx_count & RF0R_FMP0_MASK);
			if (m_rx_count >= m_rx_fifo.size()) {
				m_rf0r |= RF0R_FULL0;
			}
			m_rx0 = m_rx_fifo[m_rx_head];
			if ((m_ier & IER_FMPIE0) != 0u) {
				trigger(irq_rx0);
			}
		} else {
			m_rf0r |= RF0R_FOVR0;
			if ((m_ier & IER_FOVIE0) != 0u) {
				trigger(irq_rx0);
			}
		}
	}

	void complete_tx(unsigned idx) {
		m_tsr |= TSR_RQCP0 | TSR_TXOK0;
		if ((m_ier & IER_TMEIE) != 0u) {
			trigger(irq_tx);
		}
		push_rx0_from_tx(m_tx[idx]);
	}

	uint64_t mailbox_base(unsigned idx) const {
		return 0x180u + static_cast<uint64_t>(idx) * 0x10u;
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

		const uint64_t addr = trans.get_address();
		switch (addr) {
		case OFF_MCR:
			if (write) m_mcr = value & MCR_RW_MASK;
			result = m_mcr;
			break;
		case OFF_MSR:
			result = m_msr;
			break;
		case OFF_TSR:
			if (write) m_tsr &= ~value;
			result = m_tsr;
			break;
		case OFF_RF0R:
			if (write) {
				if ((value & 0x20u) != 0u && m_rx_count > 0u) {
					m_rx_head = (m_rx_head + 1u) % m_rx_fifo.size();
					--m_rx_count;
					if (m_rx_count > 0u) {
						m_rx0 = m_rx_fifo[m_rx_head];
					} else {
						m_rx0 = {};
					}
					m_rf0r &= ~(RF0R_FMP0_MASK | RF0R_FULL0);
					m_rf0r |= (m_rx_count & RF0R_FMP0_MASK);
				}
				if ((value & RF0R_FOVR0) != 0u) {
					m_rf0r &= ~RF0R_FOVR0;
				}
			}
			result = m_rf0r;
			break;
		case OFF_RF1R:
			if (write) m_rf1r &= ~value;
			result = m_rf1r;
			break;
		case OFF_IER:
			if (write) m_ier = value;
			result = m_ier;
			break;
		case OFF_ESR:
			result = m_esr;
			break;
		case OFF_BTR:
			if (write) m_btr = value;
			result = m_btr;
			break;
		case OFF_RI0R:
			result = m_rx0.rir;
			break;
		case OFF_RDT0R:
			result = m_rx0.rdt;
			break;
		case OFF_RDL0R:
			result = m_rx0.rdl;
			break;
		case OFF_RDH0R:
			if (write) {
				m_rx0.rdh = value;
			}
			result = m_rx0.rdh;
			break;
		default:
			if (addr >= OFF_TI0R && addr < OFF_TI0R + 3u * 0x10u) {
				const unsigned idx = static_cast<unsigned>((addr - OFF_TI0R) / 0x10u);
				const uint64_t off = (addr - OFF_TI0R) % 0x10u;
				TxMailbox &m = m_tx[idx];
				switch (off) {
				case 0x00u:
					if (write) {
						m.tir = value;
						if ((value & 1u) != 0u) {
							complete_tx(idx);
						}
					}
					result = m.tir;
					break;
				case 0x04u:
					if (write) m.tdt = value;
					result = m.tdt;
					break;
				case 0x08u:
					if (write) m.tdl = value;
					result = m.tdl;
					break;
				case 0x0Cu:
					if (write) m.tdh = value;
					result = m.tdh;
					break;
				default:
					trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
					return;
				}
			} else {
				trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
				return;
			}
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
