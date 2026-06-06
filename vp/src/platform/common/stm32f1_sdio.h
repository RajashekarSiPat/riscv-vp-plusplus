#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Sdio : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Sdio> socket{"target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t irq_id = 0u;

	explicit Stm32f1Sdio(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Sdio::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_power = 0u;
		m_clkcr = 0u;
		m_arg = 0u;
		m_cmd = 0u;
		m_respcmd = 0u;
		m_resp1 = 0u;
		m_resp2 = 0u;
		m_resp3 = 0u;
		m_resp4 = 0u;
		m_dtimer = 0u;
		m_dlen = 0u;
		m_dctrl = 0u;
		m_dcount = 0u;
		m_sta = 0u;
		m_mask = 0u;
		m_fifocnt = 0u;
		m_fifo.fill(0u);
		m_fifo_head = 0u;
		m_fifo_tail = 0u;
		m_fifo_count = 0u;
	}

private:
	static constexpr uint64_t OFF_POWER = 0x00u;
	static constexpr uint64_t OFF_CLKCR = 0x04u;
	static constexpr uint64_t OFF_ARG = 0x08u;
	static constexpr uint64_t OFF_CMD = 0x0Cu;
	static constexpr uint64_t OFF_RESPCMD = 0x10u;
	static constexpr uint64_t OFF_RESP1 = 0x14u;
	static constexpr uint64_t OFF_RESP2 = 0x18u;
	static constexpr uint64_t OFF_RESP3 = 0x1Cu;
	static constexpr uint64_t OFF_RESP4 = 0x20u;
	static constexpr uint64_t OFF_DTIMER = 0x24u;
	static constexpr uint64_t OFF_DLEN = 0x28u;
	static constexpr uint64_t OFF_DCTRL = 0x2Cu;
	static constexpr uint64_t OFF_DCOUNT = 0x30u;
	static constexpr uint64_t OFF_STA = 0x34u;
	static constexpr uint64_t OFF_ICR = 0x38u;
	static constexpr uint64_t OFF_MASK = 0x3Cu;
	static constexpr uint64_t OFF_FIFOCNT = 0x48u;
	static constexpr uint64_t OFF_FIFO = 0x80u;

	static constexpr uint32_t POWER_PWRCTRL_MASK = 0x3u;
	static constexpr uint32_t CLKCR_CLKDIV_MASK = 0xFFu;
	static constexpr uint32_t CLKCR_CLKEN = 1u << 8;
	static constexpr uint32_t CLKCR_PWRSAV = 1u << 9;
	static constexpr uint32_t CLKCR_BYPASS = 1u << 10;
	static constexpr uint32_t CLKCR_WIDBUS_MASK = 0x3u << 11;
	static constexpr uint32_t CLKCR_HWFC_EN = 1u << 14;
	static constexpr uint32_t CLKCR_RW_MASK = POWER_PWRCTRL_MASK | CLKCR_CLKDIV_MASK | CLKCR_CLKEN |
	                                          CLKCR_PWRSAV | CLKCR_BYPASS | CLKCR_WIDBUS_MASK | CLKCR_HWFC_EN;

	static constexpr uint32_t CMD_CMDINDEX_MASK = 0x3Fu;
	static constexpr uint32_t CMD_WAITRESP_MASK = 0x3u << 6;
	static constexpr uint32_t CMD_WAITINT = 1u << 8;
	static constexpr uint32_t CMD_WAITPEND = 1u << 9;
	static constexpr uint32_t CMD_CPSMEN = 1u << 10;
	static constexpr uint32_t CMD_SDIOSUSPEND = 1u << 11;
	static constexpr uint32_t CMD_ENCMDCOMPL = 1u << 12;
	static constexpr uint32_t CMD_NIEN = 1u << 13;
	static constexpr uint32_t CMD_ATACMD = 1u << 14;
	static constexpr uint32_t CMD_RW_MASK = CMD_CMDINDEX_MASK | CMD_WAITRESP_MASK | CMD_WAITINT |
	                                       CMD_WAITPEND | CMD_CPSMEN | CMD_SDIOSUSPEND | CMD_ENCMDCOMPL |
	                                       CMD_NIEN | CMD_ATACMD;

	static constexpr uint32_t DCTRL_DTEN = 1u << 0;
	static constexpr uint32_t DCTRL_DTDIR = 1u << 1;
	static constexpr uint32_t DCTRL_DTMODE = 1u << 2;
	static constexpr uint32_t DCTRL_DMAEN = 1u << 3;
	static constexpr uint32_t DCTRL_DBLOCKSIZE_MASK = 0xFu << 4;
	static constexpr uint32_t DCTRL_RW_MASK = DCTRL_DTEN | DCTRL_DTDIR | DCTRL_DTMODE | DCTRL_DMAEN |
	                                          DCTRL_DBLOCKSIZE_MASK;

	static constexpr uint32_t STA_CCRCFAIL = 1u << 0;
	static constexpr uint32_t STA_DCRCFAIL = 1u << 1;
	static constexpr uint32_t STA_CTIMEOUT = 1u << 2;
	static constexpr uint32_t STA_DTIMEOUT = 1u << 3;
	static constexpr uint32_t STA_TXUNDERR = 1u << 4;
	static constexpr uint32_t STA_RXOVERR = 1u << 5;
	static constexpr uint32_t STA_CMDREND = 1u << 6;
	static constexpr uint32_t STA_CMDSENT = 1u << 7;
	static constexpr uint32_t STA_DATAEND = 1u << 8;
	static constexpr uint32_t STA_CMDACT = 1u << 9;
	static constexpr uint32_t STA_TXACT = 1u << 12;
	static constexpr uint32_t STA_RXACT = 1u << 13;
	static constexpr uint32_t STA_TXFIFOHE = 1u << 14;
	static constexpr uint32_t STA_RXFIFOHF = 1u << 15;
	static constexpr uint32_t STA_RW_MASK = STA_CCRCFAIL | STA_DCRCFAIL | STA_CTIMEOUT | STA_DTIMEOUT |
	                                       STA_TXUNDERR | STA_RXOVERR | STA_CMDREND | STA_CMDSENT |
	                                       STA_DATAEND | STA_CMDACT | STA_TXACT | STA_RXACT |
	                                       STA_TXFIFOHE | STA_RXFIFOHF;

	static constexpr uint32_t MASK_CCRCFAILIE = 1u << 0;
	static constexpr uint32_t MASK_DCRCFAILIE = 1u << 1;
	static constexpr uint32_t MASK_CTIMEOUTIE = 1u << 2;
	static constexpr uint32_t MASK_DTIMEOUTIE = 1u << 3;
	static constexpr uint32_t MASK_TXUNDERRIE = 1u << 4;
	static constexpr uint32_t MASK_RXOVERRIE = 1u << 5;
	static constexpr uint32_t MASK_CMDRENDIE = 1u << 6;
	static constexpr uint32_t MASK_CMDSENTIE = 1u << 7;
	static constexpr uint32_t MASK_DATAENDIE = 1u << 8;
	static constexpr uint32_t MASK_CMDACTIE = 1u << 9;
	static constexpr uint32_t MASK_TXACTIE = 1u << 12;
	static constexpr uint32_t MASK_RXACTIE = 1u << 13;
	static constexpr uint32_t MASK_TXFIFOHEIE = 1u << 14;
	static constexpr uint32_t MASK_RXFIFOHFIE = 1u << 15;
	static constexpr uint32_t MASK_RW_MASK = MASK_CCRCFAILIE | MASK_DCRCFAILIE | MASK_CTIMEOUTIE |
	                                        MASK_DTIMEOUTIE | MASK_TXUNDERRIE | MASK_RXOVERRIE |
	                                        MASK_CMDRENDIE | MASK_CMDSENTIE | MASK_DATAENDIE |
	                                        MASK_CMDACTIE | MASK_TXACTIE | MASK_RXACTIE |
	                                        MASK_TXFIFOHEIE | MASK_RXFIFOHFIE;

	static constexpr unsigned FIFO_DEPTH = 16u;

	uint32_t m_power = 0u;
	uint32_t m_clkcr = 0u;
	uint32_t m_arg = 0u;
	uint32_t m_cmd = 0u;
	uint32_t m_respcmd = 0u;
	uint32_t m_resp1 = 0u;
	uint32_t m_resp2 = 0u;
	uint32_t m_resp3 = 0u;
	uint32_t m_resp4 = 0u;
	uint32_t m_dtimer = 0u;
	uint32_t m_dlen = 0u;
	uint32_t m_dctrl = 0u;
	uint32_t m_dcount = 0u;
	uint32_t m_sta = 0u;
	uint32_t m_mask = 0u;
	uint32_t m_fifocnt = 0u;
	std::array<uint32_t, FIFO_DEPTH> m_fifo{};
	unsigned m_fifo_head = 0u;
	unsigned m_fifo_tail = 0u;
	unsigned m_fifo_count = 0u;
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
		if (plic != nullptr && irq_id != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void update_fifo_status() {
		m_fifocnt = m_fifo_count;
		m_dcount = m_fifo_count * 4u;
		if (m_fifo_count == 0u) {
			m_sta &= ~(STA_TXACT | STA_RXACT | STA_TXFIFOHE | STA_RXFIFOHF);
		} else {
			m_sta |= STA_TXFIFOHE;
			if (m_fifo_count >= 8u) {
				m_sta |= STA_RXFIFOHF;
			}
		}
	}

	void set_status_and_irq(uint32_t status_bits) {
		m_sta |= status_bits;
		if ((m_mask & status_bits) != 0u) {
			trigger_irq();
		}
	}

	void complete_command() {
		const uint32_t cmd_idx = m_cmd & CMD_CMDINDEX_MASK;
		m_respcmd = cmd_idx;
		m_resp1 = m_arg ^ (0x100u | cmd_idx);
		m_resp2 = ~m_arg;
		m_resp3 = m_arg + cmd_idx;
		m_resp4 = m_arg - cmd_idx;
		if ((m_cmd & CMD_WAITRESP_MASK) == 0u) {
			set_status_and_irq(STA_CMDSENT);
		} else {
			set_status_and_irq(STA_CMDREND);
		}
	}

	void push_fifo(uint32_t value) {
		if (m_fifo_count < FIFO_DEPTH) {
			m_fifo[m_fifo_tail] = value;
			m_fifo_tail = (m_fifo_tail + 1u) % FIFO_DEPTH;
			++m_fifo_count;
		} else {
			set_status_and_irq(STA_RXOVERR);
		}
		update_fifo_status();
	}

	uint32_t pop_fifo() {
		if (m_fifo_count == 0u) {
			return 0u;
		}
		const uint32_t value = m_fifo[m_fifo_head];
		m_fifo_head = (m_fifo_head + 1u) % FIFO_DEPTH;
		--m_fifo_count;
		update_fifo_status();
		return value;
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
		case OFF_POWER:
			if (write) m_power = value & POWER_PWRCTRL_MASK;
			result = m_power;
			break;
		case OFF_CLKCR:
			if (write) m_clkcr = value & CLKCR_RW_MASK;
			result = m_clkcr;
			break;
		case OFF_ARG:
			if (write) m_arg = value;
			result = m_arg;
			break;
		case OFF_CMD:
			if (write) {
				m_cmd = value & CMD_RW_MASK;
				if ((m_cmd & CMD_CPSMEN) != 0u) {
					complete_command();
				}
			}
			result = m_cmd;
			break;
		case OFF_RESPCMD:
			result = m_respcmd;
			break;
		case OFF_RESP1:
			result = m_resp1;
			break;
		case OFF_RESP2:
			result = m_resp2;
			break;
		case OFF_RESP3:
			result = m_resp3;
			break;
		case OFF_RESP4:
			result = m_resp4;
			break;
		case OFF_DTIMER:
			if (write) m_dtimer = value;
			result = m_dtimer;
			break;
		case OFF_DLEN:
			if (write) m_dlen = value;
			result = m_dlen;
			break;
		case OFF_DCTRL:
			if (write) {
				m_dctrl = value & DCTRL_RW_MASK;
				if ((m_dctrl & DCTRL_DTEN) != 0u) {
					m_sta |= (m_dctrl & DCTRL_DTDIR) != 0u ? STA_RXACT : STA_TXACT;
				} else {
					m_sta &= ~(STA_TXACT | STA_RXACT);
				}
			}
			result = m_dctrl;
			break;
		case OFF_DCOUNT:
			result = m_dcount;
			break;
		case OFF_STA:
			result = m_sta;
			break;
		case OFF_ICR:
			if (write) {
				m_sta &= ~value;
			}
			result = m_sta;
			break;
		case OFF_MASK:
			if (write) m_mask = value & MASK_RW_MASK;
			result = m_mask;
			break;
		case OFF_FIFOCNT:
			result = m_fifocnt;
			break;
		case OFF_FIFO:
			if (write) {
				push_fifo(value);
				set_status_and_irq(STA_DATAEND);
			} else {
				result = pop_fifo();
				set_status_and_irq(STA_DATAEND);
			}
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
