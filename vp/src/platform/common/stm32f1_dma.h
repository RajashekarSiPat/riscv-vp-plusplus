#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

#include <systemc>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>

#include "core/common/irq_if.h"

class Stm32f1Dma : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Dma> socket{"target_socket"};
	tlm_utils::simple_initiator_socket<Stm32f1Dma> bus_initiator_socket{"bus_initiator_socket"};

	interrupt_gateway *plic = nullptr;
	std::array<uint32_t, 7> irq_ids{};

	explicit Stm32f1Dma(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Dma::b_transport);
		reset();
		irq_ids = {7u, 8u, 9u, 10u, 11u, 12u, 13u};
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void peripheral_reset() { reset(); }

	void reset() {
		m_isr = 0u;
		m_ifcr = 0u;
		for (auto &ch : m_channels) {
			ch = {};
		}
	}

private:
	static constexpr uint64_t OFF_ISR = 0x00u;
	static constexpr uint64_t OFF_IFCR = 0x04u;
	static constexpr uint64_t OFF_CH_BASE = 0x08u;
	static constexpr uint64_t OFF_CH_STRIDE = 0x14u;
	static constexpr uint64_t OFF_CCR = 0x00u;
	static constexpr uint64_t OFF_CNDTR = 0x04u;
	static constexpr uint64_t OFF_CPAR = 0x08u;
	static constexpr uint64_t OFF_CMAR = 0x0Cu;

	static constexpr uint32_t CCR_EN = 1u << 0;
	static constexpr uint32_t CCR_TCIE = 1u << 1;
	static constexpr uint32_t CCR_TEIE = 1u << 3;
	static constexpr uint32_t CCR_DIR = 1u << 4;
	static constexpr uint32_t CCR_CIRC = 1u << 5;
	static constexpr uint32_t CCR_PINC = 1u << 6;
	static constexpr uint32_t CCR_MINC = 1u << 7;
	static constexpr uint32_t CCR_PSIZE_MASK = 0x3u << 8;
	static constexpr uint32_t CCR_MSIZE_MASK = 0x3u << 10;
	static constexpr uint32_t CCR_MEM2MEM = 1u << 14;
	static constexpr uint32_t CCR_RW_MASK = 0x7FFFu;

	struct Channel {
		uint32_t ccr = 0u;
		uint32_t cndtr = 0u;
		uint32_t cpar = 0u;
		uint32_t cmar = 0u;
	};

	std::array<Channel, 7> m_channels{};
	uint32_t m_isr = 0u;
	uint32_t m_ifcr = 0u;
	bool m_clock_enabled = true;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	static unsigned transfer_width(uint32_t bits) {
		switch (bits & 0x3u) {
		case 0u:
			return 1u;
		case 1u:
			return 2u;
		default:
			return 4u;
		}
	}

	bool bus_read(uint64_t addr, uint8_t *data, unsigned len) {
		tlm::tlm_generic_payload trans;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		trans.set_command(tlm::TLM_READ_COMMAND);
		trans.set_address(addr);
		trans.set_data_ptr(data);
		trans.set_data_length(len);
		trans.set_streaming_width(len);
		trans.set_byte_enable_ptr(nullptr);
		trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		bus_initiator_socket->b_transport(trans, delay);
		if (delay != sc_core::SC_ZERO_TIME) {
			sc_core::wait(delay);
		}
		return trans.is_response_ok();
	}

	bool bus_write(uint64_t addr, const uint8_t *data, unsigned len) {
		tlm::tlm_generic_payload trans;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		trans.set_command(tlm::TLM_WRITE_COMMAND);
		trans.set_address(addr);
		trans.set_data_ptr(const_cast<unsigned char *>(data));
		trans.set_data_length(len);
		trans.set_streaming_width(len);
		trans.set_byte_enable_ptr(nullptr);
		trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		bus_initiator_socket->b_transport(trans, delay);
		if (delay != sc_core::SC_ZERO_TIME) {
			sc_core::wait(delay);
		}
		return trans.is_response_ok();
	}

	void set_status(unsigned channel, uint32_t bits) {
		const unsigned shift = channel * 4u;
		m_isr |= bits << shift;
	}

	void clear_status(unsigned channel, uint32_t bits) {
		const unsigned shift = channel * 4u;
		m_isr &= ~(bits << shift);
	}

	void trigger_irq(unsigned channel) {
		if (plic != nullptr) {
			plic->gateway_trigger_interrupt(irq_ids[channel]);
		}
	}

	void complete_channel(unsigned channel) {
		Channel &ch = m_channels[channel];
		if ((ch.ccr & CCR_EN) == 0u || ch.cndtr == 0u) {
			return;
		}

		const bool mem2mem = (ch.ccr & CCR_MEM2MEM) != 0u;
		const bool dir = (ch.ccr & CCR_DIR) != 0u;
		const unsigned src_width = transfer_width(ch.ccr >> 8u);
		const unsigned dst_width = transfer_width(ch.ccr >> 10u);
		const unsigned unit = std::max(src_width, dst_width);
		const bool src_inc = (mem2mem || !dir) ? ((ch.ccr & CCR_PINC) != 0u) : ((ch.ccr & CCR_MINC) != 0u);
		const bool dst_inc = (mem2mem || dir) ? ((ch.ccr & CCR_MINC) != 0u) : ((ch.ccr & CCR_PINC) != 0u);
		uint64_t src = mem2mem ? ch.cpar : (dir ? ch.cmar : ch.cpar);
		uint64_t dst = mem2mem ? ch.cmar : (dir ? ch.cpar : ch.cmar);

		std::array<uint8_t, 4> buf{};
		for (uint32_t i = 0; i < ch.cndtr; ++i) {
			if (!bus_read(src, buf.data(), unit)) {
				set_status(channel, 1u << 3);
				if ((ch.ccr & CCR_TEIE) != 0u) {
					trigger_irq(channel);
				}
				return;
			}
			if (!bus_write(dst, buf.data(), unit)) {
				set_status(channel, 1u << 3);
				if ((ch.ccr & CCR_TEIE) != 0u) {
					trigger_irq(channel);
				}
				return;
			}
			if (src_inc) {
				src += unit;
			}
			if (dst_inc) {
				dst += unit;
			}
		}

		ch.cndtr = 0u;
		set_status(channel, 1u | (1u << 1));
		if ((ch.ccr & CCR_TCIE) != 0u) {
			trigger_irq(channel);
		}
	}

	void write_channel(unsigned channel, uint64_t offset, uint32_t value) {
		Channel &ch = m_channels[channel];
		const uint32_t old_ccr = ch.ccr;

		switch (offset) {
		case OFF_CCR:
			ch.ccr = value & CCR_RW_MASK;
			if (((old_ccr & CCR_EN) == 0u) && ((ch.ccr & CCR_EN) != 0u)) {
				complete_channel(channel);
			}
			break;
		case OFF_CNDTR:
			ch.cndtr = value & 0xFFFFu;
			break;
		case OFF_CPAR:
			ch.cpar = value;
			break;
		case OFF_CMAR:
			ch.cmar = value;
			break;
		default:
			break;
		}
	}

	uint32_t read_channel(unsigned channel, uint64_t offset) const {
		const Channel &ch = m_channels[channel];
		switch (offset) {
		case OFF_CCR:
			return ch.ccr;
		case OFF_CNDTR:
			return ch.cndtr;
		case OFF_CPAR:
			return ch.cpar;
		case OFF_CMAR:
			return ch.cmar;
		default:
			return 0u;
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

		const uint64_t addr = trans.get_address();
		if (addr == OFF_ISR) {
			result = m_isr;
		} else if (addr == OFF_IFCR) {
			if (write) {
				m_ifcr = value;
				for (unsigned ch = 0; ch < m_channels.size(); ++ch) {
					const uint32_t mask = (value >> (ch * 4u)) & 0xFu;
					clear_status(ch, mask);
				}
			}
			result = 0u;
		} else if (addr >= OFF_CH_BASE && addr < OFF_CH_BASE + m_channels.size() * OFF_CH_STRIDE) {
			const unsigned channel = static_cast<unsigned>((addr - OFF_CH_BASE) / OFF_CH_STRIDE);
			const uint64_t off = (addr - OFF_CH_BASE) % OFF_CH_STRIDE;
			if (channel >= m_channels.size()) {
				trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
				return;
			}
			if (write) {
				write_channel(channel, off, value);
			}
			result = read_channel(channel, off);
		} else {
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) {
			write_word(trans, result);
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
