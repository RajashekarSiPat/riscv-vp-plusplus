#pragma once

#include <cstdint>
#include <cstring>
#include <systemc>

#include "core/common/irq_if.h"
#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
enum class I2cBusOp : uint32_t {
	START = 1u,
	ADDR = 2u,
	WRITE = 3u,
	READ = 4u,
	STOP = 5u,
};

enum class I2cSclState : uint32_t {
	toggling = 0u,
	idle = 1u,
};

struct I2cSclTlm {
	sc_core::sc_time period;
	I2cSclState state;

	I2cSclTlm() : period(sc_core::SC_ZERO_TIME), state(I2cSclState::idle) {}
	I2cSclTlm(sc_core::sc_time p, I2cSclState s) : period(p), state(s) {}
};

struct I2cBusFrame {
	uint32_t op = 0u;
	uint32_t addr = 0u;
	uint32_t read = 0u;
	uint32_t ack = 0u;
	uint32_t data = 0u;
	I2cSclTlm scl;
};

class I2cStm32 : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<I2cStm32, 32> socket{"target_socket"};
	tlm_utils::simple_initiator_socket<I2cStm32, 32> bus_initiator_socket{"bus_initiator_socket"};
	tlm_utils::simple_target_socket<I2cStm32, 32> bus_target_socket{"bus_target_socket"};

	interrupt_gateway *plic = nullptr;
	uint32_t ev_irq_id = 0u;
	uint32_t er_irq_id = 0u;
	uint32_t slave_addr = 0x50u;
	uint32_t nack_addr = 0xFFu;

	SC_HAS_PROCESS(I2cStm32);

	explicit I2cStm32(sc_core::sc_module_name name)
	    : sc_module(name) {
		socket.register_b_transport(this, &I2cStm32::b_transport);
		bus_target_socket.register_b_transport(this, &I2cStm32::bus_b_transport);
		SC_METHOD(rx_arm_method);
		sensitive << m_rx_arm_event;
		dont_initialize();
		SC_METHOD(rx_frame_done_method);
		sensitive << m_rx_frame_done_event;
		dont_initialize();
		SC_METHOD(tx_frame_done_method);
		sensitive << m_tx_frame_done_event;
		dont_initialize();
		do_reset();
	}

private:
	static constexpr uint32_t CR1_PE = (1u << 0);
	static constexpr uint32_t CR1_SMBUS = (1u << 1);
	static constexpr uint32_t CR1_SMBTYPE = (1u << 3);
	static constexpr uint32_t CR1_ENARP = (1u << 4);
	static constexpr uint32_t CR1_ENPEC = (1u << 5);
	static constexpr uint32_t CR1_ENGC = (1u << 6);
	static constexpr uint32_t CR1_NOSTRETCH = (1u << 7);
	static constexpr uint32_t CR1_START = (1u << 8);
	static constexpr uint32_t CR1_STOP = (1u << 9);
	static constexpr uint32_t CR1_ACK = (1u << 10);
	static constexpr uint32_t CR1_POS = (1u << 11);
	static constexpr uint32_t CR1_PEC = (1u << 12);
	static constexpr uint32_t CR1_ALERT = (1u << 13);
	static constexpr uint32_t CR1_SWRST = (1u << 15);
	static constexpr uint32_t CR1_RW_MASK =
	    CR1_PE | CR1_SMBUS | CR1_SMBTYPE | CR1_ENARP | CR1_ENPEC | CR1_ENGC | CR1_NOSTRETCH | CR1_ACK | CR1_POS |
	    CR1_PEC | CR1_ALERT;

	static constexpr uint32_t CR2_FREQ_MASK = 0x3Fu;
	static constexpr uint32_t CR2_ITERREN = (1u << 8);
	static constexpr uint32_t CR2_ITEVTEN = (1u << 9);
	static constexpr uint32_t CR2_ITBUFEN = (1u << 10);
	static constexpr uint32_t CR2_DMAEN = (1u << 11);
	static constexpr uint32_t CR2_LAST = (1u << 12);
	static constexpr uint32_t CR2_RW_MASK = CR2_FREQ_MASK | CR2_ITERREN | CR2_ITEVTEN | CR2_ITBUFEN | CR2_DMAEN |
	                                        CR2_LAST;

	static constexpr uint32_t SR1_SB = (1u << 0);
	static constexpr uint32_t SR1_ADDR = (1u << 1);
	static constexpr uint32_t SR1_BTF = (1u << 2);
	static constexpr uint32_t SR1_ADD10 = (1u << 3);
	static constexpr uint32_t SR1_STOPF = (1u << 4);
	static constexpr uint32_t SR1_RxNE = (1u << 6);
	static constexpr uint32_t SR1_TxE = (1u << 7);
	static constexpr uint32_t SR1_BERR = (1u << 8);
	static constexpr uint32_t SR1_ARLO = (1u << 9);
	static constexpr uint32_t SR1_AF = (1u << 10);
	static constexpr uint32_t SR1_OVR = (1u << 11);
	static constexpr uint32_t SR1_PECERR = (1u << 12);
	static constexpr uint32_t SR1_TIMEOUT = (1u << 14);
	static constexpr uint32_t SR1_SMBALERT = (1u << 15);
	static constexpr uint32_t SR1_ERR_MASK =
	    SR1_BERR | SR1_ARLO | SR1_AF | SR1_OVR | SR1_PECERR | SR1_TIMEOUT | SR1_SMBALERT;

	static constexpr uint32_t SR2_MSL = (1u << 0);
	static constexpr uint32_t SR2_BUSY = (1u << 1);
	static constexpr uint32_t SR2_TRA = (1u << 2);
	static constexpr uint32_t SR2_GENCALL = (1u << 4);
	static constexpr uint32_t SR2_SMBDEFAULT = (1u << 5);
	static constexpr uint32_t SR2_SMBHOST = (1u << 6);
	static constexpr uint32_t SR2_DUALF = (1u << 7);

	static constexpr uint32_t OAR1_ADDMODE = (1u << 15);
	static constexpr uint32_t OAR1_RW_MASK = OAR1_ADDMODE | 0x03FFu;
	static constexpr uint32_t OAR2_ENDUAL = (1u << 0);
	static constexpr uint32_t OAR2_RW_MASK = 0x00FFu;

	static constexpr uint64_t OFF_CR1 = 0x00u;
	static constexpr uint64_t OFF_CR2 = 0x04u;
	static constexpr uint64_t OFF_OAR1 = 0x08u;
	static constexpr uint64_t OFF_OAR2 = 0x0Cu;
	static constexpr uint64_t OFF_DR = 0x10u;
	static constexpr uint64_t OFF_SR1 = 0x14u;
	static constexpr uint64_t OFF_SR2 = 0x18u;
	static constexpr uint64_t OFF_CCR = 0x1Cu;
	static constexpr uint64_t OFF_TRISE = 0x20u;

	uint32_t m_cr1 = 0u;
	uint32_t m_cr2 = 0u;
	uint32_t m_oar1 = 0u;
	uint32_t m_oar2 = 0u;
	uint32_t m_dr_tx = 0u;
	uint32_t m_dr_rx = 0u;
	uint32_t m_sr1 = 0u;
	uint32_t m_sr2 = 0u;
	uint32_t m_ccr = 0u;
	uint32_t m_trise = 0u;
	enum class State {
		IDLE,
		STARTED,
		ADDR10_LOW,
		TX_DATA,
		RX_DATA,
		SLAVE_RX,
		SLAVE_TX,
	};

	State m_state = State::IDLE;
	bool m_rx_mode = false;
	bool m_rx_frame_pending = false;
	bool m_tx_frame_pending = false;
	bool m_external_transfer = false;
	uint16_t m_current_addr = 0u;
	uint16_t m_addr10_prefix = 0u;
	bool m_current_addr10 = false;
	sc_core::sc_time m_clk_period = sc_core::SC_ZERO_TIME;
	sc_core::sc_event m_rx_arm_event;
	sc_core::sc_event m_rx_frame_done_event;
	sc_core::sc_event m_tx_frame_done_event;

	void do_reset() {
		m_cr1 = 0u;
		m_cr2 = 0u;
		m_oar1 = 0u;
		m_oar2 = 0u;
		m_dr_tx = 0u;
		m_dr_rx = 0u;
		m_sr1 = 0u;
		m_sr2 = 0u;
		m_ccr = 0u;
		m_trise = 0u;
		m_state = State::IDLE;
		m_rx_mode = false;
		m_rx_frame_pending = false;
		m_tx_frame_pending = false;
		m_external_transfer = false;
		m_current_addr = 0u;
		m_addr10_prefix = 0u;
		m_current_addr10 = false;
		m_clk_period = sc_core::SC_ZERO_TIME;
		m_rx_arm_event.cancel();
		m_rx_frame_done_event.cancel();
		m_tx_frame_done_event.cancel();
	}

	sc_core::sc_time byte_frame_time() const {
		if (m_clk_period != sc_core::SC_ZERO_TIME) {
			uint32_t ccr = m_ccr & 0x0FFFu;
			if (ccr == 0u) {
				ccr = 1u;
			}
			const bool fast = (m_ccr & (1u << 15)) != 0u;
			const bool duty16_9 = (m_ccr & (1u << 14)) != 0u;
			double clocks_per_scl = fast ? static_cast<double>(duty16_9 ? 25u : 3u) * static_cast<double>(ccr)
			                             : 2.0 * static_cast<double>(ccr);
			return m_clk_period * (9.0 * clocks_per_scl);
		}

		uint32_t freq_mhz = m_cr2 & CR2_FREQ_MASK;
		if (freq_mhz < 2u) {
			freq_mhz = 2u;
		}
		uint32_t ccr = m_ccr & 0x0FFFu;
		if (ccr == 0u) {
			ccr = 1u;
		}
		const bool fast = (m_ccr & (1u << 15)) != 0u;
		const bool duty16_9 = (m_ccr & (1u << 14)) != 0u;
		uint64_t scl_period_ns = fast ? (static_cast<uint64_t>(duty16_9 ? 25u : 3u) * ccr * 1000u) / freq_mhz
		                              : (2u * static_cast<uint64_t>(ccr) * 1000u) / freq_mhz;
		if (scl_period_ns == 0u) {
			scl_period_ns = 1u;
		}
		return sc_core::sc_time(9u * scl_period_ns, sc_core::SC_NS);
	}

	bool send_bus_frame(I2cBusFrame &frame) {
		if (bus_initiator_socket.size() == 0u) {
			return false;
		}
		tlm::tlm_generic_payload trans;
		sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
		trans.set_command(tlm::TLM_WRITE_COMMAND);
		trans.set_address(0u);
		trans.set_data_ptr(reinterpret_cast<unsigned char *>(&frame));
		trans.set_data_length(sizeof(frame));
		trans.set_streaming_width(sizeof(frame));
		trans.set_byte_enable_ptr(nullptr);
		trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		bus_initiator_socket->b_transport(trans, delay);
		return trans.is_response_ok();
	}

	bool bus_start() {
		I2cBusFrame frame{};
		frame.op = static_cast<uint32_t>(I2cBusOp::START);
		frame.scl = I2cSclTlm(byte_frame_time(), I2cSclState::toggling);
		return send_bus_frame(frame);
	}

	bool bus_address(uint16_t addr, bool read) {
		I2cBusFrame frame{};
		frame.op = static_cast<uint32_t>(I2cBusOp::ADDR);
		frame.addr = addr;
		frame.read = read ? 1u : 0u;
		frame.scl = I2cSclTlm(byte_frame_time(), I2cSclState::toggling);
		return send_bus_frame(frame) && frame.ack != 0u;
	}

	bool bus_write(uint8_t data) {
		I2cBusFrame frame{};
		frame.op = static_cast<uint32_t>(I2cBusOp::WRITE);
		frame.addr = m_current_addr;
		frame.data = data;
		frame.scl = I2cSclTlm(byte_frame_time(), I2cSclState::toggling);
		return send_bus_frame(frame) && frame.ack != 0u;
	}

	bool bus_read(uint8_t &data) {
		I2cBusFrame frame{};
		frame.op = static_cast<uint32_t>(I2cBusOp::READ);
		frame.addr = m_current_addr;
		frame.read = 1u;
		frame.scl = I2cSclTlm(byte_frame_time(), I2cSclState::toggling);
		if (!send_bus_frame(frame) || frame.ack == 0u) {
			return false;
		}
		data = static_cast<uint8_t>(frame.data & 0xFFu);
		return true;
	}

	void bus_stop() {
		I2cBusFrame frame{};
		frame.op = static_cast<uint32_t>(I2cBusOp::STOP);
		frame.addr = m_current_addr;
		frame.scl = I2cSclTlm(byte_frame_time(), I2cSclState::idle);
		(void)send_bus_frame(frame);
	}

	uint16_t primary_addr() const {
		if (m_oar1 != 0u) {
			return static_cast<uint16_t>(m_oar1 & ((m_oar1 & OAR1_ADDMODE) ? 0x03FFu : 0x7Fu));
		}
		return static_cast<uint16_t>(slave_addr & 0x7Fu);
	}

	bool primary_is_10bit() const {
		return (m_oar1 & OAR1_ADDMODE) != 0u;
	}

	bool matches_own_addr(uint16_t addr, bool &dual_match, bool &general_call) const {
		dual_match = false;
		general_call = false;
		if ((m_cr1 & CR1_PE) == 0u) {
			return false;
		}
		if (addr == 0u && (m_cr1 & CR1_ENGC) != 0u) {
			general_call = true;
			return true;
		}
		if (primary_is_10bit()) {
			return addr > 0x7Fu && (addr & 0x03FFu) == primary_addr();
		}
		if (addr <= 0x7Fu && (addr & 0x7Fu) == primary_addr()) {
			return true;
		}
		if ((m_oar2 & OAR2_ENDUAL) != 0u && addr <= 0x7Fu) {
			const uint16_t addr2 = static_cast<uint16_t>((m_oar2 >> 1u) & 0x7Fu);
			if ((addr & 0x7Fu) == addr2) {
				dual_match = true;
				return true;
			}
		}
		return false;
	}

	bool ev_irq_enabled(uint32_t bits) const {
		if ((m_cr2 & CR2_ITEVTEN) == 0u) {
			return false;
		}
		const uint32_t buf_bits = SR1_TxE | SR1_RxNE;
		if ((bits & buf_bits) && (m_cr2 & CR2_ITBUFEN) == 0u) {
			bits &= ~buf_bits;
		}
		return bits != 0u;
	}

	bool er_irq_enabled(uint32_t bits) const {
		return bits != 0u && (m_cr2 & CR2_ITERREN) != 0u;
	}

	void trigger_irq(uint32_t irq_id) {
		if (plic != nullptr && irq_id != 0u) {
			plic->gateway_trigger_interrupt(irq_id);
		}
	}

	void set_ev_flags(uint32_t bits) {
		m_sr1 |= bits;
		if (ev_irq_enabled(bits)) {
			trigger_irq(ev_irq_id);
		}
	}

	void set_er_flags(uint32_t bits) {
		m_sr1 |= bits;
		if (er_irq_enabled(bits)) {
			trigger_irq(er_irq_id);
		}
	}

	void clear_bus() {
		if (m_external_transfer) {
			bus_stop();
		}
		m_sr1 &= ~(SR1_SB | SR1_ADDR | SR1_ADD10 | SR1_BTF | SR1_RxNE | SR1_TxE);
		m_sr2 &= ~(SR2_MSL | SR2_BUSY | SR2_TRA | SR2_DUALF);
		m_state = State::IDLE;
		m_rx_mode = false;
		m_rx_frame_pending = false;
		m_tx_frame_pending = false;
		m_external_transfer = false;
		m_current_addr = 0u;
		m_addr10_prefix = 0u;
		m_current_addr10 = false;
		m_rx_arm_event.cancel();
		m_rx_frame_done_event.cancel();
		m_tx_frame_done_event.cancel();
	}

	void schedule_rx_frame() {
		if (m_state != State::RX_DATA || m_rx_frame_pending) {
			return;
		}
		if ((m_cr1 & (CR1_PE | CR1_ACK)) != (CR1_PE | CR1_ACK)) {
			return;
		}
		m_rx_frame_pending = true;
		m_rx_frame_done_event.notify(byte_frame_time());
	}

	void schedule_tx_frame() {
		if (m_state != State::TX_DATA || m_tx_frame_pending) {
			return;
		}
		m_tx_frame_pending = true;
		m_tx_frame_done_event.notify(byte_frame_time());
	}

	void bus_b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		trans.set_dmi_allowed(false);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
		if (trans.get_data_length() < sizeof(I2cBusFrame) || trans.get_data_ptr() == nullptr) {
			trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
			return;
		}

		I2cBusFrame frame = *reinterpret_cast<I2cBusFrame *>(trans.get_data_ptr());
		switch (static_cast<I2cBusOp>(frame.op)) {
		case I2cBusOp::START:
			if (m_sr2 & SR2_MSL) {
				set_er_flags(SR1_ARLO);
				clear_bus();
			} else if (m_state == State::SLAVE_RX || m_state == State::SLAVE_TX) {
				set_er_flags(SR1_BERR);
				m_state = State::IDLE;
				m_sr2 &= ~(SR2_BUSY | SR2_TRA | SR2_GENCALL | SR2_DUALF);
			}
			break;

		case I2cBusOp::ADDR: {
			bool dual_match = false;
			bool general_call = false;
			const uint16_t addr = static_cast<uint16_t>(frame.addr & 0x03FFu);
			const bool selected = matches_own_addr(addr, dual_match, general_call) &&
			                      (addr & 0x7Fu) != (nack_addr & 0x7Fu);
			if (selected) {
				m_rx_mode = frame.read == 0u;
				m_state = frame.read ? State::SLAVE_TX : State::SLAVE_RX;
				m_sr1 = (m_sr1 & ~(SR1_RxNE | SR1_BTF)) | SR1_ADDR;
				if (frame.read) {
					m_sr1 |= SR1_TxE;
				}
				m_sr2 = SR2_BUSY | (frame.read ? SR2_TRA : 0u) | (dual_match ? SR2_DUALF : 0u) |
				        (general_call ? SR2_GENCALL : 0u);
				set_ev_flags(SR1_ADDR | (frame.read ? SR1_TxE : 0u));
				frame.ack = 1u;
			}
			break;
		}

		case I2cBusOp::WRITE:
			if (m_state == State::SLAVE_RX && (m_cr1 & CR1_PE) != 0u) {
				if ((m_cr1 & CR1_ACK) == 0u) {
					frame.ack = 0u;
					break;
				}
				if (m_sr1 & SR1_RxNE) {
					set_er_flags(SR1_OVR);
					frame.ack = 0u;
					break;
				}
				m_dr_rx = frame.data & 0xFFu;
				set_ev_flags(SR1_RxNE | SR1_BTF);
				frame.ack = 1u;
			}
			break;

		case I2cBusOp::READ:
			if (m_state == State::SLAVE_TX && (m_cr1 & CR1_PE) != 0u) {
				frame.data = m_dr_tx & 0xFFu;
				m_sr1 &= ~SR1_TxE;
				set_ev_flags(SR1_BTF);
				frame.ack = 1u;
			}
			break;

		case I2cBusOp::STOP:
			if (m_state == State::SLAVE_RX || m_state == State::SLAVE_TX) {
				m_state = State::IDLE;
				m_sr2 &= ~(SR2_BUSY | SR2_TRA);
				set_ev_flags(SR1_STOPF);
			}
			break;
		}

		*reinterpret_cast<I2cBusFrame *>(trans.get_data_ptr()) = frame;
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		const uint64_t off = trans.get_address();
		const bool is_rd = (trans.get_command() == tlm::TLM_READ_COMMAND);
		uint8_t *ptr = trans.get_data_ptr();
		trans.set_dmi_allowed(false);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);

		if (is_rd) {
			uint32_t val = 0u;
			switch (off) {
			case OFF_CR1: val = m_cr1; break;
			case OFF_CR2: val = m_cr2; break;
			case OFF_OAR1: val = m_oar1; break;
			case OFF_OAR2: val = m_oar2; break;
			case OFF_DR:
				val = m_dr_rx & 0xFFu;
				m_sr1 &= ~(SR1_RxNE | SR1_BTF);
				if (m_rx_mode && m_state == State::RX_DATA) {
					m_rx_arm_event.notify(sc_core::SC_ZERO_TIME);
				}
				break;
			case OFF_CCR: val = m_ccr; break;
			case OFF_TRISE: val = m_trise; break;
			case OFF_SR1: val = m_sr1; break;
			case OFF_SR2:
				val = m_sr2;
				if (m_sr1 & SR1_ADDR) {
					m_sr1 &= ~SR1_ADDR;
					if (m_rx_mode && m_state == State::RX_DATA) {
						m_rx_arm_event.notify(sc_core::SC_ZERO_TIME);
					}
				}
				break;
			default:
				break;
			}
			if (ptr != nullptr) {
				std::memcpy(ptr, &val, sizeof(val));
			}
			return;
		}

		uint32_t wval = 0u;
		if (ptr != nullptr) {
			std::memcpy(&wval, ptr, sizeof(wval));
		}

		switch (off) {
		case OFF_CR1:
			handle_cr1_write(wval);
			break;
		case OFF_CR2:
			m_cr2 = wval & CR2_RW_MASK;
			break;
		case OFF_OAR1:
			m_oar1 = wval & OAR1_RW_MASK;
			break;
		case OFF_OAR2:
			m_oar2 = wval & OAR2_RW_MASK;
			break;
		case OFF_CCR:
			m_ccr = wval & 0xCFFFu;
			break;
		case OFF_TRISE:
			m_trise = wval & 0x3Fu;
			break;
		case OFF_DR:
			handle_dr_write(wval);
			break;
		case OFF_SR1:
			m_sr1 &= (wval | ~SR1_ERR_MASK);
			break;
		default:
			break;
		}
	}

	void handle_cr1_write(uint32_t val) {
		if (val & CR1_SWRST) {
			do_reset();
			return;
		}
		if ((m_cr1 & CR1_PE) && !(val & CR1_PE)) {
			do_reset();
			return;
		}
		m_cr1 = val & CR1_RW_MASK;
		if (val & CR1_STOP) {
			clear_bus();
			return;
		}
		if ((val & CR1_START) && (val & CR1_PE)) {
			m_external_transfer = false;
			const bool repeated_10bit_start = m_current_addr10 && m_current_addr != 0u;
			if (!repeated_10bit_start) {
				m_current_addr = 0u;
				m_current_addr10 = false;
			}
			m_addr10_prefix = 0u;
			m_sr1 &= ~(SR1_ADDR | SR1_ADD10 | SR1_BTF | SR1_RxNE | SR1_TxE);
			m_sr2 = SR2_MSL | SR2_BUSY;
			m_state = State::STARTED;
			m_rx_mode = false;
			(void)bus_start();
			set_ev_flags(SR1_SB);
		}
	}

	void complete_address(uint16_t addr, bool read, bool addr10) {
		m_rx_mode = read;
		const uint32_t nack = nack_addr & 0x7Fu;
		const bool external_ack = bus_address(addr, read);
		if (external_ack && (addr & 0x7Fu) != nack) {
			m_external_transfer = true;
			m_current_addr = addr;
			m_current_addr10 = addr10;
			m_addr10_prefix = 0u;
			m_sr1 = (m_sr1 & ~(SR1_SB | SR1_ADD10 | SR1_TxE | SR1_RxNE | SR1_BTF)) | SR1_ADDR;
			m_sr2 = SR2_MSL | SR2_BUSY | (read ? 0u : SR2_TRA);
			m_state = read ? State::RX_DATA : State::TX_DATA;
			set_ev_flags(SR1_ADDR);
		} else {
			m_sr1 = (m_sr1 & ~(SR1_SB | SR1_ADD10)) | SR1_AF;
			m_state = State::IDLE;
			m_sr2 &= ~(SR2_MSL | SR2_BUSY | SR2_TRA);
			m_current_addr10 = false;
			m_addr10_prefix = 0u;
			set_er_flags(SR1_AF);
		}
	}

	void handle_dr_write(uint32_t val) {
		const uint8_t byte = static_cast<uint8_t>(val & 0xFFu);
		if (m_state == State::STARTED) {
			const bool addr10_header = (byte & 0xF8u) == 0xF0u;
			if (addr10_header) {
				const uint16_t prefix = static_cast<uint16_t>(byte & 0x06u) << 7u;
				const bool read_header = (byte & 1u) != 0u;
				if (read_header && m_current_addr10 && (m_current_addr & 0x0300u) == prefix) {
					m_rx_mode = true;
					const bool external_ack = bus_address(m_current_addr, true);
					if (external_ack) {
						m_external_transfer = true;
						m_sr1 = (m_sr1 & ~(SR1_SB | SR1_ADD10 | SR1_TxE | SR1_RxNE | SR1_BTF)) | SR1_ADDR;
						m_sr2 = SR2_MSL | SR2_BUSY;
						m_state = State::RX_DATA;
						set_ev_flags(SR1_ADDR);
					} else {
						m_sr1 = (m_sr1 & ~SR1_SB) | SR1_AF;
						m_state = State::IDLE;
						m_sr2 &= ~(SR2_MSL | SR2_BUSY | SR2_TRA);
						set_er_flags(SR1_AF);
					}
					return;
				}
				m_rx_mode = false;
				m_addr10_prefix = prefix;
				m_state = State::ADDR10_LOW;
				m_sr1 = (m_sr1 & ~SR1_SB) | SR1_ADD10;
				set_ev_flags(SR1_ADD10);
				return;
			}
			complete_address(static_cast<uint16_t>(byte >> 1u), (byte & 1u) != 0u, false);
		} else if (m_state == State::ADDR10_LOW) {
			complete_address(static_cast<uint16_t>(m_addr10_prefix | byte), false, true);
		} else if (m_state == State::TX_DATA) {
			m_dr_tx = byte;
			m_sr1 &= ~(SR1_TxE | SR1_BTF);
			if (m_external_transfer && !bus_write(byte)) {
				set_er_flags(SR1_AF);
				return;
			}
			schedule_tx_frame();
		} else if (m_state == State::SLAVE_TX) {
			m_dr_tx = byte;
			m_sr1 &= ~SR1_TxE;
		} else if (m_state == State::IDLE) {
			m_dr_tx = byte;
		}
	}

	void rx_arm_method() {
		schedule_rx_frame();
	}

	void rx_frame_done_method() {
		m_rx_frame_pending = false;
		if (m_state != State::RX_DATA || (m_cr1 & CR1_PE) == 0u) {
			return;
		}
		if (m_sr1 & SR1_RxNE) {
			set_er_flags(SR1_OVR);
			return;
		}
		if (m_external_transfer) {
			uint8_t data = 0u;
			if (!bus_read(data)) {
				set_er_flags(SR1_AF);
				return;
			}
			m_dr_rx = data;
		} else {
			set_er_flags(SR1_AF);
			return;
		}
		set_ev_flags(SR1_RxNE);
	}

	void tx_frame_done_method() {
		m_tx_frame_pending = false;
		if (m_state != State::TX_DATA || (m_cr1 & CR1_PE) == 0u) {
			return;
		}
		(void)m_dr_tx;
		set_ev_flags(SR1_TxE | SR1_BTF);
	}
};
