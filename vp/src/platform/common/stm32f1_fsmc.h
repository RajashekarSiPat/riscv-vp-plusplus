#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <unordered_map>
#include <utility>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

class Stm32f1FsmcBank : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1FsmcBank> socket{"target_socket"};

	explicit Stm32f1FsmcBank(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1FsmcBank::b_transport);
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void set_enabled(bool enabled) { m_enabled = enabled; }

	void peripheral_reset() { m_enabled = false; }

private:
	static constexpr uint64_t BASE = 0x60000000ull;
	static constexpr uint64_t LIMIT = 0x70000000ull;

	bool m_clock_enabled = true;
	bool m_enabled = false;
	std::unordered_map<uint64_t, uint8_t> m_mem;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	uint8_t read_byte(uint64_t addr) const {
		auto it = m_mem.find(addr);
		return it == m_mem.end() ? 0u : it->second;
	}

	void write_byte(uint64_t addr, uint8_t value) { m_mem[addr] = value; }

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() == 0u) {
			trans.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
			return;
		}

		const bool write = trans.get_command() == tlm::TLM_WRITE_COMMAND;
		const uint64_t addr = trans.get_address();

		if (addr < BASE || addr >= LIMIT) {
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!m_clock_enabled || !m_enabled) {
			if (!write) {
				write_word(trans, 0u);
			}
			trans.set_response_status(tlm::TLM_OK_RESPONSE);
			return;
		}

		const uint32_t len = trans.get_data_length();
		if (write) {
			for (uint32_t i = 0u; i < len; ++i) {
				write_byte(addr + i, trans.get_data_ptr()[i]);
			}
		} else {
			uint32_t value = 0u;
			for (uint32_t i = 0u; i < len; ++i) {
				value |= static_cast<uint32_t>(read_byte(addr + i)) << (8u * i);
			}
			write_word(trans, value);
		}

		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};

class Stm32f1Fsmc : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket<Stm32f1Fsmc> socket{"target_socket"};

	explicit Stm32f1Fsmc(sc_core::sc_module_name name) : sc_module(name) {
		socket.register_b_transport(this, &Stm32f1Fsmc::b_transport);
		reset();
	}

	void set_clock_enabled(bool enabled) { m_clock_enabled = enabled; }

	void set_bank1_enable(std::function<void(bool)> cb) { m_bank1_enable = std::move(cb); }

	void peripheral_reset() { reset(); }

	void reset() {
		m_bcr.fill(0u);
		m_btr.fill(0u);
		m_bwtr.fill(0u);
		m_bank1_enabled = false;
		if (m_bank1_enable) {
			m_bank1_enable(false);
		}
	}

private:
	static constexpr uint64_t OFF_BCR1 = 0x00u;
	static constexpr uint64_t OFF_BTR1 = 0x04u;
	static constexpr uint64_t OFF_BCR2 = 0x08u;
	static constexpr uint64_t OFF_BTR2 = 0x0Cu;
	static constexpr uint64_t OFF_BCR3 = 0x10u;
	static constexpr uint64_t OFF_BTR3 = 0x14u;
	static constexpr uint64_t OFF_BCR4 = 0x18u;
	static constexpr uint64_t OFF_BTR4 = 0x1Cu;

	static constexpr uint32_t BCR_MBKEN = 1u << 0;
	static constexpr uint32_t BCR_MUXEN = 1u << 1;
	static constexpr uint32_t BCR_MTYP_MASK = 0x3u << 2;
	static constexpr uint32_t BCR_MWID_MASK = 0x3u << 4;
	static constexpr uint32_t BCR_FACCEN = 1u << 6;
	static constexpr uint32_t BCR_BURSTEN = 1u << 8;
	static constexpr uint32_t BCR_WAITPOL = 1u << 9;
	static constexpr uint32_t BCR_WRAPMOD = 1u << 10;
	static constexpr uint32_t BCR_WAITCFG = 1u << 11;
	static constexpr uint32_t BCR_WREN = 1u << 12;
	static constexpr uint32_t BCR_WAITEN = 1u << 13;
	static constexpr uint32_t BCR_EXTMOD = 1u << 14;
	static constexpr uint32_t BCR_ASYNCWAIT = 1u << 15;
	static constexpr uint32_t BCR_CBURSTRW = 1u << 19;
	static constexpr uint32_t BCR_CCLKEN = 1u << 20;
	static constexpr uint32_t BCR_RW_MASK = BCR_MBKEN | BCR_MUXEN | BCR_MTYP_MASK | BCR_MWID_MASK |
	                                        BCR_FACCEN | BCR_BURSTEN | BCR_WAITPOL | BCR_WRAPMOD |
	                                        BCR_WAITCFG | BCR_WREN | BCR_WAITEN | BCR_EXTMOD |
	                                        BCR_ASYNCWAIT | BCR_CBURSTRW | BCR_CCLKEN;

	static constexpr uint32_t BTR_BUSTURN_MASK = 0xFu << 4;
	static constexpr uint32_t BTR_DATAST_MASK = 0xFFu << 8;
	static constexpr uint32_t BTR_ADDHLD_MASK = 0xFu << 4;
	static constexpr uint32_t BTR_ADDSET_MASK = 0xFu;
	static constexpr uint32_t BTR_RW_MASK = 0x0FFFFFFFu;

	std::array<uint32_t, 4> m_bcr{};
	std::array<uint32_t, 4> m_btr{};
	std::array<uint32_t, 4> m_bwtr{};
	bool m_bank1_enabled = false;
	bool m_clock_enabled = true;
	std::function<void(bool)> m_bank1_enable;

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	void update_bank1() {
		const bool enabled = (m_bcr[0] & BCR_MBKEN) != 0u;
		if (m_bank1_enabled != enabled) {
			m_bank1_enabled = enabled;
			if (m_bank1_enable) {
				m_bank1_enable(enabled);
			}
		}
	}

	void write_bcr(unsigned idx, uint32_t value) {
		m_bcr[idx] = value & BCR_RW_MASK;
		if (idx == 0u) {
			update_bank1();
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
		case OFF_BCR1:
			if (write) write_bcr(0u, value);
			result = m_bcr[0];
			break;
		case OFF_BTR1:
			if (write) m_btr[0] = value & BTR_RW_MASK;
			result = m_btr[0];
			break;
		case OFF_BCR2:
			if (write) write_bcr(1u, value);
			result = m_bcr[1];
			break;
		case OFF_BTR2:
			if (write) m_btr[1] = value & BTR_RW_MASK;
			result = m_btr[1];
			break;
		case OFF_BCR3:
			if (write) write_bcr(2u, value);
			result = m_bcr[2];
			break;
		case OFF_BTR3:
			if (write) m_btr[2] = value & BTR_RW_MASK;
			result = m_btr[2];
			break;
		case OFF_BCR4:
			if (write) write_bcr(3u, value);
			result = m_bcr[3];
			break;
		case OFF_BTR4:
			if (write) m_btr[3] = value & BTR_RW_MASK;
			result = m_btr[3];
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
