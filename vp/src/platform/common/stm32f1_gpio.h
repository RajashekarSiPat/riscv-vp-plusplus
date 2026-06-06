#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

#include <systemc>
#include <tlm_utils/simple_target_socket.h>

#include "stm32f1_exti.h"

class Stm32f1Afio;

class Stm32f1GpioPort : public sc_core::sc_module {
	public:
	tlm_utils::simple_target_socket<Stm32f1GpioPort> socket{"target_socket"};

	unsigned port_index = 0u;
	Stm32f1Afio *afio = nullptr;
	Stm32f1Exti *exti = nullptr;

	explicit Stm32f1GpioPort(sc_core::sc_module_name name, unsigned port) : sc_module(name), port_index(port) {
		socket.register_b_transport(this, &Stm32f1GpioPort::b_transport);
		reset();
	}

	void reset() {
		m_crl = 0x44444444u;
		m_crh = 0x44444444u;
		m_odr = 0u;
		m_idr = 0u;
		m_bsrr = 0u;
		m_brr = 0u;
		m_lckr = 0u;
		m_input_level.fill(false);
		m_output_level.fill(false);
		m_pad_level.fill(false);
	}

	void set_peer(unsigned pin, std::function<void(bool)> cb) {
		if (pin < m_peer.size()) {
			m_peer[pin] = std::move(cb);
		}
	}

	void set_external_level(unsigned pin, bool level) {
		if (pin >= m_input_level.size()) {
			return;
		}
		m_input_level[pin] = level;
		refresh_pin(pin);
	}

private:
	static constexpr uint64_t OFF_CRL = 0x00u;
	static constexpr uint64_t OFF_CRH = 0x04u;
	static constexpr uint64_t OFF_IDR = 0x08u;
	static constexpr uint64_t OFF_ODR = 0x0Cu;
	static constexpr uint64_t OFF_BSRR = 0x10u;
	static constexpr uint64_t OFF_BRR = 0x14u;
	static constexpr uint64_t OFF_LCKR = 0x18u;

	uint32_t m_crl = 0u;
	uint32_t m_crh = 0u;
	uint32_t m_idr = 0u;
	uint32_t m_odr = 0u;
	uint32_t m_bsrr = 0u;
	uint32_t m_brr = 0u;
	uint32_t m_lckr = 0u;
	std::array<bool, 16> m_input_level{};
	std::array<bool, 16> m_output_level{};
	std::array<bool, 16> m_pad_level{};
	std::array<std::function<void(bool)>, 16> m_peer{};

	static uint32_t read_word(const tlm::tlm_generic_payload &trans) {
		uint32_t value = 0u;
		std::memcpy(&value, trans.get_data_ptr(), std::min(trans.get_data_length(), 4u));
		return value;
	}

	static void write_word(tlm::tlm_generic_payload &trans, uint32_t value) {
		std::memcpy(trans.get_data_ptr(), &value, std::min(trans.get_data_length(), 4u));
	}

	uint32_t config_for_pin(unsigned pin) const {
		if (pin < 8u) {
			return (m_crl >> (pin * 4u)) & 0xFu;
		}
		return (m_crh >> ((pin - 8u) * 4u)) & 0xFu;
	}

	bool is_output(unsigned pin) const {
		return (config_for_pin(pin) & 0x3u) != 0u;
	}

	bool is_pullup_input(unsigned pin) const {
		return config_for_pin(pin) == 0x8u;
	}

	bool pad_level(unsigned pin) const {
		if (is_output(pin)) {
			return m_output_level[pin];
		}
		if (is_pullup_input(pin)) {
			return ((m_odr >> pin) & 1u) != 0u;
		}
		return m_input_level[pin];
	}

	void refresh_pin(unsigned pin) {
		const bool level = pad_level(pin);
		if (m_pad_level[pin] == level) {
			m_idr = (m_idr & ~(1u << pin)) | (level ? (1u << pin) : 0u);
			return;
		}
		m_pad_level[pin] = level;
		m_idr = (m_idr & ~(1u << pin)) | (level ? (1u << pin) : 0u);
		if (exti != nullptr) {
			exti->signal_gpio(port_index, pin, level);
		}
	}

	void drive_output(unsigned pin, bool level) {
		m_output_level[pin] = level;
		refresh_pin(pin);
		if (m_peer[pin]) {
			m_peer[pin](level);
		}
	}

	void apply_odr(uint32_t value) {
		m_odr = value & 0xFFFFu;
		for (unsigned pin = 0; pin < 16u; ++pin) {
			if (is_output(pin)) {
				drive_output(pin, ((m_odr >> pin) & 1u) != 0u);
			} else {
				refresh_pin(pin);
			}
		}
	}

	void apply_bsrr(uint32_t value) {
		m_bsrr = value;
		const uint32_t set = value & 0xFFFFu;
		const uint32_t reset = (value >> 16) & 0xFFFFu;
		m_odr |= set;
		m_odr &= ~reset;
		for (unsigned pin = 0; pin < 16u; ++pin) {
			if (((set | reset) >> pin) & 1u) {
				if (is_output(pin)) {
					drive_output(pin, ((m_odr >> pin) & 1u) != 0u);
				} else {
					refresh_pin(pin);
				}
			}
		}
	}

	void apply_brr(uint32_t value) {
		m_brr = value;
		m_odr &= ~((value & 0xFFFFu));
		for (unsigned pin = 0; pin < 16u; ++pin) {
			if ((value >> pin) & 1u) {
				if (is_output(pin)) {
					drive_output(pin, false);
				} else {
					refresh_pin(pin);
				}
			}
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

		switch (trans.get_address()) {
		case OFF_CRL:
			if (write) {
				m_crl = value;
				for (unsigned pin = 0; pin < 8u; ++pin) {
					if (is_output(pin)) {
						m_output_level[pin] = ((m_odr >> pin) & 1u) != 0u;
					}
					refresh_pin(pin);
				}
			}
			result = m_crl;
			break;
		case OFF_CRH:
			if (write) {
				m_crh = value;
				for (unsigned pin = 8; pin < 16u; ++pin) {
					if (is_output(pin)) {
						m_output_level[pin] = ((m_odr >> pin) & 1u) != 0u;
					}
					refresh_pin(pin);
				}
			}
			result = m_crh;
			break;
		case OFF_IDR:
			result = m_idr & 0xFFFFu;
			break;
		case OFF_ODR:
			if (write) apply_odr(value);
			result = m_odr & 0xFFFFu;
			break;
		case OFF_BSRR:
			if (write) apply_bsrr(value);
			result = m_bsrr;
			break;
		case OFF_BRR:
			if (write) apply_brr(value);
			result = m_brr;
			break;
		case OFF_LCKR:
			if (write) m_lckr = value & 0x0001FFFFu;
			result = m_lckr;
			break;
		default:
			trans.set_response_status(tlm::TLM_ADDRESS_ERROR_RESPONSE);
			return;
		}

		if (!write) write_word(trans, result);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};
