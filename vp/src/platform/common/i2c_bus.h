#pragma once

#include <systemc>

#include "tlm.h"
#include "tlm_utils/simple_initiator_socket.h"
#include "tlm_utils/simple_target_socket.h"
#include "platform/common/i2c_stm32.h"

class I2cBus : public sc_core::sc_module {
public:
	tlm_utils::simple_target_socket_tagged<I2cBus, 32> target_socket_0{"target_socket_0"};
	tlm_utils::simple_target_socket_tagged<I2cBus, 32> target_socket_1{"target_socket_1"};
	tlm_utils::simple_initiator_socket<I2cBus, 32> initiator_socket_0{"initiator_socket_0"};
	tlm_utils::simple_initiator_socket<I2cBus, 32> initiator_socket_1{"initiator_socket_1"};

	SC_HAS_PROCESS(I2cBus);

	explicit I2cBus(sc_core::sc_module_name name) : sc_module(name) {
		target_socket_0.register_b_transport(this, &I2cBus::b_transport, 0);
		target_socket_1.register_b_transport(this, &I2cBus::b_transport, 1);
	}

private:
	void b_transport(int source, tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		trans.set_dmi_allowed(false);
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
		if (trans.get_data_ptr() == nullptr || trans.get_data_length() < sizeof(I2cBusFrame)) {
			trans.set_response_status(tlm::TLM_BURST_ERROR_RESPONSE);
			return;
		}

		I2cBusFrame frame = *reinterpret_cast<I2cBusFrame *>(trans.get_data_ptr());
		bool acked = false;
		if (source != 0 && initiator_socket_0.size() > 0u) {
			acked |= forward(initiator_socket_0, frame, delay);
		}
		if (source != 1 && initiator_socket_1.size() > 0u) {
			acked |= forward(initiator_socket_1, frame, delay);
		}
		if (acked) {
			frame.ack = 1u;
		}
		*reinterpret_cast<I2cBusFrame *>(trans.get_data_ptr()) = frame;
	}

	bool forward(tlm_utils::simple_initiator_socket<I2cBus, 32> &socket,
	             I2cBusFrame &frame,
	             sc_core::sc_time &delay) {
		tlm::tlm_generic_payload fwd;
		I2cBusFrame local = frame;
		fwd.set_command(tlm::TLM_WRITE_COMMAND);
		fwd.set_address(0u);
		fwd.set_data_ptr(reinterpret_cast<unsigned char *>(&local));
		fwd.set_data_length(sizeof(local));
		fwd.set_streaming_width(sizeof(local));
		fwd.set_byte_enable_ptr(nullptr);
		fwd.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
		socket->b_transport(fwd, delay);
		if (!fwd.is_response_ok()) {
			return false;
		}
		if (local.ack) {
			frame = local;
			return true;
		}
		return false;
	}
};
