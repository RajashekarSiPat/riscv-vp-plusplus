#include <boost/program_options.hpp>

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "core/common/clint.h"
#include "core/common/gdb-mc/gdb_runner.h"
#include "core/rv32/elf_loader.h"
#include "core/rv32/iss.h"
#include "core/rv32/mem.h"
#include "platform/common/bus.h"
#include "platform/common/fe310_plic.h"
#include "platform/common/i2c_bus.h"
#include "platform/common/i2c_stm32.h"
#include "platform/common/memory.h"
#include "platform/common/options.h"
#include "util/options.h"
#include "util/propertytree.h"

using namespace rv32;
namespace po = boost::program_options;

struct I2CTestOptions : Options {
	typedef uint64_t addr_t;

	addr_t mem_start_addr = 0x80000000ull;
	addr_t mem_size = 0x01000000ull;
	addr_t mem_end_addr = mem_start_addr + mem_size - 1;
	addr_t clint_start_addr = 0x02000000ull;
	addr_t clint_end_addr = 0x0200ffffull;
	addr_t plic_start_addr = 0x40000000ull;
	addr_t plic_end_addr = 0x40ffffffull;
	addr_t i2c0_start_addr = 0x41005400ull;
	addr_t i2c0_end_addr = 0x410054ffull;
	addr_t i2c1_start_addr = 0x41005800ull;
	addr_t i2c1_end_addr = 0x410058ffull;
	addr_t console_start_addr = 0x09004000ull;
	addr_t console_end_addr = 0x09004fffull;
	addr_t exiter_start_addr = 0x09010000ull;
	addr_t exiter_end_addr = 0x09010fffull;

	void parse(int argc, char **argv) override {
		Options::parse(argc, argv);
		mem_end_addr = mem_start_addr + mem_size - 1;
	}
};

struct ConsoleUart : sc_core::sc_module {
	tlm_utils::simple_target_socket<ConsoleUart> tsock;

	explicit ConsoleUart(sc_core::sc_module_name name) : sc_module(name), tsock("tsock") {
		tsock.register_b_transport(this, &ConsoleUart::b_transport);
	}

	void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &) {
		if (trans.get_command() == tlm::TLM_WRITE_COMMAND && trans.get_address() == 0x04u) {
			const char c = static_cast<char>(*trans.get_data_ptr());
			std::cout << c << std::flush;
		}
		trans.set_response_status(tlm::TLM_OK_RESPONSE);
	}
};

struct Exiter : sc_core::sc_module {
	tlm_utils::simple_target_socket<Exiter> tsock;

	explicit Exiter(sc_core::sc_module_name name) : sc_module(name), tsock("tsock") {
		tsock.register_b_transport(this, &Exiter::b_transport);
	}

	void b_transport(tlm::tlm_generic_payload &, sc_core::sc_time &) {
		sc_core::sc_stop();
	}
};

int sc_main(int argc, char **argv) {
	I2CTestOptions opt;
	opt.parse(argc, argv);

	std::srand(std::time(nullptr));

	if (!opt.property_tree_is_loaded) {
		VPPP_PROPERTY_SET("", "clock_cycle_period", sc_core::sc_time, sc_core::sc_time(10, sc_core::SC_NS));
	}

	tlm::tlm_global_quantum::instance().set(sc_core::sc_time(opt.tlm_global_quantum, sc_core::SC_NS));

	RV_ISA_Config isa_config(opt.use_E_base_isa, opt.en_ext_Zfh);
	ISS core(&isa_config, 0);

	SimpleMemory mem("RAM", opt.mem_size);
	CombinedMemoryInterface iss_mem_if("MemIf", core);
	ELFLoader loader(opt.input_program.c_str());
	FE310_PLIC<1, 16, 32, 32> plic("PLIC");
	CLINT<1> clint("CLINT");
	I2cStm32 i2c0("I2C0");
	I2cStm32 i2c1("I2C1");
	I2cBus i2c_bus("I2C_BUS");
	ConsoleUart console("Console");
	Exiter exiter("Exiter");

	/* self-contained I2C pair */
	i2c0.bus_initiator_socket.bind(i2c_bus.target_socket_0);
	i2c1.bus_initiator_socket.bind(i2c_bus.target_socket_1);
	i2c_bus.initiator_socket_0.bind(i2c0.bus_target_socket);
	i2c_bus.initiator_socket_1.bind(i2c1.bus_target_socket);

	/* Bus: 1 initiator (ISS), 7 targets */
	SimpleBus<1, 7> bus("Bus", nullptr, opt.break_on_transaction);

	{
		unsigned i = 0;
		bus.ports[i++] = new PortMapping(opt.mem_start_addr, opt.mem_end_addr, mem);
		bus.ports[i++] = new PortMapping(opt.clint_start_addr, opt.clint_end_addr, clint);
		bus.ports[i++] = new PortMapping(opt.plic_start_addr, opt.plic_end_addr, plic);
		bus.ports[i++] = new PortMapping(opt.i2c0_start_addr, opt.i2c0_end_addr, i2c0);
		bus.ports[i++] = new PortMapping(opt.i2c1_start_addr, opt.i2c1_end_addr, i2c1);
		bus.ports[i++] = new PortMapping(opt.console_start_addr, opt.console_end_addr, console);
		bus.ports[i++] = new PortMapping(opt.exiter_start_addr, opt.exiter_end_addr, exiter);
	}
	bus.mapping_complete();

	iss_mem_if.isock.bind(bus.tsocks[0]);
	{
		unsigned i = 0;
		bus.isocks[i++].bind(mem.tsock);
		bus.isocks[i++].bind(clint.tsock);
		bus.isocks[i++].bind(plic.tsock);
		bus.isocks[i++].bind(i2c0.socket);
		bus.isocks[i++].bind(i2c1.socket);
		bus.isocks[i++].bind(console.tsock);
		bus.isocks[i++].bind(exiter.tsock);
	}

	std::shared_ptr<BusLock> bus_lock = std::make_shared<BusLock>();
	iss_mem_if.bus_lock = bus_lock;

	MemoryDMI dmi = MemoryDMI::create_start_size_mapping(mem.data, opt.mem_start_addr, mem.get_size());
	InstrMemoryProxy instr_mem(dmi, core);
	iss_mem_if.dmi_add(dmi);
	iss_mem_if.dmi_enable(opt.use_data_dmi);

	instr_memory_if *instr_mem_if = opt.use_instr_dmi ? static_cast<instr_memory_if *>(&instr_mem)
	                                                  : static_cast<instr_memory_if *>(&iss_mem_if);
	data_memory_if *data_mem_if = &iss_mem_if;

	uint64_t entry_point = loader.get_entrypoint();
	try {
		loader.load_executable_image(mem, mem.get_size(), opt.mem_start_addr);
	} catch (ELFLoader::load_executable_exception &e) {
		std::cerr << e.what() << "\nRAM: 0x" << std::hex << opt.mem_start_addr << "\n";
		return -1;
	}

	core.init(instr_mem_if, opt.use_dbbcache, data_mem_if, opt.use_lscache, &clint, entry_point, opt.mem_end_addr);

	/* Interrupt wiring */
	plic.target_harts[0] = &core;
	clint.target_harts[0] = &core;
	i2c0.plic = &plic;
	i2c0.ev_irq_id = 1u;
	i2c0.er_irq_id = 2u;
	i2c1.plic = &plic;
	i2c1.ev_irq_id = 3u;
	i2c1.er_irq_id = 4u;

	/* Keep the slave addresses fixed for the verification suite. */
	i2c0.slave_addr = 0x50u;
	i2c1.slave_addr = 0x51u;

	new DirectCoreRunner(core);
	opt.handle_property_export_and_exit();
	sc_core::sc_start();
	return 0;
}
