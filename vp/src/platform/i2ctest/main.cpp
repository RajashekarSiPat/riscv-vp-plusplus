#include <boost/program_options.hpp>

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <iostream>

#include "core/common/clint.h"
#include "core/common/gdb-mc/gdb_runner.h"
#include "core/common/gdb-mc/gdb_server.h"
#include "core/rv32/elf_loader.h"
#include "core/rv32/iss.h"
#include "core/rv32/mem.h"
#include "core/rv32/syscall.h"
#include "platform/common/bus.h"
#include "platform/common/ds1307.h"
#include "platform/common/fe310_plic.h"
#include "platform/common/fu540_i2c.h"
#include "platform/common/memory.h"
#include "platform/common/options.h"
#include "util/options.h"
#include "util/propertytree.h"

using namespace rv32;
namespace po = boost::program_options;

struct I2CTestOptions : Options {
	typedef uint64_t addr_t;

	addr_t mem_start_addr = 0x80000000;
	addr_t mem_size = 0x01000000;
	addr_t mem_end_addr = mem_start_addr + mem_size - 1;
	addr_t clint_start_addr = 0x02000000;
	addr_t clint_end_addr = 0x0200ffff;
	addr_t sys_start_addr = 0x02010000;
	addr_t sys_end_addr = 0x020103ff;
	addr_t plic_start_addr = 0x40000000;
	addr_t plic_end_addr = 0x40ffffff;
	addr_t i2c_start_addr = 0x10030000;
	addr_t i2c_end_addr = 0x10030fff;

	void parse(int argc, char **argv) override {
		Options::parse(argc, argv);
		mem_end_addr = mem_start_addr + mem_size - 1;
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
	SyscallHandler sys("SyscallHandler");
	FE310_PLIC<1, 64, 96, 32> plic("PLIC");
	CLINT<1> clint("CLINT");
	FU540_I2C i2c("I2C", 50);
	DS1307 *rtc_ds1307 = new DS1307();

	i2c.register_device(0x68, rtc_ds1307);

	SimpleBus<1, 5> bus("Bus", nullptr, opt.break_on_transaction);
	bus.ports[0] = new PortMapping(opt.mem_start_addr, opt.mem_end_addr, mem);
	bus.ports[1] = new PortMapping(opt.clint_start_addr, opt.clint_end_addr, clint);
	bus.ports[2] = new PortMapping(opt.plic_start_addr, opt.plic_end_addr, plic);
	bus.ports[3] = new PortMapping(opt.i2c_start_addr, opt.i2c_end_addr, i2c);
	bus.ports[4] = new PortMapping(opt.sys_start_addr, opt.sys_end_addr, sys);
	bus.mapping_complete();

	iss_mem_if.isock.bind(bus.tsocks[0]);
	bus.isocks[0].bind(mem.tsock);
	bus.isocks[1].bind(clint.tsock);
	bus.isocks[2].bind(plic.tsock);
	bus.isocks[3].bind(i2c.tsock);
	bus.isocks[4].bind(sys.tsock);

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
	sys.init(mem.data, opt.mem_start_addr, loader.get_heap_addr(mem.get_size(), opt.mem_start_addr));
	sys.register_core(&core);
	if (opt.intercept_syscalls) {
		core.sys = &sys;
	}
	core.error_on_zero_traphandler = opt.error_on_zero_traphandler;

	plic.target_harts[0] = &core;
	clint.target_harts[0] = &core;
	i2c.plic = &plic;

	std::cout << "[VP] I2C test platform ready\n";
	sc_core::sc_start();
	return 0;
}
