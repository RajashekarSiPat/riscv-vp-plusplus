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
#include "platform/common/stm32f1_adc.h"
#include "platform/common/stm32f1_eth.h"
#include "platform/common/stm32f1_can.h"
#include "platform/common/stm32f1_dac.h"
#include "platform/common/stm32f1_flash.h"
#include "platform/common/stm32f1_fsmc.h"
#include "platform/common/stm32f1_otg_fs.h"
#include "platform/common/stm32f1_sdio.h"
#include "platform/common/stm32f1_usb.h"
#include "platform/common/i2c_bus.h"
#include "platform/common/i2c_stm32.h"
#include "platform/common/memory.h"
#include "platform/common/options.h"
#include "platform/common/stm32f1_afio.h"
#include "platform/common/stm32f1_bkp.h"
#include "platform/common/stm32f1_crc.h"
#include "platform/common/stm32f1_dma.h"
#include "platform/common/stm32f1_exti.h"
#include "platform/common/stm32f1_gpio.h"
#include "platform/common/stm32f1_map.h"
#include "platform/common/stm32f1_rcc.h"
#include "platform/common/stm32f1_pwr.h"
#include "platform/common/stm32f1_rtc.h"
#include "platform/common/stm32f1_watchdog.h"
#include "platform/common/stm32f1_tim.h"
#include "platform/common/stm32f1_spi.h"
#include "platform/common/stm32f1_usart.h"
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
	addr_t i2c0_start_addr = stm32f1::I2C1;
	addr_t i2c0_end_addr = 0x410054ffull;
	addr_t i2c1_start_addr = stm32f1::I2C2;
	addr_t i2c1_end_addr = 0x410058ffull;
	addr_t rcc_start_addr = stm32f1::RCC;
	addr_t rcc_end_addr = stm32f1::RCC + 0xffull;
	addr_t afio_start_addr = stm32f1::AFIO;
	addr_t afio_end_addr = stm32f1::AFIO + 0xffull;
	addr_t exti_start_addr = stm32f1::EXTI;
	addr_t exti_end_addr = stm32f1::EXTI + 0xffull;
	addr_t gpioa_start_addr = stm32f1::GPIOA;
	addr_t gpioa_end_addr = stm32f1::GPIOA + 0xffull;
	addr_t gpiob_start_addr = stm32f1::GPIOB;
	addr_t gpiob_end_addr = stm32f1::GPIOB + 0xffull;
	addr_t gpioc_start_addr = stm32f1::GPIOC;
	addr_t gpioc_end_addr = stm32f1::GPIOC + 0xffull;
	addr_t gpiod_start_addr = stm32f1::GPIOD;
	addr_t gpiod_end_addr = stm32f1::GPIOD + 0xffull;
	addr_t gpioe_start_addr = stm32f1::GPIOE;
	addr_t gpioe_end_addr = stm32f1::GPIOE + 0xffull;
	addr_t dma1_start_addr = stm32f1::DMA1;
	addr_t dma1_end_addr = stm32f1::DMA1 + 0x1ffull;
	addr_t flash_start_addr = stm32f1::FLASH;
	addr_t flash_end_addr = stm32f1::FLASH + 0xffull;
	addr_t adc1_start_addr = stm32f1::ADC1;
	addr_t adc1_end_addr = stm32f1::ADC1 + 0xffull;
	addr_t adc2_start_addr = stm32f1::ADC2;
	addr_t adc2_end_addr = stm32f1::ADC2 + 0xffull;
	addr_t dac_start_addr = stm32f1::DAC;
	addr_t dac_end_addr = stm32f1::DAC + 0xffull;
	addr_t can1_start_addr = stm32f1::CAN1;
	addr_t can1_end_addr = stm32f1::CAN1 + 0x3ffull;
	addr_t sdio_start_addr = stm32f1::SDIO;
	addr_t sdio_end_addr = stm32f1::SDIO + 0xffull;
	addr_t fsmc_start_addr = stm32f1::FSMC;
	addr_t fsmc_end_addr = stm32f1::FSMC + 0xfffull;
	addr_t usb_start_addr = stm32f1::USB_FS;
	addr_t usb_end_addr = stm32f1::USB_FS + 0x3ffull;
	addr_t otgfs_start_addr = stm32f1::OTG_FS;
	addr_t otgfs_end_addr = stm32f1::OTG_FS + 0x3ffffull;
	addr_t eth_start_addr = stm32f1::ETH;
	addr_t eth_end_addr = stm32f1::ETH + 0x1fffull;
	addr_t crc_start_addr = stm32f1::CRC;
	addr_t crc_end_addr = stm32f1::CRC + 0xffull;
	addr_t spi1_start_addr = stm32f1::SPI1;
	addr_t spi1_end_addr = stm32f1::SPI1 + 0xffull;
	addr_t spi2_start_addr = stm32f1::SPI2;
	addr_t spi2_end_addr = stm32f1::SPI2 + 0xffull;
	addr_t usart1_start_addr = stm32f1::USART1;
	addr_t usart1_end_addr = stm32f1::USART1 + 0xffull;
	addr_t usart2_start_addr = stm32f1::USART2;
	addr_t usart2_end_addr = stm32f1::USART2 + 0xffull;
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
	FE310_PLIC<1, 32, 32, 32> plic("PLIC");
	CLINT<1> clint("CLINT");
	I2cStm32 i2c0("I2C0");
	I2cStm32 i2c1("I2C1");
	I2cBus i2c_bus("I2C_BUS");
	Stm32f1Rcc rcc("RCC");
	Stm32f1Flash flash("FLASH");
	Stm32f1Pwr pwr("PWR");
	Stm32f1Bkp bkp("BKP");
	Stm32f1Rtc rtc("RTC");
	Stm32f1Wwdg wwdg("WWDG");
	Stm32f1Iwdg iwdg("IWDG");
	Stm32f1Tim tim1("TIM1");
	Stm32f1Tim tim2("TIM2");
	Stm32f1Tim tim3("TIM3");
	Stm32f1Tim tim4("TIM4");
	Stm32f1Tim tim5("TIM5");
	Stm32f1Afio afio("AFIO");
	Stm32f1Exti exti("EXTI");
	Stm32f1GpioPort gpioa("GPIOA", 0u);
	Stm32f1GpioPort gpiob("GPIOB", 1u);
	Stm32f1GpioPort gpioc("GPIOC", 2u);
	Stm32f1GpioPort gpiod("GPIOD", 3u);
	Stm32f1GpioPort gpioe("GPIOE", 4u);
	Stm32f1Adc adc1("ADC1", 0u);
	Stm32f1Adc adc2("ADC2", 1u);
	Stm32f1Dac dac("DAC");
	Stm32f1Can can1("CAN1");
	Stm32f1Fsmc fsmc("FSMC");
	Stm32f1FsmcBank fsmc_bank("FSMC_BANK");
	Stm32f1UsbDeviceFs usb_dev("USBDEV");
	Stm32f1OtgFs otg_fs("OTGFS");
	Stm32f1Eth eth("ETH");
	Stm32f1Sdio sdio("SDIO");
	Stm32f1Dma dma1("DMA1");
	Stm32f1Crc crc("CRC");
	Stm32f1Spi spi1("SPI1");
	Stm32f1Spi spi2("SPI2");
	Stm32f1Usart usart1("USART1");
	Stm32f1Usart usart2("USART2");
	ConsoleUart console("Console");
	Exiter exiter("Exiter");
	sc_core::sc_fifo<uint8_t> usart1_to_2("USART1_TO_2", 16);
	sc_core::sc_fifo<uint8_t> usart2_to_1("USART2_TO_1", 16);

	/* self-contained I2C pair */
	i2c0.bus_initiator_socket.bind(i2c_bus.target_socket_0);
	i2c1.bus_initiator_socket.bind(i2c_bus.target_socket_1);
	i2c_bus.initiator_socket_0.bind(i2c0.bus_target_socket);
	i2c_bus.initiator_socket_1.bind(i2c1.bus_target_socket);
	rcc.bind_apb1(21u, [&](bool enabled) { i2c0.set_clock_enabled(enabled); },
	              [&]() { i2c0.peripheral_reset(); });
	rcc.bind_apb1(22u, [&](bool enabled) { i2c1.set_clock_enabled(enabled); },
	              [&]() { i2c1.peripheral_reset(); });
	rcc.bind_apb1(27u, [&](bool enabled) { bkp.set_clock_enabled(enabled); },
	              [&]() { bkp.peripheral_reset(); });
	rcc.bind_apb1(28u, [&](bool enabled) { pwr.set_clock_enabled(enabled); },
	              [&]() { pwr.peripheral_reset(); });
	rcc.bind_apb1(11u, [&](bool enabled) { wwdg.set_clock_enabled(enabled); },
	              [&]() { wwdg.peripheral_reset(); });
	rcc.bind_apb2(0u, [&](bool) {}, [&]() { afio.reset(); });
	rcc.bind_apb2(2u, [&](bool) {}, [&]() { gpioa.reset(); });
	rcc.bind_apb2(3u, [&](bool) {}, [&]() { gpiob.reset(); });
	rcc.bind_apb2(4u, [&](bool) {}, [&]() { gpioc.reset(); });
	rcc.bind_apb2(5u, [&](bool) {}, [&]() { gpiod.reset(); });
	rcc.bind_apb2(6u, [&](bool) {}, [&]() { gpioe.reset(); });
	rcc.bind_apb2(14u, [&](bool enabled) { usart1.set_clock_enabled(enabled); },
	              [&]() { usart1.peripheral_reset(); });
	rcc.bind_apb2(12u, [&](bool enabled) { spi1.set_clock_enabled(enabled); }, [&]() { spi1.peripheral_reset(); });
	rcc.bind_apb2(9u, [&](bool enabled) { adc1.set_clock_enabled(enabled); }, [&]() { adc1.peripheral_reset(); });
	rcc.bind_apb2(10u, [&](bool enabled) { adc2.set_clock_enabled(enabled); }, [&]() { adc2.peripheral_reset(); });
	rcc.bind_ahb(0u, [&](bool enabled) { dma1.set_clock_enabled(enabled); }, [&]() { dma1.peripheral_reset(); });
	rcc.bind_ahb(6u, [&](bool enabled) { crc.set_clock_enabled(enabled); }, [&]() { crc.peripheral_reset(); });
	rcc.bind_apb1(17u, [&](bool enabled) { usart2.set_clock_enabled(enabled); },
	              [&]() { usart2.peripheral_reset(); });
	rcc.bind_apb1(14u, [&](bool enabled) { spi2.set_clock_enabled(enabled); }, [&]() { spi2.peripheral_reset(); });
	rcc.bind_apb1(29u, [&](bool enabled) { dac.set_clock_enabled(enabled); }, [&]() { dac.peripheral_reset(); });
	rcc.bind_apb1(25u, [&](bool enabled) { can1.set_clock_enabled(enabled); }, [&]() { can1.peripheral_reset(); });
	rcc.bind_apb1(23u, [&](bool enabled) { usb_dev.set_clock_enabled(enabled); }, [&]() { usb_dev.peripheral_reset(); });
	rcc.bind_ahb(8u, [&](bool enabled) { fsmc.set_clock_enabled(enabled); fsmc_bank.set_clock_enabled(enabled); },
	              [&]() {
		              fsmc.peripheral_reset();
		              fsmc_bank.peripheral_reset();
	              });
	rcc.bind_ahb(12u, [&](bool enabled) { otg_fs.set_clock_enabled(enabled); }, [&]() { otg_fs.peripheral_reset(); });
	uint32_t eth_clk_bits = 0u;
	auto update_eth_clock = [&]() {
		const uint32_t required = stm32f1::RCC_AHB_ETHMAC | stm32f1::RCC_AHB_ETHMACTX | stm32f1::RCC_AHB_ETHMACRX;
		eth.set_clock_enabled((eth_clk_bits & required) == required);
	};
	rcc.bind_ahb(14u,
	             [&](bool enabled) {
		             if (enabled) {
			             eth_clk_bits |= stm32f1::RCC_AHB_ETHMAC;
		             } else {
			             eth_clk_bits &= ~stm32f1::RCC_AHB_ETHMAC;
		             }
		             update_eth_clock();
	             },
	             [&]() { eth.peripheral_reset(); });
	rcc.bind_ahb(15u,
	             [&](bool enabled) {
		             if (enabled) {
			             eth_clk_bits |= stm32f1::RCC_AHB_ETHMACTX;
		             } else {
			             eth_clk_bits &= ~stm32f1::RCC_AHB_ETHMACTX;
		             }
		             update_eth_clock();
	             },
	             [&]() { eth.peripheral_reset(); });
	rcc.bind_ahb(16u,
	             [&](bool enabled) {
		             if (enabled) {
			             eth_clk_bits |= stm32f1::RCC_AHB_ETHMACRX;
		             } else {
			             eth_clk_bits &= ~stm32f1::RCC_AHB_ETHMACRX;
		             }
		             update_eth_clock();
	             },
	             [&]() { eth.peripheral_reset(); });
	rcc.bind_ahb(10u, [&](bool enabled) { sdio.set_clock_enabled(enabled); }, [&]() { sdio.peripheral_reset(); });
	rcc.bind_apb2(11u, [&](bool enabled) { tim1.set_clock_enabled(enabled); }, [&]() { tim1.peripheral_reset(); });
	rcc.bind_apb1(0u, [&](bool enabled) { tim2.set_clock_enabled(enabled); }, [&]() { tim2.peripheral_reset(); });
	rcc.bind_apb1(1u, [&](bool enabled) { tim3.set_clock_enabled(enabled); }, [&]() { tim3.peripheral_reset(); });
	rcc.bind_apb1(2u, [&](bool enabled) { tim4.set_clock_enabled(enabled); }, [&]() { tim4.peripheral_reset(); });
	rcc.bind_apb1(3u, [&](bool enabled) { tim5.set_clock_enabled(enabled); }, [&]() { tim5.peripheral_reset(); });
	rcc.bind_bdcr([&](bool enabled) { rtc.set_enabled(enabled); },
	              [&]() {
		              bkp.peripheral_reset();
		              rtc.backup_reset();
	              });
	pwr.set_clock_enabled(true);
	bkp.pwr = &pwr;
	rtc.pwr = &pwr;
	rtc.irq_id = stm32f1::IRQ_RTC;
	flash.plic = &plic;
	flash.irq_id = stm32f1::IRQ_FLASH;
	wwdg.plic = &plic;
	wwdg.irq_id = stm32f1::IRQ_WWDG;
	exti.plic = &plic;
	exti.afio = &afio;
	exti.irq_ids[1u] = 6u;
	gpioa.afio = &afio;
	gpioa.exti = &exti;
	gpiob.afio = &afio;
	gpiob.exti = &exti;
	gpioc.afio = &afio;
	gpioc.exti = &exti;
	gpiod.afio = &afio;
	gpiod.exti = &exti;
	gpioe.afio = &afio;
	gpioe.exti = &exti;
	adc1.plic = &plic;
	adc1.irq_id = stm32f1::IRQ_ADC1_2;
	adc2.plic = &plic;
	adc2.irq_id = stm32f1::IRQ_ADC1_2;
	can1.plic = &plic;
	can1.irq_tx = stm32f1::IRQ_CAN1_TX;
	can1.irq_rx0 = stm32f1::IRQ_CAN1_RX0;
	can1.irq_rx1 = stm32f1::IRQ_CAN1_RX1;
	can1.irq_sce = stm32f1::IRQ_CAN1_SCE;
	fsmc.set_bank1_enable([&](bool enabled) { fsmc_bank.set_enabled(enabled); });
	usb_dev.plic = &plic;
	usb_dev.irq_id = stm32f1::IRQ_USB;
	otg_fs.plic = &plic;
	otg_fs.irq_id = stm32f1::IRQ_USB;
	eth.plic = &plic;
	eth.irq_id = stm32f1::IRQ_ETH;
	sdio.plic = &plic;
	sdio.irq_id = stm32f1::IRQ_SDIO;
	dma1.plic = &plic;
	spi1.plic = &plic;
	spi1.irq_id = 5u;
	spi2.plic = &plic;
	spi2.irq_id = 8u;
	spi1.set_peer(&spi2);
	spi2.set_peer(&spi1);
	usart1.plic = &plic;
	usart1.irq_id = 14u;
	usart2.plic = &plic;
	usart2.irq_id = 15u;
	usart1.tx_port(usart1_to_2);
	usart2.rx_port(usart1_to_2);
	usart2.tx_port(usart2_to_1);
	usart1.rx_port(usart2_to_1);
	gpioa.set_peer(0u, [&](bool level) { gpiob.set_external_level(1u, level); });
	tim1.plic = &plic;
	tim1.irq_id = stm32f1::IRQ_TIM1_UP;
	tim2.plic = &plic;
	tim2.irq_id = stm32f1::IRQ_TIM2_UP;
	tim3.plic = &plic;
	tim3.irq_id = stm32f1::IRQ_TIM3_UP;
	tim4.plic = &plic;
	tim4.irq_id = stm32f1::IRQ_TIM4_UP;
	tim5.plic = &plic;
	tim5.irq_id = stm32f1::IRQ_TIM5_UP;

	/* Bus: 3 initiators (ISS, DMA1, ETH) and 42 targets */
	SimpleBus<3, 42> bus("Bus", nullptr, opt.break_on_transaction);

	{
		unsigned i = 0;
		bus.ports[i++] = new PortMapping(opt.mem_start_addr, opt.mem_end_addr, mem);
		bus.ports[i++] = new PortMapping(opt.clint_start_addr, opt.clint_end_addr, clint);
		bus.ports[i++] = new PortMapping(opt.plic_start_addr, opt.plic_end_addr, plic);
		bus.ports[i++] = new PortMapping(opt.i2c0_start_addr, opt.i2c0_end_addr, i2c0);
		bus.ports[i++] = new PortMapping(opt.i2c1_start_addr, opt.i2c1_end_addr, i2c1);
		bus.ports[i++] = new PortMapping(opt.rcc_start_addr, opt.rcc_end_addr, rcc);
		bus.ports[i++] = new PortMapping(opt.flash_start_addr, opt.flash_end_addr, flash);
		bus.ports[i++] = new PortMapping(stm32f1::PWR, stm32f1::PWR + 0xffull, pwr);
		bus.ports[i++] = new PortMapping(stm32f1::BKP, stm32f1::BKP + 0xffull, bkp);
		bus.ports[i++] = new PortMapping(stm32f1::RTC, stm32f1::RTC + 0xffull, rtc);
		bus.ports[i++] = new PortMapping(stm32f1::WWDG, stm32f1::WWDG + 0xffull, wwdg);
		bus.ports[i++] = new PortMapping(stm32f1::IWDG, stm32f1::IWDG + 0xffull, iwdg);
		bus.ports[i++] = new PortMapping(stm32f1::TIM1, stm32f1::TIM1 + 0xffull, tim1);
		bus.ports[i++] = new PortMapping(stm32f1::TIM2, stm32f1::TIM2 + 0xffull, tim2);
		bus.ports[i++] = new PortMapping(stm32f1::TIM3, stm32f1::TIM3 + 0xffull, tim3);
		bus.ports[i++] = new PortMapping(stm32f1::TIM4, stm32f1::TIM4 + 0xffull, tim4);
		bus.ports[i++] = new PortMapping(stm32f1::TIM5, stm32f1::TIM5 + 0xffull, tim5);
		bus.ports[i++] = new PortMapping(opt.afio_start_addr, opt.afio_end_addr, afio);
		bus.ports[i++] = new PortMapping(opt.exti_start_addr, opt.exti_end_addr, exti);
		bus.ports[i++] = new PortMapping(opt.gpioa_start_addr, opt.gpioa_end_addr, gpioa);
		bus.ports[i++] = new PortMapping(opt.gpiob_start_addr, opt.gpiob_end_addr, gpiob);
		bus.ports[i++] = new PortMapping(opt.gpioc_start_addr, opt.gpioc_end_addr, gpioc);
		bus.ports[i++] = new PortMapping(opt.gpiod_start_addr, opt.gpiod_end_addr, gpiod);
		bus.ports[i++] = new PortMapping(opt.gpioe_start_addr, opt.gpioe_end_addr, gpioe);
		bus.ports[i++] = new PortMapping(opt.adc1_start_addr, opt.adc1_end_addr, adc1);
		bus.ports[i++] = new PortMapping(opt.adc2_start_addr, opt.adc2_end_addr, adc2);
		bus.ports[i++] = new PortMapping(opt.dac_start_addr, opt.dac_end_addr, dac);
		bus.ports[i++] = new PortMapping(opt.can1_start_addr, opt.can1_end_addr, can1);
		bus.ports[i++] = new PortMapping(opt.fsmc_start_addr, opt.fsmc_end_addr, fsmc);
		bus.ports[i++] = new PortMapping(0x60000000ull, 0x6fffffffull, fsmc_bank);
		bus.ports[i++] = new PortMapping(opt.usb_start_addr, opt.usb_end_addr, usb_dev);
		bus.ports[i++] = new PortMapping(opt.otgfs_start_addr, opt.otgfs_end_addr, otg_fs);
		bus.ports[i++] = new PortMapping(opt.eth_start_addr, opt.eth_end_addr, eth);
		bus.ports[i++] = new PortMapping(opt.sdio_start_addr, opt.sdio_end_addr, sdio);
		bus.ports[i++] = new PortMapping(opt.dma1_start_addr, opt.dma1_end_addr, dma1);
		bus.ports[i++] = new PortMapping(opt.crc_start_addr, opt.crc_end_addr, crc);
		bus.ports[i++] = new PortMapping(opt.spi1_start_addr, opt.spi1_end_addr, spi1);
		bus.ports[i++] = new PortMapping(opt.spi2_start_addr, opt.spi2_end_addr, spi2);
		bus.ports[i++] = new PortMapping(opt.usart1_start_addr, opt.usart1_end_addr, usart1);
		bus.ports[i++] = new PortMapping(opt.usart2_start_addr, opt.usart2_end_addr, usart2);
		bus.ports[i++] = new PortMapping(opt.console_start_addr, opt.console_end_addr, console);
		bus.ports[i++] = new PortMapping(opt.exiter_start_addr, opt.exiter_end_addr, exiter);
	}
	bus.mapping_complete();

	iss_mem_if.isock.bind(bus.tsocks[0]);
	dma1.bus_initiator_socket.bind(bus.tsocks[1]);
	eth.mem_socket.bind(bus.tsocks[2]);
	{
		unsigned i = 0;
		bus.isocks[i++].bind(mem.tsock);
		bus.isocks[i++].bind(clint.tsock);
		bus.isocks[i++].bind(plic.tsock);
		bus.isocks[i++].bind(i2c0.socket);
		bus.isocks[i++].bind(i2c1.socket);
		bus.isocks[i++].bind(rcc.socket);
		bus.isocks[i++].bind(flash.socket);
		bus.isocks[i++].bind(pwr.socket);
		bus.isocks[i++].bind(bkp.socket);
		bus.isocks[i++].bind(rtc.socket);
		bus.isocks[i++].bind(wwdg.socket);
		bus.isocks[i++].bind(iwdg.socket);
		bus.isocks[i++].bind(tim1.socket);
		bus.isocks[i++].bind(tim2.socket);
		bus.isocks[i++].bind(tim3.socket);
		bus.isocks[i++].bind(tim4.socket);
		bus.isocks[i++].bind(tim5.socket);
		bus.isocks[i++].bind(afio.socket);
		bus.isocks[i++].bind(exti.socket);
		bus.isocks[i++].bind(gpioa.socket);
		bus.isocks[i++].bind(gpiob.socket);
		bus.isocks[i++].bind(gpioc.socket);
		bus.isocks[i++].bind(gpiod.socket);
		bus.isocks[i++].bind(gpioe.socket);
		bus.isocks[i++].bind(adc1.socket);
		bus.isocks[i++].bind(adc2.socket);
		bus.isocks[i++].bind(dac.socket);
		bus.isocks[i++].bind(can1.socket);
		bus.isocks[i++].bind(fsmc.socket);
		bus.isocks[i++].bind(fsmc_bank.socket);
		bus.isocks[i++].bind(usb_dev.socket);
		bus.isocks[i++].bind(otg_fs.socket);
		bus.isocks[i++].bind(eth.socket);
		bus.isocks[i++].bind(sdio.socket);
		bus.isocks[i++].bind(dma1.socket);
		bus.isocks[i++].bind(crc.socket);
		bus.isocks[i++].bind(spi1.socket);
		bus.isocks[i++].bind(spi2.socket);
		bus.isocks[i++].bind(usart1.socket);
		bus.isocks[i++].bind(usart2.socket);
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
	spi1.plic = &plic;
	spi1.irq_id = 5u;
	spi2.plic = &plic;
	spi2.irq_id = 8u;
	usart1.plic = &plic;
	usart1.irq_id = 14u;
	usart2.plic = &plic;
	usart2.irq_id = 15u;

	/* Keep the slave addresses fixed for the verification suite. */
	i2c0.slave_addr = 0x50u;
	i2c1.slave_addr = 0x51u;

	new DirectCoreRunner(core);
	opt.handle_property_export_and_exit();
	sc_core::sc_start();
	return 0;
}
