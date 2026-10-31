// license:BSD-3-Clause
// copyright-holders:SBC3 contributors
/***************************************************************************

    SBC3 -- homebrew 68030 single-board computer

    Hardware:
      MC68030 @ 40 MHz (no FPU)
      8 MB RAM at $000000 (the first 8 bytes read back the ROM's reset
          vectors)
      ROM (up to 4 MB) at $C00000
      XR68C681 DUART @ 7.3728 MHz at $AD0001 (odd bytes, every 2nd byte)
          IPL 2, vectored through its IVR
      CompactFlash, 16-bit True IDE, at $AE0000 (registers every 2nd
          byte; CF D0-D7 on CPU D31-D24)
          INTRQ: optionally on IPL 4, auto-vectored (configuration switch)
      MC68230 PI/T at $AF0001 (odd bytes), timer clocked at 10 MHz
          IPL 1, vectored through TIACK/TIVR

    Interrupt acknowledge, as decoded by the board:
      level 1  PI/T timer vector (TIVR)
      level 2  DUART vector (IVR)
      others   auto-vectored

    Running NetBSD:
      mame sbc3 -hard card.chd -quik netbsd
    loads the kernel ELF file straight into RAM a moment after reset
    (after the ROM, if any, has initialised the DUART) and jumps to its
    entry point.  A raw binary (netbsd.bin) is loaded at $2000 instead.

    The emulation is functional, not timing-accurate: no wait states,
    no bus errors for unmapped addresses, and the CF is always ready
    immediately.

***************************************************************************/

#include "emu.h"

#include "bus/ata/ataintf.h"
#include "bus/ata/hdd.h"
#include "bus/rs232/rs232.h"
#include "cpu/m68000/m68030.h"
#include "imagedev/snapquik.h"
#include "machine/68230pit.h"
#include "machine/mc68681.h"
#include "machine/ram.h"

#include "multibyte.h"


namespace {

class sbc3_state : public driver_device
{
public:
	sbc3_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_ram(*this, RAM_TAG)
		, m_rom(*this, "rom")
		, m_duart(*this, "duart")
		, m_pit(*this, "pit")
		, m_ata(*this, "ata")
		, m_config(*this, "CONFIG")
	{ }

	void sbc3(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;
	void cpu_space_map(address_map &map) ATTR_COLD;

	u32 cf_r(offs_t offset, u32 mem_mask);
	void cf_w(offs_t offset, u32 data, u32 mem_mask);
	void cf_irq_w(int state);

	DECLARE_QUICKLOAD_LOAD_MEMBER(quickload_cb);

	required_device<m68030_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_region_ptr<u32> m_rom;
	required_device<xr68c681_device> m_duart;
	required_device<pit68230_device> m_pit;
	required_device<ata_interface_device> m_ata;
	required_ioport m_config;

	int m_cf_irq = 0;
};


/***************************************************************************
    Memory map
***************************************************************************/

void sbc3_state::mem_map(address_map &map)
{
	// Only A0-A23 are decoded, so everything repeats every 16 MB.
	map.global_mask(0x00ffffff);
	map.unmap_value_high();

	// RAM is installed in machine_start() (its size is configurable),
	// followed by the ROM vector overlay at $0-$7.

	// DUART: registers on the odd byte lane, every 2nd byte.
	map(0xad0000, 0xad001f).rw(m_duart, FUNC(xr68c681_device::read), FUNC(xr68c681_device::write)).umask32(0x00ff00ff);

	// CompactFlash: 16-bit registers every 2nd byte.
	map(0xae0000, 0xae000f).rw(FUNC(sbc3_state::cf_r), FUNC(sbc3_state::cf_w));

	// PI/T: registers on the odd byte lane, every 2nd byte.
	map(0xaf0000, 0xaf003f).rw(m_pit, FUNC(pit68230_device::read), FUNC(pit68230_device::write)).umask32(0x00ff00ff);

	map(0xc00000, 0xffffff).rom().region("rom", 0);
}

/*
 * Interrupt acknowledge cycles: FC = 7, A19-A16 = 1111, A3-A1 = level,
 * so level n is acknowledged with a byte read of $FFFFFFF1 + 2n.
 */
void sbc3_state::cpu_space_map(address_map &map)
{
	map(0xfffffff3, 0xfffffff3).lr8(NAME([this] () -> u8 { return m_pit->irq_tiack(); }));
	map(0xfffffff5, 0xfffffff5).lr8(NAME([this] () -> u8 { return m_duart->get_irq_vector(); }));
	map(0xfffffff7, 0xfffffff7).lr8(NAME([] () -> u8 { return m68000_base_device::autovector(3); }));
	map(0xfffffff9, 0xfffffff9).lr8(NAME([] () -> u8 { return m68000_base_device::autovector(4); }));
	map(0xfffffffb, 0xfffffffb).lr8(NAME([] () -> u8 { return m68000_base_device::autovector(5); }));
	map(0xfffffffd, 0xfffffffd).lr8(NAME([] () -> u8 { return m68000_base_device::autovector(6); }));
	map(0xffffffff, 0xffffffff).lr8(NAME([] () -> u8 { return m68000_base_device::autovector(7); }));
}


/***************************************************************************
    CompactFlash

    The CF sits on D31-D16 as a 16-bit port with its D0-D7 on CPU
    D31-D24, so every 16-bit value is byte-swapped relative to the ATA
    bus: task-file registers appear in the high byte, and data words
    land in memory in disk byte order.  Register n is at $AE0000 + 2n,
    i.e. the upper or lower half of a 32-bit word.
***************************************************************************/

u32 sbc3_state::cf_r(offs_t offset, u32 mem_mask)
{
	u32 data = 0;

	if (ACCESSING_BITS_16_31)
		data |= u32(swapendian_int16(m_ata->cs0_r(offset * 2))) << 16;
	if (ACCESSING_BITS_0_15)
		data |= swapendian_int16(m_ata->cs0_r(offset * 2 + 1));
	return data;
}

void sbc3_state::cf_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (ACCESSING_BITS_16_31)
		m_ata->cs0_w(offset * 2, swapendian_int16(u16(data >> 16)));
	if (ACCESSING_BITS_0_15)
		m_ata->cs0_w(offset * 2 + 1, swapendian_int16(u16(data)));
}

void sbc3_state::cf_irq_w(int state)
{
	m_cf_irq = state;
	const bool wired = BIT(m_config->read(), 0);
	m_maincpu->set_input_line(M68K_IRQ_4, (wired && state) ? ASSERT_LINE : CLEAR_LINE);
}


/***************************************************************************
    Quickload: a NetBSD kernel ELF file (or a raw binary for $2000)
***************************************************************************/

QUICKLOAD_LOAD_MEMBER(sbc3_state::quickload_cb)
{
	const u32 ramsize = m_ram->size();
	std::vector<u8> file(image.length());
	u32 entry;

	if (file.size() < 4 || image.fread(&file[0], file.size()) != file.size())
		return std::make_pair(image_error::UNSPECIFIED, "Cannot read the file");

	auto be16 = [&file] (size_t o) -> u32 { return (o + 2 <= file.size()) ? get_u16be(&file[o]) : 0; };
	auto be32 = [&file] (size_t o) -> u32 { return (o + 4 <= file.size()) ? get_u32be(&file[o]) : 0; };
	address_space &space = m_maincpu->space(AS_PROGRAM);

	if (file[0] == 0x7f && file[1] == 'E' && file[2] == 'L' && file[3] == 'F')
	{
		if (file.size() < 52 || file[4] != 1 || file[5] != 2 || be16(16) != 2 || be16(18) != 4)
			return std::make_pair(image_error::INVALIDIMAGE, "Not a 32-bit big-endian 68k ELF executable");

		const u32 phoff = be32(28), phentsize = be16(42), phnum = be16(44);
		if (phentsize < 32 || u64(phoff) + u64(phnum) * phentsize > file.size())
			return std::make_pair(image_error::INVALIDIMAGE, "Bad ELF program headers");

		for (int pass = 0; pass < 2; pass++)
		{
			for (u32 i = 0; i < phnum; i++)
			{
				const size_t ph = phoff + i * phentsize;
				const u32 offset = be32(ph + 4), pa = be32(ph + 12);
				const u32 filesz = be32(ph + 16), memsz = be32(ph + 20);

				if (be32(ph) != 1 || memsz == 0)        // PT_LOAD only
					continue;
				if (pass == 0)
				{
					if (filesz > memsz || u64(offset) + filesz > file.size() || u64(pa) + memsz > ramsize)
						return std::make_pair(image_error::INVALIDIMAGE, "ELF segment doesn't fit in RAM");
					continue;
				}
				for (u32 n = 0; n < filesz; n++)
					space.write_byte(pa + n, file[offset + n]);
				for (u32 n = filesz; n < memsz; n++)
					space.write_byte(pa + n, 0);
				logerror("quickload: segment %08x-%08x (%u bytes from file)\n", pa, pa + memsz, filesz);
			}
		}
		entry = be32(24);
	}
	else
	{
		constexpr u32 LOAD_ADDR = 0x2000;
		if (LOAD_ADDR + file.size() > ramsize)
			return std::make_pair(image_error::INVALIDLENGTH, "File too large for RAM");
		for (size_t n = 0; n < file.size(); n++)
			space.write_byte(LOAD_ADDR + n, file[n]);
		entry = LOAD_ADDR;
	}

	// Enter it the way a ROM monitor would: supervisor, interrupts off.
	m_maincpu->set_state_int(M68K_SR, 0x2700);
	m_maincpu->set_pc(entry);
	image.message(" Quickload: %u bytes, entry %08x", u32(file.size()), entry);

	return std::make_pair(std::error_condition(), std::string());
}


/***************************************************************************
    Machine
***************************************************************************/

void sbc3_state::machine_start()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);

	// No ROM image: give the CPU a stub that waits in a loop, so that
	// -quik can start a program without a monitor ROM being present.
	if (m_rom[0] == 0xffffffff && m_rom[1] == 0xffffffff)
	{
		m_rom[0] = 0x00002000;      // reset SSP
		m_rom[1] = 0x00c00008;      // reset PC
		m_rom[2] = 0x60fe4e71;      // bra.s * ; nop
	}

	// RAM from $0, at most 8 MB.
	const u32 ramsize = std::min<u32>(m_ram->size(), 0x800000);
	space.install_ram(0x000000, ramsize - 1, m_ram->pointer());

	// The ROM's first 8 bytes (reset SSP and PC) overlay RAM at $0-$7.
	space.install_rom(0x000000, 0x000007, &m_rom[0]);

	save_item(NAME(m_cf_irq));
}

static INPUT_PORTS_START( sbc3 )
	PORT_START("CONFIG")
	PORT_CONFNAME(0x01, 0x00, "CF INTRQ wired to IPL 4")
	PORT_CONFSETTING(   0x00, DEF_STR( No ))
	PORT_CONFSETTING(   0x01, DEF_STR( Yes ))
INPUT_PORTS_END

// Default terminal settings: 9600 8N1, as the SBC3 kernel console uses.
static DEVICE_INPUT_DEFAULTS_START( terminal )
	DEVICE_INPUT_DEFAULTS( "RS232_RXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_TXBAUD", 0xff, RS232_BAUD_9600 )
	DEVICE_INPUT_DEFAULTS( "RS232_DATABITS", 0xff, RS232_DATABITS_8 )
	DEVICE_INPUT_DEFAULTS( "RS232_PARITY", 0xff, RS232_PARITY_NONE )
	DEVICE_INPUT_DEFAULTS( "RS232_STOPBITS", 0xff, RS232_STOPBITS_1 )
DEVICE_INPUT_DEFAULTS_END

void sbc3_state::sbc3(machine_config &config)
{
	M68030(config, m_maincpu, 40_MHz_XTAL);
	m_maincpu->set_addrmap(AS_PROGRAM, &sbc3_state::mem_map);
	m_maincpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &sbc3_state::cpu_space_map);
	m_maincpu->set_fpu_enable(false);       // no 68881/68882 fitted

	RAM(config, m_ram).set_default_size("8M").set_extra_options("4M");

	XR68C681(config, m_duart, 7.3728_MHz_XTAL);
	m_duart->irq_cb().set_inputline(m_maincpu, M68K_IRQ_2);
	m_duart->a_tx_cb().set("rs232a", FUNC(rs232_port_device::write_txd));
	m_duart->b_tx_cb().set("rs232b", FUNC(rs232_port_device::write_txd));

	rs232_port_device &rs232a(RS232_PORT(config, "rs232a", default_rs232_devices, "terminal"));
	rs232a.rxd_handler().set(m_duart, FUNC(xr68c681_device::rx_a_w));
	rs232a.set_option_device_input_defaults("terminal", DEVICE_INPUT_DEFAULTS_NAME(terminal));
	rs232a.set_option_device_input_defaults("null_modem", DEVICE_INPUT_DEFAULTS_NAME(terminal));

	rs232_port_device &rs232b(RS232_PORT(config, "rs232b", default_rs232_devices, nullptr));
	rs232b.rxd_handler().set(m_duart, FUNC(xr68c681_device::rx_b_w));

	PIT68230(config, m_pit, 10_MHz_XTAL);
	m_pit->timer_irq_callback().set_inputline(m_maincpu, M68K_IRQ_1);

	ATA_INTERFACE(config, m_ata).options(ata_devices, "cf", nullptr, true);
	m_ata->irq_handler().set(FUNC(sbc3_state::cf_irq_w));

	QUICKLOAD(config, "quickload", "elf,bin,netbsd", attotime::from_msec(500)).set_load_callback(FUNC(sbc3_state::quickload_cb));
}

ROM_START( sbc3 )
	ROM_REGION32_BE(0x400000, "rom", ROMREGION_ERASEFF)
	// Your monitor ROM image; it may be smaller than 4 MB.  Without
	// one, use -quik to start a program directly.
	ROM_LOAD( "sbc3.bin", 0x000000, 0x400000, NO_DUMP )
ROM_END

} // anonymous namespace


//    YEAR  NAME  PARENT  COMPAT  MACHINE  INPUT  CLASS       INIT        COMPANY      FULLNAME                       FLAGS
COMP( 2026, sbc3, 0,      0,      sbc3,    sbc3,  sbc3_state, empty_init, "homebrew",  "SBC3 (68030 single-board computer)", MACHINE_NO_SOUND_HW )
