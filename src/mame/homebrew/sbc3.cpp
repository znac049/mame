// license:BSD-3-Clause
// copyright-holders:SBC3 contributors
/***************************************************************************

    SBC3 -- homebrew 68030 single-board computer

    Hardware:
      MC68030 @ 40 MHz (no FPU)
      8 MB RAM at $000000 (after reset, the first 8 bytes read the ROM's
          reset vectors until the first access to the ROM)
      Optional 32 MB expansion RAM at $02000000 (configuration switch;
          A24-A31 decoded for it, otherwise the board repeats every 16 MB)
      ROM (up to 4 MB) at $C00000
      XR68C681 DUART @ 7.3728 MHz at $AD0001 (odd bytes, every 2nd byte)
          IPL 2, vectored through its IVR
      CompactFlash, 16-bit True IDE, at $AE0000 (registers every 2nd
          byte; CF D0-D7 on CPU D31-D24)
          INTRQ: optionally on IPL 4, auto-vectored (configuration switch)
      MC68230 PI/T at $AF0001 (odd bytes), timer clocked at 10 MHz
          IPL 1, vectored through TIACK/TIVR
      DS1307 RTC on I2C, bit-banged on PI/T port C: PC0 = SDA, PC1 = SCL,
          open-drain (PCDR held at 0; a line is pulled low by setting its
          PCDDR bit)
      Optional Mega-Micros Hex Display board at $AB0000 (configuration
          switch): 8 hex digits and 32 binary LEDs.  Byte n ($AB0000+n)
          drives digits 2n+1/2n+2 and binary row n+1; word and long
          writes set 4 or 8 digits.  Only A0-A1 are decoded within the
          64K page.  The board's documentation doesn't describe reads;
          here they return $FF and don't bus-error.

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
    and the CF is always ready immediately.  Unused addresses between
    RAM and ROM ($800000-$BFFFFF) give a bus error, as on the board.

***************************************************************************/

#include "emu.h"

#include "bus/ata/ataintf.h"
#include "bus/ata/hdd.h"
#include "bus/rs232/rs232.h"
#include "cpu/m68000/m68030.h"
#include "imagedev/snapquik.h"
#include "machine/68230pit.h"
#include "machine/ds1307.h"
#include "machine/mc68681.h"
#include "machine/ram.h"

#include "multibyte.h"

#include "sbc3.lh"


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
		, m_rtc(*this, "rtc")
		, m_config(*this, "CONFIG")
		, m_links(*this, "LINKS")
		, m_hex_digits(*this, "digit%u", 0U)
		, m_hex_bits(*this, "bit%u", 0U)
	{ }

	void sbc3(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	void mem_map(address_map &map) ATTR_COLD;
	void cpu_space_map(address_map &map) ATTR_COLD;

	u32 cf_r(offs_t offset, u32 mem_mask);
	void cf_w(offs_t offset, u32 data, u32 mem_mask);
	void cf_irq_w(int state);
	void i2c_update();
	u8 pit_pc_r();
	u32 berr_r(offs_t offset, u32 mem_mask);
	void berr_w(offs_t offset, u32 data, u32 mem_mask);
	u32 hexdisp_r(offs_t offset, u32 mem_mask);
	void hexdisp_w(offs_t offset, u32 data, u32 mem_mask);
	void hexdisp_update();

	DECLARE_QUICKLOAD_LOAD_MEMBER(quickload_cb);

	required_device<m68030_device> m_maincpu;
	required_device<ram_device> m_ram;
	required_region_ptr<u32> m_rom;
	required_device<xr68c681_device> m_duart;
	required_device<pit68230_device> m_pit;
	required_device<ata_interface_device> m_ata;
	required_device<i2c_ds1307_device> m_rtc;
	required_ioport m_config;
	required_ioport m_links;
	output_finder<8> m_hex_digits;
	output_finder<32> m_hex_bits;

	memory_passthrough_handler m_overlay_tap;
	memory_passthrough_handler m_pcddr_tap;
	static constexpr u32 EXP_RAM_BASE = 0x02000000;
	static constexpr u32 EXP_RAM_SIZE = 0x02000000;     // 32 MB
	std::unique_ptr<u8[]> m_exp_ram;
	bool m_exp_mapped = false;
	static constexpr u32 HEXDISP_BASE = 0x00ab0000;
	bool m_hexdisp_mapped = false;
	u32 m_hexdisp = 0;          // latched value, digit 1 in bits 31-28
	u8 m_hexdisp_valid = 0;     // which bytes have been written since power-on
	u8 m_pcddr = 0;
	int m_rtc_sda = 1;
	int m_cf_irq = 0;
};


/***************************************************************************
    Memory map
***************************************************************************/

void sbc3_state::mem_map(address_map &map)
{
	// The on-board decode uses only A0-A23, so the board repeats every
	// 16 MB (the mirrors below).  The optional expansion RAM at
	// $02000000-$03FFFFFF decodes the upper address bits and replaces
	// those copies; it's installed in machine_reset().
	map.unmap_value_high();

	// RAM is installed in machine_start() (its size is configurable),
	// followed by the ROM vector overlay at $0-$7.

	// Nothing else is fitted between RAM and ROM: an access there gets
	// no DSACK and times out with a bus error, which is how the monitor
	// detects missing boards.  The devices below override this.
	map(0x800000, 0xbfffff).mirror(0xff000000).rw(FUNC(sbc3_state::berr_r), FUNC(sbc3_state::berr_w));

	// DUART: registers on the odd byte lane, every 2nd byte.
	map(0xad0000, 0xad001f).mirror(0xff000000).rw(m_duart, FUNC(xr68c681_device::read), FUNC(xr68c681_device::write)).umask32(0x00ff00ff);

	// CompactFlash: 16-bit registers every 2nd byte.
	map(0xae0000, 0xae000f).mirror(0xff000000).rw(FUNC(sbc3_state::cf_r), FUNC(sbc3_state::cf_w));

	// PI/T: registers on the odd byte lane, every 2nd byte.
	map(0xaf0000, 0xaf003f).mirror(0xff000000).rw(m_pit, FUNC(pit68230_device::read), FUNC(pit68230_device::write)).umask32(0x00ff00ff);

	map(0xc00000, 0xffffff).mirror(0xff000000).rom().region("rom", 0);
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


u32 sbc3_state::berr_r(offs_t offset, u32 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		m_maincpu->set_buserror_details(0x800000 + offset * 4, 1, m_maincpu->get_fc());
		m_maincpu->pulse_input_line(M68K_LINE_BUSERROR, attotime::zero);
	}
	return 0xffffffff;
}

void sbc3_state::berr_w(offs_t offset, u32 data, u32 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		m_maincpu->set_buserror_details(0x800000 + offset * 4, 0, m_maincpu->get_fc());
		m_maincpu->pulse_input_line(M68K_LINE_BUSERROR, attotime::zero);
	}
}


/***************************************************************************
    Mega-Micros Hex Display board

    Four 8-bit latches, one per byte lane, each driving two hex digits and
    eight binary LEDs.  The latches power up holding garbage; here a digit
    pair stays blank until it is first written.
***************************************************************************/

u32 sbc3_state::hexdisp_r(offs_t offset, u32 mem_mask)
{
	return 0xffffffff;
}

void sbc3_state::hexdisp_w(offs_t offset, u32 data, u32 mem_mask)
{
	COMBINE_DATA(&m_hexdisp);
	for (int n = 0; n < 4; n++)
		if (BIT(mem_mask, 31 - 8 * n))
			m_hexdisp_valid |= 1 << n;
	hexdisp_update();
}

void sbc3_state::hexdisp_update()
{
	// 7-segment patterns for 0-F (TIL311-style: A b C d E F)
	static constexpr u8 SEGS[16] = {
		0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
		0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };

	for (int d = 0; d < 8; d++)
	{
		const bool valid = BIT(m_hexdisp_valid, d / 2);
		m_hex_digits[d] = valid ? SEGS[(m_hexdisp >> (28 - 4 * d)) & 0xf] : 0;
	}
	for (int b = 0; b < 32; b++)
		m_hex_bits[b] = BIT(m_hexdisp_valid, 3 - b / 8) ? BIT(m_hexdisp, b) : 0;
}


/***************************************************************************
    I2C RTC on PI/T port C

    The lines are open-drain with pull-ups: software keeps PCDR at 0 and
    pulls a line low by making it an output in PCDDR.  The PI/T model
    doesn't report direction changes, so the PCDDR writes are watched
    with a tap.
***************************************************************************/

void sbc3_state::i2c_update()
{
	m_rtc->scl_write(BIT(m_pcddr, 1) ? 0 : 1);
	m_rtc->sda_write(BIT(m_pcddr, 0) ? 0 : 1);
}

u8 sbc3_state::pit_pc_r()
{
	const int sda = !BIT(m_pcddr, 0) && m_rtc_sda;
	const int scl = !BIT(m_pcddr, 1);
	return 0xfc | (scl << 1) | sda;
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
					const bool in_main = u64(pa) + memsz <= ramsize;
					const bool in_exp = m_exp_mapped && pa >= EXP_RAM_BASE && u64(pa) + memsz <= u64(EXP_RAM_BASE) + EXP_RAM_SIZE;
					if (filesz > memsz || u64(offset) + filesz > file.size() || !(in_main || in_exp))
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
	space.install_ram(0x000000, ramsize - 1, 0xff000000, m_ram->pointer());

	// Expansion RAM, mapped at the first reset if enabled.
	m_exp_ram = std::make_unique<u8[]>(EXP_RAM_SIZE);
	std::fill_n(m_exp_ram.get(), EXP_RAM_SIZE, 0);
	save_pointer(NAME(m_exp_ram), EXP_RAM_SIZE);

	// PCDDR is register 4: byte $AF0009, the second byte of a 32-bit word.
	m_pcddr_tap = space.install_write_tap(
			0xaf0008, 0xaf000b,
			"pcddr_w",
			[this] (offs_t offset, u32 &data, u32 mem_mask)
			{
				if (ACCESSING_BITS_16_23)
				{
					m_pcddr = u8(data >> 16);
					i2c_update();
				}
			},
			&m_pcddr_tap);

	save_item(NAME(m_cf_irq));
	save_item(NAME(m_hexdisp));
	save_item(NAME(m_hexdisp_valid));
	save_item(NAME(m_pcddr));
	save_item(NAME(m_rtc_sda));
}

/*
 * After reset the ROM's first 8 bytes (reset SSP and PC) overlay RAM at
 * $0-$7, so the CPU can fetch its reset vectors.  The overlay switches
 * off at the first access to the ROM itself (normally the first
 * instruction fetch), after which $0-$7 is ordinary RAM again.
 */
void sbc3_state::machine_reset()
{
	address_space &space = m_maincpu->space(AS_PROGRAM);

	space.install_rom(0x000000, 0x000007, &m_rom[0]);

	// Expansion RAM (configuration switch; turning it off again needs a
	// restart of MAME).
	if (BIT(m_config->read(), 1) && !m_exp_mapped)
	{
		space.install_ram(EXP_RAM_BASE, EXP_RAM_BASE + EXP_RAM_SIZE - 1, m_exp_ram.get());
		m_exp_mapped = true;
	}

	// Hex Display board: decodes A16-A23 and A0-A1 only, so its four
	// bytes repeat through the 64K page (and every 16 MB, like the rest
	// of the board).  Not fitted: the page keeps its bus error.
	if (BIT(m_config->read(), 2) && !m_hexdisp_mapped)
	{
		space.install_readwrite_handler(HEXDISP_BASE, HEXDISP_BASE + 3, 0, 0xff00fffc, 0,
				read32s_delegate(*this, FUNC(sbc3_state::hexdisp_r)),
				write32s_delegate(*this, FUNC(sbc3_state::hexdisp_w)));
		m_hexdisp_mapped = true;
	}
	hexdisp_update();

	// DUART inputs IP0-IP5 are jumpers: pulled up, and grounded when the
	// jumper is fitted.  The monitor reads them at reset (IP2 fitted
	// selects 230400 baud, IP4 open skips the ROM checksum).  IP6 is
	// unused and reads high.
	const u8 links = m_links->read();
	m_duart->ip0_w(BIT(links, 0) ? 0 : 1);
	m_duart->ip1_w(BIT(links, 1) ? 0 : 1);
	m_duart->ip2_w(BIT(links, 2) ? 0 : 1);
	m_duart->ip3_w(BIT(links, 3) ? 0 : 1);
	m_duart->ip4_w(BIT(links, 4) ? 0 : 1);
	m_duart->ip5_w(BIT(links, 5) ? 0 : 1);
	m_duart->ip6_w(1);

	// The PI/T resets port C to inputs: both I2C lines float high.
	m_pcddr = 0;
	i2c_update();

	m_overlay_tap.remove();
	m_overlay_tap = space.install_read_tap(
			0xc00000, 0xffffff,
			"rom_overlay_off",
			[this] (offs_t offset, u32 &data, u32 mem_mask)
			{
				if (!machine().side_effects_disabled())
				{
					m_overlay_tap.remove();
					m_maincpu->space(AS_PROGRAM).install_ram(0x000000, 0x000007, m_ram->pointer());
				}
			},
			&m_overlay_tap);
}

static INPUT_PORTS_START( sbc3 )
	PORT_START("CONFIG")
	PORT_CONFNAME(0x01, 0x00, "CF INTRQ wired to IPL 4")
	PORT_CONFSETTING(   0x00, DEF_STR( No ))
	PORT_CONFSETTING(   0x01, DEF_STR( Yes ))
	PORT_CONFNAME(0x02, 0x02, "Expansion RAM at $02000000")
	PORT_CONFSETTING(   0x00, DEF_STR( None ))
	PORT_CONFSETTING(   0x02, "32 MB")
	PORT_CONFNAME(0x04, 0x04, "Hex Display board at $AB0000")
	PORT_CONFSETTING(   0x00, "Not fitted")
	PORT_CONFSETTING(   0x04, "Fitted")

	// Jumpers on DUART IP0-IP5: fitted = input grounded.  Changes take
	// effect at the next reset.
	PORT_START("LINKS")
	PORT_CONFNAME(0x01, 0x01, "Jumper IP0")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x01, "Fitted")
	PORT_CONFNAME(0x02, 0x02, "Jumper IP1")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x02, "Fitted")
	PORT_CONFNAME(0x04, 0x04, "Jumper IP2 (fitted: 230400 baud)")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x04, "Fitted")
        PORT_CONFNAME(0x08, 0x00, "Jumper IP3 (fitted: auto-boot CP/M on video)")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x08, "Fitted")
	PORT_CONFNAME(0x10, 0x00, "Jumper IP4 (fitted: ROM checksum at reset)")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x10, "Fitted")
	PORT_CONFNAME(0x20, 0x00, "Jumper IP5")
	PORT_CONFSETTING(   0x00, "Open")
	PORT_CONFSETTING(   0x20, "Fitted")
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
	m_pit->pc_in_callback().set(FUNC(sbc3_state::pit_pc_r));

	I2C_DS1307(config, m_rtc);
	m_rtc->sda_callback().set([this] (int state) { m_rtc_sda = state; });

	ATA_INTERFACE(config, m_ata).options(ata_devices, "cf", nullptr, true);
	m_ata->irq_handler().set(FUNC(sbc3_state::cf_irq_w));

	config.set_default_layout(layout_sbc3);

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
