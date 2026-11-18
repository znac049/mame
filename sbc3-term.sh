#!/bin/sh
# sbc3-term.sh - run the SBC3 emulation with no window, with its console
# on this terminal.
#
#   ./sbc3-term.sh [mame options...]
#   ./sbc3-term.sh -hard card.hd -quik netbsd
#   BAUD=9600 ./sbc3-term.sh -hard card.hd -quik netbsd -ramsize 4M
#
# BAUD must match the speed the SBC software uses on DUART channel A
# (for NetBSD, the stdout-path speed in sbc3.dts).  Default 230400, which
# is what the monitor ROM uses with jumper IP2 fitted.
#
# JUMPERS lists the DUART IP0-IP5 jumpers that are fitted.  Default
# "0 1 2", as on the board.  With IP3 fitted the monitor ROM auto-boots
# CP/M-68K, which needs the video board:
#           JUMPERS="0 1 2 3" ./sbc3-term.sh -hard card.hd
#
# EXPRAM=0 leaves RAM bank 1 ($10000000) empty (default 8: 8 MB fitted).
# CLOCK=25, 33, 40 or 50 sets the CPU clock in MHz (default 40).
#
# Ctrl-] quits.  Everything else (including Ctrl-C) goes to the SBC.
# Set MAME=/path/to/sbc3 if the emulator isn't in the current directory.

MAME=${MAME:-./sbc3}
PORT=${PORT:-4321}
LOG=${LOG:-sbc3-mame.log}
BAUD=${BAUD:-230400}
JUMPERS=${JUMPERS-0 1 2}
EXPRAM=${EXPRAM:-8}
CLOCK=${CLOCK:-40}
CFGDIR=${CFGDIR:-cfg-term}

command -v socat >/dev/null 2>&1 || { echo "needs socat (apt install socat / brew install socat)" >&2; exit 1; }

# MAME's serial "null modem" has its own speed setting, which has to
# match the SBC's.  It's an input-port setting, so set it via a
# private config directory.
case $BAUD in
	1200) v=4 ;; 2400) v=5 ;; 4800) v=6 ;; 9600) v=7 ;; 19200) v=9 ;;
	38400) v=11 ;; 57600) v=12 ;; 115200) v=13 ;; 230400) v=25 ;;
	*) echo "BAUD must be one of 1200 2400 4800 9600 19200 38400 57600 115200 230400" >&2; exit 1 ;;
esac
links=0
for j in $JUMPERS; do
	case $j in
		[0-5]) links=$((links | (1 << j))) ;;
		*) echo "JUMPERS: use numbers 0-5, e.g. JUMPERS=\"0 1 2\"" >&2; exit 1 ;;
	esac
done
# One entry per jumper (MAME stores each setting separately).  MAME only
# applies an entry whose defvalue matches the driver's built-in default,
# and builds of the driver differ in their defaults (IP3 has been both),
# so write each setting for both possible defaults; the one that doesn't
# match is ignored.
jumper_cfg=
for j in 0 1 2 3 4 5; do
	bit=$((1 << j))
	for def in 0 $bit; do
		jumper_cfg="$jumper_cfg            <port tag=\":LINKS\" type=\"CONFIG\" mask=\"$bit\" defvalue=\"$def\" value=\"$((links & bit))\" />
"
	done
done
case $EXPRAM in
	0) exp=0 ;; 8) exp=2 ;;
	*) echo "EXPRAM must be 0 or 8" >&2; exit 1 ;;
esac
case $CLOCK in
	40) clk=0 ;; 25) clk=16 ;; 33) clk=32 ;; 50) clk=48 ;;
	*) echo "CLOCK must be 25, 33, 40 or 50" >&2; exit 1 ;;
esac
mkdir -p "$CFGDIR"
cat >"$CFGDIR/sbc3.cfg" <<EOF
<?xml version="1.0"?>
<mameconfig version="10">
    <system name="sbc3">
        <input>
            <port tag=":rs232a:null_modem:RS232_TXBAUD" type="CONFIG" mask="255" defvalue="7" value="$v" />
            <port tag=":rs232a:null_modem:RS232_RXBAUD" type="CONFIG" mask="255" defvalue="7" value="$v" />
$jumper_cfg            <port tag=":CONFIG" type="CONFIG" mask="2" defvalue="2" value="$exp" />
            <port tag=":CONFIG" type="CONFIG" mask="2" defvalue="0" value="$exp" />
            <port tag=":CONFIG" type="CONFIG" mask="48" defvalue="0" value="$clk" />
        </input>
    </system>
</mameconfig>
EOF

# Start MAME headless, waiting to connect to our listener.  Its own
# messages go to $LOG so they don't mix with the SBC's console.
(
	sleep 1
	exec "$MAME" sbc3 -cfg_directory "$CFGDIR" -video none -sound none -skip_gameinfo \
		-rs232a null_modem -bitb socket.127.0.0.1:$PORT "$@"
) >"$LOG" 2>&1 &
pid=$!
trap 'kill $pid 2>/dev/null' EXIT INT TERM

echo "SBC3 console at $BAUD baud, jumpers fitted: ${JUMPERS:-none}, RAM bank 1: ${EXPRAM} MB, CPU ${CLOCK} MHz (Ctrl-] to quit; MAME messages in $LOG)"
# Raw terminal: keys go straight through; Ctrl-] is the escape.
socat -,raw,echo=0,escape=0x1d TCP-LISTEN:$PORT,bind=127.0.0.1,reuseaddr
stty sane 2>/dev/null
echo
