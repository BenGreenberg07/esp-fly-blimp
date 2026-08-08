#!/usr/bin/env python3
"""
check_link.py — terminal-only drone connectivity + motor test (no panel/HTTP UI).

Talks to the drone exactly the way the panel does:
  this PC --USB serial--> XIAO C6 bridge --ESP-NOW--> blimp (S3)

It sends the SAME 0xA5 manual-control frames the panel's manual mode uses, and
reads back the drone's 0xB7 motor telemetry -- so a PASS here means the WHOLE
chain (serial port, bridge, radio link, drone armed + listening) actually
works, not just that a COM port opened.

SAFETY: --spin drives real motors at low power. Remove props, or hold/clamp
the frame down, before running it.

Examples:
  python check_link.py                      # just test the link (no spin)
  python check_link.py --spin               # + spin each channel briefly, low power
  python check_link.py --bridge-port COM5 --spin
"""
import argparse
import math
import struct
import sys
import time

FULL = 65535          # forward motor full duty (matches panel_server.py)
PITCH_MAX = 32767      # int16 cap on the vertical/turn setpoint channels
M_VERT_SIGN = -1.0     # up/down hardware orientation
BAUD = 115200


def find_bridge_port():
    """Auto-detect the XIAO C6 bridge serial port on macOS / Windows / Linux."""
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            blob = ((p.description or "") + (p.manufacturer or "") + (p.device or "")).lower()
            if any(s in blob for s in ("usbmodem", "wchusbserial", "cp210",
                                        "ch340", "esp", "uart", "silicon", "wch")):
                return p.device
        if ports:
            return ports[0].device
    except Exception:
        pass
    return None


def open_bridge(port_arg):
    print("=== USB bridge (XIAO C6) ===")
    try:
        import serial
        from serial.tools import list_ports
    except Exception:
        print("  FAIL: pyserial not installed. Run: pip install pyserial")
        return None

    ports = list(list_ports.comports())
    if ports:
        print("  Available serial ports:")
        for p in ports:
            print(f"    {p.device}  ({p.description})")
    else:
        print("  No serial ports found at all -- is the C6 plugged in?")

    port = port_arg or find_bridge_port()
    if not port:
        print("  FAIL: could not auto-detect a bridge port. Pass --bridge-port COM3 (Windows)")
        print("        or /dev/cu.usbmodemXXXX (macOS) explicitly.")
        return None

    print(f"  Opening {port} @ {BAUD}...")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1)
        time.sleep(0.3)  # let the bridge's USB-serial settle
        print(f"  PASS: opened {port}.")
        return ser
    except Exception as e:
        print(f"  FAIL: could not open {port} ({e})")
        print("        On Windows this is usually another program (Arduino Serial Monitor,")
        print("        the panel itself, a previous run of this script) holding the port")
        print("        open, or a missing USB-serial driver (CP210x/CH340) for the C6.")
        return None


_telem_buf = bytearray()


def read_telemetry(ser):
    """Non-blocking drain of the bridge's USB serial for 0xB7 motor-telemetry
    frames the drone sends back (4 LE float32: fwdL, fwdR, up, down). Returns
    the most recent frame seen since the last call, or None."""
    global _telem_buf
    try:
        n = ser.in_waiting
        if n:
            _telem_buf += ser.read(n)
    except Exception:
        return None
    last = None
    while True:
        i = _telem_buf.find(b"\xB7")
        if i < 0:
            if len(_telem_buf) > 256:
                del _telem_buf[:-1]
            return last
        if len(_telem_buf) - i < 17:
            if i > 0:
                del _telem_buf[:i]
            return last
        frame = bytes(_telem_buf[i + 1:i + 17])
        del _telem_buf[:i + 17]
        try:
            m = struct.unpack("<4f", frame)
            if all(math.isfinite(v) for v in m):
                last = m
        except Exception:
            pass


def send_manual(ser, pitch=0.0, turn=0.0, forward=0.0):
    ser.write(b"\xA5" + struct.pack("<ffff", 0.0, pitch, turn, forward))


def zero(ser, n=3):
    for _ in range(n):
        send_manual(ser)
        time.sleep(0.02)


def test_link(ser, listen_s):
    """Send idle frames and watch for ANY telemetry reply -- confirms the bridge
    is talking to the drone over ESP-NOW and the drone is alive and armed, not
    just that the serial port opened."""
    print("\n=== Drone link (bridge <-ESP-NOW-> blimp) ===")
    print(f"  Sending idle frames, listening for motor telemetry for {listen_s:.0f}s...")
    zero(ser)
    seen = None
    end = time.time() + listen_s
    while time.time() < end:
        m = read_telemetry(ser)
        if m is not None:
            seen = m
        time.sleep(0.05)
    if seen is None:
        print("  FAIL: no telemetry (0xB7) came back from the drone.")
        print("        Bridge USB is fine, but check: drone is powered ON (battery, not")
        print("        just USB), drone + bridge are on the SAME ESP-NOW channel, and the")
        print("        drone's firmware has ESPNOW_CONTROL_ENABLED=1.")
        return False
    print(f"  PASS: drone responded. last telemetry fwdL={seen[0]:.3f} fwdR={seen[1]:.3f} "
          f"up={seen[2]:.3f} down={seen[3]:.3f}")
    return True


def spin_channel(ser, name, pitch, turn, forward, duration):
    print(f"\n  -> {name}: sending for {duration:.1f}s ... watch/feel which motor(s) move")
    end = time.time() + duration
    peak = [0.0, 0.0, 0.0, 0.0]
    while time.time() < end:
        send_manual(ser, pitch, turn, forward)
        m = read_telemetry(ser)
        if m is not None:
            peak = [max(a, abs(b)) for a, b in zip(peak, m)]
        time.sleep(0.02)
    zero(ser)
    print(f"     telemetry peak (fwdL, fwdR, up, down): "
          f"{peak[0]:.3f}, {peak[1]:.3f}, {peak[2]:.3f}, {peak[3]:.3f}")
    return peak


def test_motors(ser, power, duration):
    print("\n=== Motor spin test ===")
    print("SAFETY: remove props, or hold/clamp the frame down, before this runs.")
    input("Press Enter when ready (Ctrl+C to abort)... ")
    results = {
        "FORWARD":    spin_channel(ser, "FORWARD", 0.0, 0.0, power * FULL, duration),
        "TURN RIGHT": spin_channel(ser, "TURN RIGHT", 0.0, power * PITCH_MAX, 0.0, duration),
        "TURN LEFT":  spin_channel(ser, "TURN LEFT", 0.0, -power * PITCH_MAX, 0.0, duration),
        "UP":         spin_channel(ser, "UP", M_VERT_SIGN * power * PITCH_MAX, 0.0, 0.0, duration),
        "DOWN":       spin_channel(ser, "DOWN", -M_VERT_SIGN * power * PITCH_MAX, 0.0, 0.0, duration),
    }
    zero(ser)

    print("\n  Summary -- did the drone report motor activity for each command?")
    for label, peak in results.items():
        moved = any(v > 0.01 for v in peak)
        tag = "responded" if moved else "NO telemetry response"
        print(f"    [{label:11s}] {tag} -- fwdL={peak[0]:.3f} fwdR={peak[1]:.3f} "
              f"up={peak[2]:.3f} down={peak[3]:.3f}")
    print("\n  If telemetry moved but a physical motor didn't spin, that motor/ESC/wire")
    print("  is the problem, not the connection. If telemetry stayed at 0 the whole")
    print("  time, the drone isn't receiving/decoding these frames (check firmware mode).")


def main():
    ap = argparse.ArgumentParser(
        description="Terminal-only drone connectivity + motor test (bridge <-> blimp).")
    ap.add_argument("--bridge-port", default=None, help="C6 serial port (auto-detected if omitted)")
    ap.add_argument("--spin", action="store_true", help="also spin each motor channel briefly")
    ap.add_argument("--power", type=float, default=0.25,
                     help="spin test power fraction 0..1 (default 0.25)")
    ap.add_argument("--duration", type=float, default=1.5, help="seconds per channel during --spin")
    ap.add_argument("--link-timeout", type=float, default=2.0, help="seconds to wait for telemetry")
    args = ap.parse_args()

    ser = open_bridge(args.bridge_port)
    if ser is None:
        sys.exit(1)

    link_ok = False
    try:
        link_ok = test_link(ser, args.link_timeout)
        if args.spin:
            if not link_ok:
                print("\nSkipping motor spin test -- no drone telemetry seen yet.")
                print("Fix the link first (see above), or re-run with the drone powered on.")
            else:
                test_motors(ser, args.power, args.duration)
    except KeyboardInterrupt:
        print("\nAborted -- sending stop frames.")
    finally:
        zero(ser, 5)
        ser.close()

    print(f"\n[{'PASS' if link_ok else 'FAIL'}] drone link")
    sys.exit(0 if link_ok else 1)


if __name__ == "__main__":
    main()
