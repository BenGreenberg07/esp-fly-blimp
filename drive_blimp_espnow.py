#!/usr/bin/env python3
"""
drive_blimp_espnow.py — fly the ESP-FLY BLIMP over ESP-NOW (no Wi-Fi join).

The Mac sends setpoints over USB serial to the XIAO ESP32-C6 bridge
(espnow_bridge.ino), which broadcasts them via ESP-NOW to the blimp's ESP-NOW
receiver. Your Mac's Wi-Fi stays completely free for the mocap network.

CONTROLS (same feel as drive_blimp.py)
    W  forward (hold to ramp)   S  slow      A/D  turn      Q  up   E  down
    Space  KILL                 Esc  quit

Run:
    ./.venv/bin/python drive_blimp_espnow.py            # auto-detect bridge port
    ./.venv/bin/python drive_blimp_espnow.py --port /dev/cu.usbmodemXXXX

Sends 17-byte frames: 0xA5 + 4 little-endian float32 (roll, pitch, yaw, thrust).
Values are in the firmware "passthrough" motor-duty domain (the drone ESP-NOW
receiver sets blimp.fwdScale/vertGain/turnGain=1.0 and vertScale=2.0), matching
the web panel. The DRONE zeroes the motors if frames stop (failsafe).
"""

import argparse
import glob
import struct
import time

import serial
from pynput import keyboard

FULL = 65535
PITCH_MAX = 32767          # int16 cap on the vertical/turn channels (vertScale doubles vert)
RATE_HZ = 30
THR_STEP = 0.02            # forward ramp per tick (fraction)

# Power levels (fraction of full motor duty) — no live param link over ESP-NOW,
# so these are fixed here. Match the web panel defaults.
UP_POWER = 0.80
DOWN_POWER = 0.80
FWD_POWER = 0.30
TURN_POWER = 0.25
VERT_SIGN = -1.0           # up/down inversion fix (same as drive_blimp.py)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def find_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*"))
    return ports[0] if ports else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="bridge serial port (default: auto)")
    args = ap.parse_args()
    port = args.port or find_port()
    if not port:
        print("No serial port found. Plug in the C6 bridge."); return
    # Open WITHOUT pulsing DTR/RTS — otherwise the C6 resets/halts on connect
    # (its light goes out and the bridge stops broadcasting).
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(2.0)                       # let the bridge boot + run setup()
    try: ser.reset_input_buffer()
    except Exception: pass
    print(f"Bridge on {port}. W=fwd S=slow A/D=turn Q=up E=down Space=KILL Esc=quit")

    st = {"lvl": 0.0, "W": False, "S": False, "A": False, "D": False,
          "Q": False, "E": False, "run": True}

    def on_press(k):
        try:
            if k == keyboard.Key.space: st["lvl"] = 0.0; print("\n*** KILL ***"); return
            if k == keyboard.Key.esc:   st["lvl"] = 0.0; st["run"] = False; return
            c = k.char.upper() if (hasattr(k, "char") and k.char) else None
            if c in st: st[c] = True
        except Exception:
            pass

    def on_release(k):
        try:
            c = k.char.upper() if (hasattr(k, "char") and k.char) else None
            if c in st: st[c] = False
        except Exception:
            pass

    def send(roll, pitch, yaw, thrust):
        ser.write(b"\xA5" + struct.pack("<ffff", roll, pitch, yaw, thrust))

    lis = keyboard.Listener(on_press=on_press, on_release=on_release); lis.start()
    for _ in range(5):
        send(0, 0, 0, 0); time.sleep(0.05)

    while st["run"]:
        if st["W"]: st["lvl"] = min(1.0, st["lvl"] + THR_STEP)
        if st["S"]: st["lvl"] = max(0.0, st["lvl"] - THR_STEP)

        forward = st["lvl"] * FWD_POWER * FULL
        turn = ((1.0 if st["D"] else 0.0) - (1.0 if st["A"] else 0.0)) * TURN_POWER * PITCH_MAX
        manual_v = (UP_POWER if st["Q"] else (-DOWN_POWER if st["E"] else 0.0))
        pitch = VERT_SIGN * manual_v * PITCH_MAX

        send(0.0, float(pitch), float(turn), float(forward))
        print(f"\rlvl={st['lvl']:.2f}  fwd={forward:6.0f}  turn={turn:+6.0f}  vert={pitch:+7.0f}   ", end="")
        time.sleep(1.0 / RATE_HZ)

    for _ in range(3):
        send(0, 0, 0, 0); time.sleep(0.03)
    lis.stop()
    ser.close()
    print("\nStopped.")


if __name__ == "__main__":
    main()
