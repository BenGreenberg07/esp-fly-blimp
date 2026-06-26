#!/usr/bin/env python3
"""
ESP-FLY BLIMP — fly over BLUETOOTH (BLE), so your Mac's Wi-Fi stays free for the
mocap network. Same feel as drive_blimp.py: WASD + Q/E, hold-W-to-ramp forward,
auto-engages the down motor (anti-spin), per-axis power.

WHY BLE: the lab Wi-Fi (AIRLab-BigLab) is MAC-locked and rejects the drone, and
USB Wi-Fi adapters don't work on Apple Silicon. The drone now advertises a BLE
control service ("ESP-BLIMP"); this client writes setpoints to it over Bluetooth
while macOS Wi-Fi stays on mocap. No CRTP/Wi-Fi link is used here.

CONTROLS  (identical to drive_blimp.py)
    W   forward   (hold to ramp up; auto-engages the down motor to stop the spin)
    S   slow/stop (hold to ramp forward back down)
    A   turn left
    D   turn right
    Q   up
    E   down
    1-9 forward power 10-90%, 0 = 100%
    Space  KILL (everything to 0)
    Esc    quit

Run:
    ./.venv/bin/python drive_blimp_ble.py
(Mac Bluetooth ON. Do NOT need to join the drone's Wi-Fi.)

NOTE: BLE control runs against the firmware's COMPILED-IN blimp scales
(fwdScale=0.40, vertGain=500, turnGain=60) because there's no CRTP param link
over BLE. This client pre-divides by those so the motor output matches what
drive_blimp.py produced over Wi-Fi. If you reflash with different defaults,
update FW_* below to match.
"""

import asyncio
import struct

from bleak import BleakScanner, BleakClient
from pynput import keyboard

DEVICE_NAME = "ESP-BLIMP"
RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # NUS RX: host -> drone
RATE_HZ = 20

# Firmware compiled-in BLIMP_MODE scales (stabilizer multiplies the setpoint by
# these). We divide by them so what reaches the motors equals the OUT_* targets.
FW_FWD_SCALE = 0.40
FW_VERT_GAIN = 500.0
FW_TURN_GAIN = 60.0

# Forward ramp (control-thrust domain, same as drive_blimp.py: out_fwd = thr*power)
THR_STEP = 1000
THR_MAX = 60000
FWD_POWER_START = 0.40          # adjust live with number keys

# Vertical/turn output targets at full deflection (control.pitch / control.yaw
# domain), matched to drive_blimp.py: 18deg*0.9*900=14580, 110*0.6*130=8580.
VERT_OUT_FULL = 14580.0
TURN_OUT_FULL = 8580.0
# Forward -> down anti-spin coupling: down output blended in at full forward ramp.
# drive_blimp.py: 8deg * 0.9 * 900 = 6480. Flip sign if it spins the wrong way.
COUPLE_OUT_FULL = -6480.0


def main():
    st = {"thr": 0, "power": FWD_POWER_START, "W": False, "S": False, "A": False,
          "D": False, "Q": False, "E": False, "run": True}

    def on_press(k):
        try:
            if k == keyboard.Key.space: st["thr"] = 0; print("\n*** KILL ***"); return
            if k == keyboard.Key.esc:   st["thr"] = 0; st["run"] = False; return
            c = k.char.upper() if (hasattr(k, "char") and k.char) else None
            if c and c.isdigit():
                st["power"] = 1.0 if c == "0" else int(c) / 10.0
                print(f"\nforward power = {int(st['power']*100)}%")
                return
            if c in st: st[c] = True
        except Exception:
            pass

    def on_release(k):
        try:
            c = k.char.upper() if (hasattr(k, "char") and k.char) else None
            if c in st: st[c] = False
        except Exception:
            pass

    async def run():
        print(f"Scanning for '{DEVICE_NAME}' over BLE ...")
        dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
        if dev is None:
            print(f"Could not find '{DEVICE_NAME}'. Is the drone powered and Bluetooth on?")
            return
        print(f"Found {dev.address}. Connecting ...")
        async with BleakClient(dev) as client:
            print("CONNECTED over BLE.")
            print("W=forward(+auto-down)  S=slow  A/D=turn  Q=up  E=down")
            print("FORWARD power: keys 1-9 = 10-90%, 0 = 100%  |  Space=KILL  Esc=quit\n")
            lis = keyboard.Listener(on_press=on_press, on_release=on_release); lis.start()

            async def send(roll, pitch, yaw, thrust):
                pkt = struct.pack("<ffff", roll, pitch, yaw, thrust)
                await client.write_gatt_char(RX_UUID, pkt, response=False)

            # gentle unlock
            for _ in range(5):
                await send(0, 0, 0, 0); await asyncio.sleep(0.05)

            while st["run"]:
                if st["W"]: st["thr"] = min(THR_MAX, st["thr"] + THR_STEP)
                if st["S"]: st["thr"] = max(0, st["thr"] - THR_STEP)

                out_fwd = st["thr"] * st["power"]                       # control.thrust target
                ramp = st["thr"] / THR_MAX                              # 0..1
                manual_vert = (1.0 if st["Q"] else 0.0) - (1.0 if st["E"] else 0.0)
                out_vert = manual_vert * VERT_OUT_FULL + COUPLE_OUT_FULL * ramp
                out_turn = ((1.0 if st["D"] else 0.0) - (1.0 if st["A"] else 0.0)) * TURN_OUT_FULL

                # convert control-domain targets -> BLE setpoint (firmware will re-scale)
                thrust = out_fwd / FW_FWD_SCALE
                pitch = out_vert / FW_VERT_GAIN
                yaw = out_turn / FW_TURN_GAIN
                await send(0.0, pitch, yaw, thrust)

                print(f"\rfwdP={int(st['power']*100):3d}%  fwd={out_fwd:6.0f}  "
                      f"turn={out_turn:+6.0f}  vert={out_vert:+7.0f}   ", end="")
                await asyncio.sleep(1.0 / RATE_HZ)

            for _ in range(3):
                await send(0, 0, 0, 0); await asyncio.sleep(0.03)
            lis.stop()
            print("\nStopped.")

    asyncio.run(run())


if __name__ == "__main__":
    main()
