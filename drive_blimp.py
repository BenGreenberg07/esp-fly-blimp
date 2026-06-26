#!/usr/bin/env python3
"""
ESP-FLY BLIMP — real flying (WASD + Q/E), with forward->down auto-coupling.

The trick: pushing forward on the two forward motors alone tends to make the
blimp rotate/tip. So this script automatically blends in some DOWN motor in
proportion to forward thrust, which counters that and keeps it tracking
straight. Just hold W — the coupling is handled for you.

CONTROLS
    W   forward   (hold to ramp up; auto-engages the down motor to stop the spin)
    S   slow/stop (hold to ramp forward back down)
    A   turn left
    D   turn right
    Q   up    (up motor — M4 is the disconnected one until you resolder it)
    E   down  (extra down motor, on top of the auto-coupling)
    Space  KILL (everything to 0)
    Esc    quit

TUNING
    COUPLE_DOWN_DEG = how much "down" gets blended in at full forward.
    Still spins -> raise it.  Noses down too hard -> lower it.
    Makes the spin WORSE -> flip the sign (wrong direction).

Run from DRIVE_BLIMP.command, or:
    ./.venv/bin/python drive_blimp.py
(joined to the blimp Wi-Fi ESP-DRONE_80B54EF11031, pw 12345678)
"""

import time
import cflib.crtp
import cf_udp_patch
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from pynput import keyboard

URI = "udp://192.168.43.42:2390"
THR_STEP = 1000       # forward ramp per tick (hold W/S)
THR_MAX = 60000       # forward cap (uint16 max for CRTP thrust — do NOT exceed 65535)
TILT = 18.0           # deg sent for manual Q/E (up/down authority)
YAW = 110.0           # deg/s sent for A/D turn
RATE_HZ = 20

# FORWARD power 0.1..1.0 — adjust LIVE with number keys (1=10%..9=90%, 0=100%).
FWD_POWER_START = 0.40
# UP/DOWN power — fixed strong (so vertical isn't overpowered by forward).
VERT_POWER = 0.90
# TURN power (turning uses the forward motors).
TURN_POWER = 0.60

# Forward -> down coupling: degrees of "down" blended in at full forward.
# This is the "so it doesn't spin" term. Tune it; flip sign if it's backwards.
COUPLE_DOWN_DEG = 8.0

# Vertical axis sign. Up/down came out inverted (pressing Q "up" drove it DOWN),
# so we flip the whole vertical axis. This is the same as swapping the Q and E
# buttons, and it also keeps the anti-spin down-coupling pushing the right way.
# Set back to +1.0 if you later swap MOTOR_UP/MOTOR_DOWN in the firmware mixer.
VERT_SIGN = -1.0


def main():
    st = {"thr": 0, "power": FWD_POWER_START, "W": False, "S": False, "A": False,
          "D": False, "Q": False, "E": False, "run": True}

    def on_press(k):
        try:
            if k == keyboard.Key.space: st["thr"] = 0; print("\n*** KILL ***"); return
            if k == keyboard.Key.esc:   st["thr"] = 0; st["run"] = False; return
            c = k.char.upper() if (hasattr(k, "char") and k.char) else None
            if c and c.isdigit():       # live FORWARD power level
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

    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("CONNECTED.")
        # Open up the firmware for full power; the POWER knob scales below this.
        # Coupling is done here in Python, so zero the firmware pitch feed-forward.
        try:
            cf.param.set_value("blimp.fwdScale", 1.0)    # full forward range (was 0.4)
            cf.param.set_value("blimp.vertGain", 900.0)  # stronger up/down (was 500)
            cf.param.set_value("blimp.turnGain", 130.0)  # stronger turn (was 60)
            cf.param.set_value("blimp.pitchFF", 0.0)
        except Exception:
            pass
        print("W=forward(+auto-down)  S=slow  A/D=turn  Q=up  E=down")
        print("FORWARD power: keys 1-9 = 10-90%, 0 = 100% (up/down fixed at 90%)")
        print("Space=KILL  Esc=quit\n")

        for _ in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0); time.sleep(0.05)
        lis = keyboard.Listener(on_press=on_press, on_release=on_release); lis.start()

        while st["run"]:
            if st["W"]: st["thr"] = min(THR_MAX, st["thr"] + THR_STEP)
            if st["S"]: st["thr"] = max(0, st["thr"] - THR_STEP)
            turn = (YAW if st["D"] else 0.0) - (YAW if st["A"] else 0.0)
            manual_vert = (TILT if st["Q"] else 0.0) - (TILT if st["E"] else 0.0)
            couple = -COUPLE_DOWN_DEG * (st["thr"] / THR_MAX)   # down, scales with forward
            fwd = int(min(60000, st["thr"] * st["power"]))      # forward power (number keys), uint16-safe
            vert = VERT_SIGN * (manual_vert + couple) * VERT_POWER   # up/down (sign-corrected), strong (90%)
            turn = turn * TURN_POWER
            cf.commander.send_setpoint(0, vert, turn, fwd)
            print(f"\rfwdP={int(st['power']*100):3d}%  fwd={fwd:5d}  turn={turn:+5.0f}  vert={vert:+6.1f}   ", end="")
            time.sleep(1.0 / RATE_HZ)

        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint(); lis.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
