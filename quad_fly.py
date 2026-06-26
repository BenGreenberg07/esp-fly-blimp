#!/usr/bin/env python3
"""
ESP-FLY STOCK QUADCOPTER manual flight (keyboard).  You are in control.

This is for the drone running the STOCK quad firmware (not the blimp build).
Throttle is a value YOU raise and lower; the drone does NOT hold altitude by
itself. There is an instant kill key.

CONTROLS
    W / S        forward / back   (pitch, hold)
    A / D        left / right      (roll, hold)
    Q / E        up / down         (throttle up / down, small steps)
    Arrow L/R    yaw spin left / right (optional, hold)
    SPACE        KILL  -> throttle instantly to 0 (motors stop)
    Esc          quit (also stops motors)

SAFETY
    * Fly over an open, soft area. Spinning props can cut fingers.
    * Keep a hand near SPACE the whole time.
    * Raise throttle SLOWLY with Q. It will lift off around 35000-45000.
"""

import time
import cflib.crtp
import cf_udp_patch  # patches UDP driver for ESP-Drone checksum framing
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from pynput import keyboard
import flight_config

CFG = flight_config.load()
URI = CFG["uri"]
ROLL_TRIM = CFG["roll_trim"]    # deg added to every setpoint (tune in control panel)
PITCH_TRIM = CFG["pitch_trim"]

THR_STEP   = 1500      # throttle change per W/S press
THR_MAX    = 55000     # hard ceiling so a stuck key can't go full power
TILT_DEG   = 12.0      # roll/pitch angle when arrow held
YAW_RATE   = 90.0      # deg/s when A/D held
RATE_HZ    = 50        # setpoint send rate


def main():
    st = {"thr": 0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "run": True}

    def on_press(k):
        try:
            if hasattr(k, "char") and k.char == "q":          # UP
                st["thr"] = min(THR_MAX, st["thr"] + THR_STEP)
            elif hasattr(k, "char") and k.char == "e":        # DOWN
                st["thr"] = max(0, st["thr"] - THR_STEP)
            elif hasattr(k, "char") and k.char == "w":        # FORWARD
                st["pitch"] =  TILT_DEG
            elif hasattr(k, "char") and k.char == "s":        # BACK
                st["pitch"] = -TILT_DEG
            elif hasattr(k, "char") and k.char == "a":        # LEFT
                st["roll"] = -TILT_DEG
            elif hasattr(k, "char") and k.char == "d":        # RIGHT
                st["roll"] =  TILT_DEG
            elif k == keyboard.Key.left:  st["yaw"] = -YAW_RATE
            elif k == keyboard.Key.right: st["yaw"] =  YAW_RATE
            elif k == keyboard.Key.space:
                st["thr"] = 0                                 # KILL
                print("\n*** KILL ***")
            elif k == keyboard.Key.esc:
                st["thr"] = 0
                st["run"] = False
        except Exception:
            pass

    def on_release(k):
        if hasattr(k, "char") and k.char in ("w", "s"):  st["pitch"] = 0.0
        if hasattr(k, "char") and k.char in ("a", "d"):  st["roll"] = 0.0
        if k in (keyboard.Key.left, keyboard.Key.right): st["yaw"] = 0.0

    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("CONNECTED.  WASD=move, Q=up E=down, arrows=yaw, SPACE=KILL, Esc=quit.\n")
        # Unlock: first setpoint must be thrust 0.
        for _ in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        while st["run"]:
            cf.commander.send_setpoint(st["roll"] + ROLL_TRIM,
                                       st["pitch"] + PITCH_TRIM,
                                       st["yaw"], st["thr"])
            print(f"\rthrottle={st['thr']:5d}  roll={st['roll']:+5.0f} "
                  f"pitch={st['pitch']:+5.0f} yaw={st['yaw']:+5.0f}   ", end="")
            time.sleep(1.0 / RATE_HZ)
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint()
        listener.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
