#!/usr/bin/env python3
"""
ESP-FLY autonomous hop:  go UP -> hover -> come DOWN.

IMPORTANT: the stock quad has NO altitude sensor, so this is OPEN-LOOP. It
flies fixed throttle for fixed time; it cannot truly "hold" height and will
drift. Fly over an open, soft area, hand on SPACE (abort).

Defaults come from flight_config.json (shared with the manual script and the
web control panel). Override any of them on the command line:

    ./.venv/bin/python auto_flight.py --hover-time 5
    ./.venv/bin/python auto_flight.py --hover-thrust 40000 --climb-time 2

Press SPACE or Esc at ANY time to cut the motors instantly.
"""

import argparse
import time
import cflib.crtp
import cf_udp_patch  # ESP-Drone checksum framing
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from pynput import keyboard
import flight_config

RATE_HZ = 50
abort = {"on": False}


def get_args():
    cfg = flight_config.load()
    p = argparse.ArgumentParser(description="ESP-FLY auto hop: up -> hover -> down")
    p.add_argument("--climb-thrust", type=int, default=cfg["thrust_climb"])
    p.add_argument("--hover-thrust", type=int, default=cfg["thrust_hover"])
    p.add_argument("--climb-time", type=float, default=cfg["climb_time"])
    p.add_argument("--hover-time", type=float, default=cfg["hover_time"])
    p.add_argument("--land-time", type=float, default=cfg["land_time"])
    args = p.parse_args()
    args.uri = cfg["uri"]
    args.roll_trim = cfg["roll_trim"]
    args.pitch_trim = cfg["pitch_trim"]
    return args


def _ramp(cf, thr_start, thr_end, seconds, label, trim):
    """Linearly ramp throttle start->end over `seconds`. Returns False if aborted."""
    print(f"{label} (throttle {thr_start} -> {thr_end}, {seconds}s)")
    steps = max(1, int(seconds * RATE_HZ))
    for i in range(steps + 1):
        if abort["on"]:
            return False
        thr = int(thr_start + (thr_end - thr_start) * (i / steps))
        cf.commander.send_setpoint(trim[0], trim[1], 0, thr)   # trim keeps it straight
        time.sleep(1.0 / RATE_HZ)
    return True


def on_press(k):
    if k == keyboard.Key.space or k == keyboard.Key.esc:
        abort["on"] = True
        print("\n*** ABORT - motors off ***")


def main():
    a = get_args()
    trim = (a.roll_trim, a.pitch_trim)

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    cflib.crtp.init_drivers()
    print(f"Connecting to {a.uri} ...")
    print(f"Sequence: up {a.climb_time}s -> hover {a.hover_time}s -> down {a.land_time}s "
          f"(hover thrust {a.hover_thrust})")
    with SyncCrazyflie(a.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("CONNECTED.  Starting in 3s. SPACE/Esc = abort.\n")
        for n in (3, 2, 1):
            print(f"  {n}...")
            time.sleep(1)

        for _ in range(10):                       # unlock
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.05)

        ok = (_ramp(cf, 0, a.climb_thrust, a.climb_time, "UP", trim)
              and _ramp(cf, a.hover_thrust, a.hover_thrust, a.hover_time, "HOVER", trim)
              and _ramp(cf, a.hover_thrust, 0, a.land_time, "DOWN", trim))

        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint()
        listener.stop()
        print("\nDone." if ok else "\nAborted - stopped.")


if __name__ == "__main__":
    main()
