#!/usr/bin/env python3
"""
ESP-FLY blimp control / test script (macOS).

Talks to the ESP-FLY over Wi-Fi using the Crazyflie protocol (CRTP) carried
over UDP, exactly like the official cfclient. It matches the custom blimp
firmware where the four motors are:

    M1 = forward-left      M2 = forward-right
    M3 = up (climb)        M4 = down (descend)

and the setpoint axes are remapped (see stabilizer.c / power_distribution_stock.c):

    thrust -> forward speed      (0 .. 60000)
    pitch  -> vertical (+climb / -descend)
    yaw    -> turn (differential forward motors)
    roll   -> unused

SETUP (once):
    python3 -m pip install cflib
    # for keyboard teleop also:  python3 -m pip install pynput

Then join the drone's Wi-Fi AP "ESP-DRONE_xxxxxxxxxxxx" (password 12345678)
in macOS Wi-Fi settings, and run:

    python3 blimp_control.py test      # spin each motor channel one at a time (wiring ID)
    python3 blimp_control.py demo      # scripted flight self-test sequence
    python3 blimp_control.py keys      # arrow-key teleop (needs pynput)

SAFETY: keep propellers OFF (or the gondola off the envelope) the first time
and confirm each motor spins in the right direction before real flight.
"""

import sys
import time

import cflib.crtp
import cf_udp_patch  # patches UDP driver for ESP-Drone checksum framing
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://192.168.43.42:2390"   # ESP-FLY SoftAP gateway, CRTP/UDP port 2390

# Tune these to taste (0..60000 forward, deg for pitch, deg/s for yaw):
FWD_SPEED = 18000    # forward thrust used in the demo
VERT_CMD  = 15.0     # "pitch" degrees -> ~15000 PWM on the up/down motor
TURN_RATE = 90.0     # "yaw" deg/s    -> ~9000 PWM differential
RATE_HZ   = 20       # how often we resend the setpoint (keep > 2 Hz)


def _stream(cf, roll, pitch, yaw, thrust, seconds):
    """Hold one setpoint for `seconds`, resending at RATE_HZ."""
    end = time.time() + seconds
    while time.time() < end:
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        time.sleep(1.0 / RATE_HZ)


def unlock(cf):
    """CRTP requires a thrust-0 setpoint first to leave the locked state."""
    for _ in range(10):
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.05)


def test(cf):
    """Spin each physical motor CHANNEL (M1..M4) one at a time.

    Uses the firmware's built-in motorPowerSet override, so this bypasses the
    mixer entirely - no flight, propellers can stay off. Watch which physical
    motor moves for each M#, then set the four #defines (MOTOR_FWD_LEFT etc.)
    in power_distribution_stock.c to match, and rebuild/flash.
    """
    cf.param.set_value("system.forceArm", 1)      # make sure motors are enabled
    cf.param.set_value("motorPowerSet.enable", 1)  # bypass mixer, drive m1..m4 directly
    time.sleep(0.3)
    power = 12000  # ~18% duty: enough to clearly spin, gentle on a tethered test
    try:
        for i in range(1, 5):
            print(f"--> Channel M{i} ON for 2s")
            for j in range(1, 5):
                cf.param.set_value(f"motorPowerSet.m{j}", power if j == i else 0)
            time.sleep(2)
            for j in range(1, 5):
                cf.param.set_value(f"motorPowerSet.m{j}", 0)
            time.sleep(1)
    finally:
        for j in range(1, 5):
            cf.param.set_value(f"motorPowerSet.m{j}", 0)
        cf.param.set_value("motorPowerSet.enable", 0)
    print("\nDone. Record: M1=?  M2=?  M3=?  M4=?  (forward-L / forward-R / up / down)")
    print("Then edit the #defines in power_distribution_stock.c and rebuild.")


def demo(cf):
    print("Unlocking...")
    unlock(cf)

    print("Forward 3s");        _stream(cf, 0, 0, 0, FWD_SPEED, 3)
    print("Stop 1s");           _stream(cf, 0, 0, 0, 0, 1)
    print("Turn right 2s");     _stream(cf, 0, 0, TURN_RATE, 0, 2)
    print("Turn left 2s");      _stream(cf, 0, 0, -TURN_RATE, 0, 2)
    print("Climb 2s");          _stream(cf, 0, VERT_CMD, 0, 0, 2)
    print("Descend 2s");        _stream(cf, 0, -VERT_CMD, 0, 0, 2)
    print("Forward + turn 3s"); _stream(cf, 0, 0, TURN_RATE / 2, FWD_SPEED, 3)

    print("Idle / stop");       cf.commander.send_setpoint(0, 0, 0, 0)
    cf.commander.send_stop_setpoint()


def keys(cf):
    """Arrow-key teleop:  Up/Down = forward speed, Left/Right = turn,
    W/S = climb/descend.  Release returns that axis to neutral. Esc quits."""
    from pynput import keyboard

    state = {"fwd": 0, "turn": 0.0, "vert": 0.0, "run": True}
    step_fwd = 4000

    def on_press(k):
        try:
            if k == keyboard.Key.up:    state["fwd"]  = min(60000, state["fwd"] + step_fwd)
            elif k == keyboard.Key.down: state["fwd"] = max(0, state["fwd"] - step_fwd)
            elif k == keyboard.Key.left:  state["turn"] = -TURN_RATE
            elif k == keyboard.Key.right: state["turn"] =  TURN_RATE
            elif hasattr(k, "char") and k.char == "w": state["vert"] =  VERT_CMD
            elif hasattr(k, "char") and k.char == "s": state["vert"] = -VERT_CMD
            elif k == keyboard.Key.esc:   state["run"] = False
        except Exception:
            pass

    def on_release(k):
        if k in (keyboard.Key.left, keyboard.Key.right): state["turn"] = 0.0
        if hasattr(k, "char") and k.char in ("w", "s"):  state["vert"] = 0.0

    unlock(cf)
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    print("Arrows=forward/turn, W/S=up/down, Esc=quit")
    while state["run"]:
        cf.commander.send_setpoint(0, state["vert"], state["turn"], state["fwd"])
        time.sleep(1.0 / RATE_HZ)
    cf.commander.send_stop_setpoint()
    listener.stop()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "demo"
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        print("Connected.")
        {"test": test, "keys": keys, "demo": demo}.get(mode, demo)(scf.cf)


if __name__ == "__main__":
    main()
