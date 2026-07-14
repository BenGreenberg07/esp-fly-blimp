#!/usr/bin/env python3
"""
fly_blimp.py — ESP-FLY BLIMP flyer with IMU YAW-HOLD ("fly straight").

The blimp's two forward motors steer by differential thrust, so uneven thrust (or
the nose-up/rotate from pushing forward) makes it curve and spin. This script
reads the drone's IMU yaw rate over Wi-Fi (CRTP log var gyro.z) and, while you're
driving forward and NOT manually turning, trims the turn channel to drive that
rotation to zero — i.e. it auto-corrects the left/right motors until it tracks
straight. Hold-to-ramp forward; up/down sign matches drive_blimp.py.

CONTROLS
    Up / Down arrow   forward throttle (hold to ramp; holds its level)
    Left / Right      manual turn (while held, yaw-hold is paused)
    W / S             up / down
    H                 toggle IMU yaw-hold on/off
    [ / ]             yaw-hold strength  (lower / higher)
    \\                 flip yaw-hold direction (if it makes the spin WORSE)
    Space             KILL (motors to 0)
    Esc               quit

Run from FLY_BLIMP.command, or:
    ./.venv/bin/python fly_blimp.py
(joined to the blimp Wi-Fi ESP-DRONE_80B54EF11031, pw 12345678)

NOTE: with the forward props still mounted backwards, the yaw-hold direction may
need flipping the first time — just tap '\\' if holding forward makes it spin
faster instead of straightening out.
"""

import time
import cflib.crtp
import cf_udp_patch
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from pynput import keyboard

URI = "udp://192.168.43.42:2390"
THR_STEP = 800       # forward ramp per tick (motor-duty units)
RATE_HZ = 20

# --- Passthrough convention (same as blimp_server.py) ---
# Firmware gains are set to 1.0 on connect, so the values we send ARE motor duty.
# The vertical channel (control.pitch) is int16 (max 32767 ~ 50%); the firmware
# blimp.vertScale=2.0 doubles it to reach full range, so we send up to PITCH_MAX.
MOTOR_FULL = 65535
PITCH_MAX = 32767        # int16 cap on the vertical/turn setpoint channels
VERT_POWER = 0.80        # up/down motors ~80% on full W/S  (needs firmware vertScale=2.0)
FWD_POWER = 0.30         # forward motors ~30%
TURN_POWER = 0.25        # manual A/D turn differential
TILT = VERT_POWER * PITCH_MAX            # control.pitch for W/S (vertScale doubles it -> ~80%)
YAW = TURN_POWER * PITCH_MAX             # control.yaw for A/D turn
THR_MAX = int(FWD_POWER * MOTOR_FULL)    # forward cap (control.thrust, full range) ~30%

# Up/down came out inverted (Q/up drove it down); flip the vertical axis. Same
# convention as drive_blimp.py. Set to +1.0 if you swap MOTOR_UP/DOWN in firmware.
VERT_SIGN = -1.0

# IMU yaw-hold: differential (fraction of full) added per (deg/s) of unwanted
# yaw rate. Sign may need flipping on the real blimp (props backwards) — '\' key.
YAWHOLD_KP_START = 0.010
YAWHOLD_KP_STEP = 0.003


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def main():
    st = {"thr": 0, "turn": 0.0, "vert": 0.0, "up": False, "down": False,
          "run": True, "hold": True, "kp": YAWHOLD_KP_START, "kpsign": 1.0,
          "yawrate": 0.0}

    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("CONNECTED.")
        # Open the firmware up to full range (we scale here), like drive_blimp.py.
        for k, v in (("blimp.fwdScale", 1.0), ("blimp.vertGain", 1.0), ("blimp.turnGain", 1.0),
                     ("blimp.vertScale", 2.0), ("blimp.pitchFF", 0.0)):
            try: cf.param.set_value(k, v)
            except Exception: pass

        # Subscribe to the IMU yaw rate (deg/s) for the yaw-hold loop.
        try:
            lg = LogConfig(name="imu", period_in_ms=50)
            lg.add_variable("gyro.z", "float")        # yaw rate, deg/s

            def _imu_cb(ts, data, cfg):
                st["yawrate"] = data["gyro.z"]
            cf.log.add_config(lg)
            lg.data_received_cb.add_callback(_imu_cb)
            lg.start()
            print("IMU yaw-rate log started (gyro.z).")
        except Exception as e:
            print(f"WARNING: could not start IMU log ({e}); yaw-hold disabled.")
            st["hold"] = False

        def show():
            print("\nFLY: arrows=fwd/turn  W/S=up/down  Space=KILL  Esc=quit")
            print("YAW-HOLD: H=toggle  [ ]=strength  \\=flip direction")
            print(f"hold={'ON' if st['hold'] else 'off'}  kp={st['kp']*st['kpsign']:+.1f}\n")
        show()

        def on_press(k):
            try:
                if k == keyboard.Key.space:
                    st["thr"] = 0; print("\n*** KILL ***"); return
                if k == keyboard.Key.esc:
                    st["thr"] = 0; st["run"] = False; return
                if k == keyboard.Key.up:    st["up"] = True
                elif k == keyboard.Key.down: st["down"] = True
                elif k == keyboard.Key.left:  st["turn"] = -YAW
                elif k == keyboard.Key.right: st["turn"] = YAW
                elif hasattr(k, "char") and k.char:
                    c = k.char.lower()
                    if c == "w": st["vert"] = TILT
                    elif c == "s": st["vert"] = -TILT
                    elif c == "h":
                        st["hold"] = not st["hold"]; print(f"\nyaw-hold {'ON' if st['hold'] else 'off'}")
                    elif c == "[":
                        st["kp"] = max(0.0, st["kp"] - YAWHOLD_KP_STEP); print(f"\nkp={st['kp']*st['kpsign']:+.1f}")
                    elif c == "]":
                        st["kp"] = st["kp"] + YAWHOLD_KP_STEP; print(f"\nkp={st['kp']*st['kpsign']:+.1f}")
                    elif c == "\\":
                        st["kpsign"] = -st["kpsign"]; print(f"\nyaw-hold direction flipped: kp={st['kp']*st['kpsign']:+.1f}")
            except Exception:
                pass

        def on_release(k):
            if k == keyboard.Key.up:   st["up"] = False
            if k == keyboard.Key.down: st["down"] = False
            if k in (keyboard.Key.left, keyboard.Key.right): st["turn"] = 0.0
            if hasattr(k, "char") and k.char and k.char.lower() in ("w", "s"):
                st["vert"] = 0.0

        # unlock
        for _ in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0); time.sleep(0.05)
        lis = keyboard.Listener(on_press=on_press, on_release=on_release); lis.start()

        while st["run"]:
            if st["up"]:   st["thr"] = min(THR_MAX, st["thr"] + THR_STEP)
            if st["down"]: st["thr"] = max(0, st["thr"] - THR_STEP)

            manual_turn = st["turn"]
            # IMU yaw-hold: when driving forward and not manually turning, drive
            # the measured yaw rate to zero (cancels the curve/spin).
            if st["hold"] and st["thr"] > 0 and manual_turn == 0.0:
                corr = clamp(-st["kpsign"] * st["kp"] * st["yawrate"] * PITCH_MAX, -YAW, YAW)
            else:
                corr = 0.0
            turn_out = clamp(manual_turn + corr, -YAW, YAW)
            vert_out = VERT_SIGN * st["vert"]

            cf.commander.send_setpoint(0, vert_out, turn_out, st["thr"])
            print(f"\rthr={st['thr']:5d}  turn={turn_out:+5.0f} (corr={corr:+5.1f})  "
                  f"vert={vert_out:+5.0f}  yawrate={st['yawrate']:+6.1f}  "
                  f"hold={'ON ' if st['hold'] else 'off'}   ", end="")
            time.sleep(1.0 / RATE_HZ)

        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint(); lis.stop()
        print("\nStopped.")


if __name__ == "__main__":
    main()
