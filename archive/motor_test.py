#!/usr/bin/env python3
"""
ESP-FLY motor tester / board health check (works on stock OR blimp firmware).

It drives each motor CHANNEL (M1-M4) directly via the firmware's motorPowerSet
override, so it ignores the mixer entirely — your backwards build and the
disconnected motor don't matter; each channel just spins (or doesn't).

Use it to:
  - confirm the board/chip is ALIVE (it connects + reports params/battery), and
  - map which physical motor is M1/M2/M3/M4 (spin one at a time, watch),
  - find the disconnected/dead one (it won't spin).

SAFETY: remove props or hold the frame down. A spinning prop can cut you.

Run with the .command file, or:
    cd ~/Co-Create_ESP-FLY/Firmware
    ./.venv/bin/python motor_test.py
(while joined to the drone's Wi-Fi ESP-DRONE_xxxx, password 12345678)
"""

import time
import cflib.crtp
import cf_udp_patch  # ESP-Drone UDP checksum patch
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = "udp://192.168.43.42:2390"
POWER = 12000        # 0..65535  (~18% — enough to spin, gentle)
SPIN_SECONDS = 3.0


def diagnose_no_connect(err):
    print("\n*** COULD NOT CONNECT TO THE BOARD ***")
    print(f"    ({err})\n")
    print("What this means / check in order:")
    print("  1. Is your Mac joined to the drone's Wi-Fi (ESP-DRONE_xxxx, pw 12345678)?")
    print("     If that network DOESN'T EVEN APPEAR, the chip isn't booting -> likely")
    print("     fried or not powered (check battery + the XIAO's red/charge LED).")
    print("  2. Is the battery plugged in and charged? (USB alone powers the chip but")
    print("     not the motor rail.)")
    print("  3. Try `ping 192.168.43.42` in Terminal — if it replies, re-run this.")
    print("  If the AP appears and pings but this still fails, tell Claude the error.")


def main():
    cflib.crtp.init_drivers()
    print(f"Connecting to {URI} ...")
    cf = Crazyflie(rw_cache="./cache")
    scf = SyncCrazyflie(URI, cf=cf)
    try:
        scf.open_link()
    except Exception as e:
        diagnose_no_connect(e)
        return

    print("\n==============================================")
    print(" CONNECTED — the chip is ALIVE and talking. ")
    try:
        print(f"   param table: {len(cf.param.toc.toc)} entries")
        print(f"   log table:   {len(cf.log.toc.toc)} entries")
    except Exception:
        pass
    try:
        print(f"   battery (pm.vbat): {cf.param.get_value('pm.vbat')} V")
    except Exception:
        print("   (battery read unavailable)")
    print("==============================================\n")

    # Enable direct motor override (bypasses the mixer).
    cf.param.set_value("system.forceArm", 1)
    cf.param.set_value("motorPowerSet.enable", 1)

    def all_off():
        for j in range(1, 5):
            cf.param.set_value(f"motorPowerSet.m{j}", 0)

    def spin(ch):
        print(f"  -> M{ch} ON for {SPIN_SECONDS:.0f}s ... watch which motor moves")
        for j in range(1, 5):
            cf.param.set_value(f"motorPowerSet.m{j}", POWER if j == ch else 0)
        time.sleep(SPIN_SECONDS)
        all_off()
        print(f"  -> M{ch} off\n")

    print("Remove props or hold the frame down. Commands:")
    print("   1 / 2 / 3 / 4  = spin that motor for 3 s")
    print("   a             = sweep all four in order")
    print("   s             = stop everything now")
    print("   q             = quit\n")

    try:
        while True:
            c = input("motor [1-4 / a / s / q]: ").strip().lower()
            if c == "q":
                break
            elif c == "a":
                for ch in (1, 2, 3, 4):
                    spin(ch)
            elif c == "s":
                all_off(); print("  stopped.\n")
            elif c in ("1", "2", "3", "4"):
                spin(int(c))
            else:
                print("  ? enter 1-4, a, s, or q")
    finally:
        all_off()
        cf.param.set_value("motorPowerSet.enable", 0)
        cf.param.set_value("system.forceArm", 0)
        scf.close_link()
        print("\nStopped, override disabled, link closed. Done.")


if __name__ == "__main__":
    main()
