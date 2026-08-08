#!/usr/bin/env python3
"""
ESP-FLY connection test  (READ-ONLY, no motors, no flight).

Confirms the Mac can talk to the drone over Wi-Fi using the Crazyflie
protocol over UDP. It:
  1. opens the link to 192.168.43.42 (the SoftAP gateway, UDP port 2390),
  2. waits for the param/log table-of-contents to download,
  3. reads the battery voltage,
  4. streams live IMU attitude (roll/pitch/yaw) for a few seconds.

If all of that prints, your computer<->drone connection works.

Run (from the venv):
    cd ~/Co-Create_ESP-FLY/Firmware
    ./.venv/bin/python connect_test.py
"""

import time

import cflib.crtp
import cf_udp_patch  # patches UDP driver for ESP-Drone checksum framing
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = "udp://192.168.43.42:2390"
STREAM_SECONDS = 5


def main():
    cflib.crtp.init_drivers()
    print(f"Opening link to {URI} ...")
    cf = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(URI, cf=cf) as scf:
        print("LINK UP - drone is connected.\n")

        # TOC sizes prove the param/log negotiation completed.
        n_params = len(scf.cf.param.toc.toc)
        n_logs = len(scf.cf.log.toc.toc)
        print(f"Param table entries: {n_params}")
        print(f"Log   table entries: {n_logs}")

        # Battery voltage (1S LiPo: ~4.2 V full, ~3.3 V empty).
        try:
            vbat = scf.cf.param.get_value("pm.vbat")
            print(f"Battery (param pm.vbat): {vbat} V")
        except Exception:
            pass  # not all builds expose vbat as a param; logged below anyway

        # Stream live attitude + battery to prove the data path both ways.
        lg = LogConfig(name="check", period_in_ms=200)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("pm.vbat", "float")

        print(f"\nStreaming IMU for {STREAM_SECONDS}s (tilt the drone to watch it move):")
        end = time.time() + STREAM_SECONDS
        with SyncLogger(scf, lg) as logger:
            for _, data, _ in logger:
                print(f"  roll={data['stabilizer.roll']:+6.1f}  "
                      f"pitch={data['stabilizer.pitch']:+6.1f}  "
                      f"yaw={data['stabilizer.yaw']:+7.1f}  "
                      f"vbat={data['pm.vbat']:.2f}V")
                if time.time() > end:
                    break

        print("\nConnection test PASSED. Link, params, logging all work.")


if __name__ == "__main__":
    main()
