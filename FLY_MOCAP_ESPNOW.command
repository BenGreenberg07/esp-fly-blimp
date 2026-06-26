#!/bin/bash
# =====================================================================
#  AUTONOMOUS BLIMP via MOCAP -> ESP-NOW   (the drone computes the flight)
#  Data path:  OptiTrack --lab WiFi--> Mac --USB--> C6 --ESP-NOW--> blimp
#  The Mac only forwards pose+target; blimp_guidance.c on the drone does
#  the decoupled-PID control itself. No drone Wi-Fi join, no reflash.
# =====================================================================
#
#  >>> FILL THESE IN (from Motive), then just double-click this file <<<
#
MOTIVE_IP="192.168.0.4"   # OptiTrack/Motive PC IP  (Motive > Edit > Settings > Streaming)
BODY_ID="531"                # the blimp's Rigid Body "Streaming ID" (in its properties)
#
#  Target waypoint in the MOCAP frame (meters) + final heading to hold (deg):
TARGET_X="1.0"
TARGET_Y="0.0"
TARGET_Z="1.2"
TARGET_YAW="0"
#
#  UP AXIS: if motors drive the wrong way vertically, your Motive "Up Axis" is
#  Z, not Y. Open blimp_mocap.py, line ~45, set:  UP_AXIS = "Z"   (default "Y").
# =====================================================================

cd "$(dirname "$0")"
echo "=== AUTONOMOUS BLIMP (mocap -> ESP-NOW, drone computes) ==="
echo "Checklist:"
echo "  [ ] C6 bridge plugged into this Mac's USB"
echo "  [ ] drone ON ITS LIPO and booted (ESP-NOW firmware)"
echo "  [ ] this Mac is on the LAB Wi-Fi (so it receives mocap)"
echo "  [ ] PROPS OFF for the first run"
echo "  Target = ($TARGET_X, $TARGET_Y, $TARGET_Z)  heading $TARGET_YAW deg   body #$BODY_ID @ $MOTIVE_IP"
echo
echo "Press Return to start (Ctrl-C any time = stop; motors zero in 0.5s)."
read

./.venv/bin/python blimp_mocap.py --onboard --bridge \
    --server "$MOTIVE_IP" --body "$BODY_ID" \
    --target "$TARGET_X" "$TARGET_Y" "$TARGET_Z" --target-yaw "$TARGET_YAW"

echo
echo "Stopped — streaming ended, drone failsafe zeros the motors. Press Return to close."
read
