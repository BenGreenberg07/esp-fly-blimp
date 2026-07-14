#!/bin/bash
# Double-click: fly the blimp + tune its dynamics live (no reflash).
# Be on the blimp's Wi-Fi (ESP-DRONE_80B54EF11031, pw 12345678). Balloon attached.
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp: fly + live tune ==="
echo "Arrows=fwd/turn  W/S=up/down  Space=KILL  Esc=quit"
echo "Tune: 1/2 yawTrim  3/4 pitchFF  5/6 vertTrim  7/8 fwdScale"
echo
./.venv/bin/python fly_blimp.py
echo
echo "Press Return to close."
read
