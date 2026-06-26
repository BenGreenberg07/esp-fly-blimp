#!/bin/bash
# Double-click: FLY the blimp. Forward auto-engages the down motor so it tracks straight.
# Be on the blimp's Wi-Fi (ESP-DRONE_80B54EF11031, pw 12345678). Balloon attached.
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp drive ==="
echo "W=forward (auto-down)  S=slow  A/D=turn  Q=up  E=down  Space=KILL  Esc=quit"
echo
./.venv/bin/python drive_blimp.py
echo
echo "Press Return to close."
read
