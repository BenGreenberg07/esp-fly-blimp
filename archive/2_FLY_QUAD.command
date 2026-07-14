#!/bin/bash
# Double-click this to fly the STOCK quadcopter by keyboard. YOU control throttle.
# Controls:  W/S = throttle up/down, Arrows = tilt, A/D = yaw, SPACE = KILL, Esc = quit
cd "$(dirname "$0")"
echo "=== ESP-FLY manual flight ==="
echo "Be on the drone's Wi-Fi. Fly over open soft ground. Keep a hand on SPACE (kill)."
echo "Raise throttle SLOWLY with W."
echo
./.venv/bin/python quad_fly.py
echo
echo "Press Return to close."
read
