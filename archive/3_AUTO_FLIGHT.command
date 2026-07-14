#!/bin/bash
# Double-click: autonomous hop -> UP, hover 3s, DOWN. SPACE/Esc = abort anytime.
# Open-loop (no altitude sensor): tune THRUST_HOVER in auto_flight.py first.
cd "$(dirname "$0")"
echo "=== ESP-FLY auto flight: up -> hover 3s -> down ==="
echo "Be on the drone's Wi-Fi. Open soft area. Hand on SPACE (abort)."
echo "If it climbs too hard or drifts, hit SPACE immediately."
echo
./.venv/bin/python auto_flight.py
echo
echo "Press Return to close."
read
