#!/bin/bash
# Double-click: open the blimp WEB PANEL (drive + live tuning + IMU telemetry).
# Be on the blimp's Wi-Fi (ESP-DRONE_80B54EF11031, pw 12345678). Balloon attached.
# A browser tab opens at http://127.0.0.1:8421 — click "Connect", then fly.
#
# NOT a standalone script — run it from a full clone of this repo: it runs
# control/blimp_server.py via ./.venv/bin/python, so it needs the repo folders
# and a .venv with cflib + pyserial. See the README "Build & run" section.
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp web panel ==="
echo "Opening http://127.0.0.1:8421  (keep this window open while flying)"
echo
./.venv/bin/python control/blimp_server.py
echo
echo "Press Return to close."
read
