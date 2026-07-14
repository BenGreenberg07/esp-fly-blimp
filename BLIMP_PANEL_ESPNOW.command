#!/bin/bash
# Double-click: open the blimp WEB PANEL driving over ESP-NOW (the XIAO C6 USB
# bridge), NOT Wi-Fi. Plug the XIAO C6 bridge into this Mac via USB first. The
# drone must be running the ESP-NOW firmware (ESPNOW_CONTROL_ENABLED 1) and the
# C6 must be flashed with espnow_bridge.ino.
#
# Same panel/controls as BLIMP_PANEL.command, but no telemetry and no yaw-hold
# (ESP-NOW is one-way). A browser tab opens at http://127.0.0.1:8421 — click
# "Connect", then fly. You do NOT need to join the blimp's Wi-Fi for this.
#
# NOT a standalone script — run it from a full clone of this repo: it runs
# control/blimp_server.py via ./.venv/bin/python, so it needs the repo folders
# and a .venv with cflib + pyserial. See the README "Build & run" section.
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp web panel — ESP-NOW (C6 bridge) ==="
echo "Make sure the XIAO C6 bridge is plugged into USB."
echo "Opening http://127.0.0.1:8421  (keep this window open while flying)"
echo
./.venv/bin/python control/blimp_server.py --espnow
echo
echo "Press Return to close."
read
