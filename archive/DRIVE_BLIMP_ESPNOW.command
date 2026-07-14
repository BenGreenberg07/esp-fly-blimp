#!/bin/bash
# Double-click: fly the blimp MANUALLY over ESP-NOW (via the C6 bridge).
# Plug the C6 bridge into the Mac (USB). Power the drone (battery/USB) — it does
# NOT need to be on the Mac. Mac Wi-Fi can be anywhere; control goes over ESP-NOW.
#   W=forward(hold)  S=slow  A/D=turn  Q=up  E=down  Space=KILL  Esc=quit
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp — manual ESP-NOW control ==="
echo "Bridge must be plugged into this Mac. Drone powered & nearby."
echo
./.venv/bin/python drive_blimp_espnow.py "$@"
echo
echo "Press Return to close."
read
