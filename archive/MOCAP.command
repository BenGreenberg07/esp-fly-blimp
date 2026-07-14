#!/bin/bash
# Double-click: autonomous blimp control from OptiTrack mocap.
# First time only: open blimp_mocap.py and set MOTIVE_IP and BLIMP_BODY_ID near the
# top (find them in Motive > Edit > Settings > Streaming, and the rigid body's
# Streaming ID). Your Mac's IP is auto-detected. After that, just double-click this.
#
# To test the control loop OFFLINE (no drone/mocap), run with --sim instead.
cd "$(dirname "$0")"
echo "=== ESP-FLY blimp — OptiTrack mocap autonomy ==="
echo "(edit MOTIVE_IP / BLIMP_BODY_ID in blimp_mocap.py once if you haven't)"
echo "Ctrl-C to stop."
echo
./.venv/bin/python blimp_mocap.py "$@"
echo
echo "Press Return to close."
read
