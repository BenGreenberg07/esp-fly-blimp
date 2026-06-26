#!/bin/bash
# =====================================================================
#  BLIMP MOCAP PANEL — live 3D view of the blimp + autonomous flight.
#  Opens a browser at http://127.0.0.1:8500 showing where the blimp is
#  in 3D (from OptiTrack), with a FLY button that streams pose+target to
#  the drone over the C6 ESP-NOW bridge (the drone computes the flight).
#
#  >>> set these from Motive, then double-click <<<
MOTIVE_IP="192.168.0.4"    # OptiTrack/Motive PC IP
BODY_ID="531"              # the blimp's Rigid Body Streaming ID
UP="Z"                     # initial up axis (toggle live in the panel: Y / Z)
# =====================================================================
cd "$(dirname "$0")"
echo "=== Blimp Mocap Panel (3D view + autonomous fly) ==="
echo "  Mac must be on the LAB Wi-Fi (for mocap)."
echo "  Plug the C6 bridge into USB before pressing FLY (drone on its LiPo)."
echo "  Opening http://127.0.0.1:8500 — keep this window open."
echo
./.venv/bin/python mocap_panel_server.py --server "$MOTIVE_IP" --body "$BODY_ID" --up "$UP"
echo
echo "Stopped. Press Return to close."
read
