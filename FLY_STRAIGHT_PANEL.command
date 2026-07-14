#!/bin/bash
# =====================================================================
#  BLIMP STRAIGHT PANEL — THE autonomous flight tool (+ manual fallback).
#  Opens http://127.0.0.1:8501: live 3D mocap view, GO-to-goal autonomy
#  (pure-pursuit steering, velocity-profile forward, hover w/ tilted-prop
#  lift compensation), all Mac-side over the C6 ESP-NOW bridge.
#
#  NOT a standalone script — run it from inside a full clone of this repo:
#  it runs control/straight_panel_server.py (which reads ../optitrack_natnet
#  and ../esp-drone) via ./.venv/bin/python, so it needs the repo folders AND
#  a .venv with cflib + pyserial. See the README "Build & run" section.
#
#  >>> set these from Motive, then double-click <<<
MOTIVE_IP="192.168.0.4"    # OptiTrack/Motive PC IP
BODY_ID="531"              # the blimp's Rigid Body Streaming ID
GOAL_ID="502"              # the GOAL MARKER's Rigid Body Streaming ID
UP="Z"                     # initial up axis (toggle live in the panel: Y / Z)
# =====================================================================
cd "$(dirname "$0")"
echo "=== Blimp Straight Panel (auto GO-to-goal + manual) ==="
echo "  Mac must be on the LAB Wi-Fi (for mocap)."
echo "  Plug the C6 bridge into USB before pressing ARM (drone on its LiPo)."
echo "  Opening http://127.0.0.1:8501 — keep this window open."
echo
./.venv/bin/python control/straight_panel_server.py --server "$MOTIVE_IP" --body "$BODY_ID" --goal-body "$GOAL_ID" --up "$UP"
echo
echo "Stopped. Press Return to close."
read
