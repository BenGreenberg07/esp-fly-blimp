#!/bin/bash
# Double-click: launches the ESP-FLY web control panel and opens it in your browser.
# Keep this window open while you use the panel. Close it (or Ctrl-C) to stop.
cd "$(dirname "$0")"
echo "=== ESP-FLY control panel ==="
echo "Opening http://127.0.0.1:8420 in your browser..."
echo "Be on the drone's Wi-Fi. Keep THIS window open while flying."
echo
./.venv/bin/python control_server.py
