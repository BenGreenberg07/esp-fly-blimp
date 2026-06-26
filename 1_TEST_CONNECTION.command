#!/bin/bash
# Double-click this to test the connection to the drone (NO motors spin).
cd "$(dirname "$0")"
echo "=== ESP-FLY connection test ==="
echo "Make sure your Mac is on the drone's Wi-Fi (ESP-DRONE_..., password 12345678)."
echo
./.venv/bin/python connect_test.py
echo
echo "Press Return to close."
read
