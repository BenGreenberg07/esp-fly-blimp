#!/bin/bash
# Double-click: test each motor of the blimp (and check if the board is alive).
# Be on the drone's Wi-Fi (ESP-DRONE_xxxx, password 12345678). Props off / frame held.
cd "$(dirname "$0")"
echo "=== ESP-FLY motor test / board health check ==="
echo "Remove props or hold the frame down before spinning."
echo
./.venv/bin/python motor_test.py
echo
echo "Press Return to close."
read
