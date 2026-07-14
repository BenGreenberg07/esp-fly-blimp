# `archive/` — earlier & superseded tools

These are kept for project history. **They are not the active flight path** — for that,
use the panels in [`control/`](../control/). Everything here predates the current
Mac-side pure-pursuit controller and the frame-profile manual panel.

## Blimp tools (superseded by `control/`)
| File | What it was |
|---|---|
| `drive_blimp.py` | first keyboard teleop (Wi-Fi/CRTP): WASD+QE, forward-ramp, anti-spin down-coupling |
| `drive_blimp_espnow.py` | keyboard teleop over the C6 ESP-NOW bridge |
| `drive_blimp_ble.py` | keyboard teleop over the (parked) BLE link |
| `fly_blimp.py` | IMU yaw-hold flyer + live param tuner |
| `blimp_mocap.py` | first autonomous mocap guidance (turn-to-face + drive-to-target), on-board-PID path |
| `blimp_control.py` | early per-motor channel test / demo / teleop |
| `motor_test.py` | per-motor channel spin test to identify wiring |
| `panel.html` | the original browser panel (pre-`control/` split) |

## Quadcopter tools (before the blimp conversion)
| File | What it was |
|---|---|
| `quad_fly.py`, `auto_flight.py`, `control_server.py`, `connect_test.py`, `flight_config.py` | stock-quad flight, autonomy, web control, link test, and config from the original ESP-Drone quadcopter build |

## Launchers
The `*.command` files are the macOS double-click launchers that paired with the tools
above (`FLY_BLIMP`, `DRIVE_BLIMP`, `MOCAP`, `TEST_BLIMP_MOTORS`, and the numbered quad
launchers `1_`–`4_`).
