# ESP-FLY — Setup Summary

_Controlling a Seeed ESP-FLY (XIAO ESP32-S3) micro-drone from a MacBook, plus an in-progress quad→blimp firmware conversion. Updated 2026-06-18._

**Key fact:** the physical drone still runs its **stock ESP-Drone quad firmware** — it was never re-flashed. Everything used to fly it lives on the Mac and talks over Wi-Fi. The blimp firmware was written and compiled but **not flashed** (no blimp hardware yet).

## How it connects
The drone is its own **Wi-Fi access point** speaking the Crazyflie protocol (**CRTP**) over **UDP**.

| | |
|---|---|
| Wi-Fi SSID / pass | `ESP-DRONE_xxxx` / `12345678` |
| Connection URI | `udp://192.168.43.42:2390` |
| Setpoint rate | ~50 Hz (`send_setpoint(roll,pitch,yaw,thrust)`) |

macOS shows "no internet" for this network — normal, it still works (verified by ping). Join via Wi-Fi menu or `networksetup -setairportnetwork en0 <SSID> 12345678`.

**The tricky part — `cf_udp_patch.py`:** modern cflib (0.1.32) couldn't talk to ESP-Drone until three fixes, all bundled in this file (imported by every script):
1. URI needs the explicit `:2390` port (else cflib crashes).
2. **Checksum framing** — ESP-Drone appends a byte `sum(bytes)&0xFF` to every UDP packet and rejects any without it. Stock cflib doesn't add/strip it, so the link connects but silently drops every packet. The patch monkey-patches cflib's send/receive to add it on send and strip it on receive.
3. Logger import path is `cflib.crazyflie.syncLogger`.

## Mac setup
- Virtualenv at `Firmware/.venv` with **cflib + pynput** (run scripts via `./.venv/bin/python`).
- Control panel is a **local web app** (Homebrew Python lacks Tkinter).

## Files (in `Firmware/`)
| File | Purpose |
|---|---|
| `cf_udp_patch.py` | cflib↔ESP-Drone compatibility (used by all) |
| `flight_config.py/.json` | Shared trim + auto-sequence settings |
| `connect_test.py` | Read-only link test (no motors) |
| `quad_fly.py` | Manual keyboard flight (WASD move, Q/E up-down, Space=kill) |
| `auto_flight.py` | Autonomous up→hover→down (CLI args override config) |
| `control_server.py` + `panel.html` | Web control panel (`http://127.0.0.1:8420`) |
| `blimp_control.py` | Blimp-firmware only: `test`/`demo`/`keys` |
| `1`–`4_*.command` | Double-click launchers (test / fly / auto / panel) |

## Control panel (`4_CONTROL_PANEL.command`)
Connect, manual fly (hold Q/E to ramp throttle), **Take off** button, autonomous hop, live attitude + smoothed battery, **KILL** (red button or Spacebar), and trim sliders that **auto-save** to `flight_config.json`. Failsafe: throttle cut within 0.5 s if the browser stops sending. Battery is a ~2 s moving average; warnings need voltage low for 2.5 s (ignores momentary sag). Read voltage at rest: ~4.2 V full, ~3.5 V land, ~3.3 V empty.

## Blimp firmware (built, NOT flashed)
In `Firmware/esp-drone`, build target **must** be esp32s3 (`idf.py set-target esp32s3`):
- `power_distribution_stock.c` — quad mixer → blimp mixer (thrust=forward, yaw=turn, pitch=up/down) with editable motor-channel `#define`s.
- `stabilizer.c` — bypasses IMU self-leveling; maps sticks straight to motors.

Build/flash: `. ~/esp/esp-idf/export.sh && idf.py set-target esp32s3 && idf.py build && idf.py -p <port> flash`. Revert: `git checkout Firmware/esp-drone`.

## Open issue — brownout
Barely lifts, falls, then power-cycles; LEDs flicker on battery. This is **undervoltage brownout** (battery sags under motor load until the chip resets) — a **power/battery issue, not software**. Fix: charge fully; if a full pack still browns out, suspect a tired/under-rated battery or a resistive solder/connector. Don't fly/charge a swollen LiPo.
