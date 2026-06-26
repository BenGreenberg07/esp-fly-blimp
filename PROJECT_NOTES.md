# ESP-FLY — Project Notes & Documentation

_Last updated: 2026-06-18_

This documents everything set up to control a Seeed **ESP-FLY** (XIAO ESP32-S3
micro-drone) from a MacBook, plus the in-progress firmware changes to convert
it from a quadcopter to a blimp.

---

## 0. TL;DR — current state

- **The physical drone's firmware was NOT modified.** It still runs the **stock
  ESP-Drone quad firmware** it shipped with. Everything we use to fly it lives
  on the Mac and talks to the drone over Wi-Fi.
- **Blimp firmware changes exist only in source + a local build** (`build/ESPDrone.bin`).
  They have **never been flashed** (no blimp hardware on hand yet).
- Manual + autonomous flight and a web control panel all work over Wi-Fi.
- Open bug: drone browns out / power-cycles under throttle → diagnosed as a
  **battery/power issue**, not software (see §8).

---

## 1. Hardware

| Item | Detail |
|---|---|
| Flight controller | Seeed XIAO **ESP32-S3** (dual-core, Wi-Fi) |
| IMU | MPU-6050 (6-axis) |
| Motors | 4 × 615 coreless brushed, via N-MOSFET drivers |
| Battery | 1S LiPo 3.7 V, 250 mAh |
| Firmware base | Espressif **ESP-Drone** (a Crazyflie port) |
| Programming | USB-C (also charges the LiPo, ~100 mA) |

---

## 2. How the Mac connects to the drone

The drone runs a **Wi-Fi Access Point**; no router involved. It speaks the
Crazyflie protocol (**CRTP**) over **UDP**.

| Parameter | Value |
|---|---|
| Wi-Fi SSID | `ESP-DRONE_xxxxxxxxxxxx` (e.g. `ESP-DRONE_90706910AB79`) |
| Wi-Fi password | `12345678` |
| Drone IP (gateway) | `192.168.43.42` |
| CRTP UDP port | **2390** |
| Connection URI | `udp://192.168.43.42:2390` |

macOS reports "no internet / not associated" for this network — that's normal
(the drone has no uplink); the link still works. Join it with the Wi-Fi menu, or:
```
networksetup -setairportnetwork en0 ESP-DRONE_90706910AB79 12345678
```

### Three gotchas that had to be fixed for cflib 0.1.32 to talk to ESP-Drone
All handled in **`cf_udp_patch.py`** (imported by every script):
1. **URI needs the explicit port** `:2390` (newer cflib leaves it `None` otherwise).
2. **Checksum framing.** ESP-Drone wraps every UDP packet with a trailing
   checksum byte = `sum(all prior bytes) & 0xFF`. Stock cflib neither adds it on
   send nor strips it on receive, so the link "connects" but every packet is
   silently dropped. The patch appends it on send and strips it on receive.
3. **Import path** is `cflib.crazyflie.syncLogger` (capital L).

---

## 3. Mac software setup (one-time, already done)

A Python virtualenv lives at `Firmware/.venv` with the needed packages:
```
python3 -m venv .venv
./.venv/bin/pip install cflib pynput        # cflib 0.1.32
```
All scripts are run with `./.venv/bin/python <script>` (the `.command` launchers
do this for you).

Note: Homebrew's Python 3.14 has **no Tkinter**, which is why the control panel
is a local **web app** instead of a native window.

---

## 4. Files created (all in `Firmware/`)

| File | Purpose |
|---|---|
| `cf_udp_patch.py` | Makes cflib's UDP driver ESP-Drone-compatible (see §2). Imported by all. |
| `flight_config.py` / `flight_config.json` | Shared settings: trim + auto-sequence params. All tools read/write it. |
| `connect_test.py` | **Read-only** link test — no motors. Confirms link, params, battery, IMU stream. |
| `quad_fly.py` | Manual keyboard flight (stock quad). WASD move, Q/E up-down, arrows yaw, Space kill. |
| `auto_flight.py` | Autonomous hop: up → hover → down. Args override config (`--hover-time`, etc.). |
| `control_server.py` | Backend for the web control panel; holds one cflib link, streams setpoints at 50 Hz. |
| `panel.html` | The web UI (served by `control_server.py` at `http://127.0.0.1:8420`). |
| `blimp_control.py` | For the BLIMP firmware only: `test` (ID motors), `demo`, `keys`. |
| `1_TEST_CONNECTION.command` | Double-click → runs `connect_test.py`. |
| `2_FLY_QUAD.command` | Double-click → manual flight. |
| `3_AUTO_FLIGHT.command` | Double-click → autonomous hop. |
| `4_CONTROL_PANEL.command` | Double-click → launches the web control panel + opens browser. |

---

## 5. The control panel (`4_CONTROL_PANEL.command`)

A local web page that drives everything:
- **Connect / Disconnect**, link status, live **battery** + **attitude** telemetry.
- **Manual flight:** enable keyboard control, then **WASD** = move,
  **Q** = up / **E** = down (hold to ramp continuously), **←/→** = yaw.
- **Take off** button: ramps throttle smoothly up to the hover-thrust setting.
- **Autonomous hop** button: up → hover → down using current settings.
- **Tuning & settings:** trim sliders + thrust/time inputs. **Auto-saves** the
  moment you change them (applied live AND written to `flight_config.json`).
- **KILL** (big red, or **Spacebar**): cuts motors instantly, always available.
- Failsafe: if the browser stops sending (tab loses focus), throttle is cut
  within 0.5 s.

### Battery readout
Reads firmware `pm.vbat`. It is **smoothed** (≈2 s moving average) and warnings
require the voltage to stay low for **2.5 s** (so a momentary sag under throttle
shows a calm "under load" note, not a false "charge now"). Voltage is
ballpark-accurate (±~0.1 V); the "%" is a rough linear estimate. Read it at rest,
on battery (not while charging on USB). Rule of thumb: ~4.2 V full, ~3.5 V land,
~3.3 V empty.

---

## 6. Trim (drift correction)

`roll_trim` / `pitch_trim` (degrees) are added to every setpoint by all flight
tools. Tune them with the panel's sliders (auto-saved). If it drifts **right**
going up → roll trim **negative**; drifts **back** → pitch trim **positive**.

---

## 7. Blimp firmware changes (BUILT, NOT YET FLASHED)

Target: convert the quad to a blimp (2 forward motors, 1 up, 1 down). Source is
in `Firmware/esp-drone`. Build target **must** be esp32s3
(`idf.py set-target esp32s3` — the committed sdkconfig wrongly defaulted to esp32).

Two files changed:
- **`components/core/crazyflie/modules/src/power_distribution_stock.c`** —
  replaced the quad mixer with a blimp mixer. `thrust`→forward, `yaw`→turn
  (differential), `pitch`→vertical (+climb=up motor / −descend=down motor).
  Motor roles map to channels via 4 editable `#define`s (`MOTOR_FWD_LEFT` etc.).
- **`components/core/crazyflie/modules/src/stabilizer.c`** — bypasses the IMU
  attitude PID and maps the raw operator setpoint straight to the motors (a
  blimp is pendulum-stable from buoyancy and doesn't need self-leveling).

Build (ESP-IDF v5.0.7 at `~/esp/esp-idf`):
```
cd ~/Co-Create_ESP-FLY/Firmware/esp-drone
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3      # once
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash
```
Revert to stock source: `git checkout Firmware/esp-drone` (then set-target again).

To map blimp motors after flashing: run `blimp_control.py test` (spins each
channel one at a time, props off), note which physical motor moves, set the four
`#define`s, rebuild/flash.

---

## 8. Known issue (open) — brownout under load

Symptom: barely lifts, falls, then **power-cycles**; white LEDs flicker on
battery but not on USB. Diagnosis: **undervoltage brownout** — the 1S sags under
motor current until the ESP32 resets. This is a **power/battery issue, not
software**. Actions: charge fully (~4.2 V) and retest; if a full pack still
browns out, suspect a tired/under-rated battery or resistive solder/connector.
Don't keep cycling or charging a swollen LiPo.

---

## 9. Appendix: connection internals (the "fancy stuff")

This is the deeper detail of how the Mac actually drives the drone, and the
problems that had to be solved to make a modern `cflib` talk to ESP-Drone.

### The protocol stack
```
  your script / browser
        |  cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        v
  cflib  (Crazyflie Python lib) ── builds a CRTP packet
        |
        v
  CRTP over UDP  ── one UDP datagram per packet, to 192.168.43.42:2390
        |   (+ our checksum patch)
        v
  Wi-Fi SoftAP on the drone (ESP32-S3)
        |
        v
  ESP-Drone firmware: UDP task -> CRTP router -> commander -> stabilizer
        |
        v
  motor mixer (power_distribution) -> MOSFETs -> motors
```

**CRTP** (Crazy RealTime Protocol) is Bitcraze's tiny packet format. Each packet
is one header byte + up to 30 data bytes. The header encodes a **port**
(which subsystem: commander, logging, params, …) and a **channel**. We mostly
use the **commander port (0x03)**: a setpoint packet is
`<float roll><float pitch><float yaw><uint16 thrust>` (little-endian), which is
exactly what `send_setpoint(roll, pitch, yaw, thrust)` packs and ships ~50×/sec.

### Transport: CRTP-over-UDP
Normally a Crazyflie talks over a USB radio dongle. ESP-Drone instead carries
the *same* CRTP packets inside **UDP datagrams** over its Wi-Fi AP. cflib has a
`udp` link driver for this, selected by the `udp://host:port` URI. One datagram
= one CRTP packet.

### Why stock cflib couldn't talk to it (and the fix)
Reading the two sides side by side revealed the mismatch:

- **Firmware** (`esp-drone/components/drivers/general/wifi/wifi_esp32.c`): every
  UDP packet is `[CRTP header][data...][checksum]`, where
  `checksum = (sum of all preceding bytes) & 0xFF`. On receive it validates that
  byte and drops the packet if it doesn't match; on send it appends one.
- **cflib 0.1.32** (`cflib/crtp/udpdriver.py`): `send_packet()` sends
  `[header][data...]` with **no checksum**, and the receive thread treats the
  whole datagram as CRTP (it doesn't strip a checksum). So: socket opens fine,
  handshake looks like it starts, but the drone rejects 100% of packets
  (`udp packet cksum unmatched`) and nothing happens — a silent dead link.

The fix lives in **`cf_udp_patch.py`**. Rather than fork cflib, it
**monkey-patches** two methods at runtime (just `import cf_udp_patch` before
`init_drivers()`):
- `UdpDriver.send_packet` → rebuilds the byte tuple, appends `sum(...) & 0xFF`.
- `_UdpReceiveThread.run` → strips the trailing checksum byte before handing the
  packet to cflib.

It also documents the other two snags: the URI **must** include `:2390` (newer
cflib leaves the port `None` and crashes in `socket.connect`), and the logger
import moved to `cflib.crazyflie.syncLogger` (capital L).

### Telemetry (drone → Mac)
On connect, the panel/`connect_test.py` ask the firmware for its **TOC** (table
of contents) of log + param variables, then subscribe to a `LogConfig`
(`stabilizer.roll/pitch/yaw`, `pm.vbat`) at 5 Hz. The drone streams those back as
CRTP log packets, which is how the panel shows live attitude and battery.

### Arming & the safety watchdog
- ESP-Drone auto-arms (`ARM_INIT = true`); there's also a `system.forceArm`
  param. The motor-ID `test` mode uses the `motorPowerSet` param override (drives
  motors directly, bypassing the mixer — props off).
- The firmware has a **commander watchdog**: if it stops receiving setpoints for
  a short time it cuts the motors. That's why the scripts resend at ~50 Hz, and
  why the panel cuts throttle within 0.5 s if the browser stops sending.

### Joining the AP from the command line (and the macOS quirk)
```
networksetup -setairportnetwork en0 ESP-DRONE_90706910AB79 12345678
```
macOS shows the network as "not associated / no internet" because the drone has
no uplink — but it *is* joined. Proof during setup: `ping 192.168.43.42`
returned 0% packet loss even while the menu bar looked disconnected. Don't trust
the menu bar here; trust the ping / the link status in the panel.

### The local web-app architecture (control panel)
Because Homebrew Python lacks Tkinter, the panel is a tiny **local HTTP server**
(`control_server.py`, stdlib `http.server`) that:
- holds the **single** cflib link in a background thread and streams setpoints at
  50 Hz from a shared, lock-guarded state dict;
- serves `panel.html` and a small JSON API (`/state`, `/api`) on
  `127.0.0.1:8420`;
- runs the autonomous sequence on a server-side timeline so it can't be
  interrupted by browser hiccups, with the 0.5 s manual failsafe above.
The browser is just a thin client (keyboard capture + fetch); all the real-time
flying happens in the Python process.

---

## 10. What's done vs. not

**Done:** Mac↔drone link (with the cflib patch), manual flight, autonomous hop,
web control panel, trim + persistence, battery smoothing, blimp firmware written
and building.

**Not done:** flashing blimp firmware (no blimp hardware yet), resolving the
brownout (charging/battery test pending), tuning real-world hover/trim values.
