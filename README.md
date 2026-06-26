# ESP-FLY Autonomous Blimp

Converting a tiny ESP32-S3 quadcopter into an **autonomous helium blimp** that flies
itself to motion-capture waypoints — with all flight control computed **on board** the
drone, tuned **live** from a browser, and commanded over a custom **ESP-NOW** radio link
so the laptop never has to leave the lab Wi-Fi.

This repo contains the full stack: the on-board C flight controller, the radio bridge
firmware, and a real-time 3D control/tuning panel.

---

## What it does

A 4-motor helium blimp (two forward, one up, one down) is tracked by an **OptiTrack**
motion-capture rig. The system flies it to a target pose autonomously:

- **turn** to face the target (rate-limited yaw),
- **drive** forward once roughly facing it,
- **hold altitude** with the up/down motors.

The key design choice: **the drone does the math itself.** The laptop only measures where
the blimp is and where it should go, and streams those numbers to the drone. The decoupled
PID controller runs on the ESP32-S3 in real time.

```
OptiTrack ──Wi-Fi──▶ MacBook ──USB──▶ XIAO ESP32-C6 ──ESP-NOW──▶ ESP32-S3 (blimp)
 (pose stream)      (3D panel +        (radio bridge)         (on-board control
                     pose forwarder)                            + motor mixing)
```

This keeps the Mac on the mocap network the whole time — control rides a separate
ESP-NOW radio path, solving the "one Wi-Fi radio can't be on two networks" problem.

---

## Why a blimp is not a quadcopter (the control idea)

A helium blimp is **buoyant** (it doesn't fight gravity) and **pendulum-stable** (the
gondola hangs low, so it self-rights). So the whole attitude/tilt PID cascade a quad needs
is thrown away. What's left is three **independent, decoupled** single-axis loops:

| Loop | Error | Actuator |
|------|-------|----------|
| **Altitude** | target height − current height | up / down motors (PID + buoyancy feed-forward) |
| **Heading** | bearing-to-target − heading | differential forward thrust (rate-limited) |
| **Forward** | distance to target | both forward motors (gated by how well it's facing) |

Because the brushed motors are **unidirectional** (can't reverse to brake), the heading
loop caps the turn *rate* — a big envelope can never wind up into a spin faster than the
limited differential authority can cancel.

See [`esp-drone/.../blimp_guidance.c`](esp-drone/components/core/crazyflie/modules/src/blimp_guidance.c)
for the controller and its full theory comment.

---

## Live tuning, no reflashing

Every gain is tunable in real time from the browser panel. Tuning values ride the same
ESP-NOW link as a dedicated frame type, so the controller updates in RAM instantly — you
never reflash the drone to tune it.

The **mocap panel** (`mocap_panel_server.py` + `mocap_panel.html`) shows a live 3D view of
the blimp (position, heading, trail, target) and grouped gain sliders, plus FLY / KILL.
It's self-contained (pure-canvas 3D, no external libraries) so it runs on a locked-down
lab network.

---

## Wi-Fi mode (manual fallback & how to revert)

ESP-NOW is the primary control link, but the drone **always also brings up its own
Wi-Fi access point** — so Wi-Fi control is available at any time, with **no reflashing
needed**. It's the simplest way to fly manually or fall back if the radio bridge isn't handy.

**Connect to the blimp's Wi-Fi:**

| Setting | Value |
|---|---|
| SSID | `ESP-DRONE_80B54EF11031` |
| Password | `12345678` |
| Control link (CRTP over UDP) | `udp://192.168.43.42:2390` |

**Then fly over Wi-Fi** — join that network and use either:
- **`BLIMP_PANEL.command`** — browser panel: manual drive + live tuning + telemetry, or
- **`DRIVE_BLIMP.command`** (or `python drive_blimp.py`) — keyboard teleop: `W/S` forward, `A/D` turn, `Q/E` up/down, `Space` kill.

**To make Wi-Fi the *only* radio** (fully disable ESP-NOW) — edit
`esp-drone/components/core/crazyflie/modules/src/system.c`:

```c
#define ESPNOW_CONTROL_ENABLED 0   // 1 = ESP-NOW (default), 0 = Wi-Fi only
#define BLE_CONTROL_ENABLED    0   // (BLE link, normally off)
```

then rebuild and flash the drone:

```bash
cd esp-drone
idf.py set-target esp32s3     # first time only
idf.py build flash            # add -p /dev/cu.usbmodemXXXX if needed
```

The active radio is chosen by the `*_CONTROL_ENABLED` defines in `system.c`; the Wi-Fi AP
is the base layer that's always present, with ESP-NOW layered on top when enabled.

> **Note:** macOS serial ports re-enumerate on reset — if a flash fails with *"No serial
> data received"* or *"Resource busy"*, unplug/replug the USB cable and retry.

---

## Repo layout

```
mocap_panel_server.py / mocap_panel.html   3D view + autonomous fly + live tuning panel
blimp_mocap.py                             headless mocap → on-board controller driver
blimp_server.py / blimp_panel.html         manual Wi-Fi flight + tuning panel
drive_blimp*.py                            manual teleop (Wi-Fi / BLE / ESP-NOW)
cf_udp_patch.py                            ESP-Drone CRTP-over-UDP framing fix for cflib
optitrack_natnet/                          NatNet client (incl. a macOS multicast fix)
espnow_bridge/espnow_bridge.ino            XIAO ESP32-C6 USB↔ESP-NOW bridge firmware
esp-drone/                                 the drone firmware (ESP-IDF), see below
*.command                                  one-click launchers (macOS)
```

### On-board firmware (the interesting parts)
Built on Espressif's [ESP-Drone](https://github.com/espressif/esp-drone) (a Crazyflie
port). Custom work lives in:
- `components/core/crazyflie/modules/src/blimp_guidance.c` — the decoupled PID guidance
- `.../power_distribution_stock.c` — blimp motor mixer + inrush slew limiter
- `.../stabilizer.c`, `modules/src/system.c` — blimp flight mode + radio selection
- `components/espnow_control/` — ESP-NOW receiver (manual + mocap + gains frames)
- `components/ble_control/` — BLE control link
- `hal/.../wifi_esp32.c` — STA-only ESP-NOW base init

---

## Hardware
- **Blimp:** Seeed XIAO ESP32-S3 micro-drone (ESP-Drone), re-propped as a 4-thruster blimp
- **Bridge:** Seeed XIAO ESP32-C6 (USB ↔ ESP-NOW)
- **Mocap:** OptiTrack / Motive (NatNet)

## Build & run
- Firmware: ESP-IDF v5.0.x, `idf.py set-target esp32s3 && idf.py build flash` in `esp-drone/`
- Bridge: Arduino IDE / `arduino-cli`, board `XIAO_ESP32C6`
- Host tools: Python 3 with `cflib`, `pyserial`, `bleak`; double-click a `.command` launcher

## License
The `esp-drone/` firmware is a derivative of Espressif ESP-Drone and remains **GPL-3.0**
(original license headers retained). The host-side tools and panels are original work by
the author.

---

*Built by Ben Greenberg. An exercise in re-purposing a flight controller, decoupled
control design, embedded radio links, and real-time tooling.*
