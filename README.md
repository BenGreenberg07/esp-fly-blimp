# ESP-FLY Autonomous Blimp

Converting a tiny ESP32-S3 quadcopter into an **autonomous helium blimp** that flies
itself to motion-capture waypoints — hand-flyable and self-flying from a live browser
panel, commanded over a custom **ESP-NOW** radio link so the laptop never has to leave
the lab Wi-Fi.

This repo contains the full stack: the drone firmware, the USB↔radio bridge firmware,
and the real-time 3D control/tuning panels.

```
OptiTrack ──Wi-Fi──▶ MacBook ──USB──▶ XIAO ESP32-C6 ──ESP-NOW──▶ ESP32-S3 (blimp)
 (pose stream)      (control panel +     (radio bridge)          (motor mixing)
                     guidance loop)
```

The Mac stays on the mocap network the whole time — control rides a **separate ESP-NOW
radio path**, solving the "one Wi-Fi radio can't be on two networks at once" problem.

---

## The one idea that shapes everything: it's a *forward-only* vehicle

A helium blimp is **buoyant** (it doesn't fight gravity) and **pendulum-stable** (the
gondola hangs low, so it self-rights) — so the entire attitude/tilt PID cascade a quad
needs is thrown away. But the interesting constraint is the drivetrain:

- **Two forward motors, one up, one down.** Steering is the *difference* between the two
  forward motors — there is no dedicated yaw thruster.
- **The brushed motors are unidirectional** — they can't reverse. So the craft **cannot
  brake** and **cannot rotate in place**: any turn command also pushes it forward.

Every control decision here follows from that. You don't fight the coupling — you design
around it.

---

## Two ways to fly it

### 1. Manual command panel — `BLIMP_PANEL_ESPNOW.command`
`blimp_server.py` + `blimp_panel.html`. Hand-fly from the browser (`W/S` forward,
`A/D` turn, `Q/E` up/down, `Space` = kill) with everything tunable live:

- **Constant-forward + differential turn.** Forward thrust is held steady; turning only
  changes the *difference* between the motors. The turn is auto-clamped so it can never
  drive one motor to zero and let the other run away — a clean differential, not a lurch.
- **Altitude hold** — one toggle streams a constant vertical thrust you dial in by feel
  (open-loop buoyancy trim), so you can set height once and forget it.
- **Counter-spin** — a tap sends a short opposite-direction pulse to kill leftover
  rotation (the blimp has almost no natural yaw damping, so it coasts after you let go).
- **Three frame profiles** — `tilted-up`, `tilted-down`, `straight` front-motor mounts,
  each storing its own complete tuning set. Swap the physical frame, click the profile,
  and every slider loads that frame's values. **Auto-saves** on every change.

### 2. Autonomous panel — `FLY_STRAIGHT_PANEL.command`
`straight_panel_server.py` + `straight_panel.html`. A live 3D view (blimp, heading, trail,
target) that flies the blimp to a **goal marker** — a second tracked rigid body it chases
live (move the marker, the target follows) — or to a manual X/Y point.

The guidance is **pure pursuit**, and this is the whole trick: because a forward-only
blimp can't rotate in place, it doesn't try to. Instead it **always moves and steers
toward the target, so the *path* curves onto the point** — a trajectory, not a
rotate-then-drive.

```
steer   = kp · heading_error  −  kd · yaw_rate          (differential)
forward = velocity_profile(range) · align_gate(heading_error)
```

- The **`kd · yaw_rate`** term is anticipatory counter-steer: as rotation builds it eases
  and reverses the turn *before* overshooting — the continuous form of the hand-flying
  "counter-spin at the halfway mark" technique.
- The **align gate** slows forward when badly mis-pointed (tight, near-pivoting arc) and
  opens up when facing the target (drives straight in).
- A **velocity profile** decelerates into the point so it coasts to a stop instead of
  overshooting (no reverse brake available).
- **Altitude hover** holds the height captured at launch, with a term that subtracts the
  extra lift the tilted forward props add while driving.

All control is computed **on the laptop** and streamed to the drone as ordinary manual
setpoints, so nothing here needs reflashing — you tune the whole autonomy live.

> The firmware also carries an earlier **on-board** decoupled-PID guidance controller
> (`blimp_guidance.c`) reachable over ESP-NOW; it's kept for reference but the Mac-side
> pursuit controller above is the active, tuned path.

---

## Live tuning, no reflashing

Every gain — manual powers/trims and autonomous pursuit/hover gains — is a live slider.
Values ride the ESP-NOW link (or Wi-Fi) and update the controller in RAM instantly. Both
panels are **self-contained** (pure-canvas 3D, no external libraries / CDNs) so they run
on a locked-down lab network, and both persist their tuning to disk automatically.

---

## Wi-Fi mode (manual fallback & how to revert)

ESP-NOW is the primary control link, but the drone **also brings up its own Wi-Fi access
point**, so Wi-Fi control is available at any time with **no reflashing**.

| Setting | Value |
|---|---|
| SSID | `ESP-DRONE_xxxxxxxxxxxx` |
| Password | `12345678` |
| Control link (CRTP over UDP) | `udp://192.168.43.42:2390` |

Join that network and use **`BLIMP_PANEL.command`** (browser panel over Wi-Fi). Older
keyboard-teleop clients (`drive_blimp*.py`) are kept in `archive/`.

**To make Wi-Fi the *only* radio**, edit
`esp-drone/components/core/crazyflie/modules/src/system.c`:

```c
#define ESPNOW_CONTROL_ENABLED 0   // 1 = ESP-NOW (default), 0 = Wi-Fi only
#define BLE_CONTROL_ENABLED    0   // (BLE link, normally off)
```

then rebuild and flash:

```bash
cd esp-drone
idf.py set-target esp32s3     # first time only
idf.py build flash            # add -p /dev/cu.usbmodemXXXX if needed
```

> **Note:** macOS serial ports re-enumerate on reset — if a flash fails with *"No serial
> data received"* or *"Resource busy"*, unplug/replug the USB cable and retry.

---

## Repo layout

The code is grouped by role. The three `*.command` files at the root are the
double-click launchers (macOS); everything they run lives in `control/`.

```
FLY_STRAIGHT_PANEL.command     ▶ launch autonomous flight (pursuit) — the main tool
BLIMP_PANEL_ESPNOW.command     ▶ launch manual flight over the C6 bridge
BLIMP_PANEL.command            ▶ launch manual flight over the drone's own Wi-Fi (fallback)

control/     Mac-side ground station — the live flight software (see control/README.md)
  ├─ straight_panel_server.py / .html   autonomous: mocap 3D view + go-to-goal pursuit + tuning
  ├─ blimp_server.py / blimp_panel.html manual: frame profiles, alt-hold, counter-spin
  └─ cf_udp_patch.py                     ESP-Drone CRTP-over-UDP framing fix for cflib (Wi-Fi path)

esp-drone/           on-board drone firmware (ESP-IDF, ESP-Drone derivative) — see below
espnow_bridge/       XIAO ESP32-C6 USB↔ESP-NOW bridge firmware (Arduino sketch)
optitrack_natnet/    OptiTrack NatNet client library (incl. a macOS multicast fix)

docs/        design notes and write-ups
archive/     earlier / superseded tools kept for history (see archive/README.md)
```

### On-board firmware (the interesting parts)
Built on Espressif's [ESP-Drone](https://github.com/espressif/esp-drone) (a Crazyflie
port). Custom work lives in:
- `.../power_distribution_stock.c` — blimp motor mixer (constant-forward + differential
  turn, vertical sign/dead-band) and motor inrush slew limiter
- `.../stabilizer.c`, `.../system.c` — blimp flight mode + radio-link selection
- `components/espnow_control/` — ESP-NOW receiver (manual + mocap + gains frames, with a
  link-loss failsafe)
- `.../blimp_guidance.c` — the reference on-board decoupled-PID guidance controller
- `components/ble_control/` — BLE control link (parked)

---

## Hardware
- **Blimp:** Seeed XIAO ESP32-S3 micro-drone (ESP-Drone), re-propped as a 4-thruster blimp
- **Bridge:** Seeed XIAO ESP32-C6 (USB ↔ ESP-NOW)
- **Mocap:** OptiTrack / Motive (NatNet)

## Build & run
- **Firmware:** ESP-IDF v5.0.x — `idf.py set-target esp32s3 && idf.py build flash` in `esp-drone/`
- **Bridge:** Arduino IDE / `arduino-cli`, board `XIAO_ESP32C6` (esp32 core 3.x)
- **Host tools:** Python 3 with `cflib` and `pyserial`.

### The `.command` launchers
The `*.command` files are **macOS double-click shortcuts, not standalone scripts** — each
one just `cd`s into the repo and runs the matching panel in `control/`. They only work
from inside a full clone of this repo, because they depend on:

1. **the repo layout** — a launcher calls e.g. `control/straight_panel_server.py`, which in
   turn imports `cf_udp_patch.py` and reads `../optitrack_natnet/` and `../esp-drone/`, so
   the folders must be in place next to it;
2. **a Python virtual-env at `.venv/`** in the repo root with `cflib` + `pyserial`
   installed (the launcher runs `./.venv/bin/python …`);
3. **the hardware/network** — the C6 bridge on USB, the drone powered, and (for the
   autonomous panel) the Mac on the lab Wi-Fi so OptiTrack can stream pose.

So to use one: clone the repo, create `.venv` and `pip install cflib pyserial`, open the
launcher and set your Motive IP + rigid-body IDs at the top, then double-click it.

```bash
git clone https://github.com/BenGreenberg07/esp-fly-blimp.git
cd esp-fly-blimp
python3 -m venv .venv && ./.venv/bin/pip install cflib pyserial
# edit MOTIVE_IP / BODY_ID / GOAL_ID at the top of FLY_STRAIGHT_PANEL.command, then run it
```

## License
The `esp-drone/` firmware is a derivative of Espressif ESP-Drone and remains **GPL-3.0**
(original license headers retained). The host-side tools and panels are original work by
the author.

---

*Built by Ben Greenberg. An exercise in re-purposing a flight controller, control design
around a hard actuation constraint, embedded radio links, and real-time tooling.*
