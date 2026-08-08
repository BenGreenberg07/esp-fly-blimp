# Picking up this exact blimp

Notes for whoever inherits **the physical blimp and bridge I left behind** — the specific
IDs, addresses, and settings that are already flashed and tuned, plus how to get it
flying again from a cold start.

> ### Which blimp this page is about
>
> This is the **original two-motor-forward blimp** — the one that actually flew, for
> months, and the one everything in this repo is tuned for:
>
> **2 forward motors + 1 up + 1 down.** It steers by running the two forward motors at
> different speeds, and holds altitude with the up/down pair.
>
> It is **not** the 4-motor "S-blimp" (all four motors pointing up, canted ~45°) that I
> was starting to build at the end. That's a different airframe with a different
> controller, it has **never been flown**, and nothing on this page applies to it. If you
> have both on the shelf, this page is the one with **two horizontal props on the front**.
>
> Everything below assumes the standard build. If the drone was left on the experimental
> firmware, `python flash.py drone` puts it back — see
> [step 0](#0--make-sure-its-on-the-right-firmware) below.

If you're building a *new* blimp, you want the main [README](../README.md) instead.

---

## What you should have

| Item | What it is |
|---|---|
| **The blimp** | XIAO ESP32-S3 flight board in a printed gondola, hanging under a mylar envelope. Four brushed motors in the standard layout: **2 facing forward** (side by side, canted up ~15°), **1 facing up**, **1 facing down**. |
| **The bridge** | The one I used is a XIAO **ESP32-C6** on a short USB cable. **It has a u.FL external antenna — keep it attached.** The firmware drives the antenna switch high, so running it without the antenna is *worse* than a stock board, not neutral. (If you build a replacement, an ESP32-S3 works too — `python flash.py bridge`.) |
| **A 1S LiPo** | Charge it. A tired battery is the single most common cause of "it flies badly today" — see the altitude note below. |

Both boards are already flashed and already paired. **You should not need to flash
anything to fly.**

---

## The numbers

### Motion capture (OptiTrack / Motive)

| Setting | Value |
|---|---|
| Motive PC IP | `192.168.0.4` |
| Blimp rigid-body Streaming ID | **531** |
| Up axis | **Z** (Motive is Z-up here) |
| Typical stream rate | ~200 Hz |

These are exactly what's in `config.json` at the repo root, so a fresh clone is already
pointed at the right place. If Motive got reinstalled or the rigid body was rebuilt, the
Streaming ID is the thing that changes — check it in Motive and update `config.json`.

**Your laptop must be on the mocap network** for the pose stream to arrive. Control does
*not* use that network (it goes over ESP-NOW), so this is the only reason to be on it.

### Radio

| Setting | Value |
|---|---|
| ESP-NOW channel | **1** (both boards) |
| `FRAME_MAGIC` (laptop → drone pairing tag) | `{0xB1, 0x12, 0x9F, 0x5A}` |
| `TELEM_MAGIC` (drone → laptop telemetry) | `{0xB7, 0x1E, 0x30, 0xA5}` |
| Drone STA MAC | `80:B5:4E:F1:10:30` |
| Bridge base MAC | `f0:f5:bd:2d:0c:d8` |

There's no MAC to configure anywhere — the bridge broadcasts, and the two boards accept
each other because their `FRAME_MAGIC` matches. The MACs are listed only so you can
identify the boards if you have several.

> The magic tag isn't decoration. The lab is full of other ESP-NOW traffic, and before
> this filter existed foreign packets were being parsed as gain updates and crashing the
> drone a few seconds after power-up. If you ever see unexplained reboots or jitter,
> suspect a second transmitter before you suspect the tuning.

### Wi-Fi (fallback only)

The drone also brings up its own access point, which you don't need for normal flying:
SSID `ESP-DRONE_80B54EF11031`, password `12345678`, CRTP at `udp://192.168.43.42:2390`.

### How this blimp flies (worth 30 seconds before you touch it)

It is **not** a quadcopter and it does not behave like one:

- **Turning is differential.** There is no yaw thruster — it turns by running one forward
  motor harder than the other. So **every turn also pushes it forward**; it cannot spin
  on the spot.
- **It cannot brake.** The brushed motors only spin one direction, so it coasts. It
  arrives at a point by curving onto it, not by stopping on it.
- **Altitude is a separate pair** (the up and down motors) and is handled by its own PID,
  independent of everything else.
- **It's pendulum-stable.** The gondola hangs low, so it self-rights. There is no
  attitude/tilt loop to tune — don't go looking for one.

Minimum turn radius is about **1–1.25 m**, and radius grows with speed. That's the single
most useful fact about flying it.

### Motors

Channel → role, as flashed:

```
M1 = DOWN      M2 = forward-LEFT      M3 = forward-RIGHT      M4 = UP
```

**Verify this before trusting it** — see the open issue at the bottom of this page.

### Tuning

`control/mocap_config.json` holds the tuning that was actually flying, under the profile
named **`tilted`** (the forward props on this airframe are canted up ~15°, so forward
thrust also lifts a little — that's what the profile name means).

The values that matter most:

| Gain | Value | What it does |
|---|---|---|
| `fwdMaxN` | 0.18 | constant forward cruise. **The main lever.** Raise = faster but wider turns |
| `kpHead` / `kdHead` | 2.65 / 0.037 | pursuit steering PD; `kd` is what stops it overshooting the heading |
| `turnCap` / `turnBoost` | 0.9 / 0.35 | turn authority, and how symmetric the differential is |
| `driftK` | 1.3 | crabs the nose into the sideways drift |
| `zff` / `vertMaxPwm` | 19000 / 28000 | buoyancy feedforward and its ceiling |
| `turnMaxPwm` | **−32767** | **negative on purpose.** This board's turn direction is flipped; the sign is what corrects it |

If it ever turns the wrong way, don't rewire — press **⟳ Flip turn direction** in the
panel, which just flips that sign back.

---

## Getting it flying again, from cold

### 0 — Make sure it's on the right firmware

If nobody has touched it since I left, skip this; it's already flashed and paired. But if
you're unsure — or someone tried the 4-motor experiment on it — put the standard build
back before doing anything else:

```bash
python flash.py drone
```

That forces the `BLIMP_SWING` flag off and restores the normal two-forward-motor mixer
and the on-board guidance. It is always safe to run.

### Then, every time

1. **Charge the LiPo.** Really.
2. **Check the helium.** The envelope loses lift over days. It should be *very slightly*
   heavy — sinking slowly when released. Too light is much harder to fly than too heavy.
   Trim with tape.
3. **Plug the bridge into your laptop** (antenna attached), and **power the blimp**.
4. **Join the mocap network** and confirm Motive is streaming, with the blimp's rigid
   body present and tracking.
5. **Prove the radio link before opening any panel:**
   ```bash
   python control/check_link.py
   ```
   This opens the bridge and waits for the drone's own telemetry frames. It passing means
   the whole chain works. If it fails, nothing in the browser is going to help you.
6. **Hand-fly it first:**
   ```bash
   python run.py --mode manual
   ```
   ARM, then W/S forward, A/D turn, Q/E up/down, Space to kill. Confirm up/down goes the
   right way and turning feels like a turn.
7. **Then go autonomous:**
   ```bash
   python run.py --mode auto
   ```
   ARM, check the tracking pill is green, and try the come-back-when-pushed setup from
   the main README (Path mode, one waypoint, GO).

---

## Things I learned the hard way

**Flashing the S3 fails a lot, and it's never broken.** If `idf.py flash` says *"No
serial data received"* or *"Invalid head of packet"*, use `--before usb_reset`, or unplug
the LiPo and replug the USB cable and retry immediately. esptool fails before it writes,
so a failed flash never damages what's already on the board. The BOOT/RESET buttons are
unreachable once the gondola is assembled, which is why this matters.

**The USB adapter shows one port at a time.** Flash the drone and the bridge one after
the other, not together. The port name flips between `usbmodem101` and `usbmodem1101` on
re-enumeration — that's normal.

**`arduino-cli upload` alone flashes a stale binary.** Always `compile --upload` together
when changing the bridge sketch. This cost me a whole lab session once: the bridge was
silently still running the previous sketch and nothing downstream made sense.

**If it sinks through the flight**, check the battery under load and the helium before
touching gains. There was a real bug once where `zff` had drifted above `vertMaxPwm`, so
the altitude PID's corrections were being clipped away entirely — that's fixed in the
committed config, but it's worth knowing the failure mode: the vertical motor pinned near
max the whole flight while the craft still sank.

**Wider circles than you drew is a speed problem**, not a steering-gain problem. Turn
radius scales with cruise speed. Lower `fwdMaxN` first.

**Don't fly two blimps on the same `FRAME_MAGIC`.** Give the second triplet its own
4-byte value in both `espnow_bridge.ino` and `espnow_control.c`, and reflash both of its
boards.

---

## Known open issue — check this first

The last time I bench-tested motor channels, **pressing the M3 test button appeared to
drive both forward motors**, and M4 (up) didn't spin during a flight test even though it
spun when the battery was first connected — which means the motor itself is fine and the
problem is upstream of the motor.

I never fully closed this out. It's most likely a channel-mapping mismatch rather than
damaged hardware, so **before you trust the map above, verify it empirically**:

```bash
python control/check_link.py --spin      # PROPS OFF
```

This spins each channel briefly at low power. Watch which physical motor moves for each
channel, and if it disagrees with `M1=DOWN, M2=fwd-LEFT, M3=fwd-RIGHT, M4=UP`, fix it in
firmware by editing the `MOTOR_FWD_LEFT / MOTOR_FWD_RIGHT / MOTOR_UP / MOTOR_DOWN`
defines in `power_distribution_stock.c` and reflashing. **Don't resolder** — the mapping
is deliberately a software choice so wiring mistakes stay cheap.

---

## If you do need to reflash

| Command | What it gives you |
|---|---|
| `python flash.py drone` | the standard 2-forward-motor blimp firmware — **this is the one you want, and it's what this blimp should always be running** |
| `python flash.py bridge --board c6` | reflashes **this** C6 radio bridge (use `--board s3` for an S3 one) |
| `python flash.py swing` | switches the drone to the experimental 4-motor S-blimp build. **Do not run this on this vehicle** — the mixer sends motor commands that assume four upward motors, so a 2-forward airframe will not fly. Recover with `python flash.py drone`. |
