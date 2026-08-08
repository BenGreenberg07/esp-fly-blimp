# Picking up this exact blimp

Notes for whoever inherits **the physical blimp and bridge I left behind** — the specific
IDs, addresses, and settings that are already flashed and tuned, plus how to get it
flying again from a cold start.

If you're building a *new* blimp, you want the main [README](../README.md) instead. This
page assumes the hardware on the shelf is the hardware I flew.

---

## What you should have

| Item | What it is |
|---|---|
| **The blimp** | XIAO ESP32-S3 flight board in a printed gondola, hanging under a mylar envelope. Four brushed motors: 2 forward, 1 up, 1 down. |
| **The bridge** | XIAO ESP32-C6 on a short USB cable. **It has a u.FL external antenna — keep it attached.** The firmware drives the antenna switch high, so running it without the antenna is *worse* than a stock board, not neutral. |
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
| Goal-marker Streaming ID (optional) | **502** |
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
| `python flash.py drone` | the normal flight firmware — **this is the one you want** |
| `python flash.py bridge` | reflashes the C6 radio bridge |
| `python flash.py swing` | switches the drone to the 4-motor S-blimp build (different airframe — don't use it on this vehicle) |
| `python flash.py led-test` | an LED bench test that **skips all flight code** |

One trap worth repeating: the LED test build has no flight code at all, so if it's left
on the board the motors simply never spin and it looks like dead hardware. `flash.py`
forces that flag off for every flight build, so `python flash.py drone` always gets you
back to something flyable.
