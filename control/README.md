# `control/` — Mac-side ground station

All flight software runs **here on the laptop**. The drone only ever receives plain
motor setpoints, so every control law and gain in this folder is tuned **live, with no
reflashing**. Launch these with the `*.command` files in the repo root.

```
OptiTrack ──Wi-Fi──▶ MacBook (this code) ──USB──▶ XIAO ESP32-C6 ──ESP-NOW──▶ blimp
```

## Files

### `straight_panel_server.py` + `straight_panel.html` — autonomous flight (the main tool)
Launched by `FLY_STRAIGHT_PANEL.command` (port 8501). Listens to two OptiTrack rigid
bodies — the **blimp** and a **goal marker** — and flies the blimp to wherever the marker
is (move the marker mid-flight and the target follows), or to a manual X/Y point.

The guidance is **pure pursuit**: because a forward-only blimp can't rotate in place, it
never tries to — it always moves and steers toward the target so the *path* curves onto
the point.

```
steer   = kp_head · heading_error  −  kd_head · yaw_rate     (differential turn)
forward = velocity_profile(range) · align_gate(heading_error)
```

- `kd_head · yaw_rate` is anticipatory counter-steer — eases and reverses the turn before
  it overshoots (the continuous form of the hand-flying "counter-spin at the halfway
  mark" trick).
- The align gate slows forward when badly mis-pointed (tight, near-pivoting arc) and opens
  up when facing the target.
- A velocity profile decelerates into the point so it coasts to a stop (no reverse brake).
- Altitude hover holds the height captured at launch, minus the extra lift the tilted
  forward props add while driving (`z_fwd_couple`).

Includes a live pure-canvas 3D view (blimp, heading, trail, target ring, goal marker),
per-gain sliders, X/Y error sparklines, and a manual-teleop fallback in the same panel.

### `blimp_server.py` + `blimp_panel.html` — manual command panel
Launched by `BLIMP_PANEL_ESPNOW.command` (ESP-NOW, port 8421) or `BLIMP_PANEL.command`
(the drone's own Wi-Fi). Hand-fly from the browser — `W/S` forward, `A/D` turn, `Q/E`
up/down, `Space` = kill — with:

- **Constant-forward + differential turn**, with the turn clamped so it can never zero one
  motor and let the other run away.
- **Altitude hold** — a toggle that streams a constant vertical thrust you dial in by feel.
- **Counter-spin** — a tap that fires a short opposite pulse to kill leftover rotation.
- **Three frame profiles** (`tilted-up`, `tilted-down`, `straight` front-motor mounts),
  each storing its own full tuning set and auto-saving on every change.

### `cf_udp_patch.py`
Small compatibility shim for the Wi-Fi control path. ESP-Drone's CRTP-over-UDP link needs
an explicit `:2390` port in the URI and frames every packet with a trailing checksum byte
that stock `cflib` neither appends nor strips; this patch fixes both so the Wi-Fi tools
connect reliably. (The ESP-NOW path doesn't use it.)

## Local tuning files (not committed)
Each panel persists its live tuning next to itself — `straight_config.json` and
`blimp_config.json`. These are machine-specific and git-ignored; they're created on first
run and updated automatically as you drag sliders.
