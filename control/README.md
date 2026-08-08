# control/ — the ground station

Everything that runs on your laptop. Start it from the repo root with
`python run.py --mode <auto|manual|wander|swing>`. Run these files directly only when
debugging.

## Contents

| File | What it does |
|---|---|
| `panel_server.py` | **The server.** Reads mocap (NatNet), streams pose + commands + gains to the drone over the USB↔ESP-NOW bridge, serves the browser page, and logs every run. One file, three pages. |
| `auto_panel.html` | The **Auto** page — 2D/3D mocap view, path drawing, live gain sliders, hand-fly, gamepad, sys-ID tests. |
| `manual_panel.html` | The **Manual** page — hand-fly only. |
| `wander_panel.html` | The **Wander** page — fly to a point, park, wait for a push, repeat. |
| `mocap_config.json` | Saved tuning (the numbers currently flying). Written automatically whenever you move a slider. |
| `check_link.py` | Terminal-only link + motor test. **Run this first** when something won't connect. |
| `flight_logs/` | Per-run `run_<timestamp>.csv` + plots. `plant_final_20260727.json` is the identified dynamics model and is kept in git. |

**Swing / S-blimp build (separate, untested):**

| File | What it does |
|---|---|
| `swing_panel_server.py` | Standalone server for the 4-motor airframe (port 8620). Own NatNet reader, own control thread. |
| `swing_panel.html` | Its page — per-motor bench buttons, motor-map editor, live allocation matrix. |
| `mellinger_core.py` | SE(3) Mellinger controller port + the geometry-driven damped-least-squares mixer. |
| `swing_trajectory.py` | Trajectory generator for that build. |

## Control flow

```
Motive ──NatNet──▶ panel_server.py ──USB serial──▶ C6 bridge ──ESP-NOW──▶ drone
```

Frames going down the wire are told apart by a 4-byte magic tag plus their length:

| Frame | Payload | Meaning |
|---|---|---|
| `0xA5` | 16 B | manual setpoint (roll/pitch/yaw/thrust) — also what the swing build reuses as 4 motor duties |
| `0xA6` | 32 B | mocap pose + goal — engages the drone's on-board guidance |
| `0xA7` | 84 B | the live gains — sent whenever you move a slider |
| `0xB7` | 16 B | **coming back:** motor telemetry (mL, mR, mUp, mDown), logged into every run CSV |

**The drone does the flying.** In auto mode the panel only supplies pose and tuning; the
pursuit + altitude loops run in firmware (`blimp_guidance.c`). That's why no gain change
ever needs a reflash, and why the path drawn on screen is a *mirror* of the on-board math
rather than the source of it.

## Safety behaviour

- **Browser watchdog** — if the page stops polling for 1.5 s the server stops streaming,
  and the drone's own stale-pose failsafe cuts the motors.
- **KILL** is always visible and always immediate.
- **Link-loss failsafe** — if the bridge goes silent the drone zeroes the motors within
  250 ms rather than flying away.
- **Soft-start** ramps the motors over ~0.7 s so engaging can't brown out the battery.
