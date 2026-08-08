#!/usr/bin/env python3
"""
panel_server.py — ON-BOARD autonomous flight panel for the ESP-FLY blimp.

Serves three pages from one server, chosen at launch:
  (default)   auto_panel.html    autonomous flying + live tuning
  --manual    manual_panel.html  hand-fly only
  --wander    wander_panel.html  fly to a point, park, wait for a hand push, repeat

The DRONE does ALL the control math (firmware blimp_guidance.c: path-following +
carrot + arrival hold + hover). This Mac panel ONLY:
  1. reads the mocap (NatNet) for the blimp,
  2. STREAMS the pose + target to the drone (0xA6 frames), and
  3. streams the tuning gains (0xA7 frames) when you move a slider.
Nothing about the trajectory is computed here — the sliders just retune the
loops running on the drone, live, with no reflash.

  OptiTrack --lab WiFi--> Mac (this panel) --USB--> C6 --ESP-NOW--> blimp (S3)

One tracked rigid body (--body, the blimp). The target is set in the panel:
a drawn path, a circle, or the manual Target X/Y/Z boxes.

HOW ENGAGE / DISENGAGE WORKS (one-way ESP-NOW): the drone auto-engages the
on-board controller the instant it receives a pose frame, and its own
mocap-stale failsafe (blimpc.staleMs) stops the motors if the frames stop.
So GO = start streaming pose; STOP = stop streaming pose (the panel also sends
a few zero manual frames, which drop the drone's autonomous latch immediately).

The path/carrot drawn in the panel is a LOCAL MIRROR of the on-board math, for
visualization only — the real numbers live on the drone (CRTP log group blimpc).

Run it with `python run.py` from the repo root, or directly:
  python panel_server.py --server 192.168.0.4 --body 531
then open http://127.0.0.1:8601
"""
import argparse, glob, json, math, os, struct, subprocess, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def find_bridge_port():
    """Auto-detect the XIAO C6 bridge serial port on macOS / Windows / Linux."""
    try:  # pyserial's cross-platform enumerator (works on Windows COM ports)
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:  # prefer obvious USB-serial / ESP adapters
            blob = ((p.description or "") + (p.manufacturer or "") +
                    (p.device or "")).lower()
            if any(s in blob for s in ("usbmodem", "wchusbserial", "cp210",
                                       "ch340", "esp", "uart", "silicon", "wch")):
                return p.device
        if ports:
            return ports[0].device
    except Exception:
        pass
    g = (glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.wchusbserial*") +
         glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return g[0] if g else ""

DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8601
TICK = 0.05                # 50 ms = 20 Hz pose stream (gentler on the ESP-NOW link)
FULL = 65535               # forward motor full duty (manual test path)
PITCH_MAX = 32767          # int16 cap on the vertical/turn setpoint channels
M_VERT_SIGN = -1.0         # up/down hardware orientation (manual test path)
M_FWD_RAMP = 0.05          # per-tick fwd_level ramp (hold-W)
HOVER_UP_FRAC = 0.6        # hover baseline = this fraction of the Up power slider
AXES_STALE_S = 0.4         # gamepad input older than this is ignored (pad unplugged
                           # / tab backgrounded -> fall back to keys, never latch on)
Z_HOLD_KP = 0.9            # open-loop-test altitude hold: up-fraction per metre of error
Z_HOLD_CAP = 0.20          # ...max EXTRA lift above hover baseline (caps climb -> no ceiling)
Z_PROBE_TARGET = 1.5       # altitude (m) the open-loop probes auto-hold at

# ===========================================================================
# GAIN SET — must match GAIN order in blimp_guidance.c blimpGuidanceSetGains()
# (21 float32). key, default, min, max, step, group, label.
# ===========================================================================
GAIN_SPEC = [
    # ORDER IS LOCKED to blimpGuidanceSetGains() indices — do not reorder.
    # key, default, min, max, step, group, plain-English label
    ("kpZ",        30000.0,   0.0, 60000.0, 500.0, "Altitude", "Height-hold strength"),
    ("kiZ",         1200.0,   0.0,  6000.0,  50.0, "Altitude", "Auto-trim buoyancy (stops slow drift up/down)"),
    ("kdZ",         9500.0,   0.0, 30000.0, 250.0, "Altitude", "Height damping (anti-bounce)"),
    ("zff",        11000.0,   0.0, 30000.0, 250.0, "Altitude", "Baseline lift (fights gravity)"),
    ("iLimZ",      12000.0,   0.0, 30000.0, 500.0, "Altitude", "Auto-trim limit"),
    ("kpHead",         1.3,   0.0,     3.0,  0.05, "Steering", "Steer strength (how hard it turns to face the goal)"),
    ("kdHead",       0.007,   0.0,    0.05, 0.001, "Steering", "Turn damping (stops over-turning / wobble; too high = can't keep up in turns)"),
    ("turnCap",        0.5,  0.05,     1.0,  0.05, "Steering", "Max steering effort"),
    ("alignFloor",     0.5,   0.0,     1.0,  0.05, "Forward",  "Creep speed while still turning to face goal"),
    ("turnBoost",      0.6,   0.0,     1.0,  0.05, "Steering", "Turn power (0 = gentle single-motor, 1 = full — higher = tighter turns / follows path closer)"),
    ("driftK",        0.35,   0.0,     3.0,  0.05, "Steering", "Drift correction (crab nose into sideways drift; 0 = off; no firmware clamp, watch for oscillation past ~2)"),
    ("kVel",           0.7,   0.2,     4.0,  0.1,  "Forward",  "Throttle response (bigger = pushes harder to hit speed)"),
    ("fwdMaxN",       0.25,   0.0,     1.0,  0.02, "Forward",  "Forward cruise power (CONSTANT — floats + moves; 0 = turn-only test)"),
    ("arriveR",       0.35,   0.0,     1.5,  0.05, "Path",     "Corner smoothing radius — rounds path corners into a curve the blimp can actually fly (m). 0 = sharp corners"),
    ("holdExit",       2.5,   1.0,     5.0,  0.1,  "Arrival",  "Re-chase only after drifting this × the radius"),
    ("lookahead",      0.3,   0.1,     1.5,  0.05, "Path",     "Aim-ahead on the path (smaller = tracks tighter / less orbiting wide; bigger = smoother, rounder corners)"),
    ("fwdMaxPwm",  65535.0,   0.0, 65535.0,1000.0, "Output",   "Forward motor full-scale (PWM)"),
    ("turnMaxPwm", 32767.0,-32767.0,32767.0,500.0, "Output",   "Turn full-scale — SIGN flips turn direction (PWM)"),
    ("vertMaxPwm", 32767.0,   0.0, 32767.0, 250.0, "Output",   "Up/down motor full-scale (PWM)"),
    ("replanThresh",   0.4,   0.1,     1.5,  0.05, "Path",     "Re-plan the path if the goal moves this far (m)"),
    ("staleMs",      300.0,  50.0,  2000.0,  10.0, "Safety",   "Stop motors if mocap lost this long (ms)"),
    ("yawSlew",       20.0,   0.0,   120.0,   5.0, "Steering", "Turn speed — heading setpoint steps this many °/s (0 = snap the full angle)"),
    ("rampEn",         1.0,   0.0,     1.0,   1.0, "Output",   "Soft-start ramp (1 = ramp up gently, 0 = full power at once — test without ramping)"),
]
GAIN_ORDER = [g[0] for g in GAIN_SPEC]
DEFAULT_GAINS = {g[0]: g[1] for g in GAIN_SPEC}
DEFAULT_POWERS = {"fwd": 0.36, "up": 0.92, "down": 0.50, "left": 0.56, "right": 0.56}

# Two FRAME PROFILES for the swappable front-motor mounts; each stores its own
# full gain set + manual powers so you can physically swap the frame, click the
# matching profile, and every slider loads that frame's saved values.
FRAME_PROFILES = ("tilted", "straight")
MOCAP_CFG = os.path.join(DIR, "mocap_config.json")


def _wrap_pi(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _current_tuning():
    """Snapshot the live tuning (call under lock)."""
    return {"manual_trim": S["manual_trim"], "yaw_trim": S["yaw_trim"],
            "powers": dict(S["powers"]), "gains": dict(S["gains"])}

def _apply_tuning(d):
    """Load a profile's tuning into the live state (call under lock). Missing
    keys fall back to defaults so switching frames never leaves stale values."""
    S["powers"] = dict(DEFAULT_POWERS)
    S["gains"] = dict(DEFAULT_GAINS)
    S["manual_trim"] = float(d.get("manual_trim", 0.0))
    S["yaw_trim"] = float(d.get("yaw_trim", 0.0))
    for k, v in (d.get("powers") or {}).items():
        if k in S["powers"]:
            S["powers"][k] = float(v)
    for k, v in (d.get("gains") or {}).items():
        if k in S["gains"]:
            S["gains"][k] = float(v)

def load_trim():
    try:
        d = json.load(open(MOCAP_CFG))
    except Exception:
        d = {}
    with lock:
        if isinstance(d.get("profiles"), dict):
            S["profiles"] = {n: dict(d["profiles"].get(n, {})) for n in FRAME_PROFILES}
            S["profile"] = d["active"] if d.get("active") in FRAME_PROFILES else FRAME_PROFILES[0]
        else:
            S["profiles"] = {"tilted": {}, "straight": {}}
            S["profile"] = "tilted"
        _apply_tuning(S["profiles"].get(S["profile"], {}))
        # restore the last target choice (marker vs manual X/Y/Z) across restarts
        # legacy configs may say "marker" (the goal-marker feature is gone)
        if d.get("target_source") == "manual":
            S["target_source"] = "manual"
        t = d.get("target")
        if isinstance(t, dict):
            for k in ("x", "y", "z"):
                if k in t:
                    try: S["target"][k] = float(t[k])
                    except Exception: pass
        if d.get("path_mode") in ("circle", "heading", "path"):
            S["path_mode"] = d["path_mode"]
        elif d.get("path_mode") == "point":
            S["path_mode"] = "circle"   # v5 removed go-to-point; fall back to circle
        wp = d.get("waypoints")
        if isinstance(wp, list):
            S["waypoints"] = [[float(p[0]), float(p[1])] for p in wp
                              if isinstance(p, (list, tuple)) and len(p) >= 2]
        S["wp_loop"] = bool(d.get("wp_loop", S["wp_loop"]))
        try: S["heading_deg"] = float(d.get("heading_deg", S["heading_deg"]))
        except Exception: pass
        c = d.get("circle")
        if isinstance(c, dict):
            for k in ("r", "lead", "dir", "lead_m", "look_m"):
                if k in c:
                    try: S["circle"][k] = float(c[k])
                    except Exception: pass
            if "lead_m" not in c:       # pre-arc-length config: keep the old key sane
                S["circle"]["lead_m"] = S["circle"]["r"] * math.radians(S["circle"]["lead"])
            # look_m is NOT derived from lead_m: they measure different things (distance
            # from the vehicle vs arc from its projection), so a saved lead_m of 0.29
            # would become a dangerously short lookahead. Fall back to the safe default.
            if "look_m" not in c:
                S["circle"]["look_m"] = 1.0
    print("loaded frame profiles from %s (active: %s, target: %s)" %
          (MOCAP_CFG, S["profile"], S["target_source"]))

def save_trim():
    with lock:
        S["profiles"][S["profile"]] = _current_tuning()
        out = {"active": S["profile"], "profiles": dict(S["profiles"]),
               "target_source": S["target_source"], "target": dict(S["target"]),
               "path_mode": S["path_mode"], "circle": dict(S["circle"]),
               "waypoints": [list(p) for p in S["waypoints"]], "wp_loop": S["wp_loop"],
               "heading_deg": S["heading_deg"]}
    try:
        json.dump(out, open(MOCAP_CFG, "w"), indent=2)
    except Exception as e:
        print("trim save failed:", e)


lock = threading.Lock()
S = {
    "raw": {"x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1], "valid": False, "t": 0.0},
    "target": {"x": 1.0, "y": 0.0, "z": 1.2},
    "target_source": "manual",   # the Target X/Y/Z boxes (kept for config compatibility)
    "path_mode": "circle",       # "circle" = ring; "heading" = turn-test; "path" = waypoints
    # orbit radius (m), lead_m = how far AHEAD along the ring the streamed setpoint sits
    # (metres of arc -- the "distance between consecutive setpoints"), +CCW/-CW.
    # `lead` (deg) is the LEGACY angular knob, kept only to migrate old configs: an
    # angle means a different real distance at every radius, and at r=2.0 m the saved
    # lead=0.01 deg was a 0.35 MILLIMETRE lead -- i.e. no along-track lead at all, so
    # the setpoint collapsed onto the radially-nearest ring point and the craft aimed
    # at the centre instead of around the ring. Arc-length is radius-independent.
    # look_m = TRUE pure-pursuit lookahead: the DISTANCE from the blimp to the aim
    # point, held exactly by _circle_pursuit whatever the radial error. This is the
    # live knob. `lead`/`lead_m` are the two superseded schemes, kept so old configs
    # still load; neither is read by the circle branch any more.
    "circle": {"r": 0.8, "lead": 40.0, "dir": 1.0, "lead_m": 0.35, "look_m": 1.0},
    "waypoints": [],             # PATH mode: list of [x,y] laid out by clicking the 2D view
    "wp_loop": True,             # PATH mode: loop back to the first point (flow forever, never stop)
    "heading_deg": 0.0,          # turn-test: desired absolute heading (world frame, deg)
    "hold_alt": 1.2,             # altitude (m) latched at GO -> held during the turn-test
    "up_axis": "Z",
    "flying": False,             # bridge open (manual test path live)
    "keys": {k: False for k in "WSADQE"},
    # ANALOG manual input from a gamepad (Xbox etc). Proportional, so a half-pressed
    # trigger really is half power -- unlike the on/off WASD keys. `t` is the last
    # update time: _manual_tick ignores these once stale so an unplugged pad can
    # never latch the motors on.
    "axes": {"fwd": 0.0, "turn": 0.0, "vert": 0.0, "t": 0.0, "name": ""},
    "fwd_level": 0.0,
    "hover": False,              # manual float-assist: hold a baseline up so it doesn't sink
    "motors": [float("nan")] * 4,   # latest 4 motor commands from drone telemetry (fwdL,fwdR,up,down)
    "motors_t": 0.0,             # wall-clock of the last telemetry frame (link-alive check)
    "last_client": 0.0,
    "manual_trim": 0.0, "yaw_trim": 0.0,
    "powers": dict(DEFAULT_POWERS),
    "gains": dict(DEFAULT_GAINS),
    "gains_dirty": True,         # push 0xA7 to the drone on next tick
    "profile": "tilted", "profiles": {},
    "auto_go": False,            # streaming pose -> drone flies autonomously
    "takeoff_done": False,       # False = climb straight up to Z first, THEN follow path
    "t_go": 0.0,                 # wall-clock at GO -> drives the takeoff climb ramp
    "alt_go": 0.0,               # altitude latched at GO -> ramp starts from here
    "test_id": 0,                # bumps on each sys-ID test start/abort -> stale workers exit
    "test_name": "",             # current sys-ID test running ("" = idle); shown on the panel
    "ol_active": False,          # open-loop test (yaw step+coast) is driving the manual path
    "ol_cmd": None,              # (turn, forward) injected open-loop command during that test
    "ol_hold_alt": None,         # altitude (m) to hold CLOSED-LOOP during an open-loop test (None=off)
    "latency_ms": None,          # measured control-loop latency (command -> motion onset), ms
    "latency_detail": "",        # human summary of the latency probe result
    "ol_dir": 1.0,              # yaw-step spin direction: +1 = CCW-ish (D), -1 = CW-ish (A)
    "autotrim_build": False,     # launched with --autotrim (shows the auto-trim card)
    "autotrim": False,           # host-side integral drift auto-trim ACTIVE (toggle live)
    "at_ki": 0.15,               # auto-trim learn rate (rad crab per m/s drift per s); <0 flips
    "at_icap": 0.35,             # |learned crab| clamp, rad (~20 deg)
    "at_i": 0.0,                 # current learned crab bias (rad), streamed via carrot rotation
    # ---- Wander build (--wander): go to random points, park + LED-flash, wait for a
    # push, go to the next point. Rides the existing "path" mode (2-pt path = a curve
    # the on-board pursuit law actually flies, not a straight line) -- no firmware
    # change needed for navigation; the arrival LED itself IS a firmware addition
    # (blimp_guidance.c flashes LED_GREEN when range-to-carrot < arriveR). ----
    "wander_build": False,        # launched with --wander (shows the wander page)
    "wander": False,              # wandering ACTIVE
    "wander_id": 0,               # ownership token for the worker thread (like test_id)
    "wander_bounds": {"xmin": -2.0, "xmax": 2.0, "ymin": -2.0, "ymax": 2.0},
    "wander_z": 1.0,              # altitude to fly at
    "wander_hold_s": 4.0,         # seconds parked + LED-flashing before the next point
    "wander_min_sep": 0.8,        # new point must be at least this far from the last one
    "wander_state": "idle",       # idle | goto | arrived | waitpush
    "wander_target": None,        # [x,y] current target
    "wander_predicted": [],       # [[x,y],...] predicted curved trajectory to the target
    "wander_hold_left": 0.0,      # LIVE seconds remaining in the arrived/hold phase
    "wander_push_m": 0.30,        # hand-push detected when it moves this far while parked
    "wander_push_v": 0.18,        # ...or when it's SHOVED this fast (m/s) -- rejects slow drift
    "wander_timeout_s": 60.0,     # give up flying to a point after this long, park anyway
    "wander_next": False,         # panel "next point now" override (skip waiting for a push)
    "wander_mode": "random",      # "random" = pick points in bounds | "list" = fly my clicked points
    "wander_queue": [],           # [[x,y],...] user-picked points (list mode)
    "wander_qi": 0,               # index into wander_queue
    "wander_push_dbg": "",        # LIVE readout: why the last push fired (disp vs speed)
    "wander_push_marks": [],      # [[x,y,"why"],...] where pushes were detected -> drawn in the view
    # ONE PUSH = ONE NEW POINT. After a shove the craft keeps coasting (surge time
    # constant ~5.5 s), so the displacement window stays over threshold and the
    # detector used to re-fire on the vehicle's OWN momentum, reshuffling the target
    # several times from a single hand contact. Ignore detections for this long after
    # one fires -- long enough for the shove to bleed off, short enough to feel live.
    "wander_push_lock_s": 3.0,    # s -- refractory after a detected push
    "wander_push_mute_until": 0.0,  # LIVE deadline; detection suppressed until then
    "wander_slow_m": 1.5,         # start rolling cruise off this far BEFORE the arrive ring
    "wander_fwd_scale": 1.0,      # LIVE cruise multiplier from the approach taper (readout)
    # IN-FLIGHT SHOVE = how far it moves in a short WINDOW. Measured from real logs,
    # instantaneous derivatives are unusable at this mocap rate (9.1 Hz, dt=0.11 s):
    # double-differentiating for acceleration puts the NOISE median at 0.92 m/s^2, and
    # even single-differentiated speed has p90 = 0.63 m/s against a 0.26 m/s cruise.
    # A window averages that noise down instead of amplifying it: under power the craft
    # covers ~0.26*win metres, so a hand that moves it much further is unambiguous.
    # 0.60/0.6 chosen by REPLAYING the real logs through the detector: 0.35 -> 3.5
    # false fires per 30 s run, 0.45 -> 0.78, 0.60 -> 0.20, 0.75 -> 0. Below 0.60 the
    # craft's own motion trips it, which is what made the points jump around. The cost
    # of 0.60 is that the shove has to be deliberate (~1.0 m/s); a light nudge (0.8 m/s
    # = 0.48 m per window) will not register. Lower it if you want a lighter touch and
    # can accept the occasional spurious jump.
    "wander_push_jump": 0.90,     # m -- displacement within the window that means "pushed"
    "wander_push_win": 0.6,       # s -- window the displacement is measured over
    # Braking horizon. A blimp cannot brake; cutting thrust it coasts v*tau metres.
    # The approach targets v_des = (range - arriveR)/coast_s, so at 8 s it is planning
    # to stop from cruise (0.26 m/s) over ~2.1 m -- comfortably more conservative than
    # the old 1.5 m linear taper, which is why that taper is no longer applied.
    "wander_coast_s": 8.0,        # s -- coast time constant used to plan the approach
    "manual_build": False,       # launched with --manual (dedicated hand-fly-only page)
    "path_prog": 0.0,            # arc-length committed along the path (virtual target position)
    "path_t": 0.0,               # wall-clock of last carrot advance (drives the creep rate)
    "smooth_path": [],           # rounded trajectory the carrot actually follows (for display)
    # ---- local viz mirror of the on-board controller (display only) ----
    "path": [], "path_goal": None, "carrot": None, "hold": False,
    "herr": 0.0, "range": 0.0,
    "rate": 0.0, "frames": 0,
    "bridge": "", "err": "",
}
RUNNING = True
YAW_SIGN = 1.0    # mocap heading handedness. Feeds BOTH the on-screen heading
                  # arrow AND the yaw streamed to the drone, so the picture, the
                  # heading-error number, and the drone's control all stay in the
                  # same convention. With the +sin arrow in auto_panel.html this
                  # makes "arrow points at target" == "heading error 0" == drone
                  # facing target. (If the craft turns the WRONG way in flight,
                  # that's the separate ⟳ Flip / turnMaxPwm sign, not this.)


def quat_yaw(q, up):
    qx, qy, qz, qw = q
    if up == "Y":
        return YAW_SIGN * math.atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
    return YAW_SIGN * math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def quat_euler(q):
    """Quaternion (x,y,z,w) -> (roll, pitch, yaw) radians, aerospace ZYX. Used
    for the full-state sys-ID log (yaw itself still comes from quat_yaw so it
    matches the up-axis mapping the rest of the panel uses)."""
    qx, qy, qz, qw = q
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    s = 2.0 * (qw * qy - qz * qx)
    s = 1.0 if s > 1.0 else (-1.0 if s < -1.0 else s)
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw

def _map_raw(r, up):
    x, y, z = r["x"], r["y"], r["z"]
    return (x, z, y) if up == "Y" else (x, y, z)

def mapped():
    """Blimp pose -> (h0, h1, alt, yaw_rad, valid).

    yaw_trim (DEGREES) is the rigid-body-vs-nose heading offset set by the Zero
    button. Applying it HERE is what makes it real: this one return value feeds the
    streamed cyaw, the on-screen nose arrow, and the heading error, so all three stay
    consistent. Before this it was stored and saved but never read -- the offset was
    an uncorrected bias in every heading the controller ever saw."""
    with lock:
        r = dict(S["raw"]); up = S["up_axis"]; trim = S["yaw_trim"]
    h0, h1, alt = _map_raw(r, up)
    yaw = _wrap_pi(quat_yaw(r.get("q", [0, 0, 0, 1]), up) + math.radians(trim))
    return h0, h1, alt, yaw, r.get("valid", False)

TAKEOFF_BAND = 0.15   # m below target Z at which "takeoff" is considered done
TAKEOFF_RATE = 0.35   # m/s -- how fast the climb setpoint ramps up (gentle = no overshoot)
TAKEOFF_TIMEOUT = 9.0 # s -- give up waiting to reach Z and start the path anyway
MIN_RUN_MOVE = 0.5    # m -- discard a run's log/plot if it never moved this far
CARROT_SPEED_K = 1.2  # virtual-target speed = this * fwdMaxN (m/s), ~matches cruise


def _smooth_path(pts, r, loop):
    """Round the polyline corners into a curve the craft can ACTUALLY fly.

    Sharp waypoint corners are the root of the wide-overshoot + get-stuck-orbiting
    problem: the blimp can't pivot fast enough, sails past the corner, and then
    its path projection can't advance so the carrot freezes. We replace each
    corner with a quadratic-bezier fillet of radius ~r (cut back at most half a
    leg so short segments don't collapse), giving a smooth trajectory the turn
    loop can hold. r<=0 or <3 points -> unchanged. Closed loops fillet every
    vertex; open paths keep their two endpoints."""
    P = [list(p) for p in pts]
    n = len(P)
    if n < 3 or r <= 1e-6:
        return P
    K = 6                                    # arc sample points per corner
    out = []
    idxs = range(n) if loop else range(1, n - 1)
    if not loop:
        out.append(P[0])
    for i in idxs:
        A = P[(i - 1) % n]; B = P[i]; C = P[(i + 1) % n]
        dAB = math.hypot(B[0] - A[0], B[1] - A[1])
        dBC = math.hypot(C[0] - B[0], C[1] - B[1])
        if dAB < 1e-9 or dBC < 1e-9:
            out.append(B); continue
        dIn = min(r, 0.5 * dAB); dOut = min(r, 0.5 * dBC)
        pin = [B[0] + (A[0] - B[0]) * dIn / dAB, B[1] + (A[1] - B[1]) * dIn / dAB]
        pout = [B[0] + (C[0] - B[0]) * dOut / dBC, B[1] + (C[1] - B[1]) * dOut / dBC]
        out.append(pin)
        for j in range(1, K):                # quadratic bezier pin->B->pout
            t = j / K; mt = 1.0 - t
            out.append([mt * mt * pin[0] + 2 * mt * t * B[0] + t * t * pout[0],
                        mt * mt * pin[1] + 2 * mt * t * B[1] + t * t * pout[1]])
        out.append(pout)
    if not loop:
        out.append(P[-1])
    return out


def _save_manual_rec(samples):
    """Persist a manual hand-fly recording to flight_logs/ next to the auto runs,
    but tracking x,y,yaw + trajectory (no target). samples = [[t,x,y,yaw], ...]
    streamed from the panel. Written so the same replayer can load it."""
    try:
        if not samples or len(samples) < 2:
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, "manual_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        with open(path, "w") as f:
            f.write("t,x,y,yaw\n")
            for s in samples:
                try:
                    f.write("%.3f,%.4f,%.4f,%.2f\n" % (float(s[0]), float(s[1]),
                                                       float(s[2]), float(s[3])))
                except Exception:
                    continue
        print("manual recording -> %s (%d samples)" % (path, len(samples)))
        try:
            _render_turn_plot(path)               # turn-test summary alongside it
        except Exception as e:
            print("turn plot failed:", e)
    except Exception as e:
        with lock: S["err"] = "save_rec failed: %s" % e


def _read_xyyaw(csv_path):
    """Read (t, x, y, yaw) from ANY flight CSV -- manual (t,x,y,yaw) or auto
    (t_s,h0,h1,...,yaw_deg). Time re-based to zero. Returns four lists."""
    import csv as _csv
    with open(csv_path) as f:
        rows = list(_csv.DictReader(f))
    if not rows:
        return [], [], [], []
    cols = {k.lower(): k for k in rows[0].keys()}
    pick = lambda *names: next((cols[n] for n in names if n in cols), None)
    ct, cx = pick("t", "t_s", "time"), pick("x", "h0")
    cy, cyaw = pick("y", "h1"), pick("yaw", "yaw_deg")
    if cx is None or cy is None:
        return [], [], [], []
    t = []; x = []; y = []; yaw = []; t0 = None
    for i, r in enumerate(rows):
        try:
            xx = float(r[cx]); yy = float(r[cy])
        except Exception:
            continue
        tt = float(r[ct]) if ct and r.get(ct) not in (None, "", "nan") else i * 0.05
        if t0 is None:
            t0 = tt
        t.append(tt - t0); x.append(xx); y.append(yy)
        yaw.append(float(r[cyaw]) if cyaw and r.get(cyaw) not in (None, "", "nan") else 0.0)
    return t, x, y, yaw


def _render_turn_plot(csv_path):
    """Turn-test summary PNG next to the CSV: LEFT = cumulative yaw vs time (with
    a 90 deg reference line) so you see if it hits the target angle and how fast;
    RIGHT = top-down path with heading ARROWS along it + net translation (drift)
    while turning, so 'how wide it turns / turn in place' is visible. Works for a
    manual recording or an auto turn-test run."""
    t, x, y, yaw = _read_xyyaw(csv_path)
    if len(t) < 3:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # cumulative (unwrapped) yaw so a spin past +-180 keeps counting
    cum = [0.0]
    for i in range(1, len(yaw)):
        d = ((yaw[i] - yaw[i - 1] + 540.0) % 360.0) - 180.0
        cum.append(cum[-1] + d)
    drift = math.hypot(x[-1] - x[0], y[-1] - y[0])
    net_turn = cum[-1]
    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13, 5.5))
    axA.axhline(0, color="#999", lw=0.8)
    for ref in (90, -90):
        axA.axhline(ref, color="#c98", lw=0.9, ls="--")
    axA.plot(t, cum, color="#e0a020", lw=1.6)
    axA.set_xlabel("time (s)"); axA.set_ylabel("cumulative yaw (deg)")
    axA.set_title("turned %.0f deg  (rate ~%.0f deg/s)"
                  % (net_turn, net_turn / t[-1] if t[-1] > 1e-6 else 0.0))
    axA.grid(True, alpha=0.25)
    # RIGHT: trajectory with heading arrows
    axB.plot(x, y, color="#1f9fe0", lw=1.6, alpha=0.8, zorder=1)
    step = max(1, len(x) // 24)
    for i in range(0, len(x), step):
        a = math.radians(yaw[i])
        axB.arrow(x[i], y[i], 0.18 * math.cos(a), 0.18 * math.sin(a),
                  head_width=0.06, head_length=0.06, fc="#e0a020", ec="#e0a020",
                  length_includes_head=True, zorder=3)
    axB.plot(x[0], y[0], "o", color="#3fa34d", ms=9, zorder=4, label="start")
    axB.plot(x[-1], y[-1], "s", color="#d05a5a", ms=9, zorder=4, label="end")
    axB.set_aspect("equal", "datalim"); axB.grid(True, alpha=0.25)
    axB.set_xlabel("x (m)"); axB.set_ylabel("y (m)"); axB.legend(loc="best", fontsize=8)
    axB.set_title("drift while turning: %.2f m" % drift)
    fig.suptitle(os.path.basename(csv_path) + "  —  turn test", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    png = os.path.splitext(csv_path)[0] + "_turn.png"
    fig.savefig(png, dpi=96); plt.close(fig)
    print("turn plot -> %s" % png)


def _render_state_plot(csv_path):
    """Full-state sys-ID PNG next to the CSV: 4 stacked panels vs time --
    position (x,y,z), orientation (roll,pitch,yaw), linear rates (vx,vy,vz),
    angular rates (wroll,wpitch,wyaw). This is the plant OUTPUT we fit a model
    to (paired with the motor-command INPUT once the blimp telemetry lands)."""
    import csv as _csv
    cols = {"t_s": [], "h0": [], "h1": [], "alt": [], "roll_deg": [], "pitch_deg": [],
            "yaw_deg": [], "vx": [], "vy": [], "vz": [], "wroll": [], "wpitch": [], "wyaw": [],
            "mL": [], "mR": [], "mUp": [], "mDown": []}
    with open(csv_path) as f:
        rd = _csv.DictReader(f)
        fn = rd.fieldnames or []
        if "vx" not in fn:
            return                                # old-format run, no rate columns
        has_motors = "mL" in fn
        for row in rd:
            for k in cols:
                try: cols[k].append(float(row.get(k, "nan")))
                except Exception: cols[k].append(float("nan"))
    t = cols["t_s"]
    if len(t) < 3:
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    npan = 5 if has_motors else 4
    fig, ax = plt.subplots(npan, 1, figsize=(11, 2.6 * npan), sharex=True)
    ax[0].plot(t, cols["h0"], label="x"); ax[0].plot(t, cols["h1"], label="y")
    ax[0].plot(t, cols["alt"], label="z"); ax[0].set_ylabel("position (m)")
    ax[1].plot(t, cols["roll_deg"], label="roll"); ax[1].plot(t, cols["pitch_deg"], label="pitch")
    ax[1].plot(t, cols["yaw_deg"], label="yaw"); ax[1].set_ylabel("orientation (deg)")
    ax[2].plot(t, cols["vx"], label="vx"); ax[2].plot(t, cols["vy"], label="vy")
    ax[2].plot(t, cols["vz"], label="vz"); ax[2].set_ylabel("linear rate (m/s)")
    ax[3].plot(t, cols["wroll"], label="roll rate"); ax[3].plot(t, cols["wpitch"], label="pitch rate")
    ax[3].plot(t, cols["wyaw"], label="yaw rate"); ax[3].set_ylabel("angular rate (deg/s)")
    if has_motors:
        ax[4].plot(t, cols["mL"], label="fwdL"); ax[4].plot(t, cols["mR"], label="fwdR")
        ax[4].plot(t, cols["mUp"], label="up"); ax[4].plot(t, cols["mDown"], label="down")
        ax[4].set_ylabel("motor cmd (PWM)  [INPUT]")
    ax[npan - 1].set_xlabel("time (s)")
    for a in ax:
        a.grid(True, alpha=0.25); a.axhline(0, color="#999", lw=0.6); a.legend(loc="upper right", fontsize=8)
    fig.suptitle(os.path.basename(csv_path) + "  —  full state + rates (sys-ID)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    png = os.path.splitext(csv_path)[0] + "_state.png"
    fig.savefig(png, dpi=92); plt.close(fig)
    print("state plot -> %s" % png)


def _run_moved(csv_path, thresh):
    """True if the flown trajectory ever got `thresh` metres from its start."""
    import csv as _csv
    x0 = y0 = None
    try:
        with open(csv_path) as f:
            for row in _csv.DictReader(f):
                try:
                    x = float(row["h0"]); y = float(row["h1"])
                except Exception:
                    continue
                if x0 is None:
                    x0, y0 = x, y
                elif math.hypot(x - x0, y - y0) >= thresh:
                    return True
    except Exception:
        return True                          # on error, keep the log (safer)
    return False

def _resolve_target():
    """The goal point the panel is streaming as (tx,ty,tz).
    CIRCLE mode: orbit the manual Target X/Y (center) -- the streamed goal is kept
    `lead` degrees ahead of the blimp's own angle around the ring, so the drone
    chases a point that runs around the circle at whatever pace it can manage
    (pursuit-on-a-circle; robust for a slow craft that can't be lapped).
    POINT mode: the live goal marker (if tracked) or the manual X/Y/Z."""
    with lock:
        mode = S["path_mode"]
        tgt = dict(S["target"]); circ = dict(S["circle"]); hdg = S["heading_deg"]
        hold_alt = S["hold_alt"]
        wps = [list(p) for p in S["waypoints"]]; wp_loop = S["wp_loop"]
        look = S["gains"]["lookahead"]
        auto_go = S["auto_go"]; takeoff_done = S["takeoff_done"]
        t_go = S["t_go"]; alt_go = S["alt_go"]
    # TAKEOFF-FIRST. Starting from the ground, climb straight up to the set
    # altitude (Z) BEFORE moving horizontally. Three things make this clean:
    #   (1) RAMPED setpoint: v5 has no hover-hold and a helium blimp is buoyant,
    #       so streaming the full target Z at once builds vertical momentum and
    #       overshoots (log: climbed 0.35 -> 2.09 m for a 1.10 target). We ramp
    #       the streamed Z up at TAKEOFF_RATE so altitude error stays small.
    #   (2) FORWARD ZEROED (in _gain_frame while takeoff is active): v5 always
    #       drives forward toward the streamed point, so we can't "hover in place"
    #       by streaming our own x,y -- range~=0 makes the heading garbage and it
    #       spins/creeps (log: herr=-148). Forward off => the vertical channel
    #       climbs it straight up (proven: it over-climbed with forward random).
    #   (3) HEADING PRE-ALIGN: we stream the FIRST waypoint's x,y as the goal, so
    #       while forward=0 the turn loop rotates the nose onto the path. When the
    #       climb finishes the craft is already facing waypoint 1 -> no start spin.
    # All host-side; the on-board controller is unchanged.
    if auto_go and not takeoff_done and mode in ("path", "circle"):
        h0, h1, alt, _y, valid = mapped()
        # where to point the nose during the climb (first waypoint, else center)
        if mode == "path" and wps:
            aimx, aimy = wps[0][0], wps[0][1]
        else:
            aimx, aimy = tgt["x"], tgt["y"]
        climbing = valid and alt < tgt["z"] - TAKEOFF_BAND
        # safety: never sit forward-off forever if it can't quite reach Z
        if climbing and (time.time() - t_go) < TAKEOFF_TIMEOUT:
            climb = alt_go + TAKEOFF_RATE * max(0.0, time.time() - t_go)
            if climb > tgt["z"]:
                climb = tgt["z"]
            return aimx, aimy, climb         # climb (ramped) + pre-aim, no forward
        with lock:
            S["takeoff_done"] = True         # reached altitude (or timed out) -> go
            S["gains_dirty"] = True          # restore forward cruise in the frame
    if mode == "path":
        # WAYPOINT PATH FOLLOWING. Stream a carrot that runs `lookahead` metres
        # ahead of the blimp's closest point on the polyline through the waypoints.
        # Because the carrot has already rounded onto the NEXT segment before the
        # blimp reaches a corner, the craft flows THROUGH the turn (cutting the
        # corner by ~lookahead) instead of arriving with momentum aimed the wrong
        # way. Constant cruise + never-stop = the momentum is planned for, not
        # fought. All host-side: the drone controller is unchanged.
        h0, h1, _a, _y, _v = mapped()
        with lock:
            prog = S["path_prog"]; smooth_r = S["gains"]["arriveR"]
        sm = _smooth_path(wps, smooth_r, wp_loop)      # rounded (flyable) trajectory
        look_eff = look
        c, new_prog = _path_carrot(h0, h1, sm, look_eff, wp_loop, prog)
        with lock:
            S["path_prog"] = new_prog
            S["smooth_path"] = [[round(p[0], 3), round(p[1], 3)] for p in sm]
        if c is not None:
            return c[0], c[1], tgt["z"]
        return tgt["x"], tgt["y"], tgt["z"]
    if mode == "heading":
        # Turn-test: goal is a point FAR in the desired heading from the blimp NOW,
        # so bearing == desired heading regardless of position -> pure heading loop.
        # Altitude target = the altitude LATCHED at GO, so the altitude loop still
        # actively holds height (a live current-alt target would zero its own error
        # and never correct). Forward is forced off in _gain_frame (fwdMaxN=0), so
        # this exercises the TURN plus the altitude hold, nothing else.
        h0, h1, _alt, _y, _v = mapped()
        th = math.radians(hdg)
        return h0 + 4.0 * math.cos(th), h1 + 4.0 * math.sin(th), hold_alt
    if mode == "circle":
        # Pursuit-on-a-circle. The streamed goal sits `lead_m` metres of ARC ahead of
        # the blimp's own angle around the ring, so it can never be lapped -- the
        # setpoint only advances as the craft advances.
        #
        # WHY ARC-LENGTH AND NOT DEGREES: the lead is the distance between consecutive
        # setpoints, and that has to mean the same thing at every radius. The old
        # angular knob did not: at r=2.0 m a lead of 0.01 deg is 0.35 mm, so the
        # "ahead" point was effectively the radially-NEAREST point on the ring. Sitting
        # outside the ring, the craft then aimed straight at the centre rather than
        # around it, and the bearing swung wildly for millimetre motions (run
        # 20260805_173019: |herr| p90 = 167 deg, orbiting 0.71 m wide, only 4 crossings).
        # Small lead_m => setpoint close and slow => the craft keeps cutting inside and
        # swinging back out, i.e. the in/out oscillation about the ring we want to see.
        h0, h1, _a, yaw_now, _v = mapped()
        cx0, cy0 = tgt["x"], tgt["y"]
        rr = max(0.05, circ["r"])                   # guard: no divide-by-zero ring
        LL = max(0.10, float(circ.get("look_m", 1.0)))
        tx0, ty0 = _circle_pursuit(cx0, cy0, rr, LL, circ["dir"], h0, h1, yaw_now)
        return tx0, ty0, tgt["z"]
    return tgt["x"], tgt["y"], tgt["z"]


def _plan_path(sx, sy, gx, gy, spacing):
    d = math.hypot(gx - sx, gy - sy)
    n = max(1, int(d / max(spacing, 0.05)))
    return [[sx + (gx - sx) * i / n, sy + (gy - sy) * i / n] for i in range(n + 1)]


def _circle_pursuit(cx, cy, R, L, dirn, h0, h1, yaw):
    """TRUE pure pursuit on a ring: the aim point is where a circle of radius L
    centred on the BLIMP cuts the ring, taking the forward intersection.

    WHY THIS AND NOT AN ARC OFFSET. The old scheme put the goal a fixed arc ahead of
    the blimp's angular PROJECTION onto the ring, so the actual distance to the goal
    depended on how far off-ring the craft was. Sitting on the ring it was ~lead_m
    (0.29 m); crabbing inside, the geometry collapsed and the goal could end up on top
    of the craft or behind it. Bearing to a near point is ill-conditioned -- a few cm
    of mocap noise is many degrees, and drifting PAST the point flips the bearing
    ~180 deg. Measured over every circle run: at range <0.3 m the median |heading err|
    is 28 deg and p90 is 152 deg, versus 5.6 deg / 35 deg out at 1.2-2.0 m.

    Pure pursuit fixes that by construction: |aim - blimp| == L exactly, wherever the
    craft is. Off-ring, the forward intersection also sits on the far side of the
    ring, so the craft is steered BACK onto it -- the inward pull comes free.

    Returns (tx, ty).
    """
    ux, uy = h0 - cx, h1 - cy
    d = math.hypot(ux, uy)
    if d < 1e-6:                       # dead centre: bearing undefined, drive outward
        return cx + R * math.cos(yaw), cy + R * math.sin(yaw)
    ux /= d; uy /= d
    # No intersection when the craft is further than L from the whole ring (either far
    # outside, d > R+L, or deep inside near the centre, d < R-L). Then the nearest ring
    # point IS the right target: it pulls straight back to the path, and its range
    # (|d-R|) is necessarily > L, so the bearing stays well-conditioned either way.
    if d > R + L or d < R - L:
        return cx + R * ux, cy + R * uy
    # radical line of the two circles -> the two intersection points
    a = (d * d - L * L + R * R) / (2.0 * d)
    hh = math.sqrt(max(0.0, R * R - a * a))
    mx, my = cx + a * ux, cy + a * uy
    px, py = -uy, ux                                   # unit normal to (C -> P)
    c1 = (mx + hh * px, my + hh * py)
    c2 = (mx - hh * px, my - hh * py)
    # pick the one AHEAD in the direction of travel (dirn: +1 = CCW, -1 = CW)
    ang_p = math.atan2(h1 - cy, h0 - cx)
    s = 1.0 if dirn >= 0 else -1.0
    best = None
    for c in (c1, c2):
        dth = _wrap_pi(math.atan2(c[1] - cy, c[0] - cx) - ang_p) * s
        if best is None or dth > best[0]:
            best = (dth, c)
    return best[1]


def _path_point_at(P, cum, s):
    """Point on polyline P at arc-length s (cum = cumulative vertex distances)."""
    total = cum[-1]
    if total <= 1e-9:
        return [P[0][0], P[0][1]]
    s = max(0.0, min(s, total))
    # find segment containing s
    i = 0
    while i < len(cum) - 2 and cum[i + 1] < s:
        i += 1
    seglen = cum[i + 1] - cum[i]
    f = 0.0 if seglen < 1e-9 else (s - cum[i]) / seglen
    ax, ay = P[i]; bx, by = P[i + 1]
    return [ax + f * (bx - ax), ay + f * (by - ay)]


def _path_carrot(h0, h1, pts, look, loop, prog):
    """Drone-COUPLED pursuit carrot: the aim point is always `look` metres ahead
    of WHERE THE DRONE ACTUALLY IS on the path (its arc-length projection), so it
    only moves when the drone moves -- not a free-running clock.

    `prog` tracks the drone's committed arc-length projection and is MONOTONIC:
    it never retreats (overshooting a corner won't snap the carrot backward), and
    on a loop it never jumps a whole lap forward (which would fling the carrot to
    the far side and flip direction) -- if the drone lags, projection just holds.

    Returns ([carrot_x, carrot_y], new_prog), or (None, prog) if nothing to
    follow. `loop` re-adds the first point so a closed path wraps forward."""
    if not pts:
        return None, prog
    if len(pts) == 1:
        return [pts[0][0], pts[0][1]], prog
    P = [list(p) for p in pts]
    if loop:
        P = P + [P[0]]                       # close the loop
    look = max(0.05, float(look))
    # cumulative arc length at each vertex
    cum = [0.0]
    for i in range(len(P) - 1):
        cum.append(cum[-1] + math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1]))
    total = cum[-1]
    # 1) closest point on the polyline -> its arc length s_c, but ONLY searched
    #    within a WINDOW of arc-length near where we already are (`prog`). An
    #    unrestricted nearest-point search is ambiguous wherever the path
    #    crosses itself (e.g. a figure-8's center): two far-apart arc-length
    #    positions can be geometrically right on top of each other, so a plain
    #    Euclidean-nearest search flips between them from one tick to the next
    #    -> the carrot snaps to the OTHER lobe ("lead point keeps changing
    #    direction"). Restricting candidates to near the committed progress
    #    removes the ambiguity.
    back_allow = 0.3
    fwd_win = max(2.0, 4.0 * look)
    best = (1e18, 0.0)                        # (dist2, arc length)
    for i in range(len(P) - 1):
        seg_s = cum[i]
        if loop and total > 1e-9:
            rel = ((seg_s - prog + 0.5 * total) % total) - 0.5 * total
            if rel < -back_allow or rel > fwd_win:
                continue
        elif seg_s < prog - back_allow or seg_s > prog + fwd_win:
            continue
        ax, ay = P[i]; bx, by = P[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((h0 - ax) * dx + (h1 - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d2 = (h0 - px) ** 2 + (h1 - py) ** 2
        if d2 < best[0]:
            best = (d2, cum[i] + t * math.hypot(dx, dy))
    if best[0] >= 1e18:
        # window found nothing (e.g. right after a reset/replan) -- fall back to
        # a one-off full-path search so it can re-acquire; the monotonic guard
        # below still applies afterward.
        for i in range(len(P) - 1):
            ax, ay = P[i]; bx, by = P[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-9 else max(0.0, min(1.0, ((h0 - ax) * dx + (h1 - ay) * dy) / L2))
            px, py = ax + t * dx, ay + t * dy
            d2 = (h0 - px) ** 2 + (h1 - py) ** 2
            if d2 < best[0]:
                best = (d2, cum[i] + t * math.hypot(dx, dy))
    s_c = best[1]
    # 2) MONOTONIC: progress can only move forward. For a loop, allow the closest
    #    point to be one lap ahead (handles the wrap past the seam) but never let
    #    it slide back behind where we already are.
    # advance the drone's committed projection MONOTONICALLY (never backward)
    if loop and total > 1e-9:
        # accept only a modest forward move (< half a lap); if the nearest point
        # reads as "behind" prog (drone lagged), HOLD -- never lap-jump forward,
        # which would fling the carrot across the loop and reverse direction.
        base = prog - (prog % total)
        new_prog = prog
        for c in (base - total + s_c, base + s_c, base + total + s_c):
            if prog - 1e-9 <= c <= prog + 0.5 * total:
                if new_prog == prog or c < new_prog:
                    new_prog = c
    else:
        new_prog = s_c if s_c > prog else prog        # forward only
        if new_prog > total:
            new_prog = total
    # 3) carrot sits `look` metres ahead of committed progress
    carrot_s = new_prog + look
    if loop and total > 1e-9:
        cx, cy = _path_point_at(P, cum, carrot_s % total)
    else:
        cx, cy = _path_point_at(P, cum, carrot_s)      # clamps to last point
    return [cx, cy], new_prog


def natnet_thread(server_ip, body_id, local_ip, multicast):
    sys.path.insert(0, os.path.join(DIR, "..", "optitrack_natnet"))
    try:
        from NatNetClient import NatNetClient
    except Exception as e:
        with lock: S["err"] = "NatNet import failed: %s" % e
        return

    def rb(idn, pos, rot):
        if idn == body_id:
            with lock:
                S["raw"].update(x=pos[0], y=pos[1], z=pos[2], q=list(rot),
                                valid=True, t=time.time())
                S["frames"] += 1

    try:
        c = NatNetClient()
        c.set_server_address(server_ip)
        if local_ip:
            c.set_client_address(local_ip)
        c.set_use_multicast(multicast)
        c.rigid_body_listener = rb
        c.run()
    except Exception as e:
        with lock: S["err"] = "NatNet run failed: %s" % e
        return
    last = 0
    while RUNNING:
        time.sleep(1.0)
        with lock:
            S["rate"] = S["frames"] - last
            last = S["frames"]
            if S["raw"]["valid"] and (time.time() - S["raw"]["t"]) > 1.0:
                S["raw"]["valid"] = False


def _gain_frame():
    """0xA7 + 21 LE float32 in GAIN_ORDER (matches blimpGuidanceSetGains)."""
    with lock:
        g = dict(S["gains"])
        mode = S["path_mode"]
        taking_off = (S["auto_go"] and not S["takeoff_done"]
                      and mode in ("path", "circle"))
        turn_test = (mode == "heading")
        # WANDER PARK: arrived at the random point -> cut cruise so it coasts to a
        # stop and is SAFE TO PUSH by hand (v5 never stops on its own). Altitude
        # keeps holding. Restored the moment we pick the next point.
        parked = S["wander_state"] in ("arrived", "waitpush")
        # approach taper (see _wander_worker): rolls cruise off BEFORE the target
        wander_scale = S["wander_fwd_scale"] if (S["wander"] and S["wander_state"] == "goto") else 1.0
    # Forward is zeroed in two cases so the craft doesn't translate:
    #   TAKEOFF -- climb straight up (only rotate to pre-aim at waypoint 1)
    #             instead of creeping forward off the floor.
    #   TURN TEST -- spin IN PLACE to the target heading, turning the minimum
    #             (shortest-angle) way, with zero forward drive so it doesn't
    #             drive across the room while it turns. Altitude is held by the
    #             vertical channel (latched hold_alt); if it sinks, raise kpZ/zff.
    # Both restore the cruise the instant the mode/state changes (gains_dirty).
    if taking_off or turn_test or parked:
        g = dict(g); g["fwdMaxN"] = 0.0
    if wander_scale < 0.999:
        g = dict(g); g["fwdMaxN"] = g["fwdMaxN"] * wander_scale
    vals = [float(g[k]) for k in GAIN_ORDER]
    return b"\xA7" + struct.pack("<%df" % len(vals), *vals)


_at_prev = None   # (t, h0, h1, yaw) for the auto-trim drift finite-difference

def _autotrim_carrot(h0, h1, yaw, tx, ty):
    """HOST-SIDE integral drift auto-trim. Finite-difference the mocap pose to get
    the drone's BODY-lateral velocity, integrate it into a learned crab bias, and
    rotate the streamed carrot by that bias so the drone holds a nose crab that
    cancels SUSTAINED drift -- including the turn-coupled 'sideways momentum' that
    centering the balloon can't fix. Deterministic integral, capped. Resets when
    disengaged. If it makes drift WORSE, flip the sign (set at_ki < 0) or toggle off."""
    global _at_prev
    now = time.time()
    with lock:
        ki = S["at_ki"]; icap = S["at_icap"]; ai = S["at_i"]; go = S["auto_go"]
    if not go:
        _at_prev = None
        with lock: S["at_i"] = 0.0
        return tx, ty
    if _at_prev is not None:
        dt = now - _at_prev[0]
        if 1e-3 < dt < 0.5:
            vx = (h0 - _at_prev[1]) / dt; vy = (h1 - _at_prev[2]) / dt
            vlat = -vx * math.sin(yaw) + vy * math.cos(yaw)     # body-lateral (+left)
            ai = max(-icap, min(icap, ai - ki * vlat * dt))     # oppose the drift
            with lock: S["at_i"] = ai
    _at_prev = (now, h0, h1, yaw)
    bearing = math.atan2(ty - h1, tx - h0); rng = math.hypot(tx - h0, ty - h1)
    nb = bearing + ai                                           # rotate carrot by the crab
    return h0 + rng * math.cos(nb), h1 + rng * math.sin(nb)


def _pose_frame():
    """0xA6 + 8 LE float32: cx,cy,cz,cyaw(deg), tx,ty,tz,tyaw(deg)."""
    h0, h1, alt, yaw, valid = mapped()
    tx, ty, tz = _resolve_target()
    with lock: at_on = S["autotrim"]
    if at_on and valid:
        tx, ty = _autotrim_carrot(h0, h1, yaw, tx, ty)          # host-side drift auto-trim
    return (b"\xA6" + struct.pack("<8f", h0, h1, alt, math.degrees(yaw),
                                  tx, ty, tz, 0.0), valid)


def _update_viz():
    """Mirror the on-board path/carrot/hold for the panel display (no control)."""
    h0, h1, alt, yaw, valid = mapped()
    tx, ty, tz = _resolve_target()
    with lock:
        gains = dict(S["gains"]); path = list(S["path"]); path_goal = S["path_goal"]
        holding = S["hold"]
    rng = math.hypot(tx - h0, ty - h1)
    if not valid:
        with lock: S["range"] = round(rng, 3)
        return
    with lock: pmode = S["path_mode"]
    if pmode in ("path", "circle"):
        # The streamed (tx,ty) is ALREADY the carrot (path lookahead / ring lead),
        # so draw it directly -- no straight-line replan mirror.
        bearing = math.atan2(ty - h1, tx - h0)
        with lock:
            S["carrot"] = [tx, ty]; S["hold"] = False
            S["herr"] = math.degrees(_wrap_pi(bearing - yaw)); S["range"] = round(rng, 3)
        return
    replan = gains["replanThresh"]
    moved = (path_goal is None or math.hypot(tx - path_goal[0], ty - path_goal[1]) > replan)
    if not path or moved:
        path = _plan_path(h0, h1, tx, ty, gains["lookahead"])
        path_goal = [tx, ty]
    if rng < gains["arriveR"]:
        holding = True
    elif rng > gains["arriveR"] * gains["holdExit"]:
        if holding:
            path = _plan_path(h0, h1, tx, ty, gains["lookahead"])
            path_goal = [tx, ty]
        holding = False
    # carrot on the straight segment start->goal
    sx, sy = path[0]; gx, gy = path[-1]
    dx = gx - sx; dy = gy - sy; seg2 = dx * dx + dy * dy
    t = 0.0 if seg2 < 1e-9 else _clamp(((h0 - sx) * dx + (h1 - sy) * dy) / seg2, 0.0, 1.0)
    seglen = math.sqrt(seg2)
    tc = _clamp(t + (gains["lookahead"] / seglen if seglen > 1e-9 else 0.0), 0.0, 1.0)
    carrot = [sx + tc * dx, sy + tc * dy]
    bearing = math.atan2(carrot[1] - h1, carrot[0] - h0)
    herr = _wrap_pi(bearing - yaw)
    with lock:
        S["path"] = path; S["path_goal"] = path_goal
        S["carrot"] = (None if holding else carrot); S["hold"] = holding
        S["herr"] = math.degrees(herr); S["range"] = round(rng, 3)


LOG_DIR = os.path.join(DIR, "flight_logs")
_log_fh = None
_log_n = 0
_log_t0 = 0.0
_log_path = None
_log_prev = None                # (t,x,y,z,roll,pitch,yaw) for finite-diff rates

def _log_start():
    """Open a fresh per-run CSV when GO engages. Columns capture WHERE IT IS
    (h0,h1,alt,yaw) vs WHERE IT WANTED TO BE (tx,ty,tz goal + carrot it's
    steering at), plus range, heading error, and the hold flag."""
    global _log_fh, _log_n, _log_t0, _log_path, _log_prev
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(LOG_DIR, "run_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        _log_fh = open(path, "w")
        # existing columns first (readers key by name), then full state + rates:
        # x,y,z = h0,h1,alt ; roll/pitch/yaw ; and every one of their rates.
        _log_fh.write("t_s,h0,h1,alt,yaw_deg,tx,ty,tz,carrot_x,carrot_y,"
                      "range_m,herr_deg,hold,"
                      "roll_deg,pitch_deg,vx,vy,vz,wroll,wpitch,wyaw,"
                      "mL,mR,mUp,mDown\n")   # motor commands (drone telemetry) = INPUT
        _log_n = 0; _log_t0 = time.time(); _log_path = path; _log_prev = None
        print("logging run -> %s" % path)
    except Exception as e:
        _log_fh = None; _log_path = None
        with lock: S["err"] = "log open failed: %s" % e
        return
    # SIDECAR: freeze every live tuning value NEXT TO the CSV, at GO. mocap_config.json
    # is mutable and gets overwritten by the next Save, so after a tuning session there
    # is no way to know which numbers produced which run. This snapshot makes any past
    # run reproducible: read run_<ts>.params.json and put those values back.
    try:
        with lock:
            snap = {
                "run": os.path.basename(path)[:-4],
                "saved_local": time.strftime("%Y-%m-%d %H:%M:%S"),
                "path_mode": S["path_mode"],
                "circle": dict(S["circle"]),
                "target": dict(S["target"]),
                "target_source": S["target_source"],
                "profile": S["profile"],
                "powers": dict(S["powers"]),
                "gains": dict(S["gains"]),
                "yaw_trim": S["yaw_trim"],
                "manual_trim": S["manual_trim"],
                "heading_deg": S["heading_deg"],
                "waypoints": [list(p) for p in S["waypoints"]],
                "wp_loop": S["wp_loop"],
            }
        with open(path[:-4] + ".params.json", "w") as pf:
            json.dump(snap, pf, indent=2, sort_keys=True)
    except Exception as e:                      # never let logging kill a flight
        with lock: S["err"] = "params snapshot failed: %s" % e

def _log_stop():
    global _log_fh, _log_path
    p = _log_path
    try:
        if _log_fh: _log_fh.close()
    except Exception:
        pass
    _log_fh = None
    # Only keep a run that actually FLEW somewhere -- a GO/STOP that never moved
    # (bumped the button, failed takeoff, sat on the floor) just clutters the
    # folder. Discard the CSV and skip the plot unless it travelled MIN_RUN_MOVE.
    if p:
        try:
            if _run_moved(p, MIN_RUN_MOVE):
                _render_run_plot(p)
                _render_state_plot(p)             # full state + rates (sys-ID)
                with lock: turn_mode = (S["path_mode"] == "heading")
                if turn_mode:
                    _render_turn_plot(p)          # turn-test run -> also a turn plot
            else:
                os.remove(p)
                print("run discarded (moved < %.1f m): %s" % (MIN_RUN_MOVE, p))
        except Exception as e:
            print("run plot failed:", e)
    _log_path = None


def _render_run_plot(csv_path):
    """One PNG per run, saved next to the CSV: LEFT = X/Y/Z position error and
    YAW error vs time (all one graph); RIGHT = top-down map of WHERE IT WENT
    (flown trail) vs WHERE IT SHOULD BE (planned waypoints + carrot track)."""
    import csv as _csv
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = []; h0 = []; h1 = []; alt = []; tx = []; ty = []; tz = []
    cx = []; cy = []; herr = []
    with open(csv_path) as f:
        for row in _csv.DictReader(f):
            try:
                t.append(float(row["t_s"])); h0.append(float(row["h0"]))
                h1.append(float(row["h1"])); alt.append(float(row["alt"]))
                tx.append(float(row["tx"])); ty.append(float(row["ty"]))
                tz.append(float(row["tz"])); cx.append(float(row["carrot_x"]))
                cy.append(float(row["carrot_y"])); herr.append(float(row["herr_deg"]))
            except Exception:
                pass
    if len(t) < 3:
        return
    ex = [a - b for a, b in zip(h0, tx)]     # error to where it's steering (carrot/goal)
    ey = [a - b for a, b in zip(h1, ty)]
    ez = [a - b for a, b in zip(alt, tz)]
    with lock:
        wps = [list(p) for p in S["waypoints"]]; wp_loop = S["wp_loop"]
        pmode = S["path_mode"]
        circ = dict(S["circle"]); ctgt = dict(S["target"])
    rms = lambda a: math.sqrt(sum(v * v for v in a) / len(a)) if a else 0.0

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(13, 5.5))
    # LEFT: errors vs time (X/Y/Z on left axis in metres, YAW on right in degrees)
    axA.axhline(0, color="#999", lw=0.8)
    axA.plot(t, ex, color="#e05a5a", lw=1.4, label="X err (m)")
    axA.plot(t, ey, color="#3fa34d", lw=1.4, label="Y err (m)")
    axA.plot(t, ez, color="#3b7dd8", lw=1.4, label="Z err (m)")
    axA.set_xlabel("time (s)"); axA.set_ylabel("position error (m)")
    axA.set_title("Error vs time")
    axY = axA.twinx()
    axY.plot(t, herr, color="#c68b17", lw=1.2, ls="--", alpha=0.9, label="Yaw err (°)")
    axY.set_ylabel("yaw error (deg)", color="#c68b17")
    l1, la1 = axA.get_legend_handles_labels(); l2, la2 = axY.get_legend_handles_labels()
    axA.legend(l1 + l2, la1 + la2, loc="upper right", fontsize=8)
    axA.grid(True, alpha=0.25)
    # RIGHT: top-down flown vs planned (mirrors the 2D panel view)
    axB.plot(h0, h1, color="#1f9fe0", lw=2.2, label="flown (where it went)")
    axB.plot(tx, ty, color="#c68b17", lw=1.0, ls="--", alpha=0.7, label="carrot (aim)")
    # Draw ONLY the path this run actually followed. The waypoint list persists in
    # mocap_config.json across mode switches, so plotting it in circle mode drew a
    # SECOND, unrelated ring (a stale path-mode circle centred on the origin) next to
    # the real one centred on Target X/Y -- two circles, neither labelled as the one
    # being flown. Circle mode has no waypoints; its reference is the ring itself.
    if pmode == "circle":
        rr = max(0.05, circ["r"])
        th = [i * 2.0 * math.pi / 180.0 for i in range(181)]
        axB.plot([ctgt["x"] + rr * math.cos(a) for a in th],
                 [ctgt["y"] + rr * math.sin(a) for a in th],
                 color="#3fa34d", lw=1.6, label="commanded ring (should be)")
        axB.plot([ctgt["x"]], [ctgt["y"]], "+", color="#3fa34d", ms=10)
    elif wps:
        wx = [p[0] for p in wps]; wy = [p[1] for p in wps]
        if wp_loop and len(wps) > 1:
            wx = wx + [wx[0]]; wy = wy + [wy[0]]
        axB.plot(wx, wy, color="#3fa34d", lw=1.6, marker="o", ms=6,
                 label="planned path (should be)")
    axB.plot(h0[0], h1[0], "o", color="#111", ms=8); axB.annotate("start", (h0[0], h1[0]))
    axB.plot(h0[-1], h1[-1], "s", color="#111", ms=8); axB.annotate("end", (h0[-1], h1[-1]))
    axB.set_aspect("equal", "datalim"); axB.set_xlabel("X (m)"); axB.set_ylabel("Y (m)")
    axB.set_title("Flown vs planned (top-down)")
    axB.legend(fontsize=8, loc="best"); axB.grid(True, alpha=0.25)

    # stamp the ring params into the title -- when sweeping lead_m the PNG has to say
    # which value produced it, otherwise the runs are indistinguishable after the fact
    tag = pmode
    if pmode == "circle":
        tag = "circle r=%.2f look_m=%.2f %s" % (
            circ["r"], circ.get("look_m", 1.0),
            "CCW" if circ["dir"] >= 0 else "CW")
    fig.suptitle("%s  [%s]   RMS err  X %.2f  Y %.2f  Z %.2f m   Yaw %.1f°" % (
        os.path.basename(csv_path), tag, rms(ex), rms(ey), rms(ez), rms(herr)),
        fontsize=10)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    png = csv_path[:-4] + ".png" if csv_path.endswith(".csv") else csv_path + ".png"
    fig.savefig(png, dpi=110); plt.close(fig)
    print("run plot -> %s" % png)

def _log_tick():
    """One low-rate (~5 Hz) row into the current run's file. NOTE: ESP-NOW is
    one-way, so the drone's internal turn/forward commands can't be read back —
    but position-vs-goal, the carrot, range, and heading error already show
    whether it's converging, drifting sideways, or turning the wrong way."""
    global _log_n, _log_prev
    if _log_fh is None:
        return
    _log_n += 1
    if _log_n % 2 != 0:                    # 20 Hz tick -> ~10 Hz log (cleaner sys-ID)
        return
    try:
        h0, h1, alt, yaw, valid = mapped()
        with lock:
            r = dict(S["raw"])
        roll, pitch, _y = quat_euler(r.get("q", [0, 0, 0, 1]))
        tx, ty, tz = _resolve_target()
        with lock:
            herr = S["herr"]; rng = S["range"]; hold = S["hold"]; carrot = S["carrot"]
        cx = carrot[0] if carrot else float("nan")
        cy = carrot[1] if carrot else float("nan")
        # finite-difference the 6-DOF pose to get every rate (world-frame linear,
        # body-euler angular). ~5 Hz sampling -- coarse but fine for a slow blimp.
        now = time.time() - _log_t0
        vx = vy = vz = wr = wp = wy = float("nan")
        if _log_prev is not None:
            dt = now - _log_prev[0]
            if dt > 1e-3:
                wrap = lambda a: (a + math.pi) % (2 * math.pi) - math.pi
                vx = (h0 - _log_prev[1]) / dt; vy = (h1 - _log_prev[2]) / dt
                vz = (alt - _log_prev[3]) / dt
                wr = math.degrees(wrap(roll - _log_prev[4])) / dt
                wp = math.degrees(wrap(pitch - _log_prev[5])) / dt
                wy = math.degrees(wrap(yaw - _log_prev[6])) / dt
        _log_prev = (now, h0, h1, alt, roll, pitch, yaw)
        with lock:
            mot = list(S["motors"])
        _log_fh.write("%.2f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%d,"
                      "%.1f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,"
                      "%.0f,%.0f,%.0f,%.0f\n" % (
            now, h0, h1, alt, math.degrees(yaw),
            tx, ty, tz, cx, cy, rng, herr, 1 if hold else 0,
            math.degrees(roll), math.degrees(pitch), vx, vy, vz, wr, wp, wy,
            mot[0], mot[1], mot[2], mot[3]))
        _log_fh.flush()
    except Exception:
        pass


def _manual_tick():
    """One 25 Hz manual step -> (pitch, turn, forward) for the 0xA5 test path.
    Manual hand-fly (WASD/QE) just verifies the bridge moves motors before GO."""
    with lock:
        keys = dict(S["keys"])
        ax = dict(S["axes"])
        # gamepad wins while it's actively reporting; otherwise fall back to keys
        pad = (time.time() - ax.get("t", 0.0)) < AXES_STALE_S
        if pad:
            lvl = ax["fwd"]                       # trigger position IS the throttle
            S["fwd_level"] = lvl                  # keep the readout/ramp in sync
        else:
            target_lvl = 0.0 if keys.get("S") else (1.0 if keys.get("W") else 0.0)
            lvl = S["fwd_level"]
            lvl += _clamp(target_lvl - lvl, -M_FWD_RAMP, M_FWD_RAMP)
            if keys.get("S"):
                lvl = max(0.0, lvl - M_FWD_RAMP)
            S["fwd_level"] = lvl
        pw = dict(S["powers"]); mtrim = S["manual_trim"]; hover = S["hover"]
    forward = lvl * pw["fwd"] * FULL
    if pad:
        # proportional turn: scale by the per-side power so left/right asymmetry
        # tuning still applies, exactly like the key path does.
        turn = ax["turn"] * (pw["right"] if ax["turn"] >= 0 else pw["left"]) * PITCH_MAX
    else:
        turn = ((pw["right"] if keys.get("D") else 0.0) -
                (pw["left"] if keys.get("A") else 0.0)) * PITCH_MAX
    if forward > 1.0:
        turn = _clamp(turn + mtrim * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    if pad:
        vfrac = ax["vert"] * (pw["up"] if ax["vert"] >= 0 else pw["down"])
    else:
        vfrac = (pw["up"] if keys.get("Q") else (-pw["down"] if keys.get("E") else 0.0))
    # HOVER float-assist: hold a baseline up so a buoyancy-short blimp doesn't
    # sink while you hand-fly (open-loop -- Q/E still add on top; tune with Up).
    descending = (ax["vert"] < -0.05) if pad else keys.get("E")
    if hover and not descending:
        vfrac = max(vfrac, HOVER_UP_FRAC * pw["up"])
    pitch = _clamp(M_VERT_SIGN * vfrac * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    return pitch, turn, forward


_telem_buf = bytearray()

def _read_telem(ser):
    """Drain the bridge's USB serial and parse motor-telemetry frames the drone
    sent back: 0xB7 sync + 16 bytes (4 LE float32: fwdL,fwdR,up,down). The bridge
    also prints ASCII status lines on the same port; 0xB7 is non-ASCII so it never
    collides. Updates S['motors']. Non-blocking."""
    global _telem_buf
    try:
        n = ser.in_waiting
        if n:
            _telem_buf += ser.read(n)
    except Exception:
        return
    # keep the buffer bounded; parse every complete 0xB7 frame
    while True:
        i = _telem_buf.find(b"\xB7")
        if i < 0:
            if len(_telem_buf) > 256:
                del _telem_buf[:-1]
            return
        if len(_telem_buf) - i < 17:
            if i > 0:
                del _telem_buf[:i]        # drop leading junk, wait for the rest
            return
        frame = bytes(_telem_buf[i + 1:i + 17])
        del _telem_buf[:i + 17]
        try:
            m = struct.unpack("<4f", frame)
            if all(math.isfinite(v) for v in m):
                with lock:
                    S["motors"] = list(m); S["motors_t"] = time.time()
        except Exception:
            pass


def fly_thread(bridge_port):
    ser = None
    prev_go = False
    prev_ol = False
    while RUNNING:
        with lock:
            flying = S["flying"]
        if flying and ser is None:
            try:
                import serial
                port = bridge_port or find_bridge_port()
                if not port:
                    raise RuntimeError("no C6 bridge serial port (plug in the XIAO C6)")
                ser = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(0.3)
                with lock: S["bridge"] = port; S["err"] = ""; S["gains_dirty"] = True
            except Exception as e:
                with lock: S["err"] = "bridge: %s" % e; S["flying"] = False
                time.sleep(0.4); continue
        if (not flying) and ser is not None:
            _log_stop()                       # end any open run log
            try: ser.close()
            except Exception: pass
            ser = None
            with lock: S["bridge"] = ""
        if flying and ser is not None:
            # WATCHDOG: panel gone (tab closed / asleep) -> stop streaming so the
            # drone's mocap-stale failsafe zeroes the motors.
            with lock:
                gone = (time.time() - S["last_client"]) > 1.5
            if gone:
                with lock:
                    S["flying"] = False; S["auto_go"] = False
                    S["keys"] = {k: False for k in "WSADQE"}; S["fwd_level"] = 0.0
                    S["err"] = "client disconnected -> stopped"
                continue
            with lock:
                go = S["auto_go"]; dirty = S["gains_dirty"]
            try:
                _read_telem(ser)                          # drain drone motor telemetry
                if go:
                    if not prev_go:                       # GO just engaged -> new run log
                        _log_start()
                    if dirty:
                        ser.write(_gain_frame())          # retune the on-board loops
                        with lock: S["gains_dirty"] = False
                    frame, _valid = _pose_frame()
                    ser.write(frame)                      # drone computes + engages
                    _update_viz()
                    _log_tick()                           # per-run CSV for diagnosis
                else:
                    with lock:
                        ol_on = S["ol_active"]; ol = S["ol_cmd"]
                    if prev_go:                           # GO -> STOP: drop the auto latch
                        _log_stop()
                        for _ in range(3):
                            ser.write(b"\xA5" + struct.pack("<ffff", 0, 0, 0, 0))
                            time.sleep(0.01)
                    if ol_on:                             # OPEN-LOOP test (yaw step+coast): LOG it
                        if not prev_ol:
                            _log_start()                  # own run CSV for the open-loop maneuver
                        with lock:
                            hold_z = S["ol_hold_alt"]
                            zbase = S["gains"].get("zff", 11000.0) / PITCH_MAX  # baseline hover lift
                        if hold_z is not None:            # CLOSED-LOOP altitude hold: baseline
                            _hx,_hy,alt_now,_yy,zval = mapped()   # hover lift + P correction, CAPPED
                            err = (hold_z - alt_now) if zval else 0.0   # so it can't run to the ceiling
                            vfrac = _clamp(zbase + Z_HOLD_KP*err, 0.0, zbase + Z_HOLD_CAP)
                            pitch = _clamp(M_VERT_SIGN*vfrac*PITCH_MAX, -PITCH_MAX, PITCH_MAX)
                        else:
                            pitch, _t, _f = _manual_tick()   # fall back to manual hover assist
                        turn, forward = ol if ol else (0.0, 0.0)
                        ser.write(b"\xA5" + struct.pack("<ffff", 0.0, pitch, turn, forward))
                        _log_tick()
                    else:
                        if prev_ol:                       # open-loop test just ended -> close log
                            _log_stop()
                        pitch, turn, forward = _manual_tick()
                        ser.write(b"\xA5" + struct.pack("<ffff", 0.0, pitch, turn, forward))
                    prev_ol = ol_on
            except Exception as e:
                with lock: S["err"] = "bridge write: %s" % e; S["flying"] = False
            prev_go = go
            time.sleep(TICK)
        else:
            prev_go = False; prev_ol = False
            time.sleep(0.05)


# ============================ SYS-ID TEST SEQUENCES ============================
# Two canned, LOGGED maneuvers for the plant fit, driven entirely through the
# existing autonomous path (auto_go + streamed pose/gains). A background worker
# just mutates S over time exactly like the manual actions do; fly_thread streams
# and enforces the KILL/watchdog stop unchanged. Every worker aborts the instant
# the test is superseded (test_id bumped), or GO/flying drops (KILL, watchdog).
def _test_active(my_id):
    with lock:
        return (RUNNING and S["flying"] and S["test_id"] == my_id
                and (S["auto_go"] or S["ol_active"]))   # closed-loop OR open-loop test


def _test_wait(sec, my_id):
    """Sleep in small steps; return False the moment the test is aborted/superseded."""
    end = time.time() + sec
    while time.time() < end:
        if not _test_active(my_id):
            return False
        time.sleep(0.05)
    return _test_active(my_id)


def _test_end(my_id, restore_fwd=None):
    """Finish a test: restore cruise (if we changed it) and drop the auto latch so
    fly_thread's GO->STOP path zeroes the motors. Never stomps a newer test."""
    with lock:
        if S["test_id"] != my_id:
            return
        if restore_fwd is not None:
            S["gains"]["fwdMaxN"] = restore_fwd
        S["auto_go"] = False            # GO->STOP -> motor-zero frames from fly_thread
        S["ol_active"] = False; S["ol_cmd"] = None   # end any open-loop test cleanly
        S["ol_hold_alt"] = None         # drop the probe's altitude hold
        S["test_name"] = ""
        S["gains_dirty"] = True


def _spin_worker(my_id):
    """SPIN IN PLACE: heading mode already forces forward=0, so this is pure yaw.
    Step the heading setpoint one way, the other, then back; the coast-down between
    each step (headErr->0 => turn->0) plus the final motor-cut give clean yaw-drag
    data. Angles are absolute world heading (deg)."""
    _h0, _h1, _alt, yaw, _v = mapped()
    yaw0 = math.degrees(yaw)
    for tgt, dur in ((yaw0 + 130.0, 6.0), (yaw0 - 130.0, 6.0), (yaw0, 4.0)):
        with lock:
            if S["test_id"] != my_id:
                return
            S["heading_deg"] = ((tgt + 180.0) % 360.0) - 180.0
        if not _test_wait(dur, my_id):
            return
    _test_end(my_id)


def _yawstep_worker(my_id, direction):
    """OPEN-LOOP yaw step + coast: drive a FIXED differential (forward off) via the
    manual passthrough so the drone does NOT close its heading loop, then cut to
    zero and let the spin COAST down. 3x. Steady rate -> yaw gain C; the coast-down
    decay -> yaw drag 1/Br, both cleanly (the closed-loop 'spin' test couldn't)."""
    turnmag = 0.6 * PITCH_MAX * direction        # fixed open-loop turn command
    for _ in range(3):
        with lock:
            if S["test_id"] != my_id:
                return
            S["ol_cmd"] = (turnmag, 0.0)          # STEP: constant differential, no forward
        if not _test_wait(5.0, my_id):            # let the yaw rate reach steady state
            return
        with lock:
            if S["test_id"] != my_id:
                return
            S["ol_cmd"] = (0.0, 0.0)               # RELEASE -> coast to a stop
        if not _test_wait(5.0, my_id):            # log the decay
            return
    _test_end(my_id)


def _coast_worker(my_id, cruise):
    """FORWARD-then-COAST x3: aim a point straight ahead of the current nose, drive
    at cruise a few seconds to build speed, then cut fwdMaxN to 0 and let it glide
    straight to a stop. The decay curve pins the forward drag. Re-aims ahead each
    rep so it doesn't wander. Restores the original cruise on exit."""
    # Drive at the CURRENT cruise setting (no forced-fast floor) so the coast-down
    # characterizes drag at the speed actually flown, not an artificially fast one.
    # NOTE: at very slow cruise the natural sideways drift can dominate before the
    # craft commits to a straight line, giving a driftier/noisier glide-down than
    # the old 0.55-floor version -- that's expected, not a bug, at these speeds.
    drive = cruise
    for _ in range(3):
        h0, h1, _alt, yaw, valid = mapped()
        if not valid:
            break
        with lock:
            if S["test_id"] != my_id:
                return
            S["target"]["x"] = h0 + 8.0 * math.cos(yaw)   # aim far ahead -> ~0 heading err, drives straight
            S["target"]["y"] = h1 + 8.0 * math.sin(yaw)
            S["gains"]["fwdMaxN"] = drive                  # DRIVE hard
            S["gains_dirty"] = True
        if not _test_wait(4.5, my_id):                     # longer push -> real forward speed
            return
        with lock:
            if S["test_id"] != my_id:
                return
            S["gains"]["fwdMaxN"] = 0.0                     # CUT -> coast
            S["gains_dirty"] = True
        if not _test_wait(5.0, my_id):                     # glide to a stop (logged)
            return
    _test_end(my_id, restore_fwd=cruise)


def _latency_worker(my_id):
    """CONTROL-LOOP LATENCY PROBE. Sends short alternating yaw KICKS open-loop and
    times how long until the mocap-measured yaw actually starts moving -- i.e. the
    full round trip: panel -> C6 bridge -> ESP-NOW -> drone -> motors -> motion ->
    Motive -> NatNet -> back here. That delay is what an MPC must compensate for; if
    it is large the controller plans against stale reality and can oscillate.
    Measures onset (first yaw-rate above the still-noise floor), median over pulses."""
    import statistics
    mag = 0.6 * PITCH_MAX
    delays = []
    for i in range(6):
        d = 1.0 if i % 2 == 0 else -1.0
        with lock: S["ol_cmd"] = (0.0, 0.0)
        if not _test_wait(1.5, my_id): break                 # settle still
        # baseline still-noise of the yaw-rate
        base = []; prev = None; t_end = time.time() + 0.6
        while time.time() < t_end and _test_active(my_id):
            _h0, _h1, _a, yaw, valid = mapped(); now = time.time()
            if valid and prev and now - prev[0] > 1e-3:
                base.append(abs(_wrap_pi(yaw - prev[1])) / (now - prev[0]))
            prev = (now, yaw); time.sleep(0.02)
        thr = (statistics.mean(base) + 4 * statistics.pstdev(base)) if len(base) > 1 else 0.15
        thr = max(thr, 0.08)                                  # rad/s floor above mocap noise
        # KICK: timestamp the command, watch for the first real motion
        t0 = time.time()
        with lock: S["ol_cmd"] = (mag * d, 0.0)
        onset = None; prev = None
        while time.time() - t0 < 2.0 and _test_active(my_id):
            _h0, _h1, _a, yaw, valid = mapped(); now = time.time()
            if valid and prev and now - prev[0] > 1e-3:
                rate = abs(_wrap_pi(yaw - prev[1])) / (now - prev[0])
                if rate > thr and now - t0 > 0.02:
                    onset = now - t0; break
            prev = (now, yaw); time.sleep(0.01)
        with lock: S["ol_cmd"] = (0.0, 0.0)
        if onset is not None: delays.append(onset)
        with lock:
            if delays: S["latency_ms"] = round(1000 * statistics.median(delays))
        if not _test_wait(2.0, my_id): break                 # let the yaw settle
    with lock:
        if delays:
            S["latency_ms"] = round(1000 * statistics.median(delays))
            S["latency_detail"] = "%d kicks, %d-%d ms range" % (
                len(delays), round(1000 * min(delays)), round(1000 * max(delays)))
        else:
            S["latency_detail"] = "no motion onset detected (check tracking/arming)"
    # LOG the result to a file so it can be read back later (for the lag-aware MPC sim)
    try:
        ts = time.strftime("%Y%m%d_%H%M%S")
        with open(os.path.join(LOG_DIR, "latency_%s.json" % ts), "w") as f:
            json.dump({"ts": ts,
                       "median_ms": S["latency_ms"],
                       "detail": S["latency_detail"],
                       "delays_ms": [round(1000 * x) for x in delays],
                       "n": len(delays)}, f, indent=2)
    except Exception as e:
        with lock: S["err"] = "latency log: %s" % e
    _test_end(my_id)


_PLANT_CACHE = [None]
FWD_SUM_HALF_MAX = 30000.0        # mirrors blimp_guidance.c


def _plant():
    """Identified Sim-B plant coefficients used to PREDICT the flyable curve.
    Fitted from real flight logs (flight_logs/plant_final_20260727.json)."""
    if _PLANT_CACHE[0] is None:
        try:
            with open(os.path.join(DIR, "flight_logs", "plant_final_20260727.json")) as f:
                _PLANT_CACHE[0] = json.load(f)["simB"]
        except Exception:
            _PLANT_CACHE[0] = {"A": 4.0567e-06, "Bu": 0.18279, "rho": 0.88, "d0": 0.0067115,
                               "Bv": 0.30026, "l0": -0.045444, "C": -2.0861e-05,
                               "Br": 0.13158, "Cm": 0.0, "t0": 0.0036744}
    return _PLANT_CACHE[0]


def _predict_curve(h0, h1, yaw, tx, ty, g, T=45.0, dt=0.02, every=10):
    """Roll the ON-BOARD v5 guidance law forward through the identified plant to get
    the CURVED path the blimp can ACTUALLY fly to (tx,ty) -- it cannot turn in place,
    so the real route to a point beside/behind it is an arc, never a straight line.

    Mirrors blimp_guidance.c: pure-pursuit heading (+ driftK crab) -> heading PD ->
    constant-sum differential mixer -> plant. Purely predictive//visual: nothing here
    is streamed to the drone, so a model error costs a slightly-wrong drawing, not
    a bad command.

    dt MUST stay small (the real loop runs ~200 Hz): the kdHead yaw-rate damping is
    stiff, and at dt>~0.05 the discrete lag turns the damping into positive feedback
    and the 'prediction' spirals away instead of converging. `every` decimates the
    returned polyline so the drawing stays light."""
    p = _plant()
    A, Bu, RHO, D0 = p["A"], p["Bu"], p["rho"], p["d0"]
    BV, L0 = p["Bv"], p["l0"]
    C, BR, CM, T0 = p["C"], p["Br"], p["Cm"], p["t0"]
    kpHead = g.get("kpHead", 1.3); kdHead = g.get("kdHead", 0.007)
    turnCap = g.get("turnCap", 0.5); boost = max(0.0, min(1.0, g.get("turnBoost", 0.6)))
    driftK = g.get("driftK", 0.35); fwdMaxN = g.get("fwdMaxN", 0.25)
    fwdMaxPwm = g.get("fwdMaxPwm", 65535.0); turnMaxPwm = g.get("turnMaxPwm", -32767.0)
    # stop the drawing once it's ARRIVED -- with constant cruise it can never park
    # itself, so without this the predicted curve just keeps orbiting the point.
    arrive_r = max(0.2, g.get("arriveR", 0.35))
    x, y, th = h0, h1, yaw
    u = v = r = 0.0                      # start from rest (conservative: widest arc)
    out = [[round(x, 3), round(y, 3)]]
    n = int(T / dt)
    for step in range(n):
        dx, dy = tx - x, ty - y
        rng = math.hypot(dx, dy)
        if rng < arrive_r:
            break
        headGoal = math.atan2(dy, dx)
        # crab into lateral drift (world velocity perpendicular to headGoal)
        vwx = u * math.cos(th) - v * math.sin(th)
        vwy = u * math.sin(th) + v * math.cos(th)
        vPerp = -vwx * math.sin(headGoal) + vwy * math.cos(headGoal)
        headGoal = _wrap_pi(headGoal - driftK * vPerp)
        headErr = _wrap_pi(headGoal - th)
        uTurn = max(-turnCap, min(turnCap, kpHead * headErr - kdHead * math.degrees(r)))
        fwdEach = max(0.0, min(FWD_SUM_HALF_MAX, fwdMaxN * fwdMaxPwm))
        mag = min(FWD_SUM_HALF_MAX, abs(uTurn * turnMaxPwm))
        outside = min(2.0 * FWD_SUM_HALF_MAX, fwdEach + boost * mag)
        inside = fwdEach - mag
        thrust = 0.5 * (outside + inside)
        yawMix = 0.5 * (outside - inside)
        if (uTurn * turnMaxPwm) < 0.0:
            yawMix = -yawMix
        mL = max(0.0, thrust + yawMix); mR = max(0.0, thrust - yawMix)
        Se, De = mL + mR, mL - mR
        du = A * Se - Bu * u + RHO * (r * v) + D0
        dv = -BV * v - (1.0 / RHO) * (r * u) + L0
        dr = C * De - BR * r + CM * (u * v) + T0
        u += dt * du; v += dt * dv; r += dt * dr
        th = _wrap_pi(th + dt * r)
        x += dt * (u * math.cos(th) - v * math.sin(th))
        y += dt * (u * math.sin(th) + v * math.cos(th))
        if step % every == 0:
            out.append([round(x, 3), round(y, 3)])
    if out[-1] != [round(x, 3), round(y, 3)]:
        out.append([round(x, 3), round(y, 3)])
    return out


def _wander_pick(h0, h1, b, minsep, last):
    """A random point inside the bounds, at least `minsep` from here and from the
    previous point (so it actually travels instead of re-picking where it sits)."""
    import random
    best = None
    for _ in range(80):
        tx = random.uniform(b["xmin"], b["xmax"])
        ty = random.uniform(b["ymin"], b["ymax"])
        if math.hypot(tx - h0, ty - h1) < minsep:
            continue
        if last and math.hypot(tx - last[0], ty - last[1]) < minsep:
            continue
        best = [round(tx, 3), round(ty, 3)]
        break
    if best is None:                      # bounds too tight -> just take the far corner
        cx = b["xmin"] if h0 > 0.5 * (b["xmin"] + b["xmax"]) else b["xmax"]
        cy = b["ymin"] if h1 > 0.5 * (b["ymin"] + b["ymax"]) else b["ymax"]
        best = [round(cx, 3), round(cy, 3)]
    return best


class _WanderPush:
    """Rolling push detector, usable in EVERY wander phase -- not just while parked.

    The old detector only ran in `waitpush`, so a shove during the flight to a point,
    or during the settle countdown, was simply ignored: you had to wait for it to
    arrive AND settle before it would notice you. Two triggers, by phase:

      WINDOWED DISPLACEMENT (always, flying or parked) -- how far it actually moved
        over the last `win` seconds. This is what makes a push register mid-flight,
        where the parked tests are useless (it is already moving at cruise).
      SPEED / DISPLACEMENT-FROM-ANCHOR (parked only) -- the original pair, kept
        because a slow deliberate carry never trips the window test.

    WHY A WINDOW AND NOT AN ACCELERATION. Measured over this project's own logs, at
    9.1 Hz mocap the noise floor of double-differentiated position has its MEDIAN at
    0.92 m/s^2 and p99 at 7.8 -- an acceleration trigger anywhere near a real shove
    fires on roughly half of all samples. Even instantaneous speed has p90 = 0.63 m/s
    against a 0.26 m/s cruise. Differencing two positions `win` apart divides the same
    noise by a much larger interval AND averages it, so the discriminator is stable:
    under power the craft covers ~0.26*win m, a hand moves it far more.

    All timing is against the mocap sample's own timestamp; a stale sample is skipped
    rather than treated as new data.
    """

    def __init__(self, ax, ay):
        self.ax, self.ay = ax, ay      # anchor for the parked displacement test
        self.buf = []                  # [(t, x, y), ...] trailing window
        self.prev = None               # (t, x, y) for the parked speed test
        self.fast = 0
        self.big = 0                   # consecutive over-threshold windows
        self.speed = 0.0               # LIVE speed, reused by the approach taper
        self.last_hit = None           # (x, y) where the last detection fired -> plotted

    def reset(self, ax, ay):
        self.ax, self.ay = ax, ay
        self.buf = []; self.prev = None; self.fast = 0; self.big = 0

    def check(self, h0, h1, t_sample, valid, parked, push_m, push_v, jump_m, win_s,
              muted=False):
        """Returns a reason string if a push is detected, else None.

        `muted` keeps the buffers running but suppresses firing -- used for the
        refractory window after a push so the craft's own coast cannot re-trigger."""
        if not valid:
            return None
        # only advance on a genuinely NEW mocap sample
        if self.buf and t_sample <= self.buf[-1][0] + 1e-6:
            return None
        p = self.prev
        spd = None
        if p is not None:
            dt = t_sample - p[0]
            if dt > 1e-6:
                spd = math.hypot(h0 - p[1], h1 - p[2]) / dt
        # GLITCH GATE. A sample implying >3 m/s did not happen -- this blimp cruises at
        # 0.26 and tops out under 1. Such samples used to land in the window buffer and
        # were the whole tail of the false-fire distribution (one run showed a 1.77 m
        # "move" in 0.6 s = 2.9 m/s). Drop them entirely instead of measuring them.
        if spd is not None and spd > 3.0:
            return None
        if spd is not None:
            self.speed = spd
            if parked and not muted:
                self.fast = self.fast + 1 if spd > push_v else 0
                if self.fast >= 2:               # 2 samples: one glitch can't trip it
                    self.prev = (t_sample, h0, h1)
                    self.last_hit = (h0, h1)
                    return "shove %.2f m/s" % spd
        self.prev = (t_sample, h0, h1)
        if muted:
            self.fast = 0; self.big = 0; self.ax, self.ay = h0, h1
        self.buf.append((t_sample, h0, h1))
        while len(self.buf) > 2 and t_sample - self.buf[0][0] > win_s:
            self.buf.pop(0)
        # windowed displacement -- needs a full window before it can judge
        t0, x0, y0 = self.buf[0]
        if t_sample - t0 >= win_s * 0.8:
            jump = math.hypot(h0 - x0, h1 - y0)
            # CONFIRMATION. Require two consecutive windows over threshold. A real shove
            # keeps the craft displaced; a lone bad sample does not survive the next one.
            if jump > jump_m and not muted:
                self.big = getattr(self, "big", 0) + 1
                if self.big >= 2:
                    self.big = 0
                    self.buf = [(t_sample, h0, h1)]   # re-arm, don't re-fire same shove
                    self.last_hit = (h0, h1)
                    return "pushed %.2f m in %.1fs" % (jump, t_sample - t0)
            else:
                self.big = 0
        if parked and not muted:
            disp = math.hypot(h0 - self.ax, h1 - self.ay)
            if disp > push_m:
                self.last_hit = (h0, h1)
                return "moved %.2f m" % disp
        return None


def _wander_note_push(det, why, tag):
    """Record a detected push: readout, refractory deadline, and a marker for the view."""
    with lock:
        S["wander_push_dbg"] = why + tag
        S["wander_push_mute_until"] = time.time() + S["wander_push_lock_s"]
        if det.last_hit:
            S["wander_push_marks"] = (S["wander_push_marks"] +
                                      [[round(det.last_hit[0], 3),
                                        round(det.last_hit[1], 3), why]])[-8:]


def _wander_worker(my_id):
    """RANDOM-POINT WANDER with a physical hand-off:
         goto  -> fly to a random point (on-board pursuit; curve is predicted+drawn)
         arrived -> cruise cut to 0 (coast to a stop, SAFE TO PUSH); the DRONE's own
                    LED flashes green (firmware: range < arriveR) as the 'push me' cue
         waitpush -> once settled, watch for a hand push (moved > push_m); on push,
                     pick the next point and go again.
    Everything except the LED is host-side; the on-board controller is untouched."""
    last_pt = None
    while True:
        with lock:
            if S["wander_id"] != my_id or not S["wander"] or not RUNNING:
                break
            b = dict(S["wander_bounds"]); minsep = S["wander_min_sep"]
            z = S["wander_z"]; hold_s = S["wander_hold_s"]; push_m = S["wander_push_m"]
            push_v = S["wander_push_v"]
            wmode = S["wander_mode"]; queue = [list(p) for p in S["wander_queue"]]
            qi = S["wander_qi"]
            g = dict(S["gains"])
        h0, h1, _alt, yaw, valid = mapped()
        if not valid:
            time.sleep(0.3); continue
        # ---- pick + commit a new target ----
        if wmode == "list":
            if not queue:                       # nothing clicked yet -> idle politely
                with lock:
                    S["wander_state"] = "idle"
                    S["wander_target"] = None; S["wander_predicted"] = []
                    S["err"] = "wander: click points on the view (or switch to Random)"
                time.sleep(0.4); continue
            tgt = queue[qi % len(queue)]
            with lock:
                S["wander_qi"] = (qi + 1) % len(queue)
                if S["err"].startswith("wander: click"):
                    S["err"] = ""
        else:
            tgt = _wander_pick(h0, h1, b, minsep, last_pt)
        last_pt = tgt
        curve = _predict_curve(h0, h1, yaw, tgt[0], tgt[1], g)
        with lock:
            if S["wander_id"] != my_id:
                break
            S["wander_target"] = tgt
            S["wander_predicted"] = curve
            S["wander_state"] = "goto"
            S["target_source"] = "manual"
            S["path_mode"] = "point"          # direct pursuit -> the achievable arc
            S["target"]["x"] = tgt[0]; S["target"]["y"] = tgt[1]; S["target"]["z"] = z
            S["auto_go"] = True; S["flying"] = True
            S["gains_dirty"] = True           # restore cruise after a park
        # ---- fly there ----
        t_start = time.time()
        skipped = False; pushed = None
        last_scale = [1.0]
        _sx, _sy, _sa, _sy2, _sv = mapped()
        det = _WanderPush(_sx, _sy)
        while True:
            with lock:
                if S["wander_id"] != my_id or not S["wander"]:
                    return
                arrive_r = max(0.25, S["gains"].get("arriveR", 0.35))
                slow_m = S["wander_slow_m"]
                coast_s = S["wander_coast_s"]
                jump_m = S["wander_push_jump"]; win_s = S["wander_push_win"]
                muted = time.time() < S["wander_push_mute_until"]
                push_m_l = S["wander_push_m"]; push_v_l = S["wander_push_v"]
                cruise_n = S["gains"].get("fwdMaxN", 0.25)
                if S["wander_next"]:            # "Next point now" works in ANY phase
                    S["wander_next"] = False; skipped = True
            if skipped:
                break
            h0, h1, _alt, yaw, valid = mapped()
            with lock: t_sample = S["raw"]["t"]
            # A SHOVE ABORTS THE FLIGHT. Previously a push here was ignored -- you had
            # to let it reach the point and finish settling before it would react.
            # Acceleration is the only trigger that works while it is under power.
            why = det.check(h0, h1, t_sample, valid, False,
                            push_m_l, push_v_l, jump_m, win_s, muted)
            if why:
                pushed = why
                break
            if valid:
                rng = math.hypot(tgt[0] - h0, tgt[1] - h1)
                if rng < arrive_r:
                    break
                # APPROACH ON A COAST-PLANNED SPEED PROFILE. The old taper rolled cruise
                # off linearly over the last `slow_m` metres, which ignores how fast it is
                # actually going -- arrive at that ring carrying speed and it sails on
                # through, because a blimp cannot brake (cutting thrust it still coasts
                # v*tau). Instead pick the speed it may hold HERE so it can stop in the
                # ring, v_des = (range - arriveR)/coast_s, and scale cruise to that.
                v_cruise = max(0.05, CARROT_SPEED_K * cruise_n)
                v_des = (rng - arrive_r) / max(0.5, coast_s)
                sc = _clamp(v_des / v_cruise, 0.0, 1.0)
                # NOTE: deliberately NOT min()'d with the old (rng-arriveR)/slow_m taper.
                # Doing so made this whole profile dead code -- at cruise 0.26 m/s the
                # coast distance is v*coast_s, and whenever that is under slow_m the
                # linear taper is the tighter of the two and wins everywhere. coast_s is
                # now the single knob: bigger = starts slowing earlier and harder.
                # SPEED FEEDBACK: already faster than the profile allows -> thrust off
                # entirely and let drag do the braking. This is what actually kills the
                # overshoot; an open-loop taper cannot know it came in hot.
                if det.speed > v_des * 1.3:
                    sc = 0.0
                if abs(sc - last_scale[0]) > 0.05:
                    last_scale[0] = sc
                    with lock:
                        S["wander_fwd_scale"] = round(sc, 3); S["gains_dirty"] = True
            if time.time() - t_start > S["wander_timeout_s"]:
                break                          # couldn't reach it -> park anyway, then retry
            time.sleep(0.15)
        with lock:
            S["wander_fwd_scale"] = 1.0; S["gains_dirty"] = True
        if pushed:                              # shoved mid-flight -> new point NOW
            _wander_note_push(det, pushed, " (in flight)")
            continue
        if skipped:
            continue                            # straight on to the next point
        # ---- arrived: park (cruise->0 via _gain_frame), LED flashes on the drone ----
        with lock:
            if S["wander_id"] != my_id:
                break
            S["wander_state"] = "arrived"; S["gains_dirty"] = True
        t_park = time.time(); skipped = False; pushed = None
        _ax, _ay, _aa, _ay2, _av = mapped()
        det.reset(_ax, _ay)
        while time.time() - t_park < hold_s:   # settle so its own coast can't look like a push
            with lock:
                if S["wander_id"] != my_id or not S["wander"]:
                    return
                jump_m = S["wander_push_jump"]; win_s = S["wander_push_win"]
                muted = time.time() < S["wander_push_mute_until"]
                push_m_l = S["wander_push_m"]; push_v_l = S["wander_push_v"]
                if S["wander_next"]:           # skip the settle too
                    S["wander_next"] = False; skipped = True
                S["wander_hold_left"] = round(hold_s - (time.time() - t_park), 1)
            if skipped:
                break
            # A shove DURING the settle countdown now counts. Only the acceleration
            # trigger is armed here (parked=False): the craft is still bleeding off its
            # own approach momentum, and that coast is exactly what used to false-fire
            # the speed test the instant it settled.
            h0, h1, _a2, _y2, valid = mapped()
            with lock: t_sample = S["raw"]["t"]
            why = det.check(h0, h1, t_sample, valid, False,
                            push_m_l, push_v_l, jump_m, win_s, muted)
            if why:
                pushed = why
                break
            time.sleep(0.1)
        if pushed:                              # shoved while settling -> new point NOW
            with lock: S["wander_hold_left"] = 0.0
            _wander_note_push(det, pushed, " (settling)")
            continue
        if skipped:
            with lock: S["wander_hold_left"] = 0.0
            continue
        # ---- wait for a HAND PUSH (or the panel's Next button) ----
        px, py, _a, _y, _v = mapped()
        with lock:
            if S["wander_id"] != my_id:
                break
            S["wander_state"] = "waitpush"; S["wander_hold_left"] = 0.0
            S["wander_next"] = False; S["wander_push_dbg"] = ""
        # PUSH DETECTION, parked. All three triggers armed here (see _WanderPush):
        # acceleration, speed-for-2-samples, and displacement from the park anchor.
        # The same detector has already been running through the goto and settle
        # phases on acceleration alone, so a shove no longer has to wait for this
        # phase to be reached before it counts.
        det.reset(px, py)
        while True:
            with lock:
                if S["wander_id"] != my_id or not S["wander"]:
                    return
                forced = S["wander_next"]
                jump_m = S["wander_push_jump"]; win_s = S["wander_push_win"]
                muted = time.time() < S["wander_push_mute_until"]
            if forced:
                with lock: S["wander_push_dbg"] = "panel button"
                break
            h0, h1, _a, _y, valid = mapped()
            with lock:
                t_sample = S["raw"]["t"]
            # Parked: all three triggers armed (acceleration + speed + displacement).
            why = det.check(h0, h1, t_sample, valid, True, push_m, push_v, jump_m, win_s, muted)
            if why:
                _wander_note_push(det, why, "")
                break
            time.sleep(0.08)
    with lock:
        if S["wander_id"] == my_id:
            S["wander"] = False; S["wander_state"] = "idle"
            S["wander_predicted"] = []; S["gains_dirty"] = True


def _start_test(kind):
    """Kick off (or abort) a sys-ID test. Engages autonomous, latches the current
    altitude to hold, and spawns the timed worker. 'abort' just supersedes + stops."""
    if kind == "latency":
        _h0, _h1, _alt, _yaw, valid = mapped()
        if not valid:
            with lock: S["err"] = "latency: no mocap tracking -- can't start"
            return
        with lock:
            S["test_id"] += 1; my_id = S["test_id"]
            S["flying"] = True; S["auto_go"] = False          # open-loop manual passthrough
            S["ol_active"] = True; S["ol_cmd"] = (0.0, 0.0)
            S["hover"] = False                                # NOT the runaway constant-up hover
            S["ol_hold_alt"] = Z_PROBE_TARGET                 # auto-hold 1.5 m closed-loop instead
            S["test_name"] = "latency probe"
            S["latency_ms"] = None; S["latency_detail"] = "measuring..."
        threading.Thread(target=_latency_worker, args=(my_id,), daemon=True).start()
        return
    if kind == "abort":
        with lock:
            S["test_id"] += 1; S["test_name"] = ""; S["auto_go"] = False
            S["ol_active"] = False; S["ol_cmd"] = None; S["ol_hold_alt"] = None
        return
    if kind == "yawstep":
        _h0, _h1, _alt, _yaw, valid = mapped()
        if not valid:
            with lock: S["err"] = "test: no mocap tracking -- can't start"
            return
        with lock:
            S["test_id"] += 1; my_id = S["test_id"]
            S["flying"] = True; S["auto_go"] = False   # open-loop -> manual passthrough
            S["ol_active"] = True; S["ol_cmd"] = (0.0, 0.0)
            S["hover"] = False                         # NOT the runaway constant-up hover
            S["ol_hold_alt"] = _alt                    # hold THIS altitude closed-loop instead
            direction = 1.0 if S["ol_dir"] >= 0 else -1.0
            S["test_name"] = "yaw-step+coast"
        threading.Thread(target=_yawstep_worker, args=(my_id, direction), daemon=True).start()
        return
    if kind not in ("spin", "coast"):
        return
    _h0, _h1, alt, _yaw, valid = mapped()
    if not valid:
        with lock: S["err"] = "test: no mocap tracking -- can't start"
        return
    with lock:
        S["test_id"] += 1; my_id = S["test_id"]
        cruise = S["gains"].get("fwdMaxN", 0.25)
        if cruise < 0.05:                # was a turn-only/0 cruise -> use a sane default
            cruise = 0.25
        S["ol_active"] = False; S["ol_cmd"] = None    # ensure open-loop path is off
        S["flying"] = True
        S["takeoff_done"] = True         # skip the climb; hold current altitude
        S["hold_alt"] = alt; S["target"]["z"] = alt
        S["alt_go"] = alt; S["t_go"] = time.time()
        S["path"] = []; S["carrot"] = None; S["hold"] = False; S["path_prog"] = 0.0
        if kind == "spin":
            S["path_mode"] = "heading"   # forward forced to 0 in _gain_frame
            S["test_name"] = "spin-in-place"
        else:
            S["path_mode"] = "point"     # drives toward the streamed target (forward live)
            S["target_source"] = "manual"
            S["test_name"] = "forward+coast"
        S["auto_go"] = True; S["gains_dirty"] = True
    worker = _spin_worker if kind == "spin" else _coast_worker
    args = (my_id,) if kind == "spin" else (my_id, cruise)
    threading.Thread(target=worker, args=args, daemon=True).start()


def handle(d):
    a = d.get("action")
    # Flashing shares the USB port, so drop the bridge link first, then open a
    if a == "test":
        _start_test(d.get("kind"))       # sys-ID: 'coast' | 'spin' | 'abort'
        return {"ok": True}
    with lock:
        if a == "fly":
            S["flying"] = bool(d.get("on"))
        elif a == "kill":
            S["flying"] = False; S["auto_go"] = False; S["hover"] = False
            S["ol_active"] = False; S["ol_cmd"] = None; S["ol_hold_alt"] = None
            S["keys"] = {k: False for k in "WSADQE"}; S["fwd_level"] = 0.0
        elif a == "keys":
            kk = d.get("keys") or {}
            for k in "WSADQE":
                if k in kk:
                    S["keys"][k] = bool(kk[k])
        elif a == "axes":
            # ANALOG (gamepad) manual input: proportional throttle/turn/vertical in
            # -1..1 (forward 0..1). Stamped so _manual_tick can fall back to the
            # keyboard the moment the pad stops reporting (unplugged / tab blurred).
            try:
                S["axes"]["fwd"] = _clamp(float(d.get("fwd", 0.0)), 0.0, 1.0)
                S["axes"]["turn"] = _clamp(float(d.get("turn", 0.0)), -1.0, 1.0)
                S["axes"]["vert"] = _clamp(float(d.get("vert", 0.0)), -1.0, 1.0)
                S["axes"]["t"] = time.time()
                S["axes"]["name"] = str(d.get("name", ""))[:40]
            except Exception:
                pass
        elif a == "hover":
            S["hover"] = bool(d.get("on", not S["hover"]))
        elif a == "ol_dir":
            try: S["ol_dir"] = 1.0 if float(d.get("value", 1)) >= 0 else -1.0
            except Exception: pass
        elif a == "autotrim":
            S["autotrim"] = bool(d.get("on", not S["autotrim"]))
            if not S["autotrim"]: S["at_i"] = 0.0     # drop the learned crab when off
        elif a == "at_ki":
            try: S["at_ki"] = float(d.get("value"))
            except Exception: pass
        elif a == "wander":
            on = bool(d.get("on", not S["wander"]))
            S["wander_id"] += 1
            S["wander"] = on
            if on:
                S["wander_state"] = "goto"; S["wander_next"] = False
                S["takeoff_done"] = True      # already airborne when you start wandering
                myid = S["wander_id"]
                threading.Thread(target=_wander_worker, args=(myid,), daemon=True).start()
            else:
                S["wander_state"] = "idle"; S["wander_predicted"] = []
                S["auto_go"] = False; S["gains_dirty"] = True
        elif a == "wander_next":
            S["wander_next"] = True           # skip the push wait, go to the next point
        elif a == "wander_mode":
            m = d.get("mode")
            if m in ("random", "list"):
                S["wander_mode"] = m; S["wander_qi"] = 0
        elif a == "wander_add_pt":
            try:
                S["wander_queue"].append([round(float(d["x"]), 3), round(float(d["y"]), 3)])
            except Exception: pass
        elif a == "wander_undo_pt":
            if S["wander_queue"]:
                S["wander_queue"].pop()
                S["wander_qi"] = 0
        elif a == "wander_clear_pts":
            S["wander_queue"] = []; S["wander_qi"] = 0
        elif a == "wander_cfg":
            for k in ("wander_z", "wander_hold_s", "wander_min_sep",
                      "wander_push_m", "wander_push_v", "wander_slow_m",
                      "wander_timeout_s"):
                if k in d:
                    try: S[k] = float(d[k])
                    except Exception: pass
            bnd = d.get("bounds")
            if isinstance(bnd, dict):
                for k in ("xmin", "xmax", "ymin", "ymax"):
                    if k in bnd:
                        try: S["wander_bounds"][k] = float(bnd[k])
                        except Exception: pass
        elif a == "save_rec":
            _save_manual_rec(d.get("samples") or [])
        elif a == "yawsign":
            S["gains"]["turnMaxPwm"] = -S["gains"]["turnMaxPwm"]   # flip turn dir on drone
            S["gains_dirty"] = True
        elif a == "trim":
            try: S["manual_trim"] = _clamp(float(d.get("value")), -0.5, 0.5)
            except Exception: pass
        elif a == "power":
            k = d.get("name")
            if k in S["powers"]:
                try: S["powers"][k] = _clamp(float(d.get("value")), 0.0, 1.0)
                except Exception: pass
        elif a == "goto":
            S["auto_go"] = not S["auto_go"]
            S["path"] = []; S["path_goal"] = None; S["carrot"] = None; S["hold"] = False
            S["takeoff_done"] = False   # each GO: climb to altitude before moving
            S["path_prog"] = 0.0        # restart path progress from the beginning
            S["path_t"] = time.time()   # reset the virtual-target clock
            if S["auto_go"]:
                S["flying"] = True; S["gains_dirty"] = True
                # latch current altitude so the turn-test / heading mode holds it,
                # and seed the takeoff climb ramp (start altitude + start time)
                r = S["raw"]; S["hold_alt"] = (r["y"] if S["up_axis"] == "Y" else r["z"])
                S["alt_go"] = S["hold_alt"]; S["t_go"] = time.time()
        elif a == "gain":
            k = d.get("name")
            if k in S["gains"]:
                try:
                    S["gains"][k] = float(d.get("value")); S["gains_dirty"] = True
                except Exception: pass
        elif a == "profile":
            name = d.get("name")
            if name in FRAME_PROFILES and name != S["profile"]:
                S["profiles"][S["profile"]] = _current_tuning()
                S["profile"] = name
                _apply_tuning(S["profiles"].get(name, {}))
                S["gains_dirty"] = True
        elif a == "upaxis":
            if d.get("axis") in ("Y", "Z"):
                S["up_axis"] = d["axis"]
        elif a == "target":
            for k in ("x", "y", "z"):
                if k in d:
                    try: S["target"][k] = float(d[k])
                    except Exception: pass
        elif a == "path_mode":
            if d.get("mode") in ("point", "circle", "heading", "path"):
                S["path_mode"] = d["mode"]
                S["path"] = []; S["path_goal"] = None; S["carrot"] = None
                S["path_prog"] = 0.0
                S["gains_dirty"] = True    # heading mode zeros fwdMaxN in the frame
        elif a == "add_wp":
            try:
                S["waypoints"].append([float(d["x"]), float(d["y"])])
                S["path_prog"] = 0.0
            except Exception: pass
        elif a == "undo_wp":
            if S["waypoints"]:
                S["waypoints"].pop()
            S["path_prog"] = 0.0
        elif a == "clear_wp":
            S["waypoints"] = []
            S["path_prog"] = 0.0
        elif a == "set_wps":                            # replace waypoints wholesale (circle etc.)
            try:
                pts = d.get("pts") or []
                S["waypoints"] = [[float(p[0]), float(p[1])] for p in pts][:400]
                S["path_prog"] = 0.0
            except Exception: pass
        elif a == "wp_loop":
            S["wp_loop"] = bool(d.get("on", not S["wp_loop"]))
        elif a == "heading_deg":
            try:
                S["heading_deg"] = ((float(d.get("value")) + 180.0) % 360.0) - 180.0
            except Exception: pass
        elif a == "heading_here":
            r = S["raw"]; up = S["up_axis"]
            yd = math.degrees(quat_yaw(r.get("q", [0, 0, 0, 1]), up))
            S["heading_deg"] = ((yd + 180.0) % 360.0) - 180.0
        elif a == "zero_heading":
            # CALIBRATE THE NOSE. Point the blimp at the Target X/Y marker, click Zero:
            # yaw_trim becomes whatever makes the heading error read 0 right now, i.e.
            # the constant offset between the rigid body's reported yaw and the actual
            # nose. Computed from raw S here under the lock already held -- must NOT
            # call mapped()/_resolve_target(), which take the lock again.
            r = dict(S["raw"]); up = S["up_axis"]
            if not r.get("valid", False):
                S["err"] = "Zero heading: no mocap tracking"
            else:
                zh0, zh1, _za = _map_raw(r, up)
                raw_yaw = quat_yaw(r.get("q", [0, 0, 0, 1]), up)
                brg = math.atan2(S["target"]["y"] - zh1, S["target"]["x"] - zh0)
                if math.hypot(S["target"]["x"] - zh0, S["target"]["y"] - zh1) < 0.25:
                    S["err"] = "Zero heading: too close to the target to get a bearing"
                else:
                    S["yaw_trim"] = math.degrees(_wrap_pi(brg - raw_yaw))
                    S["gains_dirty"] = True
                    S["err"] = "heading zeroed: yaw_trim = %.1f deg" % S["yaw_trim"]
        elif a == "circle":
            for k in ("r", "lead", "dir", "lead_m", "look_m"):
                if k in d:
                    try: S["circle"][k] = float(d[k])
                    except Exception: pass
    if a in ("trim", "kill", "power", "gain", "profile", "yawsign",
             "target", "target_source",
             "path_mode", "circle", "heading_deg", "heading_here", "zero_heading",
             "add_wp", "undo_wp", "clear_wp", "set_wps", "wp_loop"):
        save_trim()
    return {"ok": True}


def state_payload():
    h0, h1, alt, yaw, valid = mapped()
    with lock:
        return {
            "raw": dict(S["raw"]),
            "mapped": {"h0": round(h0, 3), "h1": round(h1, 3),
                       "alt": round(alt, 3), "yaw": round(math.degrees(yaw), 1)},
            "target": dict(S["target"]), "target_source": S["target_source"],
            "path_mode": S["path_mode"], "circle": dict(S["circle"]),
            "waypoints": [list(p) for p in S["waypoints"]], "wp_loop": S["wp_loop"],
            "smooth_path": list(S["smooth_path"]),
            "heading_deg": round(S["heading_deg"], 1),
            "up_axis": S["up_axis"], "flying": S["flying"],
            "keys": dict(S["keys"]), "fwd_level": round(S["fwd_level"], 2),
            "hover": S["hover"],
            "manual_trim": round(S["manual_trim"], 3),
            "powers": dict(S["powers"]), "profile": S["profile"],
            "auto_go": S["auto_go"], "hold": S["hold"], "test": S["test_name"],
            "ol_dir": S["ol_dir"],
            "autotrim_build": S["autotrim_build"], "autotrim": S["autotrim"],
            "at_ki": S["at_ki"], "at_i": round(S["at_i"], 3),
            "manual_build": S["manual_build"],
            "pad": {"live": (time.time() - S["axes"]["t"]) < AXES_STALE_S,
                    "name": S["axes"]["name"], "fwd": round(S["axes"]["fwd"], 2),
                    "turn": round(S["axes"]["turn"], 2), "vert": round(S["axes"]["vert"], 2)},
            "wander_build": S["wander_build"], "wander": S["wander"],
            "wander_state": S["wander_state"], "wander_target": S["wander_target"],
            "wander_predicted": S["wander_predicted"],
            "wander_hold_left": S["wander_hold_left"],
            "wander_bounds": dict(S["wander_bounds"]), "wander_z": S["wander_z"],
            "wander_hold_s": S["wander_hold_s"], "wander_min_sep": S["wander_min_sep"],
            "wander_push_m": S["wander_push_m"], "wander_push_v": S["wander_push_v"],
            "wander_mode": S["wander_mode"], "wander_queue": [list(p) for p in S["wander_queue"]],
            "wander_qi": S["wander_qi"], "wander_push_dbg": S["wander_push_dbg"],
            "wander_slow_m": S["wander_slow_m"], "wander_fwd_scale": S["wander_fwd_scale"],
            "latency_ms": S["latency_ms"], "latency_detail": S["latency_detail"],
            "herr": round(S["herr"], 1), "range": S["range"],
            "path": [[round(p[0], 3), round(p[1], 3)] for p in S["path"]],
            "carrot": ([round(S["carrot"][0], 3), round(S["carrot"][1], 3)]
                       if S["carrot"] else None),
            "gains": dict(S["gains"]), "gain_spec": GAIN_SPEC,
            "rate": S["rate"], "valid": valid,
            "yaw_trim": S["yaw_trim"],      # feeds the Zero-heading readout
            "wander_push_marks": [list(m) for m in S["wander_push_marks"]],
            "bridge": S["bridge"], "err": S["err"],
        }


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, body, ctype):
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, must-revalidate")   # never serve a stale panel
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        with lock: S["last_client"] = time.time()
        if self.path in ("/", "/auto_panel.html", "/manual_panel.html", "/wander_panel.html"):
            with lock:
                is_manual = S["manual_build"]; is_wander = S["wander_build"]
            fname = ("wander_panel.html" if is_wander else
                      "manual_panel.html" if is_manual else "auto_panel.html")
            self._send(open(os.path.join(DIR, fname), "rb").read(), "text/html")
        elif self.path == "/state":
            self._send(json.dumps(state_payload()).encode(), "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        with lock: S["last_client"] = time.time()
        if self.path == "/api":
            n = int(self.headers.get("Content-Length", 0))
            self._send(json.dumps(handle(json.loads(self.rfile.read(n) or b"{}"))).encode(),
                       "application/json")
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description="On-board autonomous blimp panel (streams pose+gains).")
    ap.add_argument("--server", required=True, help="OptiTrack/Motive PC IP")
    ap.add_argument("--body", type=int, required=True, help="blimp rigid-body Streaming ID")
    ap.add_argument("--local", default=None, help="this Mac's IP (auto if omitted)")
    ap.add_argument("--unicast", action="store_true", help="NatNet unicast (default multicast)")
    ap.add_argument("--bridge-port", default=None, help="C6 serial port (auto if omitted)")
    ap.add_argument("--up", default="Z", choices=["Y", "Z"], help="initial up axis")
    ap.add_argument("--autotrim", action="store_true",
                    help="enable host-side integral drift auto-trim (separate panel build)")
    ap.add_argument("--manual", action="store_true",
                    help="dedicated manual-only hand-fly page (no autonomy/tests)")
    ap.add_argument("--wander", action="store_true",
                    help="random-point wander page: fly to a point, park + flash the "
                         "drone LED, wait for a hand push, go to the next point")
    ap.add_argument("--port", type=int, default=None, help="HTTP port (default 8601)")
    args = ap.parse_args()
    load_trim()

    global PORT
    if args.port:
        PORT = args.port
    with lock:
        S["up_axis"] = args.up
        S["manual_build"] = args.manual
        S["wander_build"] = args.wander
        S["autotrim_build"] = args.autotrim
        S["autotrim"] = args.autotrim
    local = args.local
    if not local:
        try:
            import socket
            sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sk.connect((args.server, 80)); local = sk.getsockname()[0]; sk.close()
        except Exception:
            local = None

    threading.Thread(target=natnet_thread,
                     args=(args.server, args.body, local, not args.unicast),
                     daemon=True).start()
    threading.Thread(target=fly_thread, args=(args.bridge_port,), daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    url = "http://127.0.0.1:%d" % PORT
    print("Mocap panel at %s  (blimp #%d @ %s, local %s)" %
          (url, args.body, args.server, local))
    print("The DRONE computes guidance; this panel only streams pose + gains.")
    print("Keep this window open. Ctrl-C to stop.")
    try: webbrowser.open(url)
    except Exception: pass
    try: srv.serve_forever()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
