#!/usr/bin/env python3
"""
straight_panel_server.py — THE autonomous flight tool for the ESP-FLY blimp.
NatNet mocap -> Mac computes everything -> streams manual-domain 0xA5 setpoints
to the drone over the C6 ESP-NOW bridge. No on-board guidance, no reflash --
the same proven mixing that works hand-flown, just closed-loop.

  OptiTrack --lab WiFi--> Mac (this server) --USB--> C6 --ESP-NOW--> blimp

TWO tracked rigid bodies:
  --body       the blimp
  --goal-body  a GOAL MARKER placed in the lab (default 502). Auto mode flies
               to wherever the marker IS, live -- move the marker, the target
               follows. Toggle to manual Target X/Y boxes in the panel.

============================ THE PATH CONTROLLER ===========================
A forward-only blimp CANNOT rotate in place: the unidirectional props always
push it forward while turning. Fighting that (rotate-to-face, THEN drive) just
makes it lurch forward every time it tries to turn. So we don't fight it -- we
PURSUE: always move, and steer toward the target so the PATH curves to the
point (a pursuit trajectory), embracing the forward-while-turning coupling.

  steer   = kp_head * heading_err  -  kd_head * yaw_rate
  forward = velocity_profile(range) * align_gate(heading_err)

The kd_head yaw-rate term is the continuous form of the hand-flying trick
("counter-spin BEFORE it overshoots"): as rotation builds it eases and then
reverses the turn command early, so the nose settles without oscillating.
The align gate slows forward when badly mis-pointed (so it curves tightly,
near-pivoting on one motor) and opens up when facing the target (so it drives
straight in). Net effect: a smooth arc onto the point, always moving, never
stalling in a fight to rotate. Slow defaults = tighter, more accurate arcs.

============================ TILTED-PROP LIFT ===============================
The forward props are now tilted UP a bit, so forward thrust also lifts.
The hover loop compensates with z_fwd_couple: while driving forward it
SUBTRACTS that much up-command (scaled by actual forward fraction). If the
blimp climbs while cruising, raise z_fwd_couple; if it sinks, lower it.
Hover defaults are seeded from the proven manual settings (up 0.92 / down 0.50
after the prop tilt; z_ff 0.56 = the manual hold_alt_power that hovers).

Run via FLY_STRAIGHT_PANEL.command, or:
  ./.venv/bin/python straight_panel_server.py --server 192.168.0.4 --body 531 --goal-body 502
then open http://127.0.0.1:8501
"""
import argparse, glob, json, math, os, struct, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8501

FULL = 65535               # forward motor full duty
PITCH_MAX = 32767          # int16 cap on the vertical/turn setpoint channels
M_VERT_SIGN = -1.0         # up/down hardware orientation (proven config)
M_FWD_RAMP = 0.05          # per-tick fwd_level ramp (hold-W)
TICK = 0.04                # 25 Hz control loop

# ---- Manual heading-hold (anti-drift): hold the heading captured at
# throttle-up and LEARN the differential trim cancelling the constant bias. ----
HH_KP = 0.9
HH_KI = 0.15
HH_TRIM_MAX = 0.30
HH_OUT_MAX = 0.90

GK_SPEED = 1.5   # auto forward: fraction per (m/s) of speed error

def _wrap_pi(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a

def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# ---- Persisted tuning. powers seeded from the PROVEN manual blimp_config
# (2026-07-09): fwd 0.36, up 0.92, down 0.50 (props tilted up = they lift),
# turn 0.56. acfg = pure-pursuit path controller (slow defaults for accuracy). ----
STRAIGHT_CFG = os.path.join(DIR, "straight_config.json")
DEFAULT_POWERS = {"fwd": 0.36, "up": 0.92, "down": 0.50, "left": 0.56, "right": 0.56}
DEFAULT_ACFG = {
    # forward drive (velocity profile: eases off approaching the point). Slow
    # defaults -- a forward-only blimp is more accurate creeping in than racing.
    "cruise": 0.25, "approach": 0.5, "arrive": 0.30,
    "align_floor": 0.1,       # min forward-speed gate when facing AWAY: a
                              # forward-only craft needs some motion to steer, so
                              # never fully stop; lower = tighter pivoting arc.
    # PURE-PURSUIT steering (curve a trajectory to the point; never rotate in place)
    "kp_head": 0.9,           # heading error (rad) -> turn fraction
    "kd_head": 0.008,         # yaw-rate damping (per deg/s) = anticipatory
                              # counter-steer: eases/reverses the turn as rotation
                              # builds, BEFORE overshoot (the hand-flying trick).
    "turn_cap": 0.5,          # max turn fraction
    # hover (always on in auto)
    "z_ff": 0.56, "z_kp": 1.2, "z_kd": 0.8, "z_cap": 0.90,
    "z_fwd_couple": 0.15,     # up-command removed per unit forward (tilted props lift)
}

# Two FRAME PROFILES for the swappable front-motor mounts. The controller
# STRUCTURE is identical for both; only the tuning NUMBERS differ (mainly the
# vertical coupling). Each profile stores its own full tuning set, so you can
# physically swap the frame, click the matching profile, and every slider loads
# that frame's saved values:
#   tilted   — front motors angled UP: forward thrust also lifts, so the hover
#              loop subtracts up-command while driving (acfg z_fwd_couple > 0).
#   straight — front motors level: forward thrust is purely horizontal, no lift,
#              so z_fwd_couple = 0 and the up/down motors carry all of altitude.
# (Forward drive self-adjusts either way: it's closed-loop on the mocap-measured
# ground speed, so the tilt's reduced horizontal thrust just gets more command.)
FRAME_PROFILES = ("tilted", "straight")

def _current_tuning():
    """Snapshot the live tuning (call under lock)."""
    return {"manual_trim": S["manual_trim"], "yaw_trim": S["yaw_trim"],
            "powers": dict(S["powers"]), "acfg": dict(S["acfg"])}

def _apply_tuning(d):
    """Load a profile's tuning into the live state (call under lock). Missing
    keys fall back to defaults so switching frames never leaves stale values."""
    S["powers"] = dict(DEFAULT_POWERS)
    S["acfg"] = dict(DEFAULT_ACFG)
    S["manual_trim"] = float(d.get("manual_trim", 0.0))
    S["yaw_trim"] = float(d.get("yaw_trim", 0.0))
    for k, v in (d.get("powers") or {}).items():
        if k in S["powers"]:
            S["powers"][k] = float(v)
    for k, v in (d.get("acfg") or {}).items():
        if k in S["acfg"]:
            S["acfg"][k] = float(v)

def _straight_seed(tilted):
    """Seed the 'straight' profile from 'tilted': identical, except forward
    thrust no longer lifts, so the forward->up coupling starts at zero."""
    s = json.loads(json.dumps(tilted))          # deep copy
    s.setdefault("acfg", {})["z_fwd_couple"] = 0.0
    return s

def load_trim():
    try:
        d = json.load(open(STRAIGHT_CFG))
    except Exception:
        d = {}
    with lock:
        if isinstance(d.get("profiles"), dict):        # new multi-frame format
            S["profiles"] = {n: dict(d["profiles"].get(n, {})) for n in FRAME_PROFILES}
            S["profile"] = d["active"] if d.get("active") in FRAME_PROFILES else FRAME_PROFILES[0]
        else:                                          # migrate old flat config = the tilted frame
            S["profiles"] = {"tilted": d, "straight": _straight_seed(d)}
            S["profile"] = "tilted"
        _apply_tuning(S["profiles"].get(S["profile"], {}))
    print("loaded frame profiles from %s (active: %s)" % (STRAIGHT_CFG, S["profile"]))

def save_trim():
    with lock:
        S["profiles"][S["profile"]] = _current_tuning()   # keep the active profile in sync
        out = {"active": S["profile"], "profiles": dict(S["profiles"])}
    try:
        json.dump(out, open(STRAIGHT_CFG, "w"), indent=2)
    except Exception as e:
        print("trim save failed:", e)

lock = threading.Lock()
S = {
    "raw": {"x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1], "valid": False, "t": 0.0},
    "goal_raw": {"x": 0.0, "y": 0.0, "z": 0.0, "valid": False, "t": 0.0},
    "target": {"x": 1.0, "y": 0.0, "z": 1.2},
    "target_source": "marker",   # "marker" = chase the live goal body; "manual" = X/Y boxes
    "up_axis": "Z",
    "flying": False,
    "keys": {k: False for k in "WSADQE"},
    "fwd_level": 0.0,
    "yaw_ctrl_sign": -1.0,       # control turn direction (⟳ Flip button)
    "last_client": 0.0,
    "straight": True,            # manual-mode heading-hold anti-drift
    "hold_yaw": None,
    "yaw_trim": 0.0,
    "manual_trim": 0.0,
    "powers": dict(DEFAULT_POWERS),
    "profile": "tilted", "profiles": {},   # per-frame tuning (tilted vs straight props)
    "auto_go": False,
    "auto_prev": None,           # (t, h0, h1, alt, yaw) finite-diff state
    "auto_arrived": False,
    "hold_alt": None,            # hover altitude captured at GO
    "yawrate": 0.0,              # mocap-derived yaw rate, deg/s (LP filtered)
    "herr": 0.0,                 # heading error to target, deg (telemetry)
    "acfg": dict(DEFAULT_ACFG),
    "rate": 0.0, "frames": 0,
    "bridge": "", "err": "",
}
RUNNING = True

YAW_SIGN = -1.0   # mocap heading handedness fix (pointing left read as right)

def quat_yaw(q, up):
    qx, qy, qz, qw = q
    if up == "Y":
        return YAW_SIGN * math.atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
    return YAW_SIGN * math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def _map_raw(r, up):
    x, y, z = r["x"], r["y"], r["z"]
    return (x, z, y) if up == "Y" else (x, y, z)

def mapped():
    """Blimp pose -> (h0, h1, alt, yaw_rad, valid)."""
    with lock:
        r = dict(S["raw"]); up = S["up_axis"]
    h0, h1, alt = _map_raw(r, up)
    return h0, h1, alt, quat_yaw(r.get("q", [0, 0, 0, 1]), up), r.get("valid", False)

def goal_mapped():
    """Goal-marker pose -> (h0, h1, alt, fresh)."""
    with lock:
        r = dict(S["goal_raw"]); up = S["up_axis"]
        fresh = r["valid"] and (time.time() - r["t"]) < 1.0
    h0, h1, alt = _map_raw(r, up)
    return h0, h1, alt, fresh


def _turn_clamp(turn_raw, forward):
    """While cruising, |turn| <= forward so a turn can never zero one motor and
    let the other run away (true differential). At zero forward, allow the full
    turn (turn-in-place = one motor on, drift is expected and fine)."""
    if forward > 1.0:
        return _clamp(turn_raw, -forward, forward)
    return _clamp(turn_raw, -PITCH_MAX, PITCH_MAX)


def natnet_thread(server_ip, body_id, goal_body_id, local_ip, multicast):
    sys.path.insert(0, os.path.join(DIR, "optitrack_natnet"))
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
        elif goal_body_id is not None and idn == goal_body_id:
            with lock:
                S["goal_raw"].update(x=pos[0], y=pos[1], z=pos[2],
                                     valid=True, t=time.time())

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
            if S["goal_raw"]["valid"] and (time.time() - S["goal_raw"]["t"]) > 1.0:
                S["goal_raw"]["valid"] = False


def fly_thread(bridge_port):
    ser = None
    while RUNNING:
        with lock:
            flying = S["flying"]
        if flying and ser is None:
            try:
                import serial
                port = bridge_port or (glob.glob("/dev/cu.usbmodem*") +
                                       glob.glob("/dev/cu.wchusbserial*") + [""])[0]
                if not port:
                    raise RuntimeError("no C6 bridge serial port (plug in the XIAO C6)")
                ser = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(0.3)
                with lock: S["bridge"] = port; S["err"] = ""
            except Exception as e:
                with lock: S["err"] = "bridge: %s" % e; S["flying"] = False
                time.sleep(0.4); continue
        if (not flying) and ser is not None:
            try: ser.close()
            except Exception: pass
            ser = None
            with lock: S["bridge"] = ""
        if flying and ser is not None:
            # WATCHDOG: panel gone (tab closed / asleep) -> stop streaming so the
            # drone's own no-frame failsafe zeroes the motors.
            with lock:
                gone = (time.time() - S["last_client"]) > 1.5
            if gone:
                with lock:
                    S["flying"] = False; S["auto_go"] = False
                    S["keys"] = {k: False for k in "WSADQE"}; S["fwd_level"] = 0.0
                    S["err"] = "client disconnected -> stopped"
                continue

            with lock: go = S["auto_go"]
            pitch, turn, forward = _auto_tick() if go else _manual_tick()
            try:
                ser.write(b"\xA5" + struct.pack("<ffff", 0.0, pitch, turn, forward))
            except Exception as e:
                with lock: S["err"] = "bridge write: %s" % e; S["flying"] = False
            time.sleep(TICK)
        else:
            time.sleep(0.05)


def _auto_tick():
    """One 25 Hz autonomous step -> (pitch, turn, forward) motor-domain values."""
    h0, h1, alt, yaw, valid = mapped()
    gh0, gh1, galt, gvalid = goal_mapped()
    now = time.time()
    with lock:
        ysign = S["yaw_ctrl_sign"]; pw = dict(S["powers"])
        prev = S["auto_prev"]; cfg = dict(S["acfg"])
        if S["target_source"] == "marker" and gvalid:
            S["target"]["x"] = gh0; S["target"]["y"] = gh1   # chase the live marker
        if valid and S["hold_alt"] is None:
            S["hold_alt"] = alt                # capture hover altitude at engage
        hold_alt = S["hold_alt"]
        t = dict(S["target"])

    dx = t["x"] - h0; dy = t["y"] - h1
    rng = math.hypot(dx, dy)
    closing = 0.0; climb = 0.0; yawrate = 0.0
    if prev is not None and now > prev[0]:
        pdt = now - prev[0]
        vx = (h0 - prev[1]) / pdt; vy = (h1 - prev[2]) / pdt
        climb = (alt - prev[3]) / pdt
        yr_raw = math.degrees(_wrap_pi(yaw - prev[4])) / pdt
        with lock:
            S["yawrate"] = 0.6 * S["yawrate"] + 0.4 * _clamp(yr_raw, -400, 400)
            yawrate = S["yawrate"]
        if rng > 1e-3:
            closing = (vx * dx + vy * dy) / rng
    with lock:
        S["auto_prev"] = (now, h0, h1, alt, yaw)

    bearing = math.atan2(dy, dx) if rng > 1e-3 else yaw
    herr = _wrap_pi(bearing - yaw)
    with lock: S["herr"] = math.degrees(herr)

    forward = 0.0; turn = 0.0; ffrac = 0.0
    if valid and rng >= cfg["arrive"]:
        with lock: S["auto_arrived"] = False
        # ---- PURE-PURSUIT steering: curve a path to the point, don't rotate in
        # place. P on heading error minus D on the mocap yaw rate -- the D term
        # is the continuous "counter-spin before overshoot": as rotation builds
        # it eases and reverses the turn early so the nose settles smoothly. ----
        steer = cfg["kp_head"] * herr - cfg["kd_head"] * yawrate
        tfrac = _clamp(steer, -cfg["turn_cap"], cfg["turn_cap"])
        # ---- forward: ALWAYS moving (a forward-only craft needs motion to
        # steer), gated by how well it faces the target -- slow + tight arc when
        # mis-pointed, full drive when aligned -- and the velocity profile so it
        # decelerates into the point. ----
        align = max(cfg["align_floor"], math.cos(herr))
        v_des = min(cfg["cruise"], cfg["approach"] * rng) * align
        ffrac = _clamp(GK_SPEED * (v_des - closing), 0.0, pw["fwd"])
        forward = ffrac * FULL
        # Turn is NOT clamped to forward here (unlike manual): letting one motor
        # drop toward 0 while the other drives IS the tight pivoting arc that
        # steers a forward-only craft around. Firmware limitThrust bounds motors.
        turn = _clamp(ysign * tfrac * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    elif valid:
        with lock: S["auto_arrived"] = True

    # ---- hover: buoyancy FF + P − D, minus tilted-prop lift while driving ----
    pitch = 0.0
    if valid and hold_alt is not None:
        zerr = hold_alt - alt
        fwd_frac = ffrac / pw["fwd"] if pw["fwd"] > 1e-6 else 0.0
        vfrac = _clamp(cfg["z_ff"] + cfg["z_kp"] * zerr - cfg["z_kd"] * climb
                       - cfg["z_fwd_couple"] * fwd_frac,
                       -cfg["z_cap"], cfg["z_cap"])
        pitch = _clamp(M_VERT_SIGN * vfrac * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    return pitch, turn, forward


def _manual_tick():
    """One 25 Hz manual step -> (pitch, turn, forward)."""
    with lock:
        keys = dict(S["keys"])
        target_lvl = 0.0 if keys.get("S") else (1.0 if keys.get("W") else 0.0)
        lvl = S["fwd_level"]
        lvl += _clamp(target_lvl - lvl, -M_FWD_RAMP, M_FWD_RAMP)
        if keys.get("S"):
            lvl = max(0.0, lvl - M_FWD_RAMP)
        S["fwd_level"] = lvl
        pw = dict(S["powers"])
    forward = lvl * pw["fwd"] * FULL
    turn_raw = ((pw["right"] if keys.get("D") else 0.0) -
                (pw["left"] if keys.get("A") else 0.0)) * PITCH_MAX
    turn = _turn_clamp(turn_raw, forward)
    vfrac = (pw["up"] if keys.get("Q") else (-pw["down"] if keys.get("E") else 0.0))
    pitch = _clamp(M_VERT_SIGN * vfrac * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    # heading-hold while driving straight (learns the anti-drift trim)
    _h0, _h1, _alt, _yaw, _valid = mapped()
    _turning = keys.get("A") or keys.get("D")
    with lock:
        if S["straight"] and forward > 1.0 and _valid and not _turning:
            if S["hold_yaw"] is None:
                S["hold_yaw"] = _yaw
            _err = _wrap_pi(S["hold_yaw"] - _yaw)
            S["yaw_trim"] = _clamp(S["yaw_trim"] + HH_KI * _err * TICK,
                                   -HH_TRIM_MAX, HH_TRIM_MAX)
            _tf = _clamp(HH_KP * _err + S["yaw_trim"], -HH_OUT_MAX, HH_OUT_MAX)
            turn = _turn_clamp(S["yaw_ctrl_sign"] * _tf * PITCH_MAX, forward)
        else:
            S["hold_yaw"] = None
        if forward > 1.0:
            turn = _turn_clamp(turn + S["manual_trim"] * PITCH_MAX, forward)
    return pitch, turn, forward


def handle(d):
    a = d.get("action")
    with lock:
        if a == "fly":
            S["flying"] = bool(d.get("on"))
        elif a == "kill":
            S["flying"] = False; S["auto_go"] = False
            S["keys"] = {k: False for k in "WSADQE"}; S["fwd_level"] = 0.0
        elif a == "keys":
            kk = d.get("keys") or {}
            for k in "WSADQE":
                if k in kk:
                    S["keys"][k] = bool(kk[k])
        elif a == "yawsign":
            S["yaw_ctrl_sign"] = -S["yaw_ctrl_sign"]
        elif a == "straight":
            S["straight"] = not S["straight"]; S["hold_yaw"] = None; S["yaw_trim"] = 0.0
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
            S["auto_prev"] = None; S["auto_arrived"] = False; S["hold_alt"] = None
            S["yawrate"] = 0.0
            if S["auto_go"]: S["flying"] = True
        elif a == "acfg":
            k = d.get("name")
            if k in S["acfg"]:
                try: S["acfg"][k] = float(d.get("value"))
                except Exception: pass
        elif a == "profile":
            name = d.get("name")
            if name in FRAME_PROFILES and name != S["profile"]:
                S["profiles"][S["profile"]] = _current_tuning()   # snapshot the frame we're leaving
                S["profile"] = name
                _apply_tuning(S["profiles"].get(name, {}))          # load the frame we're switching to
        elif a == "target_source":
            if d.get("src") in ("marker", "manual"):
                S["target_source"] = d["src"]
        elif a == "upaxis":
            if d.get("axis") in ("Y", "Z"):
                S["up_axis"] = d["axis"]
        elif a == "target":
            for k in ("x", "y", "z"):
                if k in d:
                    try: S["target"][k] = float(d[k])
                    except Exception: pass
    if a in ("trim", "kill", "straight", "power", "acfg", "profile"):
        save_trim()
    return {"ok": True}


def state_payload():
    h0, h1, alt, yaw, valid = mapped()
    gh0, gh1, galt, gvalid = goal_mapped()
    with lock:
        return {
            "raw": dict(S["raw"]),
            "mapped": {"h0": round(h0, 3), "h1": round(h1, 3),
                       "alt": round(alt, 3), "yaw": round(math.degrees(yaw), 1)},
            "goal": {"h0": round(gh0, 3), "h1": round(gh1, 3),
                     "alt": round(galt, 3), "valid": gvalid},
            "target": dict(S["target"]), "target_source": S["target_source"],
            "up_axis": S["up_axis"], "flying": S["flying"],
            "keys": dict(S["keys"]),
            "fwd_level": round(S["fwd_level"], 2),
            "yaw_ctrl_sign": S["yaw_ctrl_sign"],
            "straight": S["straight"], "yaw_trim": round(S["yaw_trim"], 3),
            "manual_trim": round(S["manual_trim"], 3),
            "powers": dict(S["powers"]), "profile": S["profile"],
            "auto_go": S["auto_go"], "auto_arrived": S["auto_arrived"],
            "herr": round(S["herr"], 1), "yawrate": round(S["yawrate"], 1),
            "acfg": dict(S["acfg"]),
            "rate": S["rate"], "valid": valid,
            "bridge": S["bridge"], "err": S["err"],
        }


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, body, ctype):
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body))); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        with lock: S["last_client"] = time.time()
        if self.path in ("/", "/straight_panel.html"):
            self._send(open(os.path.join(DIR, "straight_panel.html"), "rb").read(), "text/html")
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
    ap = argparse.ArgumentParser(description="Autonomous straight-to-goal blimp panel.")
    ap.add_argument("--server", required=True, help="OptiTrack/Motive PC IP")
    ap.add_argument("--body", type=int, required=True, help="blimp rigid-body Streaming ID")
    ap.add_argument("--goal-body", type=int, default=502,
                    help="goal-marker rigid-body Streaming ID (default 502)")
    ap.add_argument("--local", default=None, help="this Mac's IP (auto if omitted)")
    ap.add_argument("--unicast", action="store_true", help="NatNet unicast (default multicast)")
    ap.add_argument("--bridge-port", default=None, help="C6 serial port (auto if omitted)")
    ap.add_argument("--up", default="Z", choices=["Y", "Z"], help="initial up axis")
    args = ap.parse_args()
    load_trim()

    with lock: S["up_axis"] = args.up
    local = args.local
    if not local:
        try:
            import socket
            sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sk.connect((args.server, 80)); local = sk.getsockname()[0]; sk.close()
        except Exception:
            local = None

    threading.Thread(target=natnet_thread,
                     args=(args.server, args.body, args.goal_body, local, not args.unicast),
                     daemon=True).start()
    threading.Thread(target=fly_thread, args=(args.bridge_port,), daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    url = "http://127.0.0.1:%d" % PORT
    print("Straight panel at %s  (blimp #%d, goal marker #%d @ %s, local %s)" %
          (url, args.body, args.goal_body, args.server, local))
    print("Keep this window open. Ctrl-C to stop.")
    try: webbrowser.open(url)
    except Exception: pass
    try: srv.serve_forever()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
