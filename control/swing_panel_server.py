#!/usr/bin/env python3
"""swing_panel_server.py -- standalone test rig for the SWING BLIMP Mellinger controller.

SEPARATE FROM EVERYTHING ELSE. This does not import, modify, or share state with
panel_server.py or any of the decoupled-blimp panels -- the proven decoupled
stack keeps working exactly as it does today. Run one panel at a time, though: they
all want the same C6 USB bridge.

    Motive --NatNet--> this Mac --mellinger_core--> USB --> C6 --ESP-NOW--> drone

The drone runs NO guidance here. It is a dumb 4-channel motor amplifier: the swing
firmware mixer takes the four values in the existing 0xA5 frame and writes them
straight to M1..M4. All the control math and all the airframe geometry live on this
side, so the cant angle / layout / gains are live-tunable with no reflash.

FRAME (unchanged length, so the C6 bridge needs NO reflash):
    0xA5 + 4 LE float32 = (M1, M2, M3, M4) motor duties, 0..32767 half-scale.
    The firmware doubles them because control_t's roll/pitch/yaw fields are int16.

Run it via FLY_SWING_MELLINGER.command.
"""
import argparse, glob, json, math, os, struct, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import mellinger_core as mel

DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8620                  # deliberately clear of 8601/8602/8603/8610/8421
TICK = 0.04                  # 25 Hz control + stream rate
HALF = 32767.0               # int16 half-scale the firmware doubles back up
CFG = os.path.join(DIR, "swing_config.json")

lock = threading.RLock()
RUNNING = True

S = {
    "raw": {"x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1], "valid": False, "t": 0.0},
    "frames": 0, "rate": 0,
    "up_axis": "Z",
    "armed": False,            # bridge open + streaming
    "engaged": False,          # Mellinger controller driving the motors
    "bridge": "", "err": "",
    "last_client": 0.0,
    "target": {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0},
    "gains": dict(mel.MEL_DEFAULTS),
    "mix": dict(mel.MIX_DEFAULTS),
    "layout": "lateral",
    "motor_enable": [1, 1, 1, 1],
    # motor_map[arm position] = firmware channel (0..3 = m1..m4). Arm positions
    # are ordered front-right, rear-right, rear-left, front-left. Set it from the
    # panel after the props-off bench test tells you which channel is which.
    "motor_map": [0, 1, 2, 3],
    # What we SENT (fx, fy, mz, fz). The per-motor duties now live only on the
    # drone, and come back via "telem".
    "wrench": [0.0, 0.0, 0.0, 0.0],
    "ctl": {}, "log": {}, "mixinfo": {},
    "alloc": [], "auth": [0.0, 0.0],
    "vel": [0.0, 0.0, 0.0],
    "gyro": [0.0, 0.0, 0.0],
    "bench": None,             # (motor_index, duty) manual per-motor bench test
    # What the DRONE says it actually commanded to m1..m4 (0..65535 PWM), read
    # back over ESP-NOW. Compare against "duty" to tell a control-side problem
    # (never commanded) apart from a hardware one (commanded, did not spin).
    "telem": [0.0, 0.0, 0.0, 0.0],
    "telem_age": 999.0,
}


def _clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)


def _wrap_pi(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ------------------------------------------------------------------ config --
def load_cfg():
    try:
        with open(CFG) as f:
            d = json.load(f)
    except Exception:
        return
    with lock:
        for k, v in (d.get("gains") or {}).items():
            if k in mel.MEL_DEFAULTS: S["gains"][k] = float(v)
        for k, v in (d.get("mix") or {}).items():
            if k in mel.MIX_DEFAULTS: S["mix"][k] = float(v)
        if d.get("layout") in mel.CANT_LAYOUTS: S["layout"] = d["layout"]
        if isinstance(d.get("target"), dict): S["target"].update(d["target"])
        if isinstance(d.get("motor_enable"), list) and len(d["motor_enable"]) == 4:
            S["motor_enable"] = [1 if v else 0 for v in d["motor_enable"]]
        mm = d.get("motor_map")
        if isinstance(mm, list) and sorted(mm) == [0, 1, 2, 3]:
            S["motor_map"] = [int(v) for v in mm]


def save_cfg():
    with lock:
        d = {"gains": dict(S["gains"]), "mix": dict(S["mix"]), "layout": S["layout"],
             "target": dict(S["target"]), "motor_enable": list(S["motor_enable"]),
             "motor_map": list(S["motor_map"])}
    try:
        with open(CFG, "w") as f:
            json.dump(d, f, indent=1)
    except Exception as e:
        with lock: S["err"] = "save failed: %s" % e


# ------------------------------------------------------------------ mocap ---
def _map_raw(r, up):
    x, y, z = r["x"], r["y"], r["z"]
    return (x, z, y) if up == "Y" else (x, y, z)


def _map_quat(q, up):
    """Motive quaternion -> our z-up control frame. For a Y-up stream the y and z
    axes swap, which for a quaternion means swapping those two components too."""
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]
    return (qx, qz, qy, qw) if up == "Y" else (qx, qy, qz, qw)


def pose():
    """(pos xyz, quat xyzw, valid, sample_time) in the control frame."""
    with lock:
        r = dict(S["raw"]); up = S["up_axis"]
    p = _map_raw(r, up)
    q = mel.qnormalize(_map_quat(r.get("q", [0, 0, 0, 1]), up))
    return p, q, r.get("valid", False), r.get("t", 0.0)


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


# ----------------------------------------------------------------- bridge ---
def find_bridge_port():
    """Auto-detect the XIAO C6 bridge serial port."""
    try:
        from serial.tools import list_ports
        for p in list_ports.comports():
            blob = ("%s %s %s" % (p.device, p.description or "", p.manufacturer or "")).lower()
            if any(s in blob for s in ("usbmodem", "wchusbserial", "cp210", "esp", "xiao")):
                return p.device
    except Exception:
        pass
    g = (glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.wchusbserial*") +
         glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return g[0] if g else None


TELEM_MAGIC = b"\xB7\x1E\x30\xA5"
_telem_buf = bytearray()


def _read_telem(ser):
    """Drain the bridge serial and parse TELEM_MAGIC + 4 LE float32 motor commands.

    The bridge also prints ASCII status lines on this port; the magic is non-ASCII,
    so scanning for it can't collide with those."""
    global _telem_buf
    try:
        n = ser.in_waiting
        if n:
            _telem_buf += ser.read(n)
    except Exception:
        return
    if len(_telem_buf) > 4096:
        del _telem_buf[:-4096]
    while True:
        i = _telem_buf.find(TELEM_MAGIC)
        if i < 0 or len(_telem_buf) < i + 4 + 16:
            if i > 0:
                del _telem_buf[:i]
            return
        vals = struct.unpack("<ffff", bytes(_telem_buf[i + 4:i + 20]))
        del _telem_buf[:i + 20]
        with lock:
            S["telem"] = [round(v, 1) for v in vals]
            S["telem_age"] = 0.0
            S["_telem_t"] = time.time()


def wrench_frame(fx, fy, mz, fz):
    """0xA5 + 4 LE float32 = (F_x, F_y, M_z, F_z), i.e. the Mellinger controller's
    control_t fields (roll, pitch, yaw, thrust) sent straight through.

    The DRONE mixes these with the real Sblimp mixer -- see blimp_swing.h. Nothing is
    pre-mixed here, so what the controller computes is what the airframe gets.

    The ranges match what the firmware's ESP-NOW validator accepts (|pitch|,|yaw| <=
    40000; thrust in -1..70000); the controller already clamps F_x/F_y/M_z to
    +/-16000, and these caps are only a backstop against a NaN or a bad gain."""
    return b"\xA5" + struct.pack("<ffff",
                                 _clamp(float(fx), -16000.0, 16000.0),
                                 _clamp(float(fy), -16000.0, 16000.0),
                                 _clamp(float(mz), -16000.0, 16000.0),
                                 _clamp(float(fz), 0.0, 65535.0))


ZERO_FRAME = b"\xA5" + struct.pack("<ffff", 0.0, 0.0, 0.0, 0.0)


# ------------------------------------------------------- the control thread --
class _Vel:
    """Finite-difference world velocity + body rates from the mocap stream.

    Differentiates against each SAMPLE'S OWN timestamp, never wall-clock: if a
    sample is late, wall-clock dt makes the next fresh one look like a velocity
    spike, and the Mellinger D-terms carry gains in the tens of thousands on
    exactly that signal.
    """

    def __init__(self, a=0.45):
        self.a = a
        self.reset()

    def reset(self):
        self.prev = None
        self.v = [0.0, 0.0, 0.0]
        self.w = [0.0, 0.0, 0.0]

    def update(self, t, p, q):
        rpy = mel.quat2rpy(q)
        if self.prev is None or t <= self.prev[0] + 1e-6:
            if self.prev is None:
                self.prev = (t, p, rpy)
            return self.v, self.w
        dt = t - self.prev[0]
        pv, prpy = self.prev[1], self.prev[2]
        raw_v = [(p[i] - pv[i]) / dt for i in range(3)]
        raw_w = [_wrap_pi(rpy[i] - prpy[i]) / dt for i in range(3)]
        if max(abs(x) for x in raw_v) <= 5.0:              # reject glitch spikes
            self.v = [(1 - self.a) * self.v[i] + self.a * raw_v[i] for i in range(3)]
        if max(abs(x) for x in raw_w) <= 30.0:
            self.w = [(1 - self.a) * self.w[i] + self.a * raw_w[i] for i in range(3)]
        self.prev = (t, p, rpy)
        return self.v, self.w


def control_thread(bridge_port):
    ser = None
    ctrl = mel.MellingerController()
    mixer = mel.SwingMixer()
    vel = _Vel()
    was_engaged = False

    while RUNNING:
        with lock:
            armed = S["armed"]
        if armed and ser is None:
            try:
                import serial
                port = bridge_port or find_bridge_port()
                if not port:
                    raise RuntimeError("no C6 bridge serial port (plug in the XIAO C6)")
                ser = serial.Serial(port, 115200, timeout=0.1)
                time.sleep(0.3)
                with lock: S["bridge"] = port; S["err"] = ""
            except Exception as e:
                with lock: S["err"] = "bridge: %s" % e; S["armed"] = False
                time.sleep(0.4); continue
        if (not armed) and ser is not None:
            try:
                for _ in range(3):
                    ser.write(ZERO_FRAME); time.sleep(0.01)
                ser.close()
            except Exception:
                pass
            ser = None
            with lock: S["bridge"] = ""; S["wrench"] = [0.0] * 4
        if not (armed and ser is not None):
            time.sleep(0.05)
            was_engaged = False
            continue

        # WATCHDOG: panel gone (tab closed / laptop asleep) -> stop streaming, and
        # the drone's own ESP-NOW link-loss failsafe zeroes the motors.
        with lock:
            gone = (time.time() - S["last_client"]) > 1.5
        if gone:
            with lock:
                S["armed"] = False; S["engaged"] = False
                S["err"] = "client disconnected -> stopped"
            continue

        with lock:
            engaged = S["engaged"]; bench = S["bench"]
            tgt = dict(S["target"])
            gains = dict(S["gains"]); mixp = dict(S["mix"])
            layout = S["layout"]; men = tuple(S["motor_enable"])
            mmap = tuple(S["motor_map"])
        ctrl.set_gains(gains)
        mixer.set_params(dict(mixp, layout=layout))
        mixer.motor_enable = men
        mixer.set_motor_map(mmap)
        with lock:
            S["alloc"] = [[round(v, 4) for v in row] for row in mixer.A]
            S["auth"] = [round(mixer.auth_x, 3), round(mixer.auth_y, 3)]

        try:
            _read_telem(ser)          # drone -> host: what it actually commanded
            with lock:
                S["telem_age"] = round(time.time() - S.get("_telem_t", 0.0), 1)
            if bench is not None:
                # BENCH TEST, PROPS OFF. With the real Sblimp mixer every motor
                # carries the collective thrust, so a single motor CANNOT be driven
                # alone -- instead this sweeps one axis at a time:
                #   axis 0 = collective thrust  -> all four spin equally
                #   axis 1 = F_x, 2 = F_y, 3 = M_z -> differential on top of thrust
                # Axis 0 is the M4 test: if a motor stays still while the other three
                # spin at equal commanded thrust, that motor is not being driven.
                axis, mag = bench
                w = [0.0, 0.0, 0.0, 0.0]        # fx, fy, mz, fz
                base = _clamp(mag, 0.0, 0.5) * 65535.0
                if axis == 0:
                    w[3] = base
                elif 1 <= axis <= 3:
                    w[3] = base
                    w[axis - 1] = 0.35 * base   # differential kept well under thrust
                ser.write(wrench_frame(w[0], w[1], w[2], w[3]))
                with lock: S["wrench"] = [round(x, 1) for x in w]
                was_engaged = False

            elif engaged:
                p, q, valid, t = pose()
                if not valid:
                    ser.write(ZERO_FRAME)
                    vel.reset(); ctrl.reset()
                    with lock:
                        S["wrench"] = [0.0] * 4; S["err"] = "no mocap -> motors off"
                    time.sleep(TICK); continue
                if not was_engaged:
                    ctrl.reset(); vel.reset()
                    was_engaged = True
                v, w = vel.update(t, p, q)
                rpy = mel.quat2rpy(q)

                setpoint = {
                    "mode": {"x": "abs", "y": "abs", "z": "abs", "yaw": "abs"},
                    "position": {"x": tgt["x"], "y": tgt["y"], "z": tgt["z"]},
                    "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "attitude": {"roll": 0.0, "pitch": 0.0, "yaw": tgt["yaw"]},
                    "attitudeRate": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                    "thrust": 0.0,
                }
                state = {
                    "position": {"x": p[0], "y": p[1], "z": p[2]},
                    "velocity": {"x": v[0], "y": v[1], "z": v[2]},
                    "attitudeQuaternion": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
                    "attitude": {"yaw": mel.degrees(rpy[2])},
                }
                # The reference reads body rates off the IMU. We get no telemetry back
                # over ESP-NOW, so they come from the mocap attitude derivative instead
                # -- same quantity, ~25 Hz instead of 500, which is why the kw_* and
                # kd_omega_rp gains want turning DOWN from the reference values.
                sensors = {"gyro": {"x": mel.degrees(w[0]), "y": mel.degrees(w[1]),
                                    "z": mel.degrees(w[2])}}

                ctl, lg = ctrl.update(setpoint, state, sensors, TICK)
                # Straight to the wire: the drone's mixer turns (F_x, F_y, M_z, F_z)
                # into m1..m4 exactly as the reference Sblimp firmware does.
                ser.write(wrench_frame(ctl["roll"], ctl["pitch"], ctl["yaw"], ctl["thrust"]))
                with lock:
                    S["vel"] = [round(x, 3) for x in v]
                    S["gyro"] = [round(mel.degrees(x), 1) for x in w]
                    S["ctl"] = {k: round(val, 2) for k, val in ctl.items()}
                    S["log"] = {k: round(val, 4) for k, val in lg.items()}
                    # what went on the wire; per-motor values come back via "telem"
                    S["wrench"] = [round(ctl["roll"], 1), round(ctl["pitch"], 1),
                                   round(ctl["yaw"], 1), round(ctl["thrust"], 1)]
            else:
                was_engaged = False
                ser.write(ZERO_FRAME)
                with lock: S["wrench"] = [0.0] * 4
        except Exception as e:
            with lock: S["err"] = "bridge write: %s" % e; S["armed"] = False
        time.sleep(TICK)


# -------------------------------------------------------------------- HTTP --
def state_payload():
    p, q, valid, _t = pose()
    rpy = mel.quat2rpy(q)
    with lock:
        return {
            "valid": valid, "rate": S["rate"],
            "pos": [round(v, 3) for v in p],
            "rpy": [round(mel.degrees(v), 1) for v in rpy],
            "vel": list(S["vel"]), "gyro": list(S["gyro"]),
            "armed": S["armed"], "engaged": S["engaged"], "bridge": S["bridge"],
            "err": S["err"], "target": dict(S["target"]),
            "gains": dict(S["gains"]), "mix": dict(S["mix"]), "layout": S["layout"],
            "motor_enable": list(S["motor_enable"]),
            "motor_map": list(S["motor_map"]),
            "wrench": list(S["wrench"]), "ctl": dict(S["ctl"]), "log": dict(S["log"]),
            "mixinfo": dict(S["mixinfo"]), "alloc": list(S["alloc"]), "auth": list(S["auth"]),
            "bench": S["bench"], "up_axis": S["up_axis"],
            "telem": list(S["telem"]), "telem_age": S["telem_age"],
            "mel_spec": mel.MEL_SPEC, "mix_spec": mel.MIX_SPEC,
            "layouts": list(mel.CANT_LAYOUTS.keys()),
        }


def handle(d):
    a = d.get("action")
    with lock:
        if a == "arm":
            S["armed"] = bool(d.get("on", not S["armed"]))
            if not S["armed"]:
                S["engaged"] = False; S["bench"] = None
        elif a == "engage":
            on = bool(d.get("on", not S["engaged"]))
            if on and not S["raw"]["valid"]:
                S["err"] = "engage refused: no mocap tracking"
            else:
                S["engaged"] = on; S["bench"] = None
                if on: S["armed"] = True
        elif a == "kill":
            S["engaged"] = False; S["armed"] = False; S["bench"] = None
        elif a == "gain":
            k = d.get("name")
            if k in mel.MEL_LIMITS:
                lo, hi = mel.MEL_LIMITS[k]
                try: S["gains"][k] = _clamp(float(d.get("value")), lo, hi)
                except Exception: pass
        elif a == "mix":
            k = d.get("name")
            if k in mel.MIX_LIMITS:
                lo, hi = mel.MIX_LIMITS[k]
                try: S["mix"][k] = _clamp(float(d.get("value")), lo, hi)
                except Exception: pass
        elif a == "layout":
            if d.get("value") in mel.CANT_LAYOUTS:
                S["layout"] = d["value"]
        elif a == "motor_enable":
            try:
                i = int(d.get("index"))
                if 0 <= i < 4: S["motor_enable"][i] = 0 if S["motor_enable"][i] else 1
            except Exception: pass
        elif a == "motor_map":
            # Assign a firmware channel to an arm position, SWAPPING with whichever
            # position currently holds that channel. Swapping (rather than plain
            # assignment) keeps the map a valid permutation at every step, so the
            # panel can never leave two positions on one channel and one motor dead.
            try:
                pos = int(d.get("index")); ch = int(d.get("channel"))
            except Exception:
                pos = ch = -1
            if 0 <= pos < 4 and 0 <= ch < 4:
                cur = S["motor_map"].index(ch)
                S["motor_map"][cur] = S["motor_map"][pos]
                S["motor_map"][pos] = ch
        elif a == "reset_map":
            S["motor_map"] = [0, 1, 2, 3]
        elif a == "target":
            for k in ("x", "y", "z", "yaw"):
                if k in d:
                    try: S["target"][k] = float(d[k])
                    except Exception: pass
        elif a == "target_here":
            px = _map_raw(dict(S["raw"]), S["up_axis"])
            S["target"]["x"], S["target"]["y"] = round(px[0], 3), round(px[1], 3)
        elif a == "bench":
            # PROPS OFF. Spins one channel so you can map M1..M4 to physical motors.
            i = d.get("index")
            S["bench"] = None if i is None else (int(i), float(d.get("duty", 0.15)))
            if S["bench"] is not None:
                S["engaged"] = False; S["armed"] = True
        elif a == "reset_gains":
            S["gains"] = dict(mel.MEL_DEFAULTS)
        elif a == "reset_mix":
            S["mix"] = dict(mel.MIX_DEFAULTS)
        elif a == "up_axis":
            if d.get("value") in ("Y", "Z"): S["up_axis"] = d["value"]
        elif a == "clear_err":
            S["err"] = ""
    if a == "save":
        save_cfg()
        with lock: S["err"] = "saved %s" % os.path.basename(CFG)
    return {"ok": True}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        with lock: S["last_client"] = time.time()
        if self.path in ("/", "/swing_panel.html"):
            self._send(open(os.path.join(DIR, "swing_panel.html"), "rb").read(), "text/html")
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
    ap = argparse.ArgumentParser(description="Swing-blimp Mellinger controller test panel.")
    ap.add_argument("--server", required=True, help="OptiTrack/Motive PC IP")
    ap.add_argument("--body", type=int, required=True, help="blimp rigid-body Streaming ID")
    ap.add_argument("--local", default=None, help="this Mac's IP (auto if omitted)")
    ap.add_argument("--unicast", action="store_true", help="NatNet unicast (default multicast)")
    ap.add_argument("--bridge-port", default=None, help="C6 serial port (auto if omitted)")
    ap.add_argument("--up", default="Z", choices=["Y", "Z"], help="up axis")
    ap.add_argument("--layout", default=None, choices=list(mel.CANT_LAYOUTS.keys()),
                    help="initial motor cant layout")
    ap.add_argument("--port", type=int, default=None, help="HTTP port (default 8620)")
    args = ap.parse_args()

    load_cfg()
    global PORT
    if args.port:
        PORT = args.port
    with lock:
        S["up_axis"] = args.up
        if args.layout:
            S["layout"] = args.layout
        S["last_client"] = time.time()

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
    threading.Thread(target=control_thread, args=(args.bridge_port,), daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    url = "http://127.0.0.1:%d" % PORT
    print("Swing-blimp Mellinger panel at %s  (blimp #%d @ %s, local %s)"
          % (url, args.body, args.server, local))
    print("The MAC runs the Mellinger controller; the drone is a dumb 4-motor amplifier.")
    print("Keep this window open. Ctrl-C to stop.")
    try: webbrowser.open(url)
    except Exception: pass
    try: srv.serve_forever()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
