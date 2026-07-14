#!/usr/bin/env python3
"""
blimp_server.py — web control + tuning panel for the ESP-FLY BLIMP.

Fly from the browser (WASD + Q/E) and tune everything live, no reflashing:
  * up motor power, down motor power (independent)
  * forward power, turn power
  * drift trims: yaw (stops slow spin), vertical (buoyancy)
  * anti-spin coupling (blends down with forward)
  * IMU YAW-HOLD: reads gyro.z and auto-trims the forward motors so it flies
    straight; on/off, strength, and direction are all tunable here.
  * vertical sign (the up/down inversion fix)

All mixing is done HERE (Python), so the firmware runs in passthrough:
  blimp.fwdScale/vertGain/turnGain = 1.0, trims = 0, and blimp.vertScale = 2.0
  (the new param that lets the int16 vertical channel reach full motor range —
  needs the matching firmware flash; without it up/down just maxes ~50%).

Live telemetry (battery, attitude, and the IMU yaw RATE) is shown so you can
SEE whether the IMU is feeding data and whether yaw-hold is correcting.

Run with BLIMP_PANEL.command, or:
    ./.venv/bin/python blimp_server.py
then open http://127.0.0.1:8421   (joined to the blimp Wi-Fi, pw 12345678)
"""

import glob, json, os, re, subprocess, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cflib.crtp
import cf_udp_patch  # noqa: F401  (ESP-Drone UDP framing patch)
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

DIR = os.path.dirname(os.path.abspath(__file__))
URI = "udp://192.168.43.42:2390"
PORT = 8421
RATE_HZ = 50
STALE = 1.0               # s: cut motors if the browser stops sending key state
CLIENT_STALE = 2.0        # s: if the panel stops polling /state AT ALL (tab closed,
                          # laptop asleep) drop hold-altitude/counter-spin too -- unlike
                          # STALE above, this is NOT tied to WASDQE (Hold Altitude is
                          # meant to run with no keys touched, so it must not trip just
                          # because you're not actively driving)

FULL = 65535              # motor full duty
PITCH_MAX = 32767         # int16 cap on the vertical/turn setpoint channels

# Firmware params we force on connect so OUR mixing maps straight to motor duty.
FW_PARAMS = {"fwdScale": 1.0, "vertGain": 1.0, "turnGain": 1.0,
             "yawTrim": 0.0, "vertTrim": 0.0, "pitchFF": 0.0, "vertScale": 2.0}

# Live tunables (0..1 powers; trims as fraction of full; yaw-hold gain etc.)
TUNABLES = {
    "up_power": 0.80, "down_power": 0.80, "fwd_power": 0.30, "turn_power": 0.30,
    "yaw_trim": 0.0, "vert_trim": 0.0, "couple": 0.0,
    "yawhold": 0.0, "yawhold_kp": 0.010, "yawhold_sign": 1.0, "vert_sign": -1.0,
    "fwd_ramp": 0.02,        # forward ramp per tick (was hardcoded THR_STEP) -- lower
                             # = slower/gentler start-stop, higher = snappier
    "hold_alt_power": 0.0,   # constant vertical baseline while "Altitude hold" is ON
                             # (bipolar: + climb / - descend). Q/E still nudge on top.
    "counter_power": 0.5,    # counter-spin pulse strength (turn fraction)
    "counter_ms": 350,       # counter-spin pulse duration, ms
}

CONFIG_FILE = os.path.join(DIR, "blimp_config.json")

# Three FRAME PROFILES for the three swappable front-motor mounts. Each stores
# its OWN full TUNABLES set, so you swap the physical frame, click the matching
# profile, and every slider/toggle loads that frame's saved values. Tuning
# AUTO-SAVES on every change (no save button) and persists across restarts.
#   tilted_up   — front motors angled UP   (forward thrust also lifts)
#   tilted_down — front motors angled DOWN (forward thrust also pushes down)
#   straight    — front motors level       (forward thrust purely horizontal)
FRAME_PROFILES = ("tilted_up", "tilted_down", "straight")

def _clean_tune(d):
    """A full TUNABLES dict overlaid with any valid saved values from d."""
    t = dict(TUNABLES)
    t.update({k: float(v) for k, v in (d or {}).items() if k in TUNABLES})
    return t

def load_profiles():
    """-> (profiles, active). Migrates an old FLAT blimp_config.json by seeding
    ALL three profiles from it, so each frame starts from the last good tune."""
    try:
        d = json.load(open(CONFIG_FILE))
    except Exception:
        d = {}
    if isinstance(d.get("profiles"), dict):
        profiles = {n: _clean_tune(d["profiles"].get(n)) for n in FRAME_PROFILES}
        active = d["active"] if d.get("active") in FRAME_PROFILES else FRAME_PROFILES[0]
        print(f"Loaded frame profiles from {CONFIG_FILE} (active: {active})")
    else:
        base = _clean_tune(d)                        # old flat config = last good tune
        profiles = {n: dict(base) for n in FRAME_PROFILES}
        active = FRAME_PROFILES[0]
        print(f"Migrated flat config into 3 frame profiles (active: {active})")
    return profiles, active

def save_profiles():
    """Auto-save: dump all profiles + which is active. Called on every change."""
    with lock:
        out = {"active": S["profile"],
               "profiles": {n: dict(p) for n, p in S["profiles"].items()}}
    try:
        json.dump(out, open(CONFIG_FILE, "w"), indent=2)
    except Exception as e:
        print("save failed:", e)

# ---- Firmware mode read + flash (drive the drone's radio mode from the panel) ----
ESPDRONE = os.path.join(DIR, "esp-drone")
SYSTEM_C = os.path.join(ESPDRONE, "components/core/crazyflie/modules/src/system.c")

def read_mode():
    try:
        txt = open(SYSTEM_C).read()
        esp = re.search(r"#define ESPNOW_CONTROL_ENABLED (\d)", txt)
        ble = re.search(r"#define BLE_CONTROL_ENABLED (\d)", txt)
        if esp and esp.group(1) == "1": return "espnow"
        if ble and ble.group(1) == "1": return "ble"
        return "wifi"
    except Exception:
        return "?"

def do_flash(mode):
    with lock:
        S["flash"] = {"busy": True, "ok": None, "log": "editing config + building…"}
    try:
        txt = open(SYSTEM_C).read()
        txt = re.sub(r"#define BLE_CONTROL_ENABLED \d",
                     "#define BLE_CONTROL_ENABLED 0", txt)
        txt = re.sub(r"#define ESPNOW_CONTROL_ENABLED \d",
                     f"#define ESPNOW_CONTROL_ENABLED {1 if mode=='espnow' else 0}", txt)
        open(SYSTEM_C, "w").write(txt)
        port = (glob.glob("/dev/cu.usbmodem*") + [""])[0]
        if not port:
            raise RuntimeError("no drone USB port found (plug the drone in via USB)")
        cmd = (f'source ~/esp/esp-idf/export.sh >/dev/null 2>&1 && cd "{ESPDRONE}" '
               f'&& idf.py build && idf.py -p {port} flash')
        p = subprocess.run(["bash", "-lc", cmd], capture_output=True, text=True, timeout=600)
        ok = (p.returncode == 0)
        tail = (p.stdout + p.stderr)[-600:]
        with lock:
            S["flash"] = {"busy": False, "ok": ok, "log": tail}
            S["mode"] = read_mode()
    except Exception as e:
        with lock:
            S["flash"] = {"busy": False, "ok": False, "log": str(e)}

lock = threading.Lock()
_PROFILES, _ACTIVE = load_profiles()
S = {
    "want_connect": False, "connected": False, "error": "",
    "keys": {k: False for k in "WSADQE"}, "last_keys": 0.0, "last_client": 0.0,
    "fwd_level": 0.0, "out": {"forward": 0, "turn": 0, "pitch": 0, "corr": 0.0},
    "vbat": 0.0, "att": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}, "yawrate": 0.0,
    "profile": _ACTIVE, "profiles": _PROFILES,
    "tune": _PROFILES[_ACTIVE],   # alias -> active profile's dict; mutating tune edits the profile
    "mode": read_mode(), "flash": {"busy": False, "ok": None, "log": ""},
    "hold_alt": False,               # altitude-hold toggle (not persisted, starts off)
    "counter_sign": 0.0, "counter_until": 0.0,  # active counter-spin pulse, if any
}
RUNNING = True
CF = [None]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ---- Transport: Wi-Fi (CRTP) by default, or ESP-NOW via the C6 USB bridge. ----
# In ESP-NOW mode the panel sends the SAME motor-domain values, but as the
# 0xA5 + 4 LE-float32 (roll=0, pitch, yaw=turn, thrust=forward) frame the C6
# rebroadcasts and the drone parses — identical convention to the Wi-Fi path.
# ESP-NOW is one-way, so there is NO telemetry (battery/attitude/yaw-rate) and
# yaw-hold is disabled (it needs gyro.z). The drone forces blimp.vertScale=2.0
# at boot in ESP-NOW mode, so vertical reaches full range without setting params.
ESPNOW = {"on": False, "port": None}


class EspnowSerial:
    """USB-serial link to the XIAO C6 bridge (then ESP-NOW to the blimp)."""
    def __init__(self, port=None):
        self.port = port
        self.ser = None

    def open(self):
        import glob as _g
        import serial
        port = self.port or (_g.glob("/dev/cu.usbmodem*") + _g.glob("/dev/cu.wchusbserial*") + [""])[0]
        if not port:
            raise RuntimeError("no C6 bridge serial port found — plug in the XIAO C6")
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.4)
        print(f"  ESP-NOW bridge on {port}")

    def send(self, pitch, turn, forward):
        import struct
        self.ser.write(b"\xA5" + struct.pack("<ffff", 0.0, float(pitch), float(turn), float(forward)))

    def close(self):
        try:
            for _ in range(3):
                self.send(0, 0, 0); time.sleep(0.03)
        finally:
            if self.ser:
                self.ser.close()


def _mix(k, T, fwd_level, yawrate, hold_alt=False, counter_sign=0.0, counter_active=False):
    """Shared client-side mixing -> (forward, turn, pitch, corr) in motor duty."""
    forward = fwd_level * T["fwd_power"] * FULL

    # ---- turn: A/D, or an active counter-spin pulse overriding them ----
    if counter_active:
        manual_turn = counter_sign
        turn_power = T["counter_power"]
    else:
        manual_turn = (1.0 if k["D"] else 0.0) - (1.0 if k["A"] else 0.0)
        turn_power = T["turn_power"]
    corr = 0.0
    if T["yawhold"] >= 0.5 and forward > 0 and manual_turn == 0.0:
        corr = -T["yawhold_sign"] * T["yawhold_kp"] * yawrate
    turn_frac = manual_turn * turn_power + T["yaw_trim"] + corr
    turn_raw = turn_frac * PITCH_MAX
    # Auto-clamp turn to the CURRENT forward thrust, but ONLY while actually
    # cruising (forward > 0): forward+turn and forward-turn are the two motors'
    # raw duty before the firmware's own 0..65535 clamp, so while driving, an
    # over-large turn would zero one motor and let the other run away
    # unbounded -- that's "the other motor goes off" / "it just turns" instead
    # of a clean differential, so we cap |turn| <= forward there.
    # AT ZERO FORWARD there is no forward thrust to protect -- capping turn to
    # 0 there just made A/D do nothing unless W was held. Below, allow the full
    # requested turn: with unidirectional motors the only way to turn in place
    # IS one motor spinning and the other at 0 (some forward drift is
    # unavoidable without reverse thrust, and that's expected here).
    turn = clamp(turn_raw, -forward, forward) if forward > 1.0 else clamp(turn_raw, -PITCH_MAX, PITCH_MAX)

    manual_v = (T["up_power"] if k["Q"] else (-T["down_power"] if k["E"] else 0.0))
    hold_v = T["hold_alt_power"] if hold_alt else 0.0
    vert_frac = hold_v + manual_v + T["vert_trim"] - T["couple"] * fwd_level
    pitch = clamp(T["vert_sign"] * vert_frac * PITCH_MAX, -PITCH_MAX, PITCH_MAX)
    return forward, turn, pitch, corr


def flight_thread():
    while RUNNING:
        with lock:
            want, conn = S["want_connect"], S["connected"]
        if want and not conn:
            try:
                if ESPNOW["on"]:
                    esp = EspnowSerial(ESPNOW["port"])
                    esp.open()                          # no params, no telemetry
                    with lock:
                        S["connected"] = True; S["error"] = ""
                    _send_loop(None, esp)
                    esp.close()
                else:
                    cf = Crazyflie(rw_cache=os.path.join(DIR, "cache"))
                    link = SyncCrazyflie(URI, cf=cf)
                    link.open_link()
                    CF[0] = cf
                    for k, v in FW_PARAMS.items():
                        try: cf.param.set_value("blimp." + k, v)
                        except Exception: pass        # old firmware may lack vertScale
                    _setup_log(cf)
                    with lock:
                        S["connected"] = True; S["error"] = ""
                    _send_loop(cf, None)
                    link.close_link()
            except Exception as e:
                with lock:
                    S["error"] = str(e); S["want_connect"] = False
            CF[0] = None
            with lock:
                S["connected"] = False
        else:
            time.sleep(0.05)


def _setup_log(cf):
    try:
        lg = LogConfig(name="t", period_in_ms=100)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("gyro.z", "float")          # yaw RATE (deg/s) for yaw-hold
        lg.add_variable("pm.vbat", "float")

        def cb(ts, d, c):
            with lock:
                S["vbat"] = round(d["pm.vbat"], 2)
                S["yawrate"] = round(d["gyro.z"], 1)
                S["att"] = {"roll": round(d["stabilizer.roll"], 1),
                            "pitch": round(d["stabilizer.pitch"], 1),
                            "yaw": round(d["stabilizer.yaw"], 1)}
        cf.log.add_config(lg); lg.data_received_cb.add_callback(cb); lg.start()
    except Exception:
        pass


def _send_loop(cf, esp):
    if cf:
        for _ in range(10):
            cf.commander.send_setpoint(0, 0, 0, 0); time.sleep(0.05)
    while RUNNING:
        with lock:
            if not S["want_connect"]:
                break
            k = dict(S["keys"])
            T = dict(S["tune"])
            if (time.time() - S["last_keys"]) > STALE:      # no WASDQE touched recently ->
                S["fwd_level"] = 0.0; k = {kk: False for kk in "WSADQE"}   # decay drive keys
            if (time.time() - S["last_client"]) > CLIENT_STALE:   # panel truly gone (stopped
                S["hold_alt"] = False; S["counter_until"] = 0.0            # polling /state at all)
            if k["W"]: S["fwd_level"] = min(1.0, S["fwd_level"] + T["fwd_ramp"])
            if k["S"]: S["fwd_level"] = max(0.0, S["fwd_level"] - T["fwd_ramp"])
            fwd_level = S["fwd_level"]
            yawrate = 0.0 if esp else S["yawrate"]          # no gyro telemetry over ESP-NOW
            hold_alt = S["hold_alt"]
            now = time.time()
            counter_active = now < S["counter_until"]
            counter_sign = S["counter_sign"]
            forward, turn, pitch, corr = _mix(k, T, fwd_level, yawrate,
                                              hold_alt, counter_sign, counter_active)
            S["out"] = {"forward": int(forward), "turn": int(turn),
                        "pitch": int(pitch), "corr": round(corr, 3)}
        if cf:
            cf.commander.send_setpoint(0.0, float(pitch), float(turn), int(forward))
        else:
            esp.send(pitch, turn, forward)
        time.sleep(1.0 / RATE_HZ)
    if cf:
        cf.commander.send_setpoint(0, 0, 0, 0)
        cf.commander.send_stop_setpoint()


def handle(d):
    a = d.get("action")
    with lock:
        if a == "connect": S["want_connect"] = True
        elif a == "disconnect":
            S["want_connect"] = False; S["fwd_level"] = 0.0
            S["hold_alt"] = False; S["counter_until"] = 0.0
        elif a == "keys":
            for kk in "WSADQE": S["keys"][kk] = bool(d.get(kk, False))
            S["last_keys"] = time.time()
        elif a == "kill":
            S["fwd_level"] = 0.0; S["keys"] = {kk: False for kk in "WSADQE"}
            S["hold_alt"] = False; S["counter_until"] = 0.0
        elif a == "hold_alt":
            S["hold_alt"] = bool(d.get("on"))
        elif a == "counter":
            sign = d.get("sign")
            if sign in (-1, 1, -1.0, 1.0):
                S["counter_sign"] = float(sign)
                S["counter_until"] = time.time() + S["tune"]["counter_ms"] / 1000.0
        elif a == "tune":
            p = d.get("param")
            if p in S["tune"]:
                try: S["tune"][p] = float(d.get("value", 0))
                except Exception: pass
        elif a == "profile":
            name = d.get("name")
            if name in FRAME_PROFILES and name != S["profile"]:
                S["profile"] = name
                S["tune"] = S["profiles"][name]   # point the live alias at that frame
        elif a == "flash_mode":
            m = d.get("mode")
            if m in ("wifi", "espnow") and not S["flash"]["busy"]:
                threading.Thread(target=do_flash, args=(m,), daemon=True).start()
    if a in ("tune", "profile"):
        save_profiles()                            # AUTO-SAVE every tuning/profile change
    return {"ok": True}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _json(self, o):
        b = json.dumps(o).encode()
        self.send_response(200); self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

    def do_GET(self):
        with lock: S["last_client"] = time.time()   # panel watchdog heartbeat
        if self.path in ("/", "/blimp_panel.html"):
            b = open(os.path.join(DIR, "blimp_panel.html"), "rb").read()
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == "/state":
            with lock: self._json(dict(S))
        else: self.send_error(404)

    def do_POST(self):
        with lock: S["last_client"] = time.time()   # panel watchdog heartbeat
        if self.path == "/api":
            n = int(self.headers.get("Content-Length", 0))
            self._json(handle(json.loads(self.rfile.read(n) or b"{}")))
        else: self.send_error(404)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="ESP-FLY blimp web panel.")
    ap.add_argument("--espnow", action="store_true",
                    help="send control over the XIAO C6 ESP-NOW bridge (USB) instead of Wi-Fi")
    ap.add_argument("--port", default=None, help="C6 bridge serial port (auto if omitted)")
    args = ap.parse_args()
    ESPNOW["on"] = args.espnow
    ESPNOW["port"] = args.port

    if not ESPNOW["on"]:
        cflib.crtp.init_drivers()
    else:
        print("ESP-NOW mode: control rides the C6 USB bridge (no Wi-Fi, no telemetry).")
    threading.Thread(target=flight_thread, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    url = f"http://127.0.0.1:{PORT}"
    print(f"Blimp panel at {url} — keep this window open.")
    try: webbrowser.open(url)
    except Exception: pass
    try: srv.serve_forever()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
