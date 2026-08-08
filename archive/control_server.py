#!/usr/bin/env python3
"""
ESP-FLY web control panel (backend).

Runs a small local web server that holds ONE cflib link to the drone and
streams setpoints at 50 Hz from a shared state. The browser page (panel.html)
drives it: connect, manual fly (WASD/QE), the auto hop, live trim + settings,
and an always-available KILL.

Start it with the 4_CONTROL_PANEL.command launcher, or:
    ./.venv/bin/python control_server.py
then open http://127.0.0.1:8420
"""

import json
import os
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cflib.crtp
import cf_udp_patch  # ESP-Drone checksum framing
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import flight_config

DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8420
RATE_HZ = 50
MANUAL_TIMEOUT = 0.5   # s: if browser stops sending, cut throttle (failsafe)

cfg = flight_config.load()
lock = threading.Lock()
S = {
    "want_connect": False, "connected": False, "error": "",
    "mode": "idle",                          # idle | manual | auto
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "thr": 0,
    "last_manual": 0.0,
    "auto_start": 0.0,
    "vbat": 0.0, "att": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    # tunables (mirrored from config)
    "roll_trim": cfg["roll_trim"], "pitch_trim": cfg["pitch_trim"],
    "thrust_climb": cfg["thrust_climb"], "thrust_hover": cfg["thrust_hover"],
    "climb_time": cfg["climb_time"], "hover_time": cfg["hover_time"],
    "land_time": cfg["land_time"],
}
RUNNING = True


def auto_thrust(t):
    """Throttle for the auto sequence at elapsed time t (s). Returns (thr, done)."""
    ct, ht, lt = S["climb_time"], S["hover_time"], S["land_time"]
    if t < ct:
        return int(S["thrust_climb"] * (t / ct)), False
    if t < ct + ht:
        return S["thrust_hover"], False
    if t < ct + ht + lt:
        frac = (t - ct - ht) / lt
        return int(S["thrust_hover"] * (1 - frac)), False
    return 0, True


def flight_thread():
    while RUNNING:
        with lock:
            want = S["want_connect"]
            connected = S["connected"]
        if want and not connected:
            try:
                cf = Crazyflie(rw_cache=os.path.join(DIR, "cache"))
                link = SyncCrazyflie(cfg["uri"], cf=cf)
                link.open_link()
                _setup_log(cf)
                with lock:
                    S["connected"] = True
                    S["error"] = ""
                _send_loop(cf)
                link.close_link()
            except Exception as e:
                with lock:
                    S["error"] = str(e)
                    S["want_connect"] = False
            with lock:
                S["connected"] = False
                S["mode"] = "idle"
        else:
            time.sleep(0.05)


def _setup_log(cf):
    try:
        lg = LogConfig(name="tlm", period_in_ms=200)
        lg.add_variable("stabilizer.roll", "float")
        lg.add_variable("stabilizer.pitch", "float")
        lg.add_variable("stabilizer.yaw", "float")
        lg.add_variable("pm.vbat", "float")

        def cb(ts, data, conf):
            with lock:
                S["vbat"] = round(data["pm.vbat"], 2)
                S["att"] = {"roll": round(data["stabilizer.roll"], 1),
                            "pitch": round(data["stabilizer.pitch"], 1),
                            "yaw": round(data["stabilizer.yaw"], 1)}
        cf.log.add_config(lg)
        lg.data_received_cb.add_callback(cb)
        lg.start()
    except Exception:
        pass  # telemetry is optional; flight still works without it


def _send_loop(cf):
    # Unlock: first setpoint must be thrust 0.
    for _ in range(10):
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.05)
    while RUNNING:
        with lock:
            if not S["want_connect"]:
                break
            mode = S["mode"]
            rt, pt = S["roll_trim"], S["pitch_trim"]
            if mode == "auto":
                thr, done = auto_thrust(time.time() - S["auto_start"])
                r, p, y = rt, pt, 0.0
                if done:
                    S["mode"] = "idle"
                    thr = 0
            elif mode == "manual":
                fresh = (time.time() - S["last_manual"]) < MANUAL_TIMEOUT
                thr = S["thr"] if fresh else 0
                r = S["roll"] + rt
                p = S["pitch"] + pt
                y = S["yaw"]
            else:
                r = p = y = 0.0
                thr = 0
        cf.commander.send_setpoint(r, p, y, int(thr))
        time.sleep(1.0 / RATE_HZ)
    cf.commander.send_setpoint(0, 0, 0, 0)
    cf.commander.send_stop_setpoint()


def handle_action(d):
    a = d.get("action")
    with lock:
        if a == "connect":
            S["want_connect"] = True
        elif a == "disconnect":
            S["want_connect"] = False
            S["mode"] = "idle"
        elif a == "manual":
            S["mode"] = "manual"
            S["roll"] = float(d.get("roll", 0))
            S["pitch"] = float(d.get("pitch", 0))
            S["yaw"] = float(d.get("yaw", 0))
            S["thr"] = int(d.get("thr", 0))
            S["last_manual"] = time.time()
        elif a == "kill":
            S["mode"] = "idle"
            S["thr"] = 0
        elif a == "auto":
            if S["connected"]:
                S["mode"] = "auto"
                S["auto_start"] = time.time()
        elif a == "settings":
            for k in ("roll_trim", "pitch_trim", "thrust_climb", "thrust_hover",
                      "climb_time", "hover_time", "land_time"):
                if k in d:
                    S[k] = float(d[k]) if "time" in k or "trim" in k else int(d[k])
            flight_config.save({k: S[k] for k in (
                "roll_trim", "pitch_trim", "thrust_climb", "thrust_hover",
                "climb_time", "hover_time", "land_time")})
    return {"ok": True}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass  # quiet

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/" or self.path == "/panel.html":
            with open(os.path.join(DIR, "panel.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            with lock:
                self._json(dict(S))
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api":
            n = int(self.headers.get("Content-Length", 0))
            d = json.loads(self.rfile.read(n) or b"{}")
            self._json(handle_action(d))
        else:
            self.send_error(404)


def main():
    cflib.crtp.init_drivers()
    threading.Thread(target=flight_thread, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    url = f"http://127.0.0.1:{PORT}"
    print(f"ESP-FLY control panel running at {url}")
    print("Leave this window open. Close it (Ctrl-C) to shut down.")
    try:
        webbrowser.open(url)
    except Exception:
        pass
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
