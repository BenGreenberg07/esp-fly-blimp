#!/usr/bin/env python3
"""
mocap_panel_server.py — real-time 3D view of the blimp's mocap pose + autonomous
flight control, in one browser panel.

  OptiTrack --lab WiFi--> Mac (this server) --USB--> C6 --ESP-NOW--> blimp

The server reads the blimp's pose over NatNet, shows it live in a 3D plot in the
browser, and (when you press FLY) streams pose+target to the drone over the C6
ESP-NOW bridge so the ON-BOARD controller flies it. You can set the target,
toggle the up-axis (Y/Z) and watch which way the blimp actually moves, and KILL
instantly. The Mac stays on the lab Wi-Fi the whole time.

Run via FLY_MOCAP_PANEL.command, or:
    ./.venv/bin/python mocap_panel_server.py --server 192.168.0.4 --body 531
then open http://127.0.0.1:8500
"""
import argparse, glob, json, math, os, struct, sys, threading, time, webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DIR = os.path.dirname(os.path.abspath(__file__))
PORT = 8500

# Guidance gains — order MUST match blimpGuidanceSetGains() in blimp_guidance.c.
GAIN_ORDER = ["kpZ", "kiZ", "kdZ", "zff", "iLimZ",
              "yawKpHead", "yawRateMax", "yawKpRate",
              "kpFwd", "fwdMaxN", "arriveR", "headGate",
              "fwdMaxPwm", "turnMaxPwm", "vertMaxPwm"]
GAIN_DEFAULTS = {"kpZ": 12000, "kiZ": 1500, "kdZ": 6000, "zff": 0, "iLimZ": 8000,
                 "yawKpHead": 25, "yawRateMax": 30, "yawKpRate": 0.02,
                 "kpFwd": 0.6, "fwdMaxN": 1.0, "arriveR": 0.25, "headGate": 60,
                 "fwdMaxPwm": 18000, "turnMaxPwm": 9000, "vertMaxPwm": 16000}

lock = threading.Lock()
S = {
    "raw": {"x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1], "valid": False, "t": 0.0},
    "target": {"x": 1.0, "y": 0.0, "z": 1.2, "yaw": 0.0},
    "up_axis": "Z",            # which raw axis is altitude ("Y" or "Z")
    "flying": False,
    "rate": 0.0, "frames": 0,
    "bridge": "", "err": "",
    "gains": dict(GAIN_DEFAULTS),
    "gains_dirty": True,       # send once on connect, then on every change
}
RUNNING = True


def quat_yaw(q, up):
    qx, qy, qz, qw = q
    if up == "Y":   # heading about Y, ground plane X-Z
        return math.atan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qy * qy + qz * qz))
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def mapped():
    """Raw mocap pose -> (h0, h1, alt, yaw_rad, valid) using the chosen up axis."""
    with lock:
        r = dict(S["raw"]); up = S["up_axis"]
    x, y, z = r["x"], r["y"], r["z"]
    q = r.get("q", [0, 0, 0, 1])
    if up == "Y":
        h0, h1, alt = x, z, y
    else:
        h0, h1, alt = x, y, z
    return h0, h1, alt, quat_yaw(q, up), r.get("valid", False)


def natnet_thread(server_ip, body_id, local_ip, multicast):
    sys.path.insert(0, os.path.join(DIR, "optitrack_natnet"))
    try:
        from NatNetClient import NatNetClient
    except Exception as e:
        with lock: S["err"] = "NatNet import failed: %s" % e
        return

    def rb(idn, pos, rot):
        if idn != body_id:
            return
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
    # rate monitor + stale flag
    last = 0
    while RUNNING:
        time.sleep(1.0)
        with lock:
            S["rate"] = S["frames"] - last
            last = S["frames"]
            if S["raw"]["valid"] and (time.time() - S["raw"]["t"]) > 1.0:
                S["raw"]["valid"] = False


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
                with lock: S["bridge"] = port; S["err"] = ""; S["gains_dirty"] = True
            except Exception as e:
                with lock: S["err"] = "bridge: %s" % e; S["flying"] = False
                time.sleep(0.4); continue
        if (not flying) and ser is not None:
            try: ser.close()
            except Exception: pass
            ser = None
            with lock: S["bridge"] = ""
        if flying and ser is not None:
            # send gains whenever they changed (event-driven, off the hot path)
            with lock:
                dirty = S["gains_dirty"]; g = dict(S["gains"]); S["gains_dirty"] = False
            if dirty:
                try:
                    ser.write(b"\xA7" + struct.pack("<15f", *[float(g[k]) for k in GAIN_ORDER]))
                except Exception as e:
                    with lock: S["err"] = "bridge gains: %s" % e
            h0, h1, alt, yaw, valid = mapped()
            with lock: t = dict(S["target"])
            if valid:
                try:
                    ser.write(b"\xA6" + struct.pack("<ffffffff", h0, h1, alt,
                              math.degrees(yaw), t["x"], t["y"], t["z"], t["yaw"]))
                except Exception as e:
                    with lock: S["err"] = "bridge write: %s" % e; S["flying"] = False
            time.sleep(0.04)            # 25 Hz pose stream
        else:
            time.sleep(0.05)


def handle(d):
    a = d.get("action")
    with lock:
        if a == "fly":
            S["flying"] = bool(d.get("on"))
        elif a == "kill":
            S["flying"] = False
        elif a == "upaxis":
            if d.get("axis") in ("Y", "Z"):
                S["up_axis"] = d["axis"]
        elif a == "target":
            for k in ("x", "y", "z", "yaw"):
                if k in d:
                    try: S["target"][k] = float(d[k])
                    except Exception: pass
        elif a == "gain":
            k = d.get("name")
            if k in S["gains"]:
                try: S["gains"][k] = float(d.get("value")); S["gains_dirty"] = True
                except Exception: pass
        elif a == "gains_reset":
            S["gains"] = dict(GAIN_DEFAULTS); S["gains_dirty"] = True
        elif a == "preset":
            p = d.get("preset")
            if p == "tune":        # forward-only, slow: no vertical, low forward
                S["gains"]["vertMaxPwm"] = 0
                S["gains"]["fwdMaxPwm"] = 8000
                S["gains_dirty"] = True
            elif p == "full":      # restore vertical + forward authority
                S["gains"]["vertMaxPwm"] = GAIN_DEFAULTS["vertMaxPwm"]
                S["gains"]["fwdMaxPwm"] = GAIN_DEFAULTS["fwdMaxPwm"]
                S["gains_dirty"] = True
    return {"ok": True}


def state_payload():
    h0, h1, alt, yaw, valid = mapped()
    with lock:
        return {
            "raw": dict(S["raw"]),
            "mapped": {"h0": round(h0, 3), "h1": round(h1, 3),
                       "alt": round(alt, 3), "yaw": round(math.degrees(yaw), 1)},
            "target": dict(S["target"]),
            "up_axis": S["up_axis"], "flying": S["flying"],
            "rate": S["rate"], "valid": valid,
            "bridge": S["bridge"], "err": S["err"],
            "gains": dict(S["gains"]), "gain_order": GAIN_ORDER,
        }


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def _send(self, body, ctype):
        self.send_response(200); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body))); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/mocap_panel.html"):
            self._send(open(os.path.join(DIR, "mocap_panel.html"), "rb").read(), "text/html")
        elif self.path == "/state":
            self._send(json.dumps(state_payload()).encode(), "application/json")
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == "/api":
            n = int(self.headers.get("Content-Length", 0))
            self._send(json.dumps(handle(json.loads(self.rfile.read(n) or b"{}"))).encode(),
                       "application/json")
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description="Real-time 3D mocap view + blimp flight panel.")
    ap.add_argument("--server", required=True, help="OptiTrack/Motive PC IP")
    ap.add_argument("--body", type=int, required=True, help="blimp rigid-body Streaming ID")
    ap.add_argument("--local", default=None, help="this Mac's IP (auto if omitted)")
    ap.add_argument("--unicast", action="store_true", help="NatNet unicast (default multicast)")
    ap.add_argument("--bridge-port", default=None, help="C6 serial port (auto if omitted)")
    ap.add_argument("--up", default="Z", choices=["Y", "Z"], help="initial up axis")
    args = ap.parse_args()

    with lock: S["up_axis"] = args.up
    local = args.local
    if not local:
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect((args.server, 80)); local = s.getsockname()[0]; s.close()
        except Exception:
            local = None

    threading.Thread(target=natnet_thread,
                     args=(args.server, args.body, local, not args.unicast),
                     daemon=True).start()
    threading.Thread(target=fly_thread, args=(args.bridge_port,), daemon=True).start()

    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    url = "http://127.0.0.1:%d" % PORT
    print("Mocap panel at %s  (body #%d @ %s, local %s)" % (url, args.body, args.server, local))
    print("Keep this window open. Ctrl-C to stop.")
    try: webbrowser.open(url)
    except Exception: pass
    try: srv.serve_forever()
    except KeyboardInterrupt: pass


if __name__ == "__main__":
    main()
