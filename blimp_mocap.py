#!/usr/bin/env python3
"""
blimp_mocap.py — autonomous ESP-FLY BLIMP control from OptiTrack (NatNet) mocap.

WHAT IT DOES
  OptiTrack/Motive tracks the blimp as a rigid body and streams its pose over the
  network (NatNet). This script reads that pose, runs a guidance loop, and drives
  the blimp toward a target (hold a point, or fly through waypoints):

      turn  -> rotate to face the target          (uses mocap yaw)
      fwd   -> drive forward once roughly facing it (uses horizontal distance)
      vert  -> up/down to match target altitude     (uses height error)

  Control is sent to the drone over Wi-Fi (CRTP), the same link drive_blimp.py uses.

TWO MODES
  --sim    No drone, no network. A simple kinematic model fakes the blimp's pose
           so you can watch/tune the WHOLE guidance loop offline. USE THIS NOW
           while we wait for the lab Wi-Fi.
  (real)   Reads live OptiTrack NatNet pose and commands the real drone. Needs:
             * Mac on the same network as the OptiTrack host (lab Wi-Fi, or a
               direct Ethernet link to the Motive PC).
             * The drone reachable over Wi-Fi (its AP, or once it's on lab Wi-Fi).
             * A NatNet client — see NatNetMocap below.

QUICK START (offline, today):
    ./.venv/bin/python blimp_mocap.py --sim
Then real, later:
    ./.venv/bin/python blimp_mocap.py --server <MOTIVE_PC_IP> --body 1 --drone udp://192.168.43.42:2390

COORDINATE NOTE: Motive's default is Y-up. Set UP_AXIS below to match your
Motive "Up Axis" setting (calibration). Get this right before flying for real.
"""

import argparse
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass

# 'Y' (Motive default) or 'Z'. The two horizontal axes form the ground plane;
# the third is altitude. Heading (yaw) is rotation about the up axis.
UP_AXIS = "Z"

# ====== EDIT THESE ONCE for your lab — then just run MOCAP.command, no typing ======
# Find them in Motive: Edit > Settings > Streaming shows the server IP; the rigid
# body's "Streaming ID" is in its properties. Your Mac's IP is auto-detected.
MOTIVE_IP = ""        # e.g. "192.168.1.50"  (the OptiTrack/Motive PC). Leave "" until you have it.
BLIMP_BODY_ID = 1     # the blimp's rigid-body Streaming ID in Motive
DRONE_URI = "udp://192.168.43.42:2390"   # CRTP link to the drone (Wi-Fi mode)
# ==================================================================================


def autodetect_local_ip(server_ip):
    """The Mac's IP on the route to the Motive PC (so --local is rarely needed)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((server_ip or "8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


@dataclass
class Pose:
    """Blimp pose in the mocap world frame. Horizontal = (h0, h1), up = alt (meters)."""
    h0: float
    h1: float
    alt: float
    yaw: float            # radians, heading in the ground plane
    t: float              # timestamp (s)
    valid: bool = True


def wrap_pi(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_yaw(qx, qy, qz, qw, up_axis=UP_AXIS):
    """Heading (rotation about the up axis) from a quaternion, in radians."""
    if up_axis == "Y":
        # rotation about Y, projected onto the X-Z ground plane
        return math.atan2(2.0 * (qw * qy + qx * qz),
                          1.0 - 2.0 * (qy * qy + qz * qz))
    # Z-up: rotation about Z, ground plane X-Y
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def xyz_to_plane(x, y, z, up_axis=UP_AXIS):
    """Split a 3D point into (horizontal0, horizontal1, altitude) per the up axis."""
    if up_axis == "Y":
        return x, z, y          # ground = X-Z, up = Y
    return x, y, z              # ground = X-Y, up = Z


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


# ===========================================================================
# Mocap sources
# ===========================================================================
class MocapSource:
    def start(self): ...
    def stop(self): ...
    def get_pose(self) -> Pose: ...


class SimMocap(MocapSource):
    """
    Offline simulator: a crude blimp kinematic model so the guidance loop can be
    developed and tuned with no hardware. It integrates the last command:
      forward thrust -> velocity along heading (with drag)
      turn           -> yaw rate
      vert           -> climb rate
    Numbers are only roughly blimp-like; it's for verifying control logic, not physics.
    """
    def __init__(self, start=(0.0, 0.0, 1.0, 0.0)):
        self.h0, self.h1, self.alt, self.yaw = start
        self.vx = self.vy = self.vz = self.wz = 0.0
        self._cmd = (0.0, 0.0, 0.0)   # forward(0..1), turn(-1..1), vert(-1..1)
        self._t = time.time()
        self._lock = threading.Lock()

    def set_command(self, forward_norm, turn_norm, vert_norm):
        with self._lock:
            self._cmd = (forward_norm, turn_norm, vert_norm)

    def _step(self):
        now = time.time()
        dt = min(0.1, now - self._t)
        self._t = now
        with self._lock:
            fwd, turn, vert = self._cmd
        # very loose model: thrust -> accel along heading, first-order drag
        ax = 0.8 * fwd * math.cos(self.yaw) - 0.6 * self.vx
        ay = 0.8 * fwd * math.sin(self.yaw) - 0.6 * self.vy
        self.vx += ax * dt
        self.vy += ay * dt
        self.h0 += self.vx * dt
        self.h1 += self.vy * dt
        self.wz = 1.2 * turn                       # yaw rate ~ turn cmd
        self.yaw = wrap_pi(self.yaw + self.wz * dt)
        self.vz = 0.5 * vert
        self.alt = max(0.0, self.alt + self.vz * dt)

    def get_pose(self) -> Pose:
        self._step()
        return Pose(self.h0, self.h1, self.alt, self.yaw, time.time())


class NatNetMocap(MocapSource):
    """
    Live OptiTrack pose via NatNet.

    Rather than hand-roll the (version-sensitive) NatNet binary protocol, this
    uses OptiTrack's official Python client. One-time setup:
      1. Download the NatNet SDK from optitrack.com, find `NatNetClient.py`
         (and its helper modules) and drop them next to this file, OR
         `pip install` a maintained NatNet client and adjust the import below.
      2. In Motive: Edit > Settings > Streaming -> enable, note Transmission Type
         (Multicast/Unicast) and the server IP.
      3. Define the blimp as a rigid body in Motive and note its Streaming ID.

    We register a frame listener and keep the latest pose for the chosen body id.
    """
    def __init__(self, server_ip, body_id, local_ip=None, use_multicast=True):
        self.server_ip = server_ip
        self.body_id = body_id
        self.local_ip = local_ip
        self.use_multicast = use_multicast
        self._pose = Pose(0, 0, 0, 0, 0, valid=False)
        self._lock = threading.Lock()
        self._client = None

    def start(self):
        # Use the swarmslab/optitrack_natnet client cloned next to this script.
        import os, sys
        natnet_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "optitrack_natnet")
        if os.path.isdir(natnet_dir) and natnet_dir not in sys.path:
            sys.path.insert(0, natnet_dir)
        try:
            from NatNetClient import NatNetClient   # from optitrack_natnet/
        except Exception as e:
            raise RuntimeError(
                "NatNet client not found. Expected ./optitrack_natnet/NatNetClient.py "
                "(git clone https://github.com/swarmslab/optitrack_natnet), or run "
                f"--sim for now. (import error: {e})\n"
                "macOS multicast note: in optitrack_natnet/NatNetClient.py ~line 280, "
                "bind to self.multicast_address instead of self.local_ip_address.")
        c = NatNetClient()
        c.set_server_address(self.server_ip)
        if self.local_ip:
            c.set_client_address(self.local_ip)
        c.set_use_multicast(self.use_multicast)
        c.rigid_body_listener = self._on_rigid_body
        c.run()
        self._client = c

    def _on_rigid_body(self, new_id, position, rotation):
        if new_id != self.body_id:
            return
        x, y, z = position
        qx, qy, qz, qw = rotation
        h0, h1, alt = xyz_to_plane(x, y, z)
        yaw = quat_to_yaw(qx, qy, qz, qw)
        with self._lock:
            self._pose = Pose(h0, h1, alt, yaw, time.time(), valid=True)

    def stop(self):
        if self._client:
            try: self._client.shutdown()
            except Exception: pass

    def get_pose(self) -> Pose:
        with self._lock:
            return self._pose


# ===========================================================================
# Guidance: pose + target -> (forward, turn, vert) setpoint
# ===========================================================================
class BlimpGuidance:
    """
    Turn-to-face + drive-to-target guidance, gentle by default (blimp lifts,
    motors only steer). Outputs are in the firmware setpoint domain:
        forward : 0 .. FWD_MAX        (thrust units, like drive_blimp's 'fwd')
        turn    : deg/s               (attitudeRate.yaw)
        vert    : degrees             (attitude.pitch, sign-corrected)
    """
    # Normalized guidance gains -> outputs are fractions: forward 0..1, turn -1..1,
    # vert -1..1 ("up" positive, physical intent). Each DroneLink maps these to its
    # own domain (sim / Wi-Fi / ESP-NOW bridge), so one guidance drives any link.
    KFWD = 0.5               # full forward ~2 m out
    KTURN = 1.2              # per radian of heading error
    KVERT = 1.5              # per meter of altitude error
    ARRIVE_RADIUS = 0.25     # m: within this, stop driving forward
    FACE_TOL = math.radians(35)   # only drive forward when facing within this

    def __init__(self, target):
        self.target = target          # (h0, h1, alt)

    def set_target(self, target):
        self.target = target

    def compute(self, pose: Pose):
        tx, ty, tz = self.target
        dx, dy = tx - pose.h0, ty - pose.h1
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        head_err = wrap_pi(bearing - pose.yaw)

        # turn toward target (normalized -1..1)
        turn = clamp(self.KTURN * head_err, -1.0, 1.0)

        # drive forward only when roughly facing the target and not yet arrived
        if dist > self.ARRIVE_RADIUS and abs(head_err) < self.FACE_TOL:
            forward = clamp(self.KFWD * dist, 0.0, 1.0) * max(0.0, math.cos(head_err))
        else:
            forward = 0.0

        # altitude hold (normalized, "up" positive — hardware VERT_SIGN applied per link)
        vert = clamp(self.KVERT * (tz - pose.alt), -1.0, 1.0)

        return forward, turn, vert


# ===========================================================================
# Drone output — maps normalized guidance (fwd 0..1, turn/vert -1..1) to a link.
# ===========================================================================
VERT_SIGN = -1.0          # hardware up/down inversion (applied to real outputs only)
# Wi-Fi (CRTP): firmware blimp gains set on connect (vertGain=900, turnGain=130).
WIFI_FWD = 22000.0; WIFI_TURN = 70.0; WIFI_VERT = 14.0
# ESP-NOW bridge: passthrough motor-duty domain (matches drive_blimp_espnow.py).
BR_FULL = 65535; BR_PITCH_MAX = 32767
BR_FWD_POWER = 0.30; BR_TURN_POWER = 0.25; BR_VERT_POWER = 0.80


class DroneLink:
    """Sends control to the blimp over Wi-Fi (CRTP), the ESP-NOW bridge (USB
    serial -> C6), or nowhere (--sim, feeds the simulator)."""
    def __init__(self, mode, uri=None, bridge_port=None, sim_mocap=None):
        self.mode = mode          # "sim" | "wifi" | "bridge"
        self.uri = uri
        self.bridge_port = bridge_port
        self.sim_mocap = sim_mocap
        self._scf = self._cf = self._ser = None

    def __enter__(self):
        if self.mode == "wifi":
            import cflib.crtp
            import cf_udp_patch  # noqa: F401  (ESP-Drone UDP framing patch)
            from cflib.crazyflie import Crazyflie
            from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
            cflib.crtp.init_drivers()
            self._scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
            self._scf.open_link()
            self._cf = self._scf.cf
            for k, v in (("blimp.fwdScale", 1.0), ("blimp.vertGain", 900.0),
                         ("blimp.turnGain", 130.0), ("blimp.pitchFF", 0.0)):
                try: self._cf.param.set_value(k, v)
                except Exception: pass
            for _ in range(10):
                self._cf.commander.send_setpoint(0, 0, 0, 0); time.sleep(0.05)
        elif self.mode == "bridge":
            import glob as _g
            import serial
            port = self.bridge_port or (_g.glob("/dev/cu.usbmodem*") + [""])[0]
            if not port:
                raise RuntimeError("no bridge serial port found — plug in the C6 bridge")
            self._ser = serial.Serial(port, 115200, timeout=0.1)
            time.sleep(0.4)
            print(f"  ESP-NOW bridge on {port}")
        return self

    def send(self, fwd_n, turn_n, vert_n):
        if self.mode == "sim":
            if self.sim_mocap:
                self.sim_mocap.set_command(fwd_n, turn_n, vert_n)
            return
        if self.mode == "wifi":
            self._cf.commander.send_setpoint(0.0, VERT_SIGN * vert_n * WIFI_VERT,
                                             turn_n * WIFI_TURN, int(fwd_n * WIFI_FWD))
            return
        # bridge: same 0xA5 + 4 LE float32 frame as drive_blimp_espnow.py
        forward = fwd_n * BR_FWD_POWER * BR_FULL
        turn = turn_n * BR_TURN_POWER * BR_PITCH_MAX
        pitch = VERT_SIGN * vert_n * BR_VERT_POWER * BR_PITCH_MAX
        self._ser.write(b"\xA5" + struct.pack("<ffff", 0.0, pitch, turn, forward))

    def __exit__(self, *a):
        try:
            if self.mode == "wifi" and self._cf:
                self._cf.commander.send_setpoint(0, 0, 0, 0)
                self._cf.commander.send_stop_setpoint()
            elif self.mode == "bridge" and self._ser:
                for _ in range(3):
                    self._ser.write(b"\xA5" + struct.pack("<ffff", 0, 0, 0, 0)); time.sleep(0.03)
        except Exception:
            pass
        if self._scf: self._scf.close_link()
        if self._ser: self._ser.close()


# ===========================================================================
# ON-BOARD mode — the drone computes the guidance itself (decoupled PID in
# firmware: blimp_guidance.c). The Mac only MEASURES (mocap) and TELLS the
# drone where it is + where to go, by streaming the `mocap` CRTP params. This
# is the "12 values in, drone does the math" architecture.
# ===========================================================================
class OnboardLink:
    """Streams pose + target into the firmware `mocap` params and turns on the
    on-board controller (blimpc.autoEn=1). No guidance is computed here."""
    def __init__(self, uri, target, gains=None):
        self.uri = uri
        self.target = target            # (h0, h1, alt, yaw_deg)
        self.gains = gains or {}
        self._scf = self._cf = None
        self._seq = 0
        self._last_target = None

    def __enter__(self):
        import cflib.crtp
        import cf_udp_patch  # noqa: F401  (ESP-Drone UDP framing patch)
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        cflib.crtp.init_drivers()
        self._scf = SyncCrazyflie(self.uri, cf=Crazyflie(rw_cache="./cache"))
        self._scf.open_link()
        self._cf = self._scf.cf
        time.sleep(0.3)
        # Push any gain overrides, then the target, then arm autonomous mode.
        for k, v in self.gains.items():
            try: self._cf.param.set_value(f"blimpc.{k}", v)
            except Exception as e: print(f"  (gain {k} failed: {e})")
        self._push_target(self.target)
        try: self._cf.param.set_value("blimpc.autoEn", 1)
        except Exception as e: print(f"  WARN could not enable autoEn: {e}")
        print("  on-board controller ENABLED (blimpc.autoEn=1)")
        return self

    def _push_target(self, target):
        h0, h1, alt, yaw_deg = target
        for k, v in (("tx", h0), ("ty", h1), ("tz", alt), ("tyaw", yaw_deg)):
            self._cf.param.set_value(f"mocap.{k}", float(v))
        self._last_target = target

    def set_target(self, target):
        if target != self._last_target:
            self._push_target(target)

    def push_pose(self, pose: Pose):
        """Send current pose + bump the freshness counter. Called every loop."""
        c = self._cf
        c.param.set_value("mocap.cx", float(pose.h0))
        c.param.set_value("mocap.cy", float(pose.h1))
        c.param.set_value("mocap.cz", float(pose.alt))
        c.param.set_value("mocap.cyaw", float(math.degrees(pose.yaw)))
        self._seq += 1
        c.param.set_value("mocap.seq", self._seq)

    def __exit__(self, *a):
        try:
            self._cf.param.set_value("blimpc.autoEn", 0)   # back to manual / safe
        except Exception:
            pass
        if self._scf:
            self._scf.close_link()


class OnboardSimController:
    """Faithful Python mirror of blimp_guidance.c (same loops + default gains),
    used ONLY for offline `--sim --onboard` so you can watch the ON-BOARD
    controller converge with no hardware. Outputs normalized (fwd 0..1,
    turn/vert -1..1) to feed SimMocap. Keep in sync with blimp_guidance.c."""
    KP_Z, KI_Z, KD_Z, VERTMAX = 12000.0, 1500.0, 6000.0, 16000.0
    KP_YAW, KD_YAW = 0.9, 0.015
    KP_FWD, FWDMAXN, ARRIVE_R, HEAD_GATE = 0.6, 1.0, 0.25, 60.0

    def __init__(self, target):
        self.target = target            # (h0, h1, alt, yaw_deg)
        self.zI = 0.0
        self._t = time.time()

    def set_target(self, target):
        self.target = target

    def compute(self, pose: Pose, yaw_rate_dps: float):
        now = time.time(); dt = clamp(now - self._t, 0.001, 0.1); self._t = now
        tx, ty, tz, tyaw = self.target
        dx, dy = tx - pose.h0, ty - pose.h1
        rng = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx) if rng > 1e-3 else pose.yaw
        arrived = rng <= self.ARRIVE_R
        head_ref = math.radians(tyaw) if arrived else bearing
        yaw_err = wrap_pi(head_ref - pose.yaw)

        turn = clamp(self.KP_YAW * yaw_err - self.KD_YAW * yaw_rate_dps, -1.0, 1.0)

        fwd = 0.0
        if not arrived:
            facing = math.cos(yaw_err)
            if facing > math.cos(math.radians(self.HEAD_GATE)) and facing > 0.0:
                fwd = clamp(self.KP_FWD * rng * facing, 0.0, self.FWDMAXN)

        z_err = tz - pose.alt
        self.zI += z_err * dt
        i_term = clamp(self.KI_Z * self.zI, -8000.0, 8000.0)
        self.zI = i_term / self.KI_Z
        vz_est = 0.0   # sim altitude rate ~ command; D omitted in mirror
        u_vert_pwm = self.KP_Z * z_err + i_term - self.KD_Z * vz_est
        vert = clamp(u_vert_pwm / self.VERTMAX, -1.0, 1.0)
        return fwd, turn, vert


class OnboardBridgeLink:
    """Streams mocap pose+target to the drone over the XIAO C6 ESP-NOW bridge as
    0xA6 + 8 LE float32 frames (cx,cy,cz,cyaw, tx,ty,tz,tyaw). The DRONE runs the
    guidance; receiving a pose auto-engages it (no CRTP param needed). This keeps
    the Mac on the lab/mocap Wi-Fi the whole time — no drone Wi-Fi join."""
    def __init__(self, target, port=None):
        self.target = target            # (h0, h1, alt, yaw_deg)
        self.port = port
        self.ser = None

    def __enter__(self):
        import glob as _g
        import serial
        port = self.port or (_g.glob("/dev/cu.usbmodem*") + _g.glob("/dev/cu.wchusbserial*") + [""])[0]
        if not port:
            raise RuntimeError("no C6 bridge serial port found — plug in the XIAO C6")
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.4)
        print(f"  ESP-NOW bridge on {port} (streaming mocap pose frames)")
        return self

    def set_target(self, target):
        self.target = target

    def push_pose(self, pose: Pose):
        h0, h1, alt = pose.h0, pose.h1, pose.alt
        yaw = math.degrees(pose.yaw)
        tx, ty, tz, tyaw = self.target
        self.ser.write(b"\xA6" + struct.pack("<ffffffff",
                       h0, h1, alt, yaw, float(tx), float(ty), float(tz), float(tyaw)))

    def __exit__(self, *a):
        # One-way link: can't disarm over ESP-NOW. We just stop streaming; the
        # drone's mocap-stale failsafe (blimpc.staleMs) zeroes the motors.
        if self.ser:
            self.ser.close()


# ===========================================================================
# Main loop
# ===========================================================================
def run(args):
    target = tuple(args.target)
    rate_hz = args.rate

    # ---- ON-BOARD mode: the DRONE computes guidance; we only stream pose+target ----
    if args.onboard:
        return run_onboard(args, target, rate_hz)

    guidance = BlimpGuidance(target)

    if args.sim:
        mocap = SimMocap(start=(0.0, 0.0, 1.0, 0.0))
        mocap.start()
        link = DroneLink("sim", sim_mocap=mocap)
        print(f"[SIM] target={target}  (no drone, no network)")
    else:
        if not MOTIVE_IP and args.server in ("", "127.0.0.1"):
            print("Set MOTIVE_IP (and BLIMP_BODY_ID) at the top of blimp_mocap.py, "
                  "or pass --server <IP> --body <id>. Find them in Motive > Settings > "
                  "Streaming. (Use --sim to test offline.)")
            return
        local_ip = args.local or autodetect_local_ip(args.server)
        mocap = NatNetMocap(args.server, args.body, local_ip=local_ip,
                            use_multicast=not args.unicast)
        mocap.start()
        if args.bridge:
            link = DroneLink("bridge", bridge_port=args.bridge_port)
            dest = "ESP-NOW bridge (C6)"
        else:
            link = DroneLink("wifi", uri=args.drone)
            dest = args.drone
        print(f"[REAL] OptiTrack {args.server} (local {local_ip}) body#{args.body} -> {dest}")

    last_print = 0.0
    with link:
        try:
            while True:
                pose = mocap.get_pose()
                if not pose.valid:
                    link.send(0, 0, 0)            # no tracking -> hold still
                    time.sleep(1.0 / rate_hz)
                    continue
                fwd, turn, vert = guidance.compute(pose)
                link.send(fwd, turn, vert)

                now = time.time()
                if now - last_print > 0.25:
                    last_print = now
                    tx, ty, tz = guidance.target
                    dist = math.hypot(tx - pose.h0, ty - pose.h1)
                    print(f"\rpos=({pose.h0:+.2f},{pose.h1:+.2f},{pose.alt:+.2f}) "
                          f"yaw={math.degrees(pose.yaw):+6.1f}  dist={dist:4.2f}m  "
                          f"fwd={fwd:.2f} turn={turn:+.2f} vert={vert:+.2f}   ", end="")
                time.sleep(1.0 / rate_hz)
        except KeyboardInterrupt:
            print("\nStopping.")
        finally:
            mocap.stop()


def run_onboard(args, target3, rate_hz):
    """On-board controller mode. The drone runs blimp_guidance.c; we just feed it
    the live pose and the target via the `mocap` params (or simulate it offline)."""
    target = (target3[0], target3[1], target3[2], float(args.target_yaw))

    if args.sim:
        mocap = SimMocap(start=(0.0, 0.0, 1.0, 0.0))
        ctrl = OnboardSimController(target)
        print(f"[SIM ON-BOARD] mirroring blimp_guidance.c -> target={target}  (no drone)")
        last_print = 0.0
        try:
            while True:
                pose = mocap.get_pose()
                fwd, turn, vert = ctrl.compute(pose, math.degrees(mocap.wz))
                mocap.set_command(fwd, turn, vert)   # VERT_SIGN is hardware-only
                now = time.time()
                if now - last_print > 0.25:
                    last_print = now
                    dist = math.hypot(target[0] - pose.h0, target[1] - pose.h1)
                    print(f"\rpos=({pose.h0:+.2f},{pose.h1:+.2f},{pose.alt:+.2f}) "
                          f"yaw={math.degrees(pose.yaw):+6.1f}  dist={dist:4.2f}m  "
                          f"fwd={fwd:.2f} turn={turn:+.2f} vert={vert:+.2f}   ", end="")
                time.sleep(1.0 / rate_hz)
        except KeyboardInterrupt:
            print("\nStopping.")
        return

    # ---- REAL: stream live mocap pose into the firmware, drone does the rest ----
    if not MOTIVE_IP and args.server in ("", "127.0.0.1"):
        print("Set MOTIVE_IP/--server. (Use --sim --onboard to test offline.)")
        return
    local_ip = args.local or autodetect_local_ip(args.server)
    mocap = NatNetMocap(args.server, args.body, local_ip=local_ip,
                        use_multicast=not args.unicast)
    mocap.start()
    if args.bridge:
        link = OnboardBridgeLink(target, port=args.bridge_port)
        dest = "ESP-NOW C6 bridge (Mac stays on lab Wi-Fi)"
    else:
        link = OnboardLink(args.drone, target)
        dest = args.drone + " (Wi-Fi CRTP)"
    print(f"[REAL ON-BOARD] OptiTrack {args.server} body#{args.body} -> {dest}  "
          f"(drone computes guidance)")
    last_print = 0.0
    with link:
        try:
            while True:
                pose = mocap.get_pose()
                if pose.valid:
                    link.push_pose(pose)             # stale-failsafe handled on drone
                now = time.time()
                if now - last_print > 0.25:
                    last_print = now
                    dist = math.hypot(target[0] - pose.h0, target[1] - pose.h1)
                    flag = "" if pose.valid else "  [NO TRACK]"
                    print(f"\rpos=({pose.h0:+.2f},{pose.h1:+.2f},{pose.alt:+.2f}) "
                          f"yaw={math.degrees(pose.yaw):+6.1f}  dist={dist:4.2f}m{flag}   ", end="")
                time.sleep(1.0 / rate_hz)
        except KeyboardInterrupt:
            print("\nStopping (autoEn->0).")
        finally:
            mocap.stop()


def main():
    ap = argparse.ArgumentParser(description="Autonomous blimp control from OptiTrack mocap.")
    ap.add_argument("--sim", action="store_true", help="offline simulator (no drone/network)")
    ap.add_argument("--server", default=MOTIVE_IP or "127.0.0.1", help="OptiTrack/Motive host IP")
    ap.add_argument("--local", default=None, help="this machine's IP (auto-detected if omitted)")
    ap.add_argument("--unicast", action="store_true", help="NatNet unicast (default multicast)")
    ap.add_argument("--body", type=int, default=BLIMP_BODY_ID, help="rigid-body streaming ID of the blimp")
    ap.add_argument("--drone", default=DRONE_URI, help="CRTP URI of the drone (Wi-Fi mode)")
    ap.add_argument("--bridge", action="store_true",
                    help="send control over the ESP-NOW C6 bridge (USB) instead of Wi-Fi")
    ap.add_argument("--bridge-port", default=None, help="bridge serial port (auto if omitted)")
    ap.add_argument("--target", type=float, nargs=3, default=[1.0, 0.0, 1.2],
                    metavar=("H0", "H1", "ALT"), help="target point in mocap frame (m)")
    ap.add_argument("--target-yaw", type=float, default=0.0,
                    help="heading (deg) to hold once arrived (on-board mode)")
    ap.add_argument("--onboard", action="store_true",
                    help="DRONE computes guidance (firmware blimp_guidance.c); we only "
                         "stream pose+target. Combine with --sim to test offline.")
    ap.add_argument("--rate", type=float, default=20.0, help="control loop Hz")
    run(ap.parse_args())


if __name__ == "__main__":
    main()
