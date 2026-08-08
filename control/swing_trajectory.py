#!/usr/bin/env python3
"""swing_trajectory.py -- trajectory generators for the swing-blimp Mellinger panel.

Ported from traj_original.py (the Crazyflie/Q-AB trajectory-tracking script) so the
same shapes can be flown through the host-side Mellinger controller.

ONE DELIBERATE CHANGE: TIME, NOT TICKS
--------------------------------------
The original parameterises every shape by a loop counter `t` that increments once per
iteration of a free-running `while True:` loop, e.g. `radius*cos(0.000175*t)`. That
makes the actual speed depend entirely on how fast the host happens to loop -- the
same script flies a different trajectory on a faster machine, and none of the
constants mean anything you can state in seconds.

Here the shapes are driven by ELAPSED SECONDS and an explicit `period_s` (seconds per
lap), so a lap takes the time you asked for regardless of tick rate. The panel runs a
fixed 25 Hz tick; feeding it the original per-iteration constants would put a circle
lap at roughly 24 minutes.

The shapes themselves are unchanged.

Each generator returns (p_d, v_d):
    p_d = [x, y, z, yaw_deg]      desired pose
    v_d = [vx, vy, vz]            exact analytic derivative of the position

NOTE ON v_d: the original computes v_d but only ever logs it -- the setpoint actually
sent is `send_position_setpoint(...)`, which is position + yaw with zero velocity. The
panel mirrors that by default so the controller behaves the same, and keeps v_d for
the error log. Turn on feed-forward in the panel to also feed v_d into the
controller's velocity setpoint, which is what you'd want for real tracking.
"""
import math

# (key, default, min, max, step, label) -- the panel builds its inputs from this.
TRAJ_SPEC = [
    ("radius",     1.00,   0.10,   4.00, 0.05,  "Radius (m)"),
    ("center_x",   0.00,  -5.00,   5.00, 0.10,  "Center X (m)"),
    ("center_y",   0.00,  -5.00,   5.00, 0.10,  "Center Y (m)"),
    ("height",     1.00,   0.20,   3.00, 0.05,  "Height (m)"),
    ("period_s",  90.0,   10.0,  600.0,  5.0,   "Seconds per lap"),
    ("yaw_deg",   90.0, -180.0,  180.0,  5.0,   "Desired yaw (deg)"),
    ("climb_mps",  0.01,   0.0,    0.20, 0.005, "Helix climb rate (m/s)"),
    ("max_height", 2.00,   0.20,   3.50, 0.05,  "Helix max height (m)"),
]
TRAJ_DEFAULTS = {k: d for (k, d, _lo, _hi, _st, _l) in TRAJ_SPEC}
TRAJ_LIMITS = {k: (lo, hi) for (k, _d, lo, hi, _st, _l) in TRAJ_SPEC}

SHAPES = ("hold", "circle", "figure8", "helix", "circle_vertical")


def _omega(period_s):
    """rad/s for one lap in period_s seconds."""
    return 2.0 * math.pi / max(1e-3, float(period_s))


def circle(tau, p):
    """Horizontal circle. Original: radius*cos(.000175*t) + center."""
    w = _omega(p["period_s"])
    r, cx, cy = p["radius"], p["center_x"], p["center_y"]
    a = w * tau
    pos = [r * math.cos(a) + cx, r * math.sin(a) + cy, p["height"], p["yaw_deg"]]
    # NOTE: the original returns +w*r*sin(a) for vy; d/dt of sin is cos, so that is a
    # typo. It was invisible there because v_d only feeds the error log, never the
    # setpoint. Corrected here.
    vel = [-w * r * math.sin(a), w * r * math.cos(a), 0.0]
    return pos, vel


def figure8(tau, p):
    """Gerono lemniscate. Original: theta = omega*t + pi/2, y = r*sin(2*theta)."""
    w = _omega(p["period_s"])
    r, cx, cy = p["radius"], p["center_x"], p["center_y"]
    th = w * tau + math.pi / 2.0
    pos = [r * math.cos(th) + cx, r * math.sin(2 * th) + cy, p["height"], p["yaw_deg"]]
    vel = [-w * r * math.sin(th), 2 * w * r * math.cos(2 * th), 0.0]
    return pos, vel


def helix(tau, p):
    """Circle with a steady climb, capped at max_height.

    The original ramps `height += 0.00002` once per loop iteration and clamps to
    max_height; here that becomes climb_mps in metres per second."""
    w = _omega(p["period_s"])
    r, cx, cy = p["radius"], p["center_x"], p["center_y"]
    a = w * tau
    z = p["height"] + p["climb_mps"] * tau
    climbing = z < p["max_height"]
    if not climbing:
        z = p["max_height"]
    vel = [-w * r * math.sin(a), w * r * math.cos(a),
           p["climb_mps"] if climbing else 0.0]
    return [r * math.cos(a) + cx, r * math.sin(a) + cy, z, p["yaw_deg"]], vel


def circle_vertical(tau, p):
    """Vertical circle in the x-z plane. The original starts at -90 deg so it begins
    at the bottom of the ring and climbs, rather than jumping to the side."""
    w = _omega(p["period_s"])
    r, cx, cy = p["radius"], p["center_x"], p["center_y"]
    a = w * tau - math.pi / 2.0
    pos = [r * math.cos(a) + cx, cy, r * math.sin(a) + p["height"], p["yaw_deg"]]
    vel = [-w * r * math.sin(a), 0.0, w * r * math.cos(a)]
    return pos, vel


_GEN = {"circle": circle, "figure8": figure8, "helix": helix,
        "circle_vertical": circle_vertical}


def evaluate(shape, tau, params, hold_pos=None):
    """(p_d [x,y,z,yaw_deg], v_d [vx,vy,vz]) for `shape` at elapsed time `tau`.

    shape "hold" parks at hold_pos -- the position captured when the run started --
    which is the safe default and what ENGAGE uses before a trajectory is picked."""
    p = dict(TRAJ_DEFAULTS)
    p.update({k: float(v) for k, v in (params or {}).items() if k in TRAJ_DEFAULTS})
    if shape == "hold" or shape not in _GEN:
        h = hold_pos or [p["center_x"], p["center_y"], p["height"]]
        return [h[0], h[1], h[2], p["yaw_deg"]], [0.0, 0.0, 0.0]
    return _GEN[shape](float(tau), p)


def preview(shape, params, n=180):
    """Sample one full lap for drawing in the panel. Returns [[x, y], ...]."""
    if shape == "hold" or shape not in _GEN:
        return []
    p = dict(TRAJ_DEFAULTS)
    p.update({k: float(v) for k, v in (params or {}).items() if k in TRAJ_DEFAULTS})
    per = p["period_s"]
    out = []
    for i in range(n + 1):
        q, _v = _GEN[shape](per * i / n, p)
        out.append([round(q[0], 3), round(q[1], 3)])
    return out


# ---- the reference vehicle's own tuning, straight out of traj_original.py ----
# What the real Sblimp actually flies with. These are nowhere near the
# controller_mellinger.c header defaults: the attitude gains are ~6 orders of
# magnitude smaller, and yaw control is switched OFF entirely (kR_z = kw_z =
# ki_m_z = 0). Anything derived from the C defaults will be violently overdriven on
# this airframe, so this is the sane starting point.
REFERENCE_GAINS = {
    "mass": 0.01, "massThrust": 45000.0, "scale_fb": 0.25,
    "kp_x": 1.0,  "kd_x": 2.0,  "ki_x": 0.07, "i_range_x": 2.0,
    "kp_y": 1.75, "kd_y": 1.0,  "ki_y": 0.05,
    "kp_z": 1.20, "kd_z": 0.5,  "ki_z": 0.05,
    "kR_x": 0.1,  "kw_x": 0.05, "ki_m_x": 0.1,
    "kR_y": 0.1,  "kw_y": 0.05, "ki_m_y": 0.1,
    "kR_z": 0.0,  "kw_z": 0.0,  "ki_m_z": 0.0,
    "kd_omega_rp": 0.1,
}


if __name__ == "__main__":
    P = dict(TRAJ_DEFAULTS)
    P["radius"] = 2.0
    P["period_s"] = 60.0
    for shape in ("circle", "figure8", "helix", "circle_vertical"):
        print("\n%s" % shape)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            pos, vel = evaluate(shape, P["period_s"] * frac, P)
            print("  t=%5.1fs  p=[%6.2f %6.2f %5.2f %5.1f]  v=[%6.3f %6.3f %6.3f]"
                  % (P["period_s"] * frac, pos[0], pos[1], pos[2], pos[3],
                     vel[0], vel[1], vel[2]))
        h, worst = 1e-5, 0.0
        for frac in [i / 40 for i in range(40)]:
            t0 = P["period_s"] * frac
            a, v = evaluate(shape, t0, P)
            b, _ = evaluate(shape, t0 + h, P)
            for i in range(3):
                worst = max(worst, abs((b[i] - a[i]) / h - v[i]))
        print("  max |numeric - analytic| velocity error: %.2e" % worst)
