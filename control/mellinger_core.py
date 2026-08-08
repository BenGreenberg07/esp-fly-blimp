#!/usr/bin/env python3
"""mellinger_core.py -- the SWING BLIMP controller.

A line-for-line port of `controller_mellinger.c` (the Crazyflie/Bitcraze Mellinger
SE(3) controller, in the form the reference blimp build uses) from C to Python,
plus the motor mixer for the new 4-motor canted airframe.

SELF-CONTAINED. Nothing here imports from, or is imported by, panel_server.py
or any of the decoupled-controller panels -- this is a separate test rig so the
proven decoupled blimp stack keeps working untouched.

WHY IT RUNS ON THE MAC AND NOT ON THE DRONE
-------------------------------------------
The reference controller needs the FULL attitude quaternion + body rates every
tick. The ESP-NOW pose frame (0xA6) carries only 8 floats -- position plus a single
YAW angle -- so the drone physically cannot run this controller without growing
that frame, which means reflashing the drone AND the C6 bridge every time it
changes. Running it here instead gets the full quaternion straight from NatNet
(Motive already streams it), keeps every gain live-tunable, and needs exactly ONE
firmware change ever (the swing mixer passthrough).

WHAT COMES OUT
--------------
`MellingerController.update()` returns the same four numbers the C writes into
`control_t`, with the same meanings as the reference build:

    control->thrust  = massThrust * compensated_fb.z    -> body-Z force
    control->roll    = scale_fb*massThrust*fb.x         -> body-X force
    control->pitch   = scale_fb*massThrust*fb.y         -> body-Y force
    control->yaw     = M.z                              -> body-Z moment

`SwingMixer` then turns that (Fx, Fy, Fz, Mz) wrench into four motor duties using
an explicit table of each motor's POSITION and THRUST AXIS. That table -- not
hardcoded algebra -- is what encodes "all four pointing up, two per side canted 45
degrees", so when the real cant turns out to be a few degrees off, or mounted about
a different axis, you change numbers instead of rewriting the mixer.

THE SWING
---------
With every motor axis canted purely sideways, the airframe has NO direct body-X
force at all: the Fx row of the allocation matrix is identically zero. That is the
whole point of the design -- forward motion comes from PITCHING the envelope and
letting the gondola swing, so the tilted lift vector pulls the craft along. The
mixer therefore routes any force component the motors cannot produce directly into
the corresponding MOMENT (swing_kx / swing_ky), which is exactly the trade the
`R_c_T` tilt-compensation block in the reference C is written around.
"""
import math

# ---------------------------------------------------------------- constants --
GRAVITY_MAGNITUDE = 9.81
CF_MASS = 0.027          # kg -- reference default; override with the `mass` gain


# ============================== cmath3d, in Python ==========================
# Direct equivalents of the Bitcraze cmath3d helpers the controller calls. Vectors
# are plain 3-tuples, matrices are row-major 3x3 tuples-of-tuples, quaternions are
# (x, y, z, w) to match mkquat().

def mkvec(x, y, z): return (x, y, z)
def vadd(a, b):     return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def vsub(a, b):     return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def vscl(s, a):     return (s * a[0], s * a[1], s * a[2])
def vdot(a, b):     return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def vmag(a):        return math.sqrt(vdot(a, a))
def fsqr(x):        return x * x
def radians(d):     return d * math.pi / 180.0
def degrees(r):     return r * 180.0 / math.pi


def vcross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vnormalize(a):
    m = vmag(a)
    return (0.0, 0.0, 0.0) if m < 1e-12 else vscl(1.0 / m, a)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def mcolumn(m, i):
    """i-th COLUMN of a row-major 3x3 -- the body axes live in the columns of R."""
    return (m[0][i], m[1][i], m[2][i])


def mcolumns(c0, c1, c2):
    return ((c0[0], c1[0], c2[0]),
            (c0[1], c1[1], c2[1]),
            (c0[2], c1[2], c2[2]))


def mtranspose(m):
    return ((m[0][0], m[1][0], m[2][0]),
            (m[0][1], m[1][1], m[2][1]),
            (m[0][2], m[1][2], m[2][2]))


def mvmul(m, v):
    return (m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2])


def mkquat(x, y, z, w): return (x, y, z, w)


def qnormalize(q):
    n = math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2)
    return (0.0, 0.0, 0.0, 1.0) if n < 1e-12 else (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat2rotmat(q):
    x, y, z, w = q
    return ((1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w,     2 * x * z + 2 * y * w),
            (2 * x * y + 2 * z * w,     1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w),
            (2 * x * z - 2 * y * w,     2 * y * z + 2 * x * w,     1 - 2 * x * x - 2 * y * y))


def rpy2quat(rpy):
    """ZYX (yaw-pitch-roll) Euler -> quaternion, matching cmath3d rpy2quat()."""
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat2rpy(q):
    x, y, z, w = q
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = math.asin(clamp(2 * (w * y - x * z), -1.0, 1.0))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (r, p, yw)


# ============================ the controller itself =========================
# Field-for-field the `controllerMellinger_t` struct. Each entry is
# (key, default, min, max, step, group, label) so the panel builds its sliders from it.
#
# DEFAULTS AND RANGES ARE THE REAL SBLIMP'S, from traj_original.py -- NOT the header
# values in controller_mellinger.c. That file ships Crazyflie-QUADROTOR tuning
# (kR_x 70000, kw_x 20000, kd_omega_rp 200); the flying blimp overrides every one of
# them with values ~6 orders of magnitude smaller (kR_x 0.1, kw_x 0.05, kd_omega_rp
# 0.1) and switches yaw off entirely (kR_z = kw_z = ki_m_z = 0). Keeping the C
# ranges would make the real values unselectable on a slider and the defaults wildly
# overdriven. The CONTROL LAW is unchanged -- only these numbers.
MEL_SPEC = [
    ("mass",        0.01,        0.002,   0.500,   0.001, "Vehicle",  "Total mass (kg)"),
    ("massThrust",  45000.0,    1000.0, 400000.0,  500.0, "Vehicle",  "Force -> PWM stretch factor"),
    ("scale_fb",    0.25,          0.0,     2.0,   0.01,  "Vehicle",  "Lateral feedback scale"),

    ("kp_x",        1.0,           0.0,     6.0,   0.01,  "Position", "X position P"),
    ("kd_x",        2.0,           0.0,     6.0,   0.01,  "Position", "X position D"),
    ("ki_x",        0.07,          0.0,     2.0,   0.005, "Position", "X position I"),
    ("i_range_x",   2.0,           0.0,    10.0,   0.1,   "Position", "X integral limit"),

    ("kp_y",        1.75,          0.0,     6.0,   0.01,  "Position", "Y position P"),
    ("kd_y",        1.0,           0.0,     6.0,   0.01,  "Position", "Y position D"),
    ("ki_y",        0.05,          0.0,     2.0,   0.005, "Position", "Y position I"),
    ("i_range_y",   2.0,           0.0,    10.0,   0.1,   "Position", "Y integral limit"),

    ("kp_z",        1.20,          0.0,    10.0,   0.01,  "Altitude", "Z position P"),
    ("kd_z",        0.5,           0.0,    10.0,   0.01,  "Altitude", "Z position D"),
    ("ki_z",        0.05,          0.0,     2.0,   0.005, "Altitude", "Z position I"),
    ("i_range_z",   0.4,           0.0,     5.0,   0.05,  "Altitude", "Z integral limit"),

    ("kR_x",        0.1,           0.0,     5.0,   0.005, "Attitude", "Roll attitude P"),
    ("kw_x",        0.05,          0.0,     5.0,   0.005, "Attitude", "Roll rate D"),
    ("ki_m_x",      0.1,           0.0,     5.0,   0.005, "Attitude", "Roll attitude I"),
    ("i_range_m_x", 1.0,           0.0,  5000.0,    1.0, "Attitude", "Roll integral limit"),

    ("kR_y",        0.1,           0.0,     5.0,   0.005, "Attitude", "Pitch attitude P"),
    ("kw_y",        0.05,          0.0,     5.0,   0.005, "Attitude", "Pitch rate D"),
    ("ki_m_y",      0.1,           0.0,     5.0,   0.005, "Attitude", "Pitch attitude I"),
    ("i_range_m_y", 1500.0,        0.0,   5000.0,   10.0, "Attitude", "Pitch integral limit"),

    ("kR_z",        0.0,           0.0,     5.0,   0.005, "Yaw",      "Yaw attitude P"),
    ("kw_z",        0.0,           0.0,     5.0,   0.005, "Yaw",      "Yaw rate D"),
    ("ki_m_z",      0.0,           0.0,     5.0,   0.005, "Yaw",      "Yaw attitude I"),
    ("i_range_m_z", 1500.0,        0.0,   5000.0,   10.0, "Yaw",      "Yaw integral limit"),

    ("kd_omega_rp", 0.1,           0.0,     5.0,   0.005, "Attitude", "Roll/pitch rate D (2nd order)"),
]
MEL_DEFAULTS = {k: d for (k, d, _lo, _hi, _st, _g, _l) in MEL_SPEC}
# Present in the reference controller's struct + PARAM_GROUP(ctrlMel) but never read by
# its control law. Mirrored here so the object matches the C struct field-for-field, and
# kept OUT of MEL_SPEC so the panel doesn't show sliders that visibly do nothing.
MEL_DEFAULTS["switch_fx"] = 1.0
MEL_DEFAULTS["switch_fy"] = 1.0
MEL_LIMITS = {k: (lo, hi) for (k, _d, lo, hi, _st, _g, _l) in MEL_SPEC}


class MellingerController:
    """Port of controllerMellinger(). One instance per flight; call update() per tick.

    DELIBERATE DEVIATION FROM THE C, and the only one: the reference uses a FIXED
    `dt = 1/ATTITUDE_RATE` (500 Hz) because it is called from the 500 Hz stabilizer
    task. This runs at the panel's tick rate (20-30 Hz off NatNet), so it integrates
    with the REAL measured dt instead. Using the C's hardcoded 1/500 here would make
    every integral term and every derivative ~20x wrong. Set use_fixed_dt=True for a
    bit-identical-to-C comparison run.
    """

    def __init__(self, gains=None, use_fixed_dt=False, attitude_rate=500.0):
        self.use_fixed_dt = use_fixed_dt
        self.attitude_rate = attitude_rate
        for k, v in MEL_DEFAULTS.items():
            setattr(self, k, v)
        if gains:
            self.set_gains(gains)
        self.reset()

    def set_gains(self, gains):
        for k, v in (gains or {}).items():
            if k in MEL_DEFAULTS:
                setattr(self, k, float(v))

    def reset(self):
        """controllerMellingerReset()."""
        self.i_error_x = 0.0
        self.i_error_y = 0.0
        self.i_error_z = 0.0
        self.i_error_m_x = 0.0
        self.i_error_m_y = 0.0
        self.i_error_m_z = 0.0
        # D-part-initialised sentinel (the C uses a NaN self-compare for this)
        self.prev_omega_roll = None
        self.prev_omega_pitch = None
        self.prev_setpoint_omega_roll = 0.0
        self.prev_setpoint_omega_pitch = 0.0
        # telemetry mirrors of the LOG_GROUP(ctrlMel) variables
        self.cmd_thrust = self.cmd_roll = self.cmd_pitch = self.cmd_yaw = 0.0
        self.z_axis_desired = (0.0, 0.0, 1.0)
        self.M = (0.0, 0.0, 0.0)
        self.unclamped_M = (0.0, 0.0, 0.0)
        self.compensated_fb = (0.0, 0.0, 0.0)

    # ---------------------------------------------------------------- update --
    def update(self, setpoint, state, sensors, dt):
        """One tick of controllerMellinger().

        setpoint: dict with position/velocity/acceleration {x,y,z}, attitude
                  {roll,pitch,yaw} in DEG, attitudeRate {roll,pitch,yaw} in DEG/S,
                  mode {x,y,z,yaw,quat} using "abs" | "velocity" | "disable",
                  thrust (used only when mode.z == "disable")
        state:    position/velocity {x,y,z}, attitudeQuaternion {x,y,z,w},
                  attitude {yaw} in DEG
        sensors:  gyro {x,y,z} in DEG/S (body rates)
        Returns (control, log); control has thrust/roll/pitch/yaw exactly as the C
        writes them into control_t.
        """
        if self.use_fixed_dt:
            dt = 1.0 / self.attitude_rate
        dt = max(1e-4, float(dt))

        sp_mode = setpoint.get("mode", {})
        sp_pos = setpoint.get("position", {})
        sp_vel = setpoint.get("velocity", {})
        sp_acc = setpoint.get("acceleration", {})
        sp_att = setpoint.get("attitude", {})
        sp_rate = setpoint.get("attitudeRate", {})

        setpointPos = mkvec(sp_pos.get("x", 0.0), sp_pos.get("y", 0.0), sp_pos.get("z", 0.0))
        setpointVel = mkvec(sp_vel.get("x", 0.0), sp_vel.get("y", 0.0), sp_vel.get("z", 0.0))
        statePos = mkvec(state["position"]["x"], state["position"]["y"], state["position"]["z"])
        stateVel = mkvec(state["velocity"]["x"], state["velocity"]["y"], state["velocity"]["z"])

        # Position Error (ep)
        r_error = vsub(setpointPos, statePos)
        # Velocity Error (ev)
        v_error = vsub(setpointVel, stateVel)

        # Integral Error
        self.i_error_z = clamp(self.i_error_z + r_error[2] * dt, -self.i_range_z, self.i_range_z)
        self.i_error_x = clamp(self.i_error_x + r_error[0] * dt, -self.i_range_x, self.i_range_x)
        self.i_error_y = clamp(self.i_error_y + r_error[1] * dt, -self.i_range_y, self.i_range_y)

        # Desired thrust [F_des]
        if sp_mode.get("x") == "abs":
            tt_x = (self.mass * sp_acc.get("x", 0.0)
                    + self.kp_x * r_error[0] + self.kd_x * v_error[0] + self.ki_x * self.i_error_x)
            tt_y = (self.mass * sp_acc.get("y", 0.0)
                    + self.kp_y * r_error[1] + self.kd_y * v_error[1] + self.ki_y * self.i_error_y)
            tt_z = (self.mass * (sp_acc.get("z", 0.0) + GRAVITY_MAGNITUDE)
                    + self.kp_z * r_error[2] + self.kd_z * v_error[2] + self.ki_z * self.i_error_z)
        else:
            tt_x = -math.sin(radians(sp_att.get("pitch", 0.0)))
            tt_y = -math.sin(radians(sp_att.get("roll", 0.0)))
            # In case of a timeout, the commander tries to level, ie. x/y are disabled,
            # but z will use the previous setting; then ignore the accel feedforward.
            if sp_mode.get("z") == "abs":
                tt_z = (self.mass * GRAVITY_MAGNITUDE
                        + self.kp_z * r_error[2] + self.kd_z * v_error[2] + self.ki_z * self.i_error_z)
            else:
                tt_z = 1.0
        target_thrust = mkvec(tt_x, tt_y, tt_z)

        # Rate-controlled YAW is moving YAW angle setpoint
        desiredYaw = 0.0
        if sp_mode.get("yaw") == "velocity":
            desiredYaw = state["attitude"]["yaw"] + sp_rate.get("yaw", 0.0) * dt
        elif sp_mode.get("yaw") == "abs":
            desiredYaw = sp_att.get("yaw", 0.0)
        elif sp_mode.get("quat") == "abs":
            sq = setpoint.get("attitudeQuaternion", {})
            rpy = quat2rpy(mkquat(sq.get("x", 0.0), sq.get("y", 0.0),
                                  sq.get("z", 0.0), sq.get("w", 1.0)))
            desiredYaw = degrees(rpy[2])

        # Z-Axis [zB]
        sq = state["attitudeQuaternion"]
        q = qnormalize(mkquat(sq["x"], sq["y"], sq["z"], sq["w"]))
        R = quat2rotmat(q)
        z_axis = mcolumn(R, 2)
        x_axis = mcolumn(R, 0)
        y_axis = mcolumn(R, 1)

        # yaw correction (only if position control is not used)
        if sp_mode.get("x") != "abs":
            x_yaw = mcolumn(R, 0)
            x_yaw = vnormalize((x_yaw[0], x_yaw[1], 0.0))
            y_yaw = vcross(mkvec(0, 0, 1), x_yaw)
            R_yaw_only = mcolumns(x_yaw, y_yaw, mkvec(0, 0, 1))
            target_thrust = mvmul(R_yaw_only, target_thrust)

        # desired thrust [F] in {B}
        current_thrust = vdot(target_thrust, z_axis)
        current_thrust_x = vdot(target_thrust, x_axis)
        current_thrust_y = vdot(target_thrust, y_axis)

        # this thing works for compensating the tilting and yaw
        R_des = quat2rotmat(rpy2quat(mkvec(0, 0, radians(desiredYaw))))
        z_axis_desired = mcolumn(R_des, 2)
        x_axis_desired = mcolumn(R_des, 0)
        y_axis_desired = mcolumn(R_des, 1)
        self.z_axis_desired = z_axis_desired

        # [eR] -- the fast Mathematica-generated form, transcribed verbatim
        x, y, z, w = q
        xad, yad, zad = x_axis_desired
        xbd, ybd, zbd = y_axis_desired
        xcd, ycd, zcd = z_axis_desired

        eR_x = ((-1 + 2 * fsqr(x) + 2 * fsqr(y)) * zbd + ycd
                - 2 * (x * xbd * z + y * ybd * z - x * y * xcd + fsqr(x) * ycd
                       + fsqr(z) * ycd - y * z * zcd)
                + 2 * w * (-(y * xbd) - z * xcd + x * (ybd + zcd)))
        eR_y = (zad - xcd
                - 2 * (fsqr(x) * zad + y * (zad * y - yad * z) - (fsqr(y) + fsqr(z)) * xcd
                       + x * (-(xad * z) + y * ycd + z * zcd)
                       + w * (x * yad + z * ycd - y * (xad + zcd))))
        eR_z = (xbd
                - 2 * (y * (x * xad + y * xbd - x * ybd) + w * (x * zad + y * zbd))
                + 2 * (-(zad * y) + w * (xad + ybd) + x * zbd) * z
                - 2 * xbd * fsqr(z)
                + yad * (-1 + 2 * fsqr(x) + 2 * fsqr(z)))
        eR = (-eR_x, -eR_y, -eR_z)

        # [ew]
        err_d_roll = 0.0
        err_d_pitch = 0.0
        stateAttitudeRateRoll = radians(sensors["gyro"]["x"])
        stateAttitudeRatePitch = radians(sensors["gyro"]["y"])
        stateAttitudeRateYaw = radians(sensors["gyro"]["z"])

        sp_rr = radians(sp_rate.get("roll", 0.0))
        sp_rp = radians(sp_rate.get("pitch", 0.0))
        sp_ry = radians(sp_rate.get("yaw", 0.0))
        ew = (sp_rr - stateAttitudeRateRoll,
              sp_rp - stateAttitudeRatePitch,
              sp_ry - stateAttitudeRateYaw)
        if self.prev_omega_roll is not None:      # C: `prev_omega_roll == prev_omega_roll`
            err_d_roll = ((sp_rr - self.prev_setpoint_omega_roll)
                          - (stateAttitudeRateRoll - self.prev_omega_roll)) / dt
            err_d_pitch = ((sp_rp - self.prev_setpoint_omega_pitch)
                           - (stateAttitudeRatePitch - self.prev_omega_pitch)) / dt
        self.prev_omega_roll = stateAttitudeRateRoll
        self.prev_omega_pitch = stateAttitudeRatePitch
        self.prev_setpoint_omega_roll = sp_rr
        self.prev_setpoint_omega_pitch = sp_rp

        # Integral Error
        self.i_error_m_x = clamp(self.i_error_m_x + eR[0] * dt, -self.i_range_m_x, self.i_range_m_x)
        self.i_error_m_y = clamp(self.i_error_m_y + eR[1] * dt, -self.i_range_m_y, self.i_range_m_y)
        self.i_error_m_z = clamp(self.i_error_m_z + eR[2] * dt, -self.i_range_m_z, self.i_range_m_z)

        # Moment
        M_x = (self.kR_x * eR[0] + self.kw_x * ew[0]
               + self.ki_m_x * self.i_error_m_x + self.kd_omega_rp * err_d_roll)
        M_y = (self.kR_y * eR[1] + self.kw_y * ew[1]
               + self.ki_m_y * self.i_error_m_y + self.kd_omega_rp * err_d_pitch)
        M_z = (self.kR_z * eR[2] + self.kw_z * ew[2] + self.ki_m_z * self.i_error_m_z)

        self.unclamped_M = (M_x, M_y, M_z)
        M_x = clamp(M_x, -0.2, 0.2)
        M_y = clamp(M_y, -0.2, 0.2)
        M = (M_x, M_y, M_z)
        self.M = M

        R_c_T = mtranspose(quat2rotmat(rpy2quat(mkvec(M[0], M[1], 0))))
        compensated_fb = mvmul(R_c_T, mkvec(current_thrust_x, current_thrust_y, current_thrust))
        self.compensated_fb = compensated_fb

        # Output
        if sp_mode.get("z") == "disable":
            thrust = float(setpoint.get("thrust", 0.0))
        else:
            thrust = self.massThrust * compensated_fb[2]

        self.cmd_thrust = thrust
        if thrust > 0:
            # NOTE: switch_fx / switch_fy are deliberately NOT applied here. They exist
            # in the reference controller's struct and its PARAM_GROUP, but its control
            # law never references them -- so applying them would be a behaviour change,
            # not a port. Kept as attributes to mirror the struct; see MEL_SPEC.
            roll = clamp(self.scale_fb * self.massThrust * compensated_fb[0], -16000.0, 16000.0)
            pitch = clamp(self.scale_fb * self.massThrust * compensated_fb[1], -16000.0, 16000.0)
            yaw = clamp(M[2], -16000.0, 16000.0)
        else:
            roll = pitch = yaw = 0.0
            self.reset()
        self.cmd_roll, self.cmd_pitch, self.cmd_yaw = roll, pitch, yaw

        control = {"thrust": thrust, "roll": roll, "pitch": pitch, "yaw": yaw}
        log = {
            "cmd_thrust": thrust, "cmd_roll": roll, "cmd_pitch": pitch, "cmd_yaw": yaw,
            "Mx": M[0], "My": M[1], "Mz": M[2],
            "unclamped_Mx": self.unclamped_M[0], "unclamped_My": self.unclamped_M[1],
            "unclamped_Mz": self.unclamped_M[2],
            "Fx_body": compensated_fb[0], "Fy_body": compensated_fb[1], "Fz_body": compensated_fb[2],
            "eRx": eR[0], "eRy": eR[1], "eRz": eR[2],
            "i_err_x": self.i_error_x, "i_err_y": self.i_error_y, "i_err_z": self.i_error_z,
            "err_x": r_error[0], "err_y": r_error[1], "err_z": r_error[2],
        }
        return control, log


# ================================ swing mixer ===============================
SQ2 = 1.0 / math.sqrt(2.0)
PWM_FULL = 65535.0        # the PWM domain massThrust stretches the wrench into

# Body frame is x FORWARD, y LEFT, z UP -- the same convention the controller's R
# columns use. Motor order is ALWAYS (M1, M2, M3, M4) = the firmware's physical
# channels; run motor_test.py to confirm which physical motor each channel drives
# before flying. X-frame arm ends, arms at +/-45 deg from the nose.
_ARM_POS = {
    "M1": (SQ2, -SQ2, 0.0),    # front-right
    "M2": (-SQ2, -SQ2, 0.0),   # rear-right
    "M3": (-SQ2, SQ2, 0.0),    # rear-left
    "M4": (SQ2, SQ2, 0.0),     # front-left
}

# Which HORIZONTAL direction each motor's thrust axis leans toward. The axis itself
# is built as  a = cos(cant)*z_hat + sin(cant)*lean,  so cant=0 is a plain upward
# quad and cant=45 is this build.
CANT_LAYOUTS = {
    # "2 on each side canted 45 deg": LEFT pair leans left, RIGHT pair leans right.
    # Direct sideways force + yaw, but NO direct forward force -- forward is the
    # swing. Default reading of the photo.
    "lateral": {"M1": (0.0, -1.0, 0.0), "M2": (0.0, -1.0, 0.0),
                "M3": (0.0, 1.0, 0.0),  "M4": (0.0, 1.0, 0.0)},
    # Front pair leans forward, rear pair leans back: direct forward force, sideways
    # is the swing instead. Pick this if the mounts are canted nose/tail.
    "foreaft": {"M1": (1.0, 0.0, 0.0),  "M2": (-1.0, 0.0, 0.0),
                "M3": (-1.0, 0.0, 0.0), "M4": (1.0, 0.0, 0.0)},
    # Every motor leans tangentially around the frame. COLLECTIVELY the four lean
    # directions cancel, so equal thrust on all four is pure yaw -- but they do NOT
    # cancel differentially: front-vs-rear gives net Fx and left-vs-right gives net
    # Fy. So this layout is actually FULLY ACTUATED in x, y, z and yaw, and needs no
    # swing at all. Pick it only if the mounts really are rotated about the arms.
    "tangential": {"M1": (SQ2, SQ2, 0.0),   "M2": (SQ2, -SQ2, 0.0),
                   "M3": (-SQ2, -SQ2, 0.0), "M4": (-SQ2, SQ2, 0.0)},
}

MIX_SPEC = [
    ("cant_deg",  45.0,   0.0,  90.0, 1.0,   "Geometry", "Motor cant from vertical (deg)"),
    # Measured on the real frame: ~30-35 mm from the ESP hub centre to each motor.
    ("arm_m",     0.032, 0.01,  0.20, 0.002, "Geometry", "Arm length, hub to motor (m)"),
    # Sized so a swing-driven axis gets roughly the same duty split as a directly
    # actuated one at the same force demand (measured: 0.086 either way on the real
    # 32 mm arm). NOTE this value is arm-length dependent -- the moment is converted
    # to a vertical differential by dividing by the lever arm, so HALVING arm_m
    # DOUBLES the effective swing gain. Re-check it if the frame changes: at the
    # earlier 90 mm guess the matching value was ~0.12, at 32 mm it is ~0.045.
    ("swing_kx",  0.045,  0.0,  0.50, 0.005, "Swing",    "Forward force -> pitch moment"),
    ("swing_ky",  0.045,  0.0,  0.50, 0.005, "Swing",    "Sideways force -> roll moment"),
    ("yaw_scale", 1.0,   -4.0,   4.0, 0.05,  "Output",   "Yaw authority (sign flips direction)"),
    ("hover",     0.30,   0.0,   1.0, 0.01,  "Output",   "Hover duty feed-forward (buoyancy trim)"),
    ("duty_max",  0.60,   0.0,   1.0, 0.01,  "Output",   "Per-motor duty cap"),
    ("force_gain", 1.0,    0.0,  8.0, 0.05,  "Output",   "Wrench -> thrust scale (sim to real)"),
]
MIX_DEFAULTS = {k: d for (k, d, _lo, _hi, _st, _g, _l) in MIX_SPEC}
MIX_LIMITS = {k: (lo, hi) for (k, _d, lo, hi, _st, _g, _l) in MIX_SPEC}


class SwingMixer:
    """(Fx, Fy, Fz, Mz) wrench -> four motor duties, for the canted swing airframe.

    Builds the 4x4 allocation matrix A from the geometry so that  A @ f = wrench,
    where f is the per-motor thrust. Motor i contributes force a_i*f_i and moment
    cross(p_i, a_i)*f_i. Rows are [Fx, Fy, Fz, Mz].

    A is solved with a DAMPED least-squares (Tikhonov) inverse rather than a plain
    inverse, because for this airframe A is singular BY DESIGN -- with a purely
    lateral cant the Fx row is all zeros. Damping makes that request degrade to "do
    nothing in X" instead of exploding, and swing_kx/swing_ky then supply the missing
    axis as a MOMENT, the way the airframe actually produces it.
    """

    def __init__(self, layout="lateral", params=None, motor_enable=(1, 1, 1, 1),
                 motor_map=(0, 1, 2, 3), damping=1e-3):
        self.layout = layout if layout in CANT_LAYOUTS else "lateral"
        self.motor_enable = tuple(motor_enable)
        self.set_motor_map(motor_map)
        self.damping = damping
        for k, v in MIX_DEFAULTS.items():
            setattr(self, k, v)
        if params:
            self.set_params(params)
        self._rebuild()

    def set_params(self, params):
        changed = False
        for k, v in (params or {}).items():
            if k in MIX_DEFAULTS:
                setattr(self, k, float(v)); changed = True
            elif k == "layout" and v in CANT_LAYOUTS:
                self.layout = v; changed = True
        if changed:
            self._rebuild()

    def set_motor_map(self, m):
        """Which physical firmware channel drives each ARM POSITION.

        motor_map[i] = channel index (0..3 -> m1..m4) for arm position i, where the
        positions are ordered (front-right, rear-right, rear-left, front-left).

        This is kept separate from the geometry on purpose: the cant layout describes
        how the airframe is BUILT, this describes how it happens to be WIRED. Getting
        a motor backwards should be a one-line wiring fix, not an edit to the physics.

        Must be a permutation of 0..3 -- anything else would drive one channel twice
        and leave another dead, so a bad map falls back to identity rather than
        silently flying with two motors off.
        """
        try:
            m = tuple(int(v) for v in m)
        except Exception:
            m = (0, 1, 2, 3)
        if sorted(m) != [0, 1, 2, 3]:
            m = (0, 1, 2, 3)
        self.motor_map = m
        return self.motor_map

    def axes_and_positions(self):
        c = math.cos(radians(self.cant_deg))
        s = math.sin(radians(self.cant_deg))
        dirs = CANT_LAYOUTS[self.layout]
        out = []
        for name in ("M1", "M2", "M3", "M4"):
            p = vscl(self.arm_m, _ARM_POS[name])
            d = vnormalize(dirs[name])
            a = vnormalize((s * d[0], s * d[1], c))
            out.append((p, a))
        return out

    def _rebuild(self):
        """A[row][motor], rows = Fx, Fy, Fz, Mz."""
        cols = []
        for p, a in self.axes_and_positions():
            m = vcross(p, a)
            cols.append((a[0], a[1], a[2], m[2]))
        self.A = [[cols[j][i] for j in range(4)] for i in range(4)]
        self.Ainv = _damped_pinv(self.A, self.damping)
        # How much body-X / body-Y force the airframe can actually DELIVER, 0..1:
        # ask the allocator for one unit of that axis, then measure how much of it
        # comes back out. This is diag(A @ Ainv).
        #
        # It deliberately is NOT "sum of the |contributions| in that row" -- that
        # counts motors which cancel each other. The tangential layout is exactly
        # that trap: every motor has a big horizontal component, but they point
        # around the frame and sum to no net force at all, so a row-magnitude test
        # calls it fully actuated when its true translational authority is zero.
        AA = [[sum(self.A[i][k] * self.Ainv[k][j] for k in range(4)) for j in range(4)]
              for i in range(4)]
        self.auth_x = clamp(AA[0][0], 0.0, 1.0)
        self.auth_y = clamp(AA[1][1], 0.0, 1.0)

    def mix(self, Fx, Fy, Fz, Mz):
        """Wrench (controller units) -> (duties 0..1 per motor, info dict).

        The controller's outputs are in the reference firmware's PWM domain, not
        newtons: `massThrust` has already stretched force into 0..65535 counts, and
        roll/pitch are clamped to +/-16000 counts. Normalising by PWM_FULL here is
        what makes `hover`, `duty_max` and the returned duties all mean the same
        thing -- fraction of full motor duty."""
        g = self.force_gain / PWM_FULL
        Fx, Fy, Fz, Mz = Fx * g, Fy * g, Fz * g, Mz * g

        # Route the force the airframe CANNOT make directly into a swing moment. A
        # nose-down pitch drives the craft forward, so +Fx wants -My; a roll toward +y
        # drives it left, so +Fy wants +Mx. Those moments are realised as the vertical
        # front/rear and left/right differentials below.
        lack_x = 1.0 - min(1.0, self.auth_x)
        lack_y = 1.0 - min(1.0, self.auth_y)
        My_cmd = -self.swing_kx * Fx * lack_x
        Mx_cmd = self.swing_ky * Fy * lack_y

        w = (Fx, Fy, Fz, Mz * self.yaw_scale)
        f = [sum(self.Ainv[i][j] * w[j] for j in range(4)) for i in range(4)]

        # Each motor's vertical lever arms are px (pitch) and py (roll), so a pitch
        # moment is +/-px-weighted and a roll moment +/-py-weighted.
        denom = 4.0 * max(1e-4, self.arm_m) ** 2
        for i, (p, a) in enumerate(self.axes_and_positions()):
            az = max(0.15, a[2])                  # vertical share of this motor's thrust
            f[i] += (-My_cmd * p[0] + Mx_cmd * p[1]) / (az * denom)

        # Duty = hover feed-forward + allocated thrust, clipped. Brushed motors are
        # unidirectional, so negatives become 0 -- the same limitThrust() floor the
        # firmware applies, done here so the panel readout matches what actually flies.
        #
        # f[] is in ARM-POSITION order; motor_map routes each position to the physical
        # channel that actually drives it, so the returned list is in CHANNEL order
        # (m1..m4) ready for the wire. motor_enable is applied after the permutation
        # because a muted motor is a physical fact (unplugged, dead) about a channel.
        duties = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            duties[self.motor_map[i]] = clamp(self.hover + f[i], 0.0, self.duty_max)
        for ch in range(4):
            if not self.motor_enable[ch]:
                duties[ch] = 0.0

        info = {"f": [round(v, 4) for v in f], "duty": [round(v, 3) for v in duties],
                "motor_map": list(self.motor_map),
                "auth_x": round(self.auth_x, 3), "auth_y": round(self.auth_y, 3),
                "Mx_cmd": round(Mx_cmd, 4), "My_cmd": round(My_cmd, 4),
                "layout": self.layout}
        return duties, info


def _damped_pinv(A, lam):
    """(A^T A + lam I)^-1 A^T for a 4x4 -- stable even when A is rank-deficient."""
    n = 4
    AtA = [[sum(A[k][i] * A[k][j] for k in range(n)) + (lam if i == j else 0.0)
            for j in range(n)] for i in range(n)]
    At = [[A[j][i] for j in range(n)] for i in range(n)]
    inv = _inv(AtA)
    return [[sum(inv[i][k] * At[k][j] for k in range(n)) for j in range(n)] for i in range(n)]


def _inv(M):
    """Gauss-Jordan inverse of a 4x4 (keeps the control path numpy-free)."""
    n = len(M)
    a = [list(M[i]) + [1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            a[col][col] += 1e-9
            piv = col
        a[col], a[piv] = a[piv], a[col]
        d = a[col][col]
        a[col] = [v / d for v in a[col]]
        for r in range(n):
            if r != col and a[r][col] != 0.0:
                fct = a[r][col]
                a[r] = [vr - fct * vc for vr, vc in zip(a[r], a[col])]
    return [row[n:] for row in a]


# ================================ self-test =================================
if __name__ == "__main__":
    # Hold station, then nudge the target forward / sideways / up and confirm the
    # duties split the way the geometry says they should.
    c = MellingerController()
    mx = SwingMixer()
    base = {"position": {"x": 0.0, "y": 0.0, "z": 1.0},
            "velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            "attitudeQuaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            "attitude": {"yaw": 0.0}}
    sens = {"gyro": {"x": 0.0, "y": 0.0, "z": 0.0}}

    def sp(x, y, z):
        return {"mode": {"x": "abs", "y": "abs", "z": "abs", "yaw": "abs"},
                "position": {"x": x, "y": y, "z": z},
                "velocity": {"x": 0, "y": 0, "z": 0},
                "acceleration": {"x": 0, "y": 0, "z": 0},
                "attitude": {"yaw": 0.0},
                "attitudeRate": {"roll": 0, "pitch": 0, "yaw": 0}}

    print("motor order M1 front-right, M2 rear-right, M3 rear-left, M4 front-left\n")
    for name, tgt in (("hold", (0, 0, 1.0)), ("forward", (1.0, 0, 1.0)),
                      ("left", (0, 1.0, 1.0)), ("climb", (0, 0, 1.5))):
        c.reset()
        for _ in range(5):
            ctl, lg = c.update(sp(*tgt), base, sens, 0.05)
        d, info = mx.mix(ctl["roll"], ctl["pitch"], ctl["thrust"], ctl["yaw"])
        print("%-8s Fx=%9.1f Fy=%9.1f Fz=%9.1f Mz=%8.3f  duty=%s" %
              (name, ctl["roll"], ctl["pitch"], ctl["thrust"], ctl["yaw"],
               ["%.3f" % v for v in d]))
    print("\nallocation rows [Fx,Fy,Fz,Mz]:")
    for lab, r in zip(("Fx", "Fy", "Fz", "Mz"), mx.A):
        print("  %s %s" % (lab, ["%+.3f" % v for v in r]))
    print("authority  x=%.3f  y=%.3f  (near 0 => that axis is swing-only)"
          % (mx.auth_x, mx.auth_y))
