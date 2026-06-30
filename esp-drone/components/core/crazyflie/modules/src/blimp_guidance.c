/**
 * blimp_guidance.c - On-board decoupled PID guidance for the ESP-FLY blimp.
 *
 * ============================ THE PHYSICS ============================
 * A helium blimp is fundamentally different from a quadrotor:
 *
 *   - It is BUOYANT, so it does not have to spend thrust fighting gravity.
 *   - The gondola hangs well below the envelope (a long pendulum), so the
 *     craft is PASSIVELY STABLE in roll and pitch -- left alone it hangs
 *     level and self-rights. We therefore DO NOT need an attitude (tilt)
 *     control loop at all. This is why we throw away the quad's inner
 *     rate/attitude PID cascade.
 *
 *   - Its actuation is fully DECOUPLED: two forward motors push along the
 *     body X axis, one motor pushes up, one pushes down. Vertical motion is
 *     mechanically independent from horizontal motion. Because the axes do
 *     not fight each other, three INDEPENDENT single-input/single-output PID
 *     loops are sufficient -- no cross-coupling terms, no state matrix.
 *
 * This is the "decoupled PID" architecture. Each loop maps one error signal
 * to one actuator group:
 *
 *   1. ALTITUDE LOOP  (vertical):
 *        error  = z_target - z_now                     (meters, world up)
 *        u_z    = Kp*e + Ki*integral(e) + Kd*d/dt(e) + z_ff
 *        e>0 (too low)  -> spin UP motor;  e<0 -> spin DOWN motor.
 *        z_ff is a small constant feed-forward that cancels any residual
 *        non-neutral buoyancy so the integral term does not have to.
 *
 *   2. HEADING LOOP  (yaw):
 *        bearing = atan2(y_target - y_now, x_target - x_now)  (world)
 *        error   = wrap(bearing - yaw_now)             (radians, [-pi,pi])
 *        u_yaw   = Kp*error - Kd*yawrate
 *        The D term uses the MEASURED gyro yaw-rate (not a differentiated
 *        error) -- this is rate damping and is what stops the nose from
 *        oscillating / overshooting the target bearing. Output is applied
 *        DIFFERENTIALLY to the two forward motors (turn right = more left
 *        motor, less right motor).
 *
 *   3. FORWARD LOOP  (surge):
 *        range = sqrt(dx^2 + dy^2)                     (meters to target)
 *        u_fwd = Kp*range, clamped to [0, fwd_max]
 *        Gated by heading: we only push forward when we are roughly facing
 *        the target, scaled by cos(heading_error). As range -> 0 the command
 *        spins down so we coast to a stop instead of overshooting.
 *
 * Motor mixing (done downstream in power_distribution_stock.c):
 *        Motor_FwdLeft  = u_fwd + u_yaw
 *        Motor_FwdRight = u_fwd - u_yaw
 *        Motor_Up       =  u_z   (if u_z > 0)
 *        Motor_Down     = -u_z   (if u_z < 0)
 *
 * ============================ THE CODE ==============================
 * The host (Mac) does NOT compute any of this. It only measures where the
 * blimp is (mocap) and where we want it, and streams those numbers into the
 * CRTP `mocap` param group. Everything above runs HERE, on the drone, every
 * control tick. Gains live in the CRTP `blimpc` param group so they are tuned
 * live over the link with no reflash.
 *
 * Output convention matches the existing BLIMP_MODE manual path so it reuses
 * the proven mixer, motor map and per-axis limits:
 *        control->thrust = forward PWM   (0 .. fwdMaxPwm)
 *        control->yaw    = turn PWM      (+/- turnMaxPwm)   differential
 *        control->pitch  = vertical PWM  (+/- vertMaxPwm)   (mixer * vertScale)
 * ====================================================================
 */

#include <math.h>
#include "blimp_guidance.h"
#include "param.h"
#include "log.h"
#include "num.h"
#include "esp_timer.h"

/* ---- Host-streamed mocap state (CRTP param group "mocap") ---------------
 * Current pose comes from the motion-capture rig; target pose is the waypoint.
 * Positions in meters, yaw in DEGREES (mocap world frame). The blimp is
 * pendulum-stable so roll/pitch are not needed -- only yaw orientation. The
 * host bumps `seq` on every update; we use that as a liveness heartbeat. */
static float mc_cx = 0.0f, mc_cy = 0.0f, mc_cz = 0.0f, mc_cyaw = 0.0f; // current
static float mc_tx = 0.0f, mc_ty = 0.0f, mc_tz = 0.0f, mc_tyaw = 0.0f; // target
static uint32_t mc_seq = 0;          // host increments each fresh sample

/* ---- Guidance gains / limits (CRTP param group "blimpc") ----------------
 * All live-tunable over Wi-Fi. Conservative, gentle defaults: a blimp is slow
 * and you want to creep up on these. Output max PWMs mirror the proven manual
 * "good-flying" powers (fwd~0.30, turn~0.30, up/down~0.80 of full scale). */
static uint8_t bc_auto      = 0;     // 0 = manual passthrough, 1 = autonomous

// Altitude loop
static float bc_kp_z   = 12000.0f;   // PWM per meter of height error
static float bc_ki_z   = 1500.0f;    // PWM per (meter*second)
static float bc_kd_z   = 6000.0f;    // PWM per (meter/second) of climb rate
static float bc_zff    = 0.0f;       // constant buoyancy feed-forward, PWM
static float bc_iLim_z = 8000.0f;    // |Ki*integral| clamp (anti-windup), PWM

// Heading loop — CASCADE (heading -> rate-limited). A big helium envelope has
// high yaw inertia and almost no natural damping, and the forward motors are
// unidirectional (no reverse braking), so we never command turn *power* directly.
// Instead: heading error -> a desired turn RATE (capped), then drive the gyro to
// it. The cap means it can never wind up into a spin faster than the differential
// thrust can cancel; at the target heading the desired rate is 0 so it brakes.
static float bc_yawKpHead  = 25.0f;  // heading err (rad) -> desired yaw rate (deg/s)
static float bc_yawRateMax = 30.0f;  // cap on desired yaw rate (deg/s)
static float bc_yawKpRate  = 0.02f;  // (deg/s of rate error) -> normalized turn

// Forward loop
static float bc_kp_fwd   = 0.6f;     // (normalized fwd) per meter of range
static float bc_fwdMaxN  = 1.0f;     // cap on normalized forward [0..1]
static float bc_arriveR  = 0.25f;    // m: inside this, stop & hold target yaw
static float bc_headGate  = 60.0f;   // deg: don't drive fwd if mis-pointed beyond
static float bc_kdFwd     = 0.5f;    // VELOCITY DAMPING: (normalized fwd) per (m/s)
                                     // of closing speed. The motors can't reverse to
                                     // brake, so this eases off thrust as the blimp
                                     // approaches fast -> it coasts to the mark
                                     // instead of overshooting. Raise to stop sooner.

// Horizontal velocity estimate, finite-differenced from successive mocap samples
// (low-pass filtered) so we know the closing speed without the host sending it.
static float s_velX = 0.0f, s_velY = 0.0f;
static int64_t s_lastMocapUs = 0;

// Output scaling (normalized command -> PWM counts)
static float bc_fwdMaxPwm  = 18000.0f;  // forward full scale
static float bc_turnMaxPwm = 9000.0f;   // turn  full scale (differential)
static float bc_vertMaxPwm = 16000.0f;  // vertical full scale (int16, *vertScale)

// Failsafe: if no new mocap sample within this many ms, stop driving.
static uint32_t bc_staleMs = 500;

// Telemetry (CRTP log group "blimpc")
static float lg_range = 0.0f, lg_yawErr = 0.0f, lg_zErr = 0.0f;
static float lg_uFwd = 0.0f, lg_uTurn = 0.0f, lg_uVert = 0.0f;

// Internal state
static float zIntegral = 0.0f;
static uint32_t lastSeqSeen = 0;
static uint32_t lastFreshMs = 0;
static bool s_extAuto = false;   // set when pose arrives via ESP-NOW (non-CRTP path)

static inline float wrapPi(float a)
{
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

bool blimpGuidanceAutoEnabled(void)
{
  return bc_auto != 0 || s_extAuto;
}

void blimpGuidanceSetMocap(const float p[8])
{
  int64_t nowUs = esp_timer_get_time();          // estimate horizontal velocity
  if (s_lastMocapUs > 0) {
    float dt = (nowUs - s_lastMocapUs) * 1e-6f;
    if (dt > 1e-3f && dt < 1.0f) {
      const float a = 0.4f;                       // low-pass to tame mocap noise
      s_velX = (1.0f - a) * s_velX + a * (p[0] - mc_cx) / dt;
      s_velY = (1.0f - a) * s_velY + a * (p[1] - mc_cy) / dt;
    }
  }
  s_lastMocapUs = nowUs;
  mc_cx = p[0]; mc_cy = p[1]; mc_cz = p[2]; mc_cyaw = p[3];
  mc_tx = p[4]; mc_ty = p[5]; mc_tz = p[6]; mc_tyaw = p[7];
  mc_seq++;            // bump freshness heartbeat (blimpGuidanceMocapFresh)
  s_extAuto = true;    // a pose arrived over ESP-NOW -> engage autonomous mode
}

void blimpGuidanceSetGains(const float g[BLIMP_NUM_GAINS])
{
  for (int i = 0; i < BLIMP_NUM_GAINS; i++) {
    if (!isfinite(g[i])) return;                 // reject a corrupt frame wholesale
  }
  bc_kp_z = g[0]; bc_ki_z = g[1]; bc_kd_z = g[2]; bc_zff = g[3]; bc_iLim_z = g[4];
  bc_yawKpHead = g[5]; bc_yawRateMax = g[6]; bc_yawKpRate = g[7];
  bc_kp_fwd = g[8]; bc_fwdMaxN = g[9]; bc_arriveR = g[10]; bc_headGate = g[11];
  bc_fwdMaxPwm = g[12]; bc_turnMaxPwm = g[13]; bc_vertMaxPwm = g[14];
  bc_kdFwd = g[15];
}

void blimpGuidanceClearAuto(void)
{
  s_extAuto = false;
  zIntegral = 0.0f;
}

bool blimpGuidanceMocapFresh(uint32_t nowMs)
{
  // Detect a new host sample (seq bumped) and stamp the time it arrived.
  if (mc_seq != lastSeqSeen) {
    lastSeqSeen = mc_seq;
    lastFreshMs = nowMs;
  }
  if (lastFreshMs == 0) {
    return false;                       // never received anything yet
  }
  return (nowMs - lastFreshMs) <= bc_staleMs;
}

void blimpGuidanceUpdate(control_t *control, const state_t *state,
                         float gyroYawDps, float dt)
{
  (void)state;  // we deliberately use mocap yaw, not the drifting IMU yaw
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.1f)   dt = 0.1f;

  // ---- Geometry to target (world frame) ----
  float dx = mc_tx - mc_cx;
  float dy = mc_ty - mc_cy;
  float range = sqrtf(dx * dx + dy * dy);

  float yawNow = mc_cyaw * (float)M_PI / 180.0f;     // mocap yaw -> rad
  float bearing = (range > 1e-3f) ? atan2f(dy, dx) : yawNow;

  // When parked at the waypoint, hold the commanded heading instead of the
  // (now ill-defined) bearing to a coincident point.
  bool arrived = (range <= bc_arriveR);
  float headRef = arrived ? (mc_tyaw * (float)M_PI / 180.0f) : bearing;
  float yawErr = wrapPi(headRef - yawNow);

  // ---- 2. HEADING LOOP (cascade: heading -> rate-limited) -> turn [-1,1] ----
  float desiredRate = constrain(bc_yawKpHead * yawErr, -bc_yawRateMax, bc_yawRateMax);
  float uTurn = bc_yawKpRate * (desiredRate - gyroYawDps);   // drive gyro to desired
  uTurn = constrain(uTurn, -1.0f, 1.0f);

  // ---- 3. FORWARD LOOP -> normalized forward [0,1] ----
  float uFwd = 0.0f;
  if (!arrived) {
    float facing = cosf(yawErr);
    if (facing > cosf(bc_headGate * (float)M_PI / 180.0f) && facing > 0.0f) {
      // closing speed = velocity component toward the target (+ = approaching)
      float closing = (range > 1e-3f) ? (s_velX * dx + s_velY * dy) / range : 0.0f;
      uFwd = bc_kp_fwd * range * facing - bc_kdFwd * closing;  // brake by easing off
      uFwd = constrain(uFwd, 0.0f, bc_fwdMaxN);
    }
  }

  // ---- 1. ALTITUDE LOOP -> PWM (signed) ----
  float zErr = mc_tz - mc_cz;
  float vz = state->velocity.z;                      // estimator climb rate, m/s
  zIntegral += zErr * dt;
  float iTerm = bc_ki_z * zIntegral;
  iTerm = constrain(iTerm, -bc_iLim_z, bc_iLim_z);   // anti-windup
  if (bc_ki_z > 1e-6f) {                             // keep integral consistent w/ clamp
    zIntegral = iTerm / bc_ki_z;
  }
  float uVert = bc_kp_z * zErr + iTerm - bc_kd_z * vz + bc_zff;

  // ---- Pack into control_t (PWM domain the mixer expects) ----
  float vLim = (bc_vertMaxPwm < 32767.0f) ? bc_vertMaxPwm : 32767.0f;
  control->thrust = uFwd  * bc_fwdMaxPwm;
  control->yaw    = (int16_t)constrain(uTurn * bc_turnMaxPwm, -32767.0f, 32767.0f);
  control->pitch  = (int16_t)constrain(-uVert,                -vLim, vLim);  // up/down hardware-swapped: flip vertical sign
  control->roll   = 0;

  // ---- Telemetry ----
  lg_range = range; lg_yawErr = yawErr; lg_zErr = zErr;
  lg_uFwd = uFwd;   lg_uTurn = uTurn;   lg_uVert = uVert;
}

PARAM_GROUP_START(mocap)
PARAM_ADD(PARAM_FLOAT,  cx,   &mc_cx)
PARAM_ADD(PARAM_FLOAT,  cy,   &mc_cy)
PARAM_ADD(PARAM_FLOAT,  cz,   &mc_cz)
PARAM_ADD(PARAM_FLOAT,  cyaw, &mc_cyaw)
PARAM_ADD(PARAM_FLOAT,  tx,   &mc_tx)
PARAM_ADD(PARAM_FLOAT,  ty,   &mc_ty)
PARAM_ADD(PARAM_FLOAT,  tz,   &mc_tz)
PARAM_ADD(PARAM_FLOAT,  tyaw, &mc_tyaw)
PARAM_ADD(PARAM_UINT32, seq,  &mc_seq)
PARAM_GROUP_STOP(mocap)

PARAM_GROUP_START(blimpc)
PARAM_ADD(PARAM_UINT8,  autoEn,    &bc_auto)
PARAM_ADD(PARAM_FLOAT,  kpZ,       &bc_kp_z)
PARAM_ADD(PARAM_FLOAT,  kiZ,       &bc_ki_z)
PARAM_ADD(PARAM_FLOAT,  kdZ,       &bc_kd_z)
PARAM_ADD(PARAM_FLOAT,  zff,       &bc_zff)
PARAM_ADD(PARAM_FLOAT,  iLimZ,     &bc_iLim_z)
PARAM_ADD(PARAM_FLOAT,  yawKpHead,  &bc_yawKpHead)
PARAM_ADD(PARAM_FLOAT,  yawRateMax, &bc_yawRateMax)
PARAM_ADD(PARAM_FLOAT,  yawKpRate,  &bc_yawKpRate)
PARAM_ADD(PARAM_FLOAT,  kpFwd,     &bc_kp_fwd)
PARAM_ADD(PARAM_FLOAT,  fwdMaxN,   &bc_fwdMaxN)
PARAM_ADD(PARAM_FLOAT,  arriveR,   &bc_arriveR)
PARAM_ADD(PARAM_FLOAT,  headGate,  &bc_headGate)
PARAM_ADD(PARAM_FLOAT,  kdFwd,     &bc_kdFwd)
PARAM_ADD(PARAM_FLOAT,  fwdMaxPwm, &bc_fwdMaxPwm)
PARAM_ADD(PARAM_FLOAT,  turnMaxPwm,&bc_turnMaxPwm)
PARAM_ADD(PARAM_FLOAT,  vertMaxPwm,&bc_vertMaxPwm)
PARAM_ADD(PARAM_UINT32, staleMs,   &bc_staleMs)
PARAM_GROUP_STOP(blimpc)

LOG_GROUP_START(blimpc)
LOG_ADD(LOG_FLOAT, range,  &lg_range)
LOG_ADD(LOG_FLOAT, yawErr, &lg_yawErr)
LOG_ADD(LOG_FLOAT, zErr,   &lg_zErr)
LOG_ADD(LOG_FLOAT, uFwd,   &lg_uFwd)
LOG_ADD(LOG_FLOAT, uTurn,  &lg_uTurn)
LOG_ADD(LOG_FLOAT, uVert,  &lg_uVert)
LOG_GROUP_STOP(blimpc)
