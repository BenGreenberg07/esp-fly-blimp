/**
 * blimp_guidance.c - On-board guidance for the ESP-FLY blimp (v2).
 *
 * ============================ THE PHYSICS ============================
 * A helium blimp is fundamentally different from a quadrotor:
 *
 *   - It is BUOYANT, so it does not have to spend thrust fighting gravity.
 *   - The gondola hangs well below the envelope (a long pendulum), so the
 *     craft is PASSIVELY STABLE in roll and pitch -- no attitude loop needed.
 *   - Actuation is DECOUPLED: two forward motors (body X), one up, one down.
 *   - The forward motors are UNIDIRECTIONAL (brushed, no reverse): the blimp
 *     cannot brake, and differential turn always adds a little forward push.
 *   - THIS blimp specifically: the gondola is slightly off-center, so forward
 *     thrust produces a constant parasitic yaw torque, and it is slightly
 *     under-buoyant, so it sinks unless the vertical loop works continuously.
 *
 * ============================ V2 DESIGN ==============================
 * v1 spun up violently. Two structural causes, both fixed here:
 *
 *   (1) SIGN FRAGILITY. v1 damped the yaw cascade with the IMU gyro while the
 *       heading error came from MOCAP yaw, which the host mirrors/sign-flips
 *       for the lab display. If those two conventions disagree, the "damping"
 *       term becomes POSITIVE feedback and the cascade actively spins the
 *       blimp up -- exactly the observed failure. v2 derives the yaw rate by
 *       differentiating the mocap yaw itself (wrapped, low-pass filtered), so
 *       the rate feedback is sign-consistent with the heading error BY
 *       CONSTRUCTION, no matter how the host flips axes.
 *
 *   (2) UNMODELED YAW BIAS. The off-center thrust line is a constant torque
 *       disturbance a pure PD loop can only cancel with a standing heading
 *       error (or by oscillating). v2 adds a slow INTEGRAL (adaptive trim) on
 *       the rate error that learns the standing differential needed to fly
 *       straight -- the same trick the manual straight-line panel proved.
 *
 * Per the prof: fly SLOW and rotate-first, with the hover loop always on so
 * slowing down doesn't cost altitude. v2 is a 3-mode state machine:
 *
 *   ALIGN  : rotate in place toward the bearing to target. No forward drive.
 *   CRUISE : drive forward at a *commanded ground speed* (vCruise, slow),
 *            decelerating linearly as range shrinks so it coasts to the mark
 *            (motors can't brake). Heading cascade keeps it on the bearing.
 *   HOLD   : inside the arrive radius -- forward off, damp any rotation, keep
 *            hovering. (Target yaw is NOT chased: unidirectional props turn
 *            by thrusting, which would arc the blimp back out of the radius.)
 *
 * Transitions have hysteresis so it can't chatter between modes.
 * The ALTITUDE loop runs identically in every mode (that IS the hover):
 *            u_z = Kp*e + Ki*integral(e) - Kd*vz + z_ff
 * with vz differentiated from mocap z (the on-board estimator has no usable
 * climb rate in BLIMP_MODE), and z_ff canceling the residual buoyancy.
 *
 * The HEADING loop is a cascade with a hard rate cap:
 *            desiredRate = clamp(KpHead * yawErr, +/-yawRateMax)   [deg/s]
 *            uTurn       = KpRate * (desiredRate - yawRateMocap) + trim
 * The cap bounds how fast it may EVER rotate; at the setpoint desiredRate=0
 * so the rate loop actively brakes the rotation. `trim` is the learned
 * integral above. As a last line of defense, if |yaw rate| exceeds
 * `spinAbort` the controller cuts forward thrust and commands pure
 * counter-rotation until the spin is back under control.
 *
 * The FORWARD loop is velocity control, not position-P:
 *            vDes = min(vCruise, kApp * (range - arriveR/2)) * cos(yawErr)
 *            uFwd = clamp(kVel * (vDes - closingSpeed), 0, fwdMaxN)
 * closingSpeed comes from mocap-differentiated world velocity projected onto
 * the target direction. Commanding speed (instead of thrust ~ distance) is
 * what makes "slow" actually slow, regardless of trim/battery/drag.
 *
 * Motor mixing stays downstream in power_distribution_stock.c:
 *        control->thrust = forward PWM   (0 .. fwdMaxPwm)
 *        control->yaw    = turn PWM      (+/- turnMaxPwm)   differential
 *        control->pitch  = vertical PWM  (+/- vertMaxPwm)   (mixer * vertScale)
 *
 * The host only STREAMS pose+target (CRTP `mocap` params or ESP-NOW 0xA6) and
 * gains (`blimpc` params or ESP-NOW 0xA7); all control math runs here.
 * ====================================================================
 */

#include <math.h>
#include "blimp_guidance.h"
#include "param.h"
#include "log.h"
#include "num.h"
#include "esp_timer.h"

/* ---- Host-streamed mocap state (CRTP param group "mocap") ---------------
 * Positions in meters, yaw in DEGREES (mocap world frame). The host bumps
 * `seq` on every update; we use that as a liveness heartbeat. */
static float mc_cx = 0.0f, mc_cy = 0.0f, mc_cz = 0.0f, mc_cyaw = 0.0f; // current
static float mc_tx = 0.0f, mc_ty = 0.0f, mc_tz = 0.0f, mc_tyaw = 0.0f; // target
static uint32_t mc_seq = 0;          // host increments each fresh sample

/* ---- Guidance gains / limits (CRTP param group "blimpc") ----------------
 * All live-tunable (Wi-Fi params or ESP-NOW 0xA7). Defaults are DELIBERATELY
 * gentle: slow cruise, low rate cap -- creep up from here. */
static uint8_t bc_auto      = 0;     // 0 = manual passthrough, 1 = autonomous

// Altitude loop (the hover -- runs in every mode)
static float bc_kp_z   = 24000.0f;   // PWM per meter of height error
static float bc_ki_z   = 1200.0f;    // PWM per (meter*second): trims buoyancy
static float bc_kd_z   = 9500.0f;    // PWM per (m/s) of mocap climb rate
static float bc_zff    = 11000.0f;   // constant buoyancy feed-forward, PWM
static float bc_iLim_z = 12000.0f;   // |Ki*integral| clamp (anti-windup), PWM

// Heading cascade (heading -> rate-limited -> PI on rate)
static float bc_yawKpHead  = 20.0f;  // heading err (rad) -> desired yaw rate (deg/s)
static float bc_yawRateMax = 25.0f;  // hard cap on desired yaw rate (deg/s)
static float bc_yawKpRate  = 0.02f;  // (deg/s of rate error) -> normalized turn
static float bc_yawKiRate  = 0.01f;  // adaptive trim learn rate: turn per (deg/s * s).
                                     // Cancels the off-center-gondola torque. 0 = off.

// Forward loop (velocity-commanded, slow)
static float bc_vCruise = 0.25f;     // cruise ground speed toward target, m/s (SLOW)
static float bc_kApp    = 0.6f;      // decel slope: vDes=kApp*(range-arriveR/2), 1/s
static float bc_kVel    = 1.2f;      // (normalized fwd) per (m/s) of speed error
static float bc_fwdMaxN = 0.5f;      // cap on normalized forward [0..1]
static float bc_arriveR = 0.4f;      // m: inside this -> HOLD (fwd off, target yaw)
static float bc_alignDeg = 25.0f;    // deg: mis-pointed beyond this -> ALIGN (no fwd)

// Safety
static float bc_spinAbort = 120.0f;  // deg/s: |yaw rate| above this -> cut fwd,
                                     // counter-rotate only, until back under half.
static float bc_yawRateDeadband = 6.0f;  // deg/s: mocap-differentiated yaw rate is
                                     // noisy at rest (quantization/jitter in the
                                     // finite difference); a rate error smaller
                                     // than this is treated as zero so the forward
                                     // motors don't twitch/spin while sitting still
                                     // (observed during hover testing -- L/R would
                                     // creep on pure noise with no deadband).

// Output scaling (normalized command -> PWM counts)
static float bc_fwdMaxPwm  = 18000.0f;  // forward full scale
static float bc_turnMaxPwm = 9000.0f;   // turn  full scale (differential)
static float bc_vertMaxPwm = 28000.0f;  // vertical full scale (int16, *vertScale)

// Failsafe: if no new mocap sample within this many ms, stop driving. Live-tunable
// (gains[20]) since ESP-NOW has no CRTP path to reach this param otherwise.
static uint32_t bc_staleMs = 300;

// ---- Mocap-derived motion estimates (filled in blimpGuidanceSetMocap) ----
// World velocity + yaw rate by finite difference of successive mocap samples,
// low-pass filtered. The yaw rate is in the SAME frame/sign as mc_cyaw, which
// is what makes the cascade sign-safe (see header comment).
static float s_velX = 0.0f, s_velY = 0.0f, s_velZ = 0.0f;   // m/s, world
static float s_yawRateDps = 0.0f;                            // deg/s, mocap frame
static float s_lastYawDeg = 0.0f;
static bool  s_havePrev = false;
static int64_t s_lastMocapUs = 0;

// ---- Controller state ----
typedef enum { MODE_ALIGN = 0, MODE_CRUISE = 1, MODE_HOLD = 2, MODE_SPIN = 3 } gmode_t;
static gmode_t s_mode = MODE_ALIGN;
static float zIntegral = 0.0f;
static float s_yawTrim = 0.0f;       // learned standing differential, [-1..1] domain
static uint32_t lastSeqSeen = 0;
static uint32_t lastFreshMs = 0;
static bool s_extAuto = false;   // set when pose arrives via ESP-NOW (non-CRTP path)

// Telemetry (CRTP log group "blimpc")
static float lg_range = 0.0f, lg_yawErr = 0.0f, lg_zErr = 0.0f;
static float lg_uFwd = 0.0f, lg_uTurn = 0.0f, lg_uVert = 0.0f;
static float lg_yawRate = 0.0f, lg_vDes = 0.0f, lg_closing = 0.0f, lg_trim = 0.0f;
static uint8_t lg_mode = 0;

static inline float wrapPi(float a)
{
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

static inline float wrap180(float a)
{
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool blimpGuidanceAutoEnabled(void)
{
  return bc_auto != 0 || s_extAuto;
}

void blimpGuidanceSetMocap(const float p[8])
{
  int64_t nowUs = esp_timer_get_time();
  if (s_havePrev && s_lastMocapUs > 0) {
    float dt = (nowUs - s_lastMocapUs) * 1e-6f;
    if (dt > 5e-3f && dt < 0.5f) {
      // World velocity (for closing speed + climb-rate damping)
      const float a = 0.35f;                      // low-pass to tame mocap noise
      s_velX = (1.0f - a) * s_velX + a * (p[0] - mc_cx) / dt;
      s_velY = (1.0f - a) * s_velY + a * (p[1] - mc_cy) / dt;
      s_velZ = (1.0f - a) * s_velZ + a * (p[2] - mc_cz) / dt;
      // Yaw rate from mocap yaw (wrapped). Sign-consistent with mc_cyaw by
      // construction -- immune to host-side axis mirrors / gyro sign mismatch.
      const float ay = 0.3f;
      float r = wrap180(p[3] - s_lastYawDeg) / dt;         // deg/s
      r = constrain(r, -400.0f, 400.0f);                    // reject glitches
      s_yawRateDps = (1.0f - ay) * s_yawRateDps + ay * r;
    }
  }
  s_lastMocapUs = nowUs;
  s_lastYawDeg = p[3];
  s_havePrev = true;
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
  bc_yawKpHead = g[5]; bc_yawRateMax = g[6]; bc_yawKpRate = g[7]; bc_yawKiRate = g[8];
  bc_vCruise = g[9]; bc_kApp = g[10]; bc_kVel = g[11]; bc_fwdMaxN = g[12];
  bc_arriveR = g[13]; bc_alignDeg = g[14];
  bc_spinAbort = g[15];
  bc_fwdMaxPwm = g[16]; bc_turnMaxPwm = g[17]; bc_vertMaxPwm = g[18];
  bc_yawRateDeadband = g[19];
  bc_staleMs = (uint32_t)constrain(g[20], 50.0f, 5000.0f);   // sanity-clamp the ms
}

void blimpGuidanceClearAuto(void)
{
  s_extAuto = false;
  zIntegral = 0.0f;
  s_yawTrim = 0.0f;
  s_mode = MODE_ALIGN;
  s_havePrev = false;              // don't finite-diff across a gap
  s_velX = s_velY = s_velZ = 0.0f;
  s_yawRateDps = 0.0f;
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
  (void)state;        // BLIMP_MODE estimator has no usable velocity/attitude
  (void)gyroYawDps;   // rate feedback comes from mocap (sign-safe); see header
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.1f)   dt = 0.1f;

  // ---- Geometry to target (world frame) ----
  float dx = mc_tx - mc_cx;
  float dy = mc_ty - mc_cy;
  float range = sqrtf(dx * dx + dy * dy);

  float yawNow = mc_cyaw * (float)M_PI / 180.0f;     // mocap yaw -> rad
  float bearing = (range > 1e-3f) ? atan2f(dy, dx) : yawNow;
  float bearErr = wrapPi(bearing - yawNow);          // how mis-pointed we are
  float bearErrDeg = bearErr * 180.0f / (float)M_PI;

  // ---- Mode state machine (with hysteresis so it can't chatter) ----
  if (fabsf(s_yawRateDps) > bc_spinAbort) {
    s_mode = MODE_SPIN;                              // spinning out: recover first
  }
  switch (s_mode) {
    case MODE_SPIN:
      if (fabsf(s_yawRateDps) < 0.5f * bc_spinAbort) s_mode = MODE_ALIGN;
      break;
    case MODE_HOLD:
      if (range > 1.5f * bc_arriveR) s_mode = MODE_ALIGN;   // drifted away: re-run
      break;
    case MODE_CRUISE:
      if (range <= bc_arriveR)                    s_mode = MODE_HOLD;
      else if (fabsf(bearErrDeg) > bc_alignDeg)   s_mode = MODE_ALIGN;
      break;
    case MODE_ALIGN:
    default:
      if (range <= bc_arriveR)                          s_mode = MODE_HOLD;
      else if (fabsf(bearErrDeg) < 0.6f * bc_alignDeg)  s_mode = MODE_CRUISE;
      break;
  }

  // ---- HEADING cascade: heading err -> capped desired rate -> PI on rate ----
  // En route we track the bearing to target. In HOLD we do NOT chase the target
  // yaw: the forward motors are unidirectional, so any commanded rotation also
  // pushes the blimp forward and arcs it OUT of the arrive radius -- sim showed
  // this oscillating ALIGN->CRUISE->HOLD forever. Instead HOLD just damps the
  // rotation to zero and hovers; final heading is best-effort, not enforced.
  float yawErr = wrapPi(bearing - yawNow);
  float desiredRate = constrain(bc_yawKpHead * yawErr, -bc_yawRateMax, bc_yawRateMax);
  if (s_mode == MODE_SPIN || s_mode == MODE_HOLD) desiredRate = 0.0f;  // brake only
  float rateErr = desiredRate - s_yawRateDps;
  // Deadband: the mocap-differentiated yaw rate is noisy at rest (quantization in
  // the finite difference), so small rate errors are noise, not a real rotation to
  // correct. Without this the forward motors twitch/spin even sitting still, since
  // yaw authority on THIS airframe is only the differential forward thrust.
  float rateCmd = (fabsf(rateErr) < bc_yawRateDeadband) ? 0.0f : bc_yawKpRate * rateErr;
  float uTurn = rateCmd + s_yawTrim;
  // Adaptive trim: slowly learn the standing differential that zeroes the rate
  // error (cancels off-center thrust torque). Learned only while actually
  // driving (ALIGN/CRUISE) -- the disturbance it models is thrust-induced --
  // and not when the output is already saturated the way it pushes (anti-windup).
  if (s_mode == MODE_ALIGN || s_mode == MODE_CRUISE) {
    bool sat = (uTurn > 1.0f && rateErr > 0.0f) || (uTurn < -1.0f && rateErr < 0.0f);
    if (!sat) {
      s_yawTrim = constrain(s_yawTrim + bc_yawKiRate * rateErr * dt, -0.35f, 0.35f);
    }
  }
  uTurn = constrain(uTurn, -1.0f, 1.0f);

  // ---- FORWARD: commanded ground speed with linear decel to the mark ----
  float uFwd = 0.0f;
  float vDes = 0.0f;
  float closing = (range > 1e-3f) ? (s_velX * dx + s_velY * dy) / range : 0.0f;
  if (s_mode == MODE_CRUISE) {
    float rampR = range - 0.5f * bc_arriveR;         // start stopping BEFORE the ring
    vDes = bc_kApp * (rampR > 0.0f ? rampR : 0.0f);
    if (vDes > bc_vCruise) vDes = bc_vCruise;
    float facing = cosf(yawErr);                     // bleed speed off-axis
    vDes *= (facing > 0.0f) ? facing : 0.0f;
    uFwd = constrain(bc_kVel * (vDes - closing), 0.0f, bc_fwdMaxN);
  }

  // ---- ALTITUDE (the hover -- runs in every mode) ----
  float zErr = mc_tz - mc_cz;
  zIntegral += zErr * dt;
  float iTerm = bc_ki_z * zIntegral;
  iTerm = constrain(iTerm, -bc_iLim_z, bc_iLim_z);   // anti-windup
  if (bc_ki_z > 1e-6f) {                             // keep integral consistent w/ clamp
    zIntegral = iTerm / bc_ki_z;
  }
  float uVert = bc_kp_z * zErr + iTerm - bc_kd_z * s_velZ + bc_zff;

  // ---- Pack into control_t (PWM domain the mixer expects) ----
  // Vertical sign chain is flight-proven: this -uVert together with the mixer's
  // blimp.vertSign=-1 yields net +uVert = climb. Do not "simplify" either alone.
  float vLim = (bc_vertMaxPwm < 32767.0f) ? bc_vertMaxPwm : 32767.0f;
  control->thrust = uFwd  * bc_fwdMaxPwm;
  control->yaw    = (int16_t)constrain(uTurn * bc_turnMaxPwm, -32767.0f, 32767.0f);
  control->pitch  = (int16_t)constrain(-uVert,                -vLim, vLim);
  control->roll   = 0;

  // ---- Telemetry ----
  lg_range = range; lg_yawErr = yawErr; lg_zErr = zErr;
  lg_uFwd = uFwd;   lg_uTurn = uTurn;   lg_uVert = uVert;
  lg_yawRate = s_yawRateDps; lg_vDes = vDes; lg_closing = closing;
  lg_trim = s_yawTrim; lg_mode = (uint8_t)s_mode;
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
PARAM_ADD(PARAM_FLOAT,  yawKiRate,  &bc_yawKiRate)
PARAM_ADD(PARAM_FLOAT,  vCruise,   &bc_vCruise)
PARAM_ADD(PARAM_FLOAT,  kApp,      &bc_kApp)
PARAM_ADD(PARAM_FLOAT,  kVel,      &bc_kVel)
PARAM_ADD(PARAM_FLOAT,  fwdMaxN,   &bc_fwdMaxN)
PARAM_ADD(PARAM_FLOAT,  arriveR,   &bc_arriveR)
PARAM_ADD(PARAM_FLOAT,  alignDeg,  &bc_alignDeg)
PARAM_ADD(PARAM_FLOAT,  spinAbort, &bc_spinAbort)
PARAM_ADD(PARAM_FLOAT,  yawRateDeadband, &bc_yawRateDeadband)
PARAM_ADD(PARAM_FLOAT,  fwdMaxPwm, &bc_fwdMaxPwm)
PARAM_ADD(PARAM_FLOAT,  turnMaxPwm,&bc_turnMaxPwm)
PARAM_ADD(PARAM_FLOAT,  vertMaxPwm,&bc_vertMaxPwm)
PARAM_ADD(PARAM_UINT32, staleMs,   &bc_staleMs)
PARAM_GROUP_STOP(blimpc)

LOG_GROUP_START(blimpc)
LOG_ADD(LOG_FLOAT, range,   &lg_range)
LOG_ADD(LOG_FLOAT, yawErr,  &lg_yawErr)
LOG_ADD(LOG_FLOAT, zErr,    &lg_zErr)
LOG_ADD(LOG_FLOAT, uFwd,    &lg_uFwd)
LOG_ADD(LOG_FLOAT, uTurn,   &lg_uTurn)
LOG_ADD(LOG_FLOAT, uVert,   &lg_uVert)
LOG_ADD(LOG_FLOAT, yawRate, &lg_yawRate)
LOG_ADD(LOG_FLOAT, vDes,    &lg_vDes)
LOG_ADD(LOG_FLOAT, closing, &lg_closing)
LOG_ADD(LOG_FLOAT, trim,    &lg_trim)
LOG_ADD(LOG_UINT8, mode,    &lg_mode)
LOG_GROUP_STOP(blimpc)
