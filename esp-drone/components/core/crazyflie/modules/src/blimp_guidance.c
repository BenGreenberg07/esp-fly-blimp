/**
 * blimp_guidance.c - On-board guidance for the ESP-FLY blimp (v5: constant-cruise circle-follow).
 *
 * ============================ THE PHYSICS ============================
 * A helium blimp is fundamentally different from a quadrotor:
 *
 *   - It is BUOYANT, so it does not have to spend thrust fighting gravity.
 *   - The gondola hangs well below the envelope (a long pendulum), so the
 *     craft is PASSIVELY STABLE in roll and pitch -- no attitude loop needed.
 *   - Actuation is DECOUPLED: two forward motors (body X), one up, one down.
 *   - The forward motors are UNIDIRECTIONAL (brushed, no reverse). The craft
 *     carries a lot of momentum: it CANNOT stop on a point, and it turns in a
 *     wide arc. So we do NOT try to go-to-a-point and park -- we keep it moving.
 *
 * ============================ V5 DESIGN =============================
 * v5 drops "drive to a point and stop" entirely. Instead:
 *
 *   1. CONSTANT FORWARD. Both forward motors run at a fixed cruise level
 *      (bc_fwdMaxN). The operator picks a level where the forward motors also
 *      keep the blimp floating, so cruise = trim. Forward is never modulated by
 *      range or facing -- the blimp just keeps flying forward.
 *
 *   2. STEER by DIFFERENTIAL, keeping the two forward motors' SUM CONSTANT.
 *      The mixer makes  L = forward + turn,  R = forward - turn. We clamp
 *      |turn| <= forward, so neither motor clips and  L + R = 2*forward stays
 *      constant no matter how hard it steers -- turning does not change total
 *      forward thrust (so it does not upset the float / cruise speed).
 *
 *   3. FOLLOW A CIRCLE by pure pursuit ("donkey and carrot"). The host streams
 *      a CARROT -- a point a little way AHEAD of the blimp around the ring (the
 *      panel's circle mode computes it). We simply steer the nose at that carrot
 *      with a heading PD; because the carrot keeps moving around the circle, the
 *      blimp chases it around the circle. No path plan lives on the drone.
 *
 * STEERING is a PD on the heading error to the carrot:
 *        headErr = wrap(bearing_to_carrot - yaw)
 *        turn    = clamp(kpHead*headErr - kdHead*yawRate, +/-turnCap)
 * The D term (gyro yaw rate) is the anti-spin: it eases/reverses the turn before
 * the nose overshoots. An optional setpoint slew (yawSlew) makes the commanded
 * heading step gently instead of snapping.
 *
 * MOTION comes from the STOCK KALMAN ESTIMATOR (system.c feeds mocap position in
 * as an external measurement): the fused world velocity is read from
 * state->velocity (used for the altitude D term); the yaw RATE is the raw IMU
 * gyro.z. No hand-rolled finite differences.
 *
 * The ALTITUDE loop (the hover) runs identically every tick:
 *        u_z = Kp*e + Ki*integral(e) - Kd*vz + z_ff
 * holding the streamed target z. With constant forward providing part of the
 * float, the integral only has to trim the small residual.
 *
 * Motor mixing stays downstream in power_distribution_stock.c:
 *        control->thrust = forward PWM (constant cruise, 0 .. FWD_SUM_HALF_MAX)
 *        control->yaw    = turn PWM    (differential, |turn| <= forward)
 *        control->pitch  = vertical PWM
 * ====================================================================
 */

#include <math.h>
#include "blimp_guidance.h"
#include "param.h"
#include "log.h"
#include "num.h"
#include "esp_timer.h"
#include "espnow_control.h"     /* espnowControlSendTelem: motor telemetry -> host */
#include "led.h"                /* ledSet: arrival-signal LED */

extern float blimpVertSign;      /* power_distribution_stock.c: flips control->pitch
                                     to correct reversed vertical-motor wiring. Motor
                                     telemetry below must apply the SAME flip, or the
                                     logged mUp/mDown are swapped from what's physically
                                     spinning (bit us 2026-07-31: logs showed "down"
                                     pegged near max while the craft was actually
                                     climbing near max and still losing altitude). */

/* ---- Host-streamed mocap state (CRTP param group "mocap") ---------------
 * Positions in meters, yaw in DEGREES (mocap world frame). The host bumps
 * `seq` on every update; we use that as a liveness heartbeat. The SAME mocap
 * position is also fed to the Kalman estimator (system.c) so state->velocity
 * is the fused world velocity in this same frame. The TARGET (mc_t*) is the
 * pure-pursuit CARROT -- a point ahead of the blimp on the circle. */
static float mc_cx = 0.0f, mc_cy = 0.0f, mc_cz = 0.0f, mc_cyaw = 0.0f; // current
static float mc_tx = 0.0f, mc_ty = 0.0f, mc_tz = 0.0f, mc_tyaw = 0.0f; // carrot
static uint32_t mc_seq = 0;          // host increments each fresh sample

/* ---- Guidance gains / limits (CRTP param group "blimpc") ----------------
 * All live-tunable (Wi-Fi params or ESP-NOW 0xA7). */
static uint8_t bc_auto = 0;          // 0 = manual passthrough, 1 = autonomous

// Altitude loop (the hover -- runs every tick). PWM domain, flight-proven.
// kp_z + vertMaxPwm raised 2026-07-21: flight log showed it holding ~0.5 m under
// a 1.1 m target while already near the old 28000 cap -- the brownout that once
// motivated detuning vertical turned out to be foreign-ESP-NOW crashes, so it's
// safe to push authority back up.
static float bc_kp_z   = 30000.0f;   // PWM per meter of height error
static float bc_ki_z   = 1200.0f;    // PWM per (meter*second): trims buoyancy
static float bc_kd_z   = 9500.0f;    // PWM per (m/s) of fused climb rate
static float bc_zff    = 11000.0f;   // constant buoyancy feed-forward, PWM
static float bc_iLim_z = 12000.0f;   // |Ki*integral| clamp (anti-windup), PWM

// Steering: PD on the heading error to the carrot. The D term IS anti-spin.
static float bc_kpHead   = 0.9f;     // heading err (rad) -> turn fraction
static float bc_kdHead   = 0.008f;   // yaw-rate damping, turn frac per (deg/s)
static float bc_turnCap  = 0.5f;     // max |turn| fraction [0..1]
static float bc_alignFloor = 0.1f;   // (unused in v5; kept for gain-frame layout)

// Crab / lateral-drift compensation. A blimp has NO sideways actuator, so the
// only way to cancel lateral drift is to yaw the nose INTO the drift so the
// constant forward thrust opposes it. bc_driftK = radians of heading bias per
// (m/s) of measured cross-track velocity (from the Kalman world velocity).
// Lives in the (v5-unused) gain slot g[10] so it is LIVE-tunable from the panel.
static float bc_driftK = 0.35f;      // crab gain: rad heading bias per m/s drift

// Turn mix: blend single-motor turn (tight, but turn authority capped at cruise)
// toward symmetric (outside motor BOOSTS above cruise -> more yaw torque -> smaller
// turn radius). 0 = pure single-motor; 1 = full symmetric. Lives in gain slot g[9]
// (was the v5-unused vCruise) so it is LIVE-tunable from the panel.
static float bc_turnBoost = 0.6f;    // 0..1 outside-motor boost during turns

// Forward loop. In v5 fwdMaxN is the CONSTANT cruise level (0..1); the others
// are unused (kept so the 23-float gain frame layout / panel sliders are stable).
static float bc_vCruise = 0.25f;     // (unused in v5)
static float bc_kApp    = 0.5f;      // (unused in v5; slot g[10] now = bc_driftK)
static float bc_kVel    = 1.5f;      // (unused in v5)
static float bc_fwdMaxN = 0.36f;     // CONSTANT forward cruise level [0..1]; 0 = turn-only test
static float bc_arriveR = 0.30f;     // (unused in v5)
static float bc_holdExit = 2.5f;     // (unused in v5)
static float bc_lookahead = 0.5f;    // (unused in v5; the carrot is computed host-side)
static float bc_replanThresh = 0.4f; // (unused in v5)

// Output scaling (normalized command -> PWM counts). turnMaxPwm may be NEGATIVE
// to flip the turn direction (equivalent to the host panel's "Flip" button).
static float bc_fwdMaxPwm  = 65535.0f;  // forward full scale
static float bc_turnMaxPwm = 32767.0f;  // turn full scale (differential, signed)
static float bc_vertMaxPwm = 32767.0f;  // vertical full scale (int16, *vertScale)

// Failsafe: if no new mocap sample within this many ms, stop driving.
static uint32_t bc_staleMs = 300;

// Heading-setpoint slew rate (deg/s). 0 = steer straight at the carrot (pure
// pursuit). >0 = ramp the commanded heading toward the carrot this many deg/s,
// so the turn is gentler. For circle-following, 0 (direct) usually tracks best.
static float bc_yawSlew = 0.0f;

// Soft-start ramp enable (1 = ramp outputs up over SOFT_START_S at engage to
// avoid the inrush brownout; 0 = full output immediately, to test WITHOUT ramping).
static float bc_rampEn = 1.0f;

// Fixed safety: if |yaw rate| exceeds this, cut forward and counter-rotate only
// until it drops under half.
#define SPIN_ABORT_DPS 120.0f

// Soft-start ramp time (s). See bc_rampEn.
#define SOFT_START_S 0.7f

// Gyro yaw-rate low-pass time constant (s). The raw MPU6050 gyro.z is noisy; the
// steering D-term multiplies it and the mixer feeds it straight into BOTH forward
// motors (L = fwd + turn, R = fwd - turn), so unfiltered gyro noise shows up as
// the forward motors "randomly" surging. Smooth the rate before the D-term.
#define YAW_RATE_TAU 0.06f

// IMU gyro.z sign vs the mocap yaw frame. The board mounting fixes this; if the
// turn-test shows the D-term ADDING to a spin instead of damping it (nose runs
// away), flip this to -1.0f (or flip turnMaxPwm's sign on the panel).
#define YAW_GYRO_SIGN 1.0f

// Constant-sum guard: the per-motor forward cruise is capped here so that
// forward + turn (= up to 2*forward) never exceeds the motor range -- which is
// what keeps L + R exactly constant while steering. Half of the ~60000 motor max.
#define FWD_SUM_HALF_MAX 30000.0f

// ---- Controller state ----
static float zIntegral = 0.0f;
static bool  s_spin = false;         // spin-abort latch
static float s_soft = 0.0f;          // soft-start ramp 0..1, resets each engage
static float s_headSet = 0.0f;       // slewed heading setpoint, rad (world)
static bool  s_haveHeadSet = false;  // false -> init s_headSet to current yaw
static float s_yawRateFilt = 0.0f;   // low-passed gyro yaw rate (anti-jitter)
static uint32_t lastSeqSeen = 0;
static uint32_t lastFreshMs = 0;
static bool s_extAuto = false;   // set when pose arrives via ESP-NOW (non-CRTP path)

// Telemetry (CRTP log group "blimpc")
static float lg_range = 0.0f, lg_headErr = 0.0f, lg_zErr = 0.0f, lg_xtrack = 0.0f;
static float lg_uFwd = 0.0f, lg_uTurn = 0.0f, lg_uVert = 0.0f;
static float lg_yawRate = 0.0f, lg_vDes = 0.0f, lg_closing = 0.0f;
static float lg_carrotX = 0.0f, lg_carrotY = 0.0f;
static uint8_t lg_hold = 0;

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
  // Latch the streamed pose + carrot. Velocity/rate come from the Kalman + IMU.
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
  bc_kpHead = g[5]; bc_kdHead = g[6]; bc_turnCap = g[7]; bc_alignFloor = g[8];
  bc_turnBoost = g[9]; bc_driftK = g[10]; bc_kVel = g[11]; bc_fwdMaxN = g[12];
  bc_arriveR = g[13]; bc_holdExit = g[14]; bc_lookahead = g[15];
  bc_fwdMaxPwm = g[16]; bc_turnMaxPwm = g[17]; bc_vertMaxPwm = g[18];
  bc_replanThresh = g[19];
  bc_staleMs = (uint32_t)constrain(g[20], 50.0f, 5000.0f);   // sanity-clamp the ms
  bc_yawSlew = constrain(g[21], 0.0f, 360.0f);
  bc_rampEn  = g[22];
}

void blimpGuidanceClearAuto(void)
{
  s_extAuto = false;
  zIntegral = 0.0f;
  s_spin = false;
  s_soft = 0.0f;                   // re-ramp outputs from zero on the next engage
  s_haveHeadSet = false;           // re-init the heading setpoint to the nose
  s_yawRateFilt = 0.0f;            // clear the gyro low-pass
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
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.1f)   dt = 0.1f;

  //! KEY EQ 1 — Geometry to the CARROT (streamed point ahead on the circle)
  float dx = mc_tx - mc_cx;
  float dy = mc_ty - mc_cy;
  float range = sqrtf(dx * dx + dy * dy);
  float yawNow = mc_cyaw * (float)M_PI / 180.0f;     // mocap yaw -> rad

  //! KEY EQ 2 — Motion from the stock filter: fused velZ + gyro yaw rate
  float velZ = state->velocity.z;                    // Kalman (IMU + mocap), for altitude D
  float yawRate = YAW_GYRO_SIGN * gyroYawDps;        // deg/s in the mocap yaw frame
  // Low-pass the raw gyro so the steering D-term (and thus the differential forward
  // motors) stops jittering on gyro noise -- the "forward motors randomly spinning".
  float rc = dt / YAW_RATE_TAU; if (rc > 1.0f) rc = 1.0f;
  s_yawRateFilt += rc * (yawRate - s_yawRateFilt);
  yawRate = s_yawRateFilt;

  // ---- spin-abort safety latch (rarely hit; the PD damps rotation) ----
  if (fabsf(yawRate) > SPIN_ABORT_DPS) s_spin = true;
  else if (fabsf(yawRate) < 0.5f * SPIN_ABORT_DPS) s_spin = false;

  float uFwd = 0.0f, uTurn = 0.0f, headErr = 0.0f;

  if (s_spin) {
    // Recover: cut forward, counter-rotate against the spin only.
    uFwd = 0.0f;
    uTurn = constrain(-bc_kdHead * yawRate, -bc_turnCap, bc_turnCap);
    s_haveHeadSet = false;
  } else {
    //! KEY EQ 3a — Pure pursuit: steer toward the carrot (optionally slewed)
    float headGoal = atan2f(dy, dx);                 // straight at the carrot

    //! KEY EQ 3c — Crab into lateral drift (no sideways actuator on a blimp):
    // vPerp = component of world velocity perpendicular (to the left) of headGoal;
    // rotating the nose by -driftK*vPerp aims the constant forward thrust so it
    // opposes the sideways slip. Uses the Kalman-fused world velocity.
    float vPerp = -state->velocity.x * sinf(headGoal)
                +  state->velocity.y * cosf(headGoal);
    headGoal = wrapPi(headGoal - bc_driftK * vPerp);

    if (!s_haveHeadSet) { s_headSet = yawNow; s_haveHeadSet = true; }
    if (bc_yawSlew <= 0.0f) {
      s_headSet = headGoal;                          // direct pursuit
    } else {
      float step = bc_yawSlew * (float)M_PI / 180.0f * dt;
      float d = wrapPi(headGoal - s_headSet);
      if (d >  step) d =  step;
      if (d < -step) d = -step;
      s_headSet = wrapPi(s_headSet + d);
    }
    headErr = wrapPi(s_headSet - yawNow);

    //! KEY EQ 3b — Heading PD: P toward the carrot − D on the gyro rate (anti-spin)
    uTurn = constrain(bc_kpHead * headErr - bc_kdHead * yawRate, -bc_turnCap, bc_turnCap);

    //! KEY EQ 4 — Forward speeds UP when facing the target, eases OFF while turning.
    // uFwd = fwdMaxN·max(alignFloor, cos(headErr)): full cruise when aimed at the
    // carrot (drives ONTO the path fast, closing cross-track), reduced while turning
    // (keeps turns tight); alignFloor is the minimum creep so it never fully stalls.
    float align = cosf(headErr);
    if (align < bc_alignFloor) align = bc_alignFloor;
    uFwd = bc_fwdMaxN * align;
  }

  //! KEY EQ 5 — Altitude PID: kpZ·e + I(learns buoyancy) − kdZ·vz(fused) + zff
  float zErr = mc_tz - mc_cz;
  zIntegral += zErr * dt;
  float iTerm = bc_ki_z * zIntegral;
  iTerm = constrain(iTerm, -bc_iLim_z, bc_iLim_z);   // anti-windup
  if (bc_ki_z > 1e-6f) {                             // keep integral consistent w/ clamp
    zIntegral = iTerm / bc_ki_z;
  }
  float uVert = bc_kp_z * zErr + iTerm - bc_kd_z * velZ + bc_zff;

  // ---- Soft-start: ramp every output up from zero over SOFT_START_S at engage,
  // unless ramping is disabled for a test (bc_rampEn == 0). ----
  if (bc_rampEn > 0.5f) {
    s_soft += dt / SOFT_START_S;
    if (s_soft > 1.0f) s_soft = 1.0f;
  } else {
    s_soft = 1.0f;
  }
  uFwd  *= s_soft;
  uTurn *= s_soft;
  uVert *= s_soft;

  //! KEY EQ 6 — TURN MIX (blended single-motor <-> symmetric). The INSIDE forward
  // motor is always cut toward 0 as it steers (keeps turns tight); bc_turnBoost sets
  // how much the OUTSIDE motor BOOSTS above cruise to add yaw torque:
  //   boost = 0 -> outside stays at cruise            (pure single-motor: authority = cruise)
  //   boost = 1 -> outside = cruise + |turn|          (symmetric: up to 2x the turn authority)
  // The mixer builds L = thrust + yaw, R = thrust - yaw, so we pick (thrust,yaw) to hit
  // the desired outside/inside motor pair. More boost -> more turn authority -> smaller
  // turn radius (the flight log showed ~1.8 m orbit because single-motor authority is
  // capped at cruise level), without raising the average forward speed.
  float fwdEach = uFwd * bc_fwdMaxPwm;
  if (fwdEach < 0.0f) fwdEach = 0.0f;
  if (fwdEach > FWD_SUM_HALF_MAX) fwdEach = FWD_SUM_HALF_MAX;
  float mag = fabsf(uTurn * bc_turnMaxPwm);
  if (mag > FWD_SUM_HALF_MAX) mag = FWD_SUM_HALF_MAX;               // motor headroom for the boost
  float boost = constrain(bc_turnBoost, 0.0f, 1.0f);
  float outside = fwdEach + boost * mag;                            // outside motor (boosts with turn)
  float inside  = fwdEach - mag;                                    // inside motor (cut; mixer floors at 0)
  float outCap  = 2.0f * FWD_SUM_HALF_MAX;
  if (outside > outCap) outside = outCap;
  float thrustMix = 0.5f * (outside + inside);                      // L=thrust+yaw=outside, R=thrust-yaw=inside
  float yawMix    = 0.5f * (outside - inside);
  if ((uTurn * bc_turnMaxPwm) < 0.0f) yawMix = -yawMix;             // flip turn direction

  float vLim = (bc_vertMaxPwm < 32767.0f) ? bc_vertMaxPwm : 32767.0f;
  control->thrust = thrustMix;
  control->yaw    = (int16_t)constrain(yawMix, -32767.0f, 32767.0f);
  control->pitch  = (int16_t)constrain(-uVert,  -vLim, vLim);        // vertical (flight-proven sign)
  control->roll   = 0;

  //! ARRIVAL LED — flashes GREEN when parked at the streamed target (within
  // bc_arriveR, repurposed here as the arrival radius -- unused elsewhere in
  // v5) so a person watching knows it's safe to grab/push. Off otherwise.
  // Blimp doesn't stop on its own (v5 never holds), so this doesn't change
  // flight behavior -- it's purely a signal for a host "wander" mode that
  // stops STREAMING a new carrot (parking it) once arrived and wants a
  // physical hand-off cue before picking the next point.
  {
    bool arrived = (range < bc_arriveR);
    static uint32_t s_ledToggleMs = 0;
    static bool s_ledOn = false;
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
    if (arrived) {
      if (nowMs - s_ledToggleMs > 250) {           // ~2 Hz flash
        s_ledOn = !s_ledOn;
        ledSet(LED_GREEN, s_ledOn);
        s_ledToggleMs = nowMs;
      }
    } else if (s_ledOn) {
      ledSet(LED_GREEN, false);
      s_ledOn = false;
    }
  }

  // ---- Telemetry ----
  lg_range = range; lg_headErr = headErr; lg_zErr = zErr; lg_xtrack = 0.0f;
  lg_uFwd = uFwd;   lg_uTurn = uTurn;   lg_uVert = uVert;
  lg_yawRate = yawRate; lg_vDes = uFwd; lg_closing = 0.0f;
  lg_carrotX = mc_tx; lg_carrotY = mc_ty;
  lg_hold = (uint8_t)(s_spin ? 2 : 0);

  // ---- MOTOR TELEMETRY back to the host (system ID) ----
  // The 4 actual motor commands, derived from the final mixer outputs:
  //   fwdL = thrust + yaw   (outside/left forward motor)
  //   fwdR = thrust - yaw   (inside/right forward motor; hardware floors <0 at 0)
  //   up   = +vertical      down = -vertical   (vertical = control->pitch)
  // Throttled to ~1-in-8 control ticks so ESP-NOW isn't flooded.
  static uint32_t s_telN = 0;
  if ((s_telN++ & 0x7) == 0) {
    // blimpVertSign (mixer, power_distribution_stock.c) flips control->pitch
    // before it picks the physical up/down motor -- apply the same flip here
    // so telemetry reports which motor ACTUALLY spins, not the pre-mixer sign.
    float vert = blimpVertSign * (float)control->pitch;
    float m4[4];
    m4[0] = control->thrust + (float)control->yaw;   // fwdL
    m4[1] = control->thrust - (float)control->yaw;   // fwdR
    if (m4[1] < 0.0f) m4[1] = 0.0f;
    m4[2] = (vert > 0.0f) ? vert : 0.0f;             // up
    m4[3] = (vert < 0.0f) ? -vert : 0.0f;            // down
    espnowControlSendTelem(m4);
  }
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
PARAM_ADD(PARAM_FLOAT,  kpHead,    &bc_kpHead)
PARAM_ADD(PARAM_FLOAT,  kdHead,    &bc_kdHead)
PARAM_ADD(PARAM_FLOAT,  turnCap,   &bc_turnCap)
PARAM_ADD(PARAM_FLOAT,  alignFloor,&bc_alignFloor)
PARAM_ADD(PARAM_FLOAT,  vCruise,   &bc_vCruise)
PARAM_ADD(PARAM_FLOAT,  kApp,      &bc_kApp)
PARAM_ADD(PARAM_FLOAT,  kVel,      &bc_kVel)
PARAM_ADD(PARAM_FLOAT,  fwdMaxN,   &bc_fwdMaxN)
PARAM_ADD(PARAM_FLOAT,  arriveR,   &bc_arriveR)
PARAM_ADD(PARAM_FLOAT,  holdExit,  &bc_holdExit)
PARAM_ADD(PARAM_FLOAT,  lookahead, &bc_lookahead)
PARAM_ADD(PARAM_FLOAT,  fwdMaxPwm, &bc_fwdMaxPwm)
PARAM_ADD(PARAM_FLOAT,  turnMaxPwm,&bc_turnMaxPwm)
PARAM_ADD(PARAM_FLOAT,  vertMaxPwm,&bc_vertMaxPwm)
PARAM_ADD(PARAM_FLOAT,  replanThresh, &bc_replanThresh)
PARAM_ADD(PARAM_UINT32, staleMs,   &bc_staleMs)
PARAM_ADD(PARAM_FLOAT,  yawSlew,   &bc_yawSlew)
PARAM_ADD(PARAM_FLOAT,  rampEn,    &bc_rampEn)
PARAM_GROUP_STOP(blimpc)

LOG_GROUP_START(blimpc)
LOG_ADD(LOG_FLOAT, range,   &lg_range)
LOG_ADD(LOG_FLOAT, headErr, &lg_headErr)
LOG_ADD(LOG_FLOAT, zErr,    &lg_zErr)
LOG_ADD(LOG_FLOAT, xtrack,  &lg_xtrack)
LOG_ADD(LOG_FLOAT, uFwd,    &lg_uFwd)
LOG_ADD(LOG_FLOAT, uTurn,   &lg_uTurn)
LOG_ADD(LOG_FLOAT, uVert,   &lg_uVert)
LOG_ADD(LOG_FLOAT, yawRate, &lg_yawRate)
LOG_ADD(LOG_FLOAT, vDes,    &lg_vDes)
LOG_ADD(LOG_FLOAT, closing, &lg_closing)
LOG_ADD(LOG_FLOAT, carrotX, &lg_carrotX)
LOG_ADD(LOG_FLOAT, carrotY, &lg_carrotY)
LOG_ADD(LOG_UINT8, hold,    &lg_hold)
LOG_GROUP_STOP(blimpc)
