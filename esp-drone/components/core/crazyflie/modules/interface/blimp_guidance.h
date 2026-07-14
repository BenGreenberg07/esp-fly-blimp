/**
 * blimp_guidance.h - On-board decoupled PID guidance for the ESP-FLY blimp.
 *
 * The blimp is buoyancy/pendulum stable, so it needs NO attitude (tilt) loop.
 * Movement is fully decoupled into three independent single-axis PID loops:
 *
 *   1. ALTITUDE  (Z)     : z error  -> up/down motors        (control.pitch)
 *   2. HEADING   (yaw)   : bearing error -> differential fwd (control.yaw)
 *   3. FORWARD   (surge) : range error  -> both fwd motors   (control.thrust)
 *
 * The host (Mac) streams the live mocap pose + the target pose into the CRTP
 * `mocap` param group; this module turns them into a control_t each loop.
 * All gains live in the CRTP `blimpc` param group so they tune live over
 * Wi-Fi with no reflash. See blimp_guidance.c for the full theory comment.
 */
#ifndef BLIMP_GUIDANCE_H_
#define BLIMP_GUIDANCE_H_

#include <stdbool.h>
#include "stabilizer_types.h"

/* True when host has selected autonomous mode: CRTP param blimpc.autoEn != 0
 * (Wi-Fi path) OR a mocap pose has arrived over ESP-NOW (sets it implicitly). */
bool blimpGuidanceAutoEnabled(void);

/* Inject a mocap pose+target from OUTSIDE the CRTP param path (used by the
 * ESP-NOW receiver): pose8 = cx,cy,cz,cyaw, tx,ty,tz,tyaw (m, m, m, deg). This
 * updates the same state the params do, bumps the freshness heartbeat, and
 * implicitly engages autonomous mode. */
void blimpGuidanceSetMocap(const float pose8[8]);

/* Live-tune the guidance gains from outside the CRTP param path (ESP-NOW).
 * The 21 floats are, in order:
 *   kpZ, kiZ, kdZ, zff, iLimZ,                       (altitude / hover)
 *   yawKpHead, yawRateMax, yawKpRate, yawKiRate,     (heading cascade + trim)
 *   vCruise, kApp, kVel, fwdMaxN, arriveR, alignDeg, (forward speed control)
 *   spinAbort,                                       (safety)
 *   fwdMaxPwm, turnMaxPwm, vertMaxPwm,               (output caps)
 *   yawRateDeadband,                                 (reject mocap-jitter turn cmds)
 *   staleMs.                                          (link-loss failsafe window, ms)
 * Must match GAIN_ORDER in mocap_panel_server.py and the 0xA7 frame length
 * (1 + 4*BLIMP_NUM_GAINS bytes on serial, 4*BLIMP_NUM_GAINS over ESP-NOW).
 * Sent only when the host changes a value (event-driven), so it adds ~nothing. */
#define BLIMP_NUM_GAINS 21
void blimpGuidanceSetGains(const float g[BLIMP_NUM_GAINS]);

/* Drop out of autonomous mode (clears the ESP-NOW auto-engage latch + resets the
 * integrator). Called when the mocap stream goes stale so a lost link fully
 * disarms instead of holding the last command. */
void blimpGuidanceClearAuto(void);

/* True when fresh mocap data has arrived within the failsafe window.
 * stale => the caller should NOT drive autonomously (hold/zero instead). */
bool blimpGuidanceMocapFresh(uint32_t nowMs);

/* Run one guidance step. Fills `control` (thrust=forward, pitch=vertical,
 * yaw=turn) from the latest mocap pose/target and the blimpc gains.
 *   state      : unused (kept for call-site stability)
 *   gyroYawDps : unused -- v2 derives the yaw rate from mocap yaw so the rate
 *                feedback can never be sign-inverted vs. the heading error
 *   dt         : seconds since last call (for the integral terms)
 */
void blimpGuidanceUpdate(control_t *control, const state_t *state,
                         float gyroYawDps, float dt);

#endif /* BLIMP_GUIDANCE_H_ */
