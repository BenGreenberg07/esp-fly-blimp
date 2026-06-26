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

/* Live-tune the guidance gains from outside the CRTP param path (ESP-NOW). The
 * 15 floats are, in order: kpZ,kiZ,kdZ,zff,iLimZ, yawKpHead,yawRateMax,yawKpRate,
 * kpFwd,fwdMaxN,arriveR,headGate, fwdMaxPwm,turnMaxPwm,vertMaxPwm. Sent only when
 * the host changes a value (event-driven), so it adds ~nothing to the loop. */
#define BLIMP_NUM_GAINS 15
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
 *   state      : on-board estimate (we only use gyro-derived yaw rate for D)
 *   gyroYawDps : measured yaw rate, deg/s (from sensorData.gyro.z)
 *   dt         : seconds since last call (for the integral / derivative terms)
 */
void blimpGuidanceUpdate(control_t *control, const state_t *state,
                         float gyroYawDps, float dt);

#endif /* BLIMP_GUIDANCE_H_ */
