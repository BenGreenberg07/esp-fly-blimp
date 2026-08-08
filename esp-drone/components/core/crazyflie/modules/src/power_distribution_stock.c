/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */

#include <string.h>

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#define DEBUG_MODULE "PWR_DIST"
#include "debug_cf.h"
#include "blimp_swing.h"   /* BLIMP_SWING build flag; defaults to 0 = no change */
#include "espnow_control.h" /* espnowControlSendTelem: commanded motor values -> host */

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

#ifndef DEFAULT_IDLE_THRUST
#define DEFAULT_IDLE_THRUST 0
#endif

static uint32_t idleThrust = 0;   // blimp: inactive motors fully off (no idle spin)

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

/* Motor slew-rate limiter: ramp each motor toward its target instead of
 * stepping to it. The brushed motors draw a big inrush current spike when they
 * jump from 0 to a high duty; on a LiPo with no bulk decoupling cap that spike
 * sags the 3V3 rail and resets the board the instant a command arrives. Ramping
 * over a few tens of ms spreads the current draw so the rail holds.
 * Called at the ~1kHz stabilizer rate; MOTOR_SLEW_MAX per tick => 0..full ~55ms. */
#define MOTOR_SLEW_MAX 1200
static uint16_t mSlew[4] = {0, 0, 0, 0};
static inline uint16_t slewTo(uint16_t cur, uint32_t tgt)
{
  if ((uint32_t)cur < tgt) {
    return (tgt - cur > MOTOR_SLEW_MAX) ? (uint16_t)(cur + MOTOR_SLEW_MAX) : (uint16_t)tgt;
  }
  return ((uint32_t)cur - tgt > MOTOR_SLEW_MAX) ? (uint16_t)(cur - MOTOR_SLEW_MAX) : (uint16_t)tgt;
}

void powerStop()
{
  mSlew[0] = mSlew[1] = mSlew[2] = mSlew[3] = 0;   /* drop ramp state on stop */
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

/* =====================  ESP-FLY  BLIMP  MIXER  =========================
 *
 * This replaces the quadrotor mixer. The vehicle has FOUR brushed motors
 * that each push in ONE direction only, grouped into four ROLES:
 *
 *   FWD_LEFT  + FWD_RIGHT -> forward thrust (differential = turn)
 *   UP                    -> climb
 *   DOWN                  -> descend
 *
 * Each role is assigned to a physical motor CHANNEL (MOTOR_M1..MOTOR_M4)
 * below. You don't have to match any particular soldering: run
 * `blimp_control.py test` to spin each channel one at a time, note which
 * physical motor moves, then set these four #defines accordingly and
 * rebuild. The mixing logic never has to change.
 *
 *   control->thrust : forward speed   (0 .. 60000)
 *   control->yaw    : turn / steering (+ = turn right, - = turn left)
 *   control->pitch  : vertical command (+ = climb -> UP, - = descend -> DOWN)
 *   control->roll   : unused
 *
 * limitThrust() clamps each result into 0..65535, so negative sums become 0.
 * ======================================================================= */

/* ---- Motor map from bench test (motor_test.py): -------------------
 *   M1 = DOWN, M2 = forward-LEFT, M3 = forward-RIGHT, M4 = UP        */
#define MOTOR_FWD_LEFT   MOTOR_M2
#define MOTOR_FWD_RIGHT  MOTOR_M3
#define MOTOR_UP         MOTOR_M4
#define MOTOR_DOWN       MOTOR_M1
/* ------------------------------------------------------------------- */

/* ---- Blimp tuning — LIVE-ADJUSTABLE over CRTP params (group "blimp") ----
 * Set these from the fly script / control panel while flying; no reflash.
 *   fwdScale  forward stick -> thrust       (gentle cruise)
 *   vertGain  pitch deg     -> up/down PWM
 *   turnGain  yaw deg/s     -> turn PWM
 *   yawTrim   PWM bias to make "both forward" track STRAIGHT (not rotate)
 *   vertTrim  constant up/down bias for slightly +/- buoyancy
 *   pitchFF   up/down per unit forward, cancels nose-up pitch in cruise
 * (fwdScale/vertGain/turnGain are read by stabilizer.c via extern.)
 */
float blimpFwdScale = 0.40f;
float blimpVertGain = 500.0f;
float blimpTurnGain = 60.0f;
float blimpYawTrim  = 0.0f;
float blimpVertTrim = 0.0f;
float blimpPitchFF  = 0.0f;
/* control.pitch is int16 (max 32767 ~= 50% duty). This multiplier lets the
 * vertical command reach full motor range. Default 1.0 keeps old behavior;
 * clients wanting strong up/down (fly_blimp.py, the web panel) set it to ~2.0. */
float blimpVertScale = 1.0f;
/* Vertical dead-band (PWM, post-vertScale): the down motor stays fully OFF
 * until a descent command exceeds this, and the up motor until a climb command
 * does. Stops the down motor from dithering against the up motor on small
 * altitude-loop overshoot (it can't reverse-brake, so opposing spin is wasted
 * thrust + bobbing). Set 0 to restore the old always-responsive behavior. */
float blimpVertDead = 2000.0f;
/* Vertical direction sign. The up/down motors were observed reversed (commanding
 * UP drove the blimp DOWN) in BOTH manual and autonomous flight — expected, since
 * both paths feed control->pitch through this one mixer. Flipping it here (-1)
 * corrects every client at once. Set +1 if the hardware is later rewired. */
float blimpVertSign = -1.0f;
/* ------------------------------------------------------------------------ */

void powerDistribution(const control_t *control)
{
#if BLIMP_SWING
  /* SWING BLIMP mixer -- a transcription of powerDistributionLegacy() from the real
   * Sblimp's power_distribution_quadrotor.c.
   *
   * The Mellinger controller's roll/pitch fields are not attitude here, they are
   * BODY FORCES:  roll -> F_x,  pitch -> F_y.  Every motor carries the collective
   * thrust; the two body forces and the yaw moment ride on top as differentials
   * across the X frame. This is the whole reason all four motors are canted -- each
   * one contributes to lift AND to a horizontal axis at the same time.
   *
   *   m1 = thrust - f_x - f_y - yaw      m3 = thrust + f_x + f_y - yaw
   *   m2 = thrust + f_x - f_y + yaw      m4 = thrust - f_x + f_y + yaw
   *
   * The reference keeps f_x_n/f_x_p and f_y_n/f_y_p as separate variables so the
   * commented-out one-sided branches (negative component to one pair, positive to
   * the other) can be re-enabled without touching the mixer rows. That structure is
   * kept verbatim, currently all four set to the plain value, so re-enabling it here
   * is the same edit as re-enabling it there. */
  {
    const int32_t f_x = (int32_t)control->roll;
    const int32_t f_y = (int32_t)control->pitch;
    const int32_t yaw = (int32_t)control->yaw;
    const int32_t thr = (int32_t)control->thrust;

    const int32_t f_x_n = f_x, f_x_p = f_x;
    const int32_t f_y_n = f_y, f_y_p = f_y;

    int32_t m1 = thr - f_x_n - f_y_n - yaw;
    int32_t m2 = thr + f_x_p - f_y_n + yaw;
    int32_t m3 = thr + f_x_p + f_y_p - yaw;
    int32_t m4 = thr - f_x_n + f_y_p + yaw;

    /* Reference floors each motor at idleThrust; limitThrust() then clamps into the
     * 0..65535 the ESP-Drone LEDC driver takes (the Crazyflie caps later instead). */
    if (m1 < (int32_t)idleThrust) { m1 = (int32_t)idleThrust; }
    if (m2 < (int32_t)idleThrust) { m2 = (int32_t)idleThrust; }
    if (m3 < (int32_t)idleThrust) { m3 = (int32_t)idleThrust; }
    if (m4 < (int32_t)idleThrust) { m4 = (int32_t)idleThrust; }

    motorPower.m1 = limitThrust(m1);
    motorPower.m2 = limitThrust(m2);
    motorPower.m3 = limitThrust(m3);
    motorPower.m4 = limitThrust(m4);
  }
  /* Publish what we ACTUALLY commanded back to the host so the panel can show
   * per-motor ground truth. In the decoupled build blimp_guidance.c does this, but
   * that never runs here -- and without it there is no way to tell "the controller
   * never asked for this motor" apart from "the pin never drove it", which is
   * exactly the ambiguity a dead motor leaves you with.
   * powerDistribution runs at ~1 kHz, so throttle hard: 100:1 => ~10 Hz. */
  {
    static uint16_t swingTelemDiv = 0;
    if (++swingTelemDiv >= 100) {
      swingTelemDiv = 0;
      const float cmd[4] = { (float)motorPower.m1, (float)motorPower.m2,
                             (float)motorPower.m3, (float)motorPower.m4 };
      espnowControlSendTelem(cmd);
    }
  }
#else
  int32_t forward = (int32_t)control->thrust;                          // both forward motors
  int32_t turn    = control->yaw   + (int32_t)blimpYawTrim;            // differential + straighten
  int32_t vert    = (int32_t)(blimpVertSign * blimpVertScale * control->pitch)  // up vs down (sign corrects reversed hw)
                    + (int32_t)blimpVertTrim
                    + (int32_t)(blimpPitchFF * forward);               // cancel cruise pitch

  /* Compute power per ROLE, then route each role to its physical channel. */
  uint16_t ch[NBR_OF_MOTORS] = {0};
  ch[MOTOR_FWD_LEFT]  = limitThrust(forward + turn);
  ch[MOTOR_FWD_RIGHT] = limitThrust(forward - turn);
  int32_t vdead = (int32_t)blimpVertDead;
  ch[MOTOR_UP]        = limitThrust(vert >  vdead ?  vert : 0);   // climb only past dead-band
  ch[MOTOR_DOWN]      = limitThrust(vert < -vdead ? -vert : 0);   // descend only past dead-band

  motorPower.m1 = ch[MOTOR_M1];
  motorPower.m2 = ch[MOTOR_M2];
  motorPower.m3 = ch[MOTOR_M3];
  motorPower.m4 = ch[MOTOR_M4];
#endif /* BLIMP_SWING */

  if (motorSetEnable)
  {
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
  }
  else
  {
    if (motorPower.m1 < idleThrust) {
      motorPower.m1 = idleThrust;
    }
    if (motorPower.m2 < idleThrust) {
      motorPower.m2 = idleThrust;
    }
    if (motorPower.m3 < idleThrust) {
      motorPower.m3 = idleThrust;
    }
    if (motorPower.m4 < idleThrust) {
      motorPower.m4 = idleThrust;
    }

    mSlew[0] = slewTo(mSlew[0], motorPower.m1);
    mSlew[1] = slewTo(mSlew[1], motorPower.m2);
    mSlew[2] = slewTo(mSlew[2], motorPower.m3);
    mSlew[3] = slewTo(mSlew[3], motorPower.m4);
    motorsSetRatio(MOTOR_M1, mSlew[0]);
    motorsSetRatio(MOTOR_M2, mSlew[1]);
    motorsSetRatio(MOTOR_M3, mSlew[2]);
    motorsSetRatio(MOTOR_M4, mSlew[3]);
  }
}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

PARAM_GROUP_START(powerDist)
PARAM_ADD(PARAM_UINT32, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)

PARAM_GROUP_START(blimp)
PARAM_ADD(PARAM_FLOAT, fwdScale, &blimpFwdScale)
PARAM_ADD(PARAM_FLOAT, vertGain, &blimpVertGain)
PARAM_ADD(PARAM_FLOAT, turnGain, &blimpTurnGain)
PARAM_ADD(PARAM_FLOAT, yawTrim,  &blimpYawTrim)
PARAM_ADD(PARAM_FLOAT, vertTrim, &blimpVertTrim)
PARAM_ADD(PARAM_FLOAT, pitchFF,  &blimpPitchFF)
PARAM_ADD(PARAM_FLOAT, vertScale, &blimpVertScale)
PARAM_ADD(PARAM_FLOAT, vertDead, &blimpVertDead)
PARAM_ADD(PARAM_FLOAT, vertSign, &blimpVertSign)
PARAM_GROUP_STOP(blimp)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
