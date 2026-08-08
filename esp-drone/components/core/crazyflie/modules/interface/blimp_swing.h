/**
 * blimp_swing.h - SWING BLIMP build flag.
 *
 * The swing blimp is the airframe with FOUR motors that all point roughly UP, two
 * per side canted ~45 degrees. It has no dedicated forward/up/down motors, so the
 * decoupled blimp mixer (forward + differential turn + vertical) does not describe
 * it at all: the four motors have to be driven independently.
 *
 * The Mac runs the Mellinger controller (Firmware/control/mellinger_core.py) and
 * streams its four raw control_t outputs down the EXISTING 0xA5 manual frame; the
 * drone mixes them with the real Sblimp's own mixer (a transcription of
 * powerDistributionLegacy() from power_distribution_quadrotor.c).
 *
 *     0xA5 + 4 LE float32  =  (F_x, F_y, M_z, F_z)
 *                          =  (control->roll, ->pitch, ->yaw, ->thrust)
 *
 * NOTE the Mellinger convention: roll/pitch are NOT attitude here, they are BODY
 * FORCES. Every motor carries the collective thrust, with the two body forces and
 * the yaw moment riding on top as differentials -- which is exactly why all four
 * motors are canted, so each contributes to lift and to a horizontal axis at once.
 *
 * Ranges: F_x/F_y/M_z arrive as int16 (the controller clamps them to +/-16000) and
 * F_z as a float up to 65535, so no scaling is needed anywhere.
 *
 * The frame LENGTH is unchanged, so the C6 bridge does NOT need reflashing -- it
 * relays 0xA5 exactly as it always has. Only the drone changes, and only in how it
 * interprets those four floats.
 *
 * ============================ DEFAULT IS 0 =================================
 * With BLIMP_SWING 0 this file changes NOTHING. The decoupled blimp firmware, its
 * on-board guidance, the mocap panel and every existing flight mode behave exactly
 * as before -- both swing branches compile out entirely.
 *
 * Flash the two variants with:
 *     Firmware/FLASH_SWING.command       (sets this to 1)
 *     Firmware/FLASH_DECOUPLED.command   (sets this back to 0)
 * ===========================================================================
 */
#ifndef BLIMP_SWING_H_
#define BLIMP_SWING_H_

#define BLIMP_SWING 0

#endif /* BLIMP_SWING_H_ */
