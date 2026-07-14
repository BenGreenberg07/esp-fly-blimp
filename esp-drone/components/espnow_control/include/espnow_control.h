/*
 * espnow_control.h — ESP-NOW control link for the ESP-FLY blimp.
 *
 * Lets a host drive the blimp over ESP-NOW (via a USB-attached ESP32 bridge),
 * leaving the Mac's Wi-Fi free for mocap. No Wi-Fi association, router, or IP.
 *
 * Like ble_control, this depends only on the Wi-Fi/ESP-NOW stack (not the
 * flight core); the core registers a callback that turns the incoming control
 * floats into a setpoint_t + commanderSetSetpoint().
 *
 * ESP-NOW payload = 16 bytes, 4 little-endian float32: roll, pitch, yaw, thrust
 * (same layout the BLE link and the C6 bridge use). Broadcast on a fixed channel.
 */
#ifndef ESPNOW_CONTROL_H
#define ESPNOW_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*espnow_setpoint_cb_t)(float roll, float pitch, float yaw, float thrust);

/* Optional second frame type for AUTONOMOUS control: the host streams the mocap
 * pose + target (8 LE float32 = cx,cy,cz,cyaw, tx,ty,tz,tyaw; pos m, yaw deg) and
 * the DRONE runs its on-board guidance. Distinguished purely by ESP-NOW payload
 * length: 16 bytes = manual setpoint, 32 bytes = mocap pose. */
typedef void (*espnow_mocap_cb_t)(const float pose8[8]);

/* Third frame type: live GAIN tuning (84-byte payload = 21 LE float32, order
 * defined by BLIMP_NUM_GAINS in blimp_guidance.h). Sent only when the host
 * moves a tuning slider. Distinguished by payload length:
 * 16 = manual, 32 = mocap pose, 84 = gains. */
typedef void (*espnow_gains_cb_t)(const float gains21[21]);

/* Register the handlers BEFORE espnowControlInit(). The mocap + gains handlers
 * are optional (leave unset for manual-only). */
typedef void (*espnow_failsafe_cb_t)(void);   /* fired once on link-loss timeout */

void espnowControlSetHandler(espnow_setpoint_cb_t cb);
void espnowControlSetMocapHandler(espnow_mocap_cb_t cb);
void espnowControlSetGainsHandler(espnow_gains_cb_t cb);
void espnowControlSetFailsafeHandler(espnow_failsafe_cb_t cb);

/* Bring up Wi-Fi (STA, unconnected) on a fixed channel + ESP-NOW receiver +
 * the failsafe watchdog. Call this INSTEAD of wifiInit() (it owns the radio). */
void espnowControlInit(void);

#ifdef __cplusplus
}
#endif

#endif /* ESPNOW_CONTROL_H */
