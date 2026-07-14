/*
 * ble_control.h — BLE (NimBLE) control link for the ESP-FLY blimp.
 *
 * Lets a host (e.g. a MacBook whose Wi-Fi is busy on a mocap network) drive the
 * blimp over Bluetooth Low Energy instead of the drone's Wi-Fi AP.
 *
 * Design note: this component depends ONLY on the BT stack — NOT on the
 * crazyflie flight-core component. To avoid a circular component dependency,
 * the flight core registers a callback here (bleControlSetHandler) that turns
 * incoming control floats into a setpoint_t + commanderSetSetpoint() call.
 *
 * GATT layout (Nordic UART Service UUIDs, so off-the-shelf BLE tools/bleak work):
 *   Service 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 *     RX  6E400002-...  WRITE / WRITE_NO_RSP  <- host sends control
 *     TX  6E400003-...  NOTIFY                -> (reserved for telemetry)
 *
 * Control packet written to RX = 16 bytes, 4 little-endian float32:
 *   [0] roll  (deg)      — normally 0 for the blimp
 *   [1] pitch (deg)      — BLIMP_MODE maps this to vertical (up/down)
 *   [2] yaw   (deg/s)    — BLIMP_MODE maps this to turn
 *   [3] thrust(0..60000) — BLIMP_MODE maps this to forward
 * These mirror exactly what the Wi-Fi commander would deliver, so the existing
 * BLIMP_MODE mixer in stabilizer.c needs no changes.
 */
#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Called for every control packet (and with all-zeros by the failsafe
 * watchdog when the host stops sending). Registered by the flight core. */
typedef void (*ble_setpoint_cb_t)(float roll, float pitch, float yaw, float thrust);

/* Register the handler BEFORE bleControlInit(). */
void bleControlSetHandler(ble_setpoint_cb_t cb);

/* Bring up NimBLE, the GATT service, advertising, and the failsafe watchdog. */
void bleControlInit(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_CONTROL_H */
