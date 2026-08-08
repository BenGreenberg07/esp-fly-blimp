/*
 * espnow_bridge.ino — USB-serial <-> ESP-NOW bridge for the ESP-FLY blimp.
 *
 * Runs on a Seeed XIAO ESP32-C6 OR a Seeed XIAO ESP32-S3 plugged into the Mac
 * (either works as the bridge — pick whichever board you have; nothing else in
 * this file needs to change). The Mac sends control setpoints over USB serial;
 * this relays them to the blimp over ESP-NOW, so the Mac's Wi-Fi stays free for
 * the mocap network (no router / no Wi-Fi join).
 *
 * Board: "XIAO_ESP32C6" or "XIAO_ESP32S3" (install "esp32 by Espressif" core
 * v3.x in Arduino IDE; on S3 also enable "USB CDC On Boot" in Tools so Serial
 * routes over the native USB port). See espnow_bridge/README.md, "Flashing",
 * for the exact arduino-cli command for each board.
 *
 * SERIAL FRAME (Mac -> bridge), 17 bytes:
 *     0xA5  +  16 payload bytes  (4 little-endian float32: roll, pitch, yaw, thrust)
 * The 16 payload bytes are broadcast verbatim over ESP-NOW; the drone parses them.
 *
 * ESP-NOW: broadcast (FF:FF:FF:FF:FF:FF) on a FIXED channel so no MAC pairing is
 * needed. The drone must listen on the SAME channel (see ESPNOW_CHANNEL).
 *
 * Failsafe is on the DRONE side (it zeroes the motors if frames stop), so this
 * bridge just relays — if the Mac stops sending, the drone coasts to safe.
 *
 * FLYING A SECOND DRONE IN THE SAME ROOM? Search this file for FRAME_MAGIC
 * and give this pair its own value (change it here AND in the matching
 * espnow_control.c on the drone, then reflash both). See also
 * espnow_bridge/README.md, "Multiple drones in the same room".
 */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint8_t ESPNOW_CHANNEL = 1;
// BROADCAST (FF:FF:FF:FF:FF:FF). The drone now runs ESP-NOW on top of its Wi-Fi
// SoftAP, so its active interface MAC is the AP MAC, not the STA MAC we used to
// unicast to. Broadcast is delivered regardless of which interface/MAC is up, so
// it "just works" with the AP-coexist firmware. (1 Mbps 11b rate forced below
// keeps the bridge->drone link decodable regardless of which chip the bridge
// is — C6 or S3 — since it forces a common legacy rate both sides support.)
static uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Two frame types from the Mac:
//   0xA5 + 16 bytes  = MANUAL setpoint  (4 float32: roll,pitch,yaw,thrust)
//   0xA6 + 32 bytes  = MOCAP pose       (8 float32: cx,cy,cz,cyaw,tx,ty,tz,tyaw)
// The matching ESP-NOW payload length (16 or 32) is how the drone tells them
// apart, so manual control is never confused with autonomous poses.
// ============================================================================
// FRAME_MAGIC -- unique ID for THIS drone/bridge pair. Every frame this bridge
// broadcasts starts with this 4-byte tag; the drone ignores anything without
// it. This is what lets multiple ESP-FLY blimps share one room without
// cross-talk -- give each pair its own value here AND in the matching
// FRAME_MAGIC in esp-drone/components/espnow_control/espnow_control.c, then
// reflash both boards. (Also filters out other labs' ESP-NOW traffic, which
// is why this exists at all -- see espnow_bridge/README.md.)
// ============================================================================
static const uint8_t FRAME_MAGIC[4] = {0xB1, 0x12, 0x9F, 0x5A};
static uint8_t txbuf[100];       // FRAME_MAGIC + payload, broadcast over ESP-NOW

// TELEMETRY back from the drone: it broadcasts TELEM_MAGIC + 16 bytes (4 float32:
// fwdL, fwdR, up, down). We relay the 16 payload bytes to the Mac over USB serial
// framed as 0xB7 + 16 bytes, so the panel can log the motor commands. Keep this
// tag in sync with espnow_control.c TELEM_MAGIC.
static const uint8_t TELEM_MAGIC[4] = {0xB7, 0x1E, 0x30, 0xA5};

static volatile uint32_t rxAll = 0, rxTel = 0;   // DEBUG: all ESP-NOW rx vs telemetry
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  rxAll++;
  if (len == 4 + 16 && memcmp(data, TELEM_MAGIC, 4) == 0) {
    rxTel++;
    uint8_t out[1 + 16];
    out[0] = 0xB7;                        // serial sync byte (non-ASCII)
    memcpy(out + 1, data + 4, 16);
    Serial.write(out, sizeof(out));
  }
}

static uint8_t payload[96];
static int idx = -1;            // -1 = waiting for a sync byte
static int want = 0;            // payload length expected for the current frame
static uint32_t frames = 0;

// Delivery ACK from the drone (unicast): SUCCESS = drone's radio received it.
static volatile uint32_t txOk = 0, txFail = 0;
static void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) txOk++; else txFail++;
}

static void selectOnboardAntenna() {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
  // XIAO ESP32-C6 RF switch: GPIO3 LOW powers it, GPIO14 selects the antenna
  // (LOW = onboard ceramic, HIGH = external u.FL). Set to 1 ONLY if the external
  // antenna is physically attached — HIGH with no antenna radiates into an open
  // connector and makes the link WORSE.
  #define USE_EXTERNAL_ANTENNA 1
  pinMode(3, OUTPUT);  digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, USE_EXTERNAL_ANTENNA ? HIGH : LOW);
#endif
  // XIAO ESP32-S3 uses its fixed onboard antenna — nothing to switch.
}

void setup() {
  Serial.begin(115200);
  selectOnboardAntenna();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Force LEGACY Wi-Fi rates so the bridge (C6, Wi-Fi 6, or S3) transmits
  // ESP-NOW frames the S3 drone can actually decode (cross-chip/cross-radio
  // rate mismatch is the usual "bridge sends but drone never receives" cause;
  // this is needed for a C6 bridge and is harmless/still-recommended for an
  // S3 bridge, so it's left on for both).
  esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERR esp_now_init");
    return;
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // Force the broadcast to go out at 1 Mbps 802.11b — the most universally
  // decodable ESP-NOW rate (so the S3 drone can hear the C6).
  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_11B;
  rate.rate = WIFI_PHY_RATE_1M_L;
  esp_now_set_peer_rate_config(BCAST, &rate);

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);   // relay drone motor telemetry -> USB serial
  uint8_t ach = 0; wifi_second_chan_t s2;
  esp_wifi_get_channel(&ach, &s2);
  Serial.print("BRIDGE READY cfg_ch="); Serial.print(ESPNOW_CHANNEL);
  Serial.print(" ACTUAL_ch="); Serial.println(ach);
}

void loop() {
  static unsigned long lastSerial = 0, lastHb = 0;
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (idx < 0) {
      if (b == 0xA5) { idx = 0; want = 16; }       // manual setpoint frame
      else if (b == 0xA6) { idx = 0; want = 32; }  // mocap pose frame
      else if (b == 0xA7) { idx = 0; want = 92; }  // gain-tuning frame (23 float32)
    } else {
      payload[idx++] = b;
      if (idx >= want) {               // full frame -> tag + broadcast it
        memcpy(txbuf, FRAME_MAGIC, 4);
        memcpy(txbuf + 4, payload, want);
        esp_now_send(BCAST, txbuf, want + 4);
        idx = -1;
        frames++;
        lastSerial = millis();
      }
    }
  }
  // SAFETY: do NOT broadcast anything when the Mac stops driving us. Earlier this
  // sent a 5 Hz zero "heartbeat", but that kept the drone's link alive so its
  // 400 ms no-frame failsafe NEVER fired — a dropped laptop/USB/panel left the
  // blimp flying. Now silence on serial => silence on ESP-NOW => the drone times
  // out and hard-stops (zeros motors + drops autonomy). Just print a status line.
  if (millis() - lastSerial > 500 && millis() - lastHb > 1500) {
    Serial.printf("idle (no serial) — not transmitting; TX ok=%u fail=%u rxAll=%u rxTel=%u\n",
                  (unsigned)txOk, (unsigned)txFail, (unsigned)rxAll, (unsigned)rxTel);
    lastHb = millis();
  }
}
