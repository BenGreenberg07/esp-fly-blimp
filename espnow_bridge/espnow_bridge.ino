/*
 * espnow_bridge.ino — USB-serial <-> ESP-NOW bridge for the ESP-FLY blimp.
 *
 * Runs on a Seeed XIAO ESP32-C6 plugged into the Mac. The Mac sends control
 * setpoints over USB serial; this relays them to the blimp over ESP-NOW, so the
 * Mac's Wi-Fi stays free for the mocap network (no router / no Wi-Fi join).
 *
 * Board: "XIAO_ESP32C6"  (install "esp32 by Espressif" core v3.x in Arduino IDE).
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
 */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint8_t ESPNOW_CHANNEL = 1;
// BROADCAST (FF:FF:FF:FF:FF:FF). The drone now runs ESP-NOW on top of its Wi-Fi
// SoftAP, so its active interface MAC is the AP MAC, not the STA MAC we used to
// unicast to. Broadcast is delivered regardless of which interface/MAC is up, so
// it "just works" with the AP-coexist firmware. (1 Mbps 11b rate forced below
// keeps the C6->S3 cross-chip link decodable.)
static uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Two frame types from the Mac:
//   0xA5 + 16 bytes  = MANUAL setpoint  (4 float32: roll,pitch,yaw,thrust)
//   0xA6 + 32 bytes  = MOCAP pose       (8 float32: cx,cy,cz,cyaw,tx,ty,tz,tyaw)
// The matching ESP-NOW payload length (16 or 32) is how the drone tells them
// apart, so manual control is never confused with autonomous poses.
static uint8_t payload[64];
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
  // XIAO ESP32-C6 RF switch: GPIO3 LOW powers it, GPIO14 LOW = onboard ceramic.
  pinMode(3, OUTPUT);  digitalWrite(3, LOW);
  pinMode(14, OUTPUT); digitalWrite(14, LOW);
#endif
  // XIAO ESP32-S3 uses its fixed onboard antenna — nothing to switch.
}

void setup() {
  Serial.begin(115200);
  selectOnboardAntenna();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  // Force LEGACY Wi-Fi rates so a C6 (Wi-Fi 6) transmits ESP-NOW frames an
  // S3 receiver can actually decode (cross-chip rate mismatch is the usual
  // "C6 sends but S3 never receives" cause).
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
      else if (b == 0xA7) { idx = 0; want = 56; }  // gain-tuning frame
    } else {
      payload[idx++] = b;
      if (idx >= want) {               // full frame -> broadcast it (16 or 32 bytes)
        esp_now_send(BCAST, payload, want);
        idx = -1;
        frames++;
        lastSerial = millis();
      }
    }
  }
  // DIAGNOSTIC heartbeat: when not driven over serial (e.g. C6 on a battery),
  // send a zero setpoint at 5 Hz so the drone's "rx #" log can confirm the link.
  // Zero = motors stay off, so this is safe.
  if (millis() - lastSerial > 500 && millis() - lastHb > 200) {
    // Idle heartbeat: ZERO setpoint at 5 Hz when the Mac isn't driving. Lets the
    // drone's "rx #" log confirm the link with NO laptop, while keeping motors OFF
    // (safe). To bench-test motor response without a laptop, temporarily set
    // thrust (4th value) to e.g. 7000 — PROPS OFF. 4 little-endian floats.
    float tf[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    esp_now_send(BCAST, (uint8_t *)tf, 16);
    lastHb = millis();
    static unsigned long lastRep = 0;
    if (millis() - lastRep > 1500) {   // is the drone ACKing our unicast?
      Serial.printf("TX ok=%u fail=%u\n", (unsigned)txOk, (unsigned)txFail);
      lastRep = millis();
    }
  }
}
