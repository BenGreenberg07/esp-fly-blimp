/*
 * espnow_control.c — ESP-NOW receiver for the ESP-FLY blimp.
 * See espnow_control.h for the protocol and design rationale.
 */
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"

#include "espnow_control.h"

#define TAG "ESPNOW_CTRL"
#define ESPNOW_CHANNEL 1        /* must match the C6 bridge's ESPNOW_CHANNEL */
#define FAILSAFE_MS 250         /* no packet this long -> zero the motors (was 400;
                                 * tightened -- this is only a BACKSTOP for total
                                 * ESP-NOW loss now, since blimp_guidance's own
                                 * mocap-staleness check (bc_staleMs, default 300ms)
                                 * fires first and faster whenever pose frames stop,
                                 * even if manual/gain frames are still arriving) */

static espnow_setpoint_cb_t s_cb = NULL;
static espnow_mocap_cb_t s_mocap_cb = NULL;
static espnow_gains_cb_t s_gains_cb = NULL;
static espnow_failsafe_cb_t s_fail_cb = NULL;
static volatile uint32_t s_last_ms = 0;
static volatile bool s_active = false;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ============================================================================
 * FRAME_MAGIC -- unique ID for THIS drone/bridge pair. Every one of OUR frames
 * carries this 4-byte tag as its first bytes; anything without it is ignored.
 * This is what lets multiple ESP-FLY blimps share one room without cross-talk
 * -- give each pair its own value here AND in the matching FRAME_MAGIC in
 * espnow_bridge/espnow_bridge.ino, then reflash both boards. (Also filters
 * out other labs' ESP-NOW traffic, which is why this exists at all -- see
 * espnow_bridge/README.md, "Multiple drones in the same room".)
 * ============================================================================ */
static const uint8_t FRAME_MAGIC[4] = { 0xB1, 0x12, 0x9F, 0x5A };

/* TELEMETRY tag (drone -> bridge -> Mac). Distinct from FRAME_MAGIC so the bridge
 * and any listeners can tell our motor-telemetry frames apart from control frames. */
static const uint8_t TELEM_MAGIC[4] = { 0xB7, 0x1E, 0x30, 0xA5 };
static const uint8_t BCAST_ADDR[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static bool s_bcast_ready = false;

static volatile float s_telem_motors[4] = {0, 0, 0, 0};  /* last motor cmds (cached) */

/* Broadcast the cached 4 motor values (TELEM_MAGIC + 16 bytes). Called both from
 * the guidance loop (fresh values) and the failsafe task (heartbeat), so the Mac
 * ALWAYS gets a telemetry stream -- zeros when idle, real commands in flight --
 * which makes the drone->bridge->Mac link verifiable on the bench (motors off). */
static esp_err_t send_telem_now(void)
{
    if (!s_bcast_ready) return ESP_ERR_INVALID_STATE;
    uint8_t buf[4 + 16];
    memcpy(buf, TELEM_MAGIC, 4);
    memcpy(buf + 4, (const void *)s_telem_motors, 16);  /* 4 LE float32: fwdL,fwdR,up,down */
    return esp_now_send(BCAST_ADDR, buf, sizeof(buf));
}

void espnowControlSendTelem(const float motors4[4])
{
    s_telem_motors[0] = motors4[0]; s_telem_motors[1] = motors4[1];
    s_telem_motors[2] = motors4[2]; s_telem_motors[3] = motors4[3];
    send_telem_now();
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* Reject any frame that is not ours (no magic tag). Room has MANY drones on
     * ESP-NOW; log rejected foreign traffic occasionally so it stays visible. */
    if (len < 4 || memcmp(data, FRAME_MAGIC, 4) != 0) {
        static uint32_t rxForeign = 0;
        if ((rxForeign++ % 200) == 0) {
            const uint8_t *m = info->src_addr;
            ESP_LOGI(TAG, "ignored FOREIGN rx from %02x:%02x:%02x:%02x:%02x:%02x len=%d (no magic)",
                     m[0], m[1], m[2], m[3], m[4], m[5], len);
        }
        return;
    }
    /* strip the 4-byte magic: dispatch on the real payload only */
    data += 4;
    len  -= 4;

    /* 92-byte payload = live GAIN tuning (23 float32, matches BLIMP_NUM_GAINS).
     * Checked first; never drives motors, just updates gains. Validated by the
     * handler. */
    if (len >= 92 && s_gains_cb) {
        float g[23];
        memcpy(g, data, sizeof(g));
        s_gains_cb(g);
        ESP_LOGI(TAG, "gains rx (kpZ=%.0f yawKpHead=%.1f rateMax=%.0f vCruise=%.2f)",
                 g[0], g[5], g[6], g[9]);
        return;
    }
    /* 32-byte payload = AUTONOMOUS mocap pose (8 float32: cx,cy,cz,cyaw,tx,ty,tz,tyaw).
     * Checked before the 16-byte manual case, so it is never mistaken for one.
     * Validated so noise/other devices can't engage autonomy. */
    if (len >= 32 && len < 92 && s_mocap_cb) {
        float p[8];
        memcpy(p, data, sizeof(p));
        for (int i = 0; i < 8; i++) {
            if (!isfinite(p[i])) return;
        }
        /* positions must be within a sane arena (+/-50 m); reject junk */
        if (fabsf(p[0]) > 50.0f || fabsf(p[1]) > 50.0f || fabsf(p[2]) > 50.0f ||
            fabsf(p[4]) > 50.0f || fabsf(p[5]) > 50.0f || fabsf(p[6]) > 50.0f) {
            return;
        }
        s_mocap_cb(p);
        s_last_ms = now_ms();
        s_active = true;
        static uint32_t m = 0;
        if ((m++ % 50) == 0) {
            ESP_LOGI(TAG, "mocap rx #%u pos=(%.2f,%.2f,%.2f) tgt=(%.2f,%.2f,%.2f)",
                     (unsigned)m, p[0], p[1], p[2], p[4], p[5], p[6]);
        }
        return;
    }
    if (len >= 16 && s_cb) {
        float v[4];
        memcpy(v, data, sizeof(v));        /* roll, pitch, yaw, thrust (LE float32) */
        /* Reject stray/garbage ESP-NOW frames (other devices, noise): the values
         * must be finite and in a sane setpoint range, or motors would spin on junk. */
        if (!isfinite(v[0]) || !isfinite(v[1]) || !isfinite(v[2]) || !isfinite(v[3]) ||
            fabsf(v[1]) > 40000.0f || fabsf(v[2]) > 40000.0f ||
            v[3] < -1.0f || v[3] > 70000.0f) {
            return;
        }
        s_cb(v[0], v[1], v[2], v[3]);
        s_last_ms = now_ms();
        s_active = true;
        static uint32_t n = 0;             /* link-confirm: log every ~50th frame */
        if ((n++ % 50) == 0) {
            ESP_LOGI(TAG, "rx #%u pitch=%.0f yaw=%.0f thr=%.0f",
                     (unsigned)n, v[1], v[2], v[3]);
        }
    }
}

static void failsafe_task(void *param)
{
    for (;;) {
        if ((now_ms() - s_last_ms) > FAILSAFE_MS) {
            /* No frames: CONTINUOUSLY hold a zero setpoint (not a one-shot) so a
             * lost link can never leave the motors latched on. */
            if (s_cb) s_cb(0, 0, 0, 0);
            if (s_active) {
                s_active = false;
                if (s_fail_cb) s_fail_cb();   /* drop autonomous latch on link-loss */
                ESP_LOGW(TAG, "ESP-NOW timeout -> motors zeroed + autonomy off (holding)");
            }
        }
        static uint32_t s_hb = 0;
        if ((s_hb % 5) == 0) {                    // ~10 Hz motor-telemetry heartbeat
            esp_err_t rc = send_telem_now();
            if ((s_hb % 100) == 0)                // ~every 2 s: DEBUG the telemetry link
                ESP_LOGI(TAG, "telem hb: ready=%d send_rc=0x%x", (int)s_bcast_ready, (int)rc);
        }
        s_hb++;
        vTaskDelay(pdMS_TO_TICKS(20));  // tighter poll (was 50ms) to match FAILSAFE_MS
    }
}

void espnowControlSetHandler(espnow_setpoint_cb_t cb)
{
    s_cb = cb;
}

void espnowControlSetMocapHandler(espnow_mocap_cb_t cb)
{
    s_mocap_cb = cb;
}

void espnowControlSetGainsHandler(espnow_gains_cb_t cb)
{
    s_gains_cb = cb;
}

void espnowControlSetFailsafeHandler(espnow_failsafe_cb_t cb)
{
    s_fail_cb = cb;
}

/* IMPORTANT: call this AFTER wifiInit() has brought up the Wi-Fi radio.
 * ESP-NOW shares the SoftAP's single radio (no second-radio brownout — that
 * was BLE). Running on the AP also pins the channel to the AP's fixed channel
 * (WIFI_CH=1), which avoids the old pure-STA "set_channel doesn't stick" bug
 * that left the drone deaf to the bridge. We do NOT re-init wifi/netif/event
 * here (wifiInit already did) — doing so would tear down the AP. */
void espnowControlInit(void)
{
    /* No modem sleep, or the radio naps and drops incoming ESP-NOW frames. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (esp_now_init() != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_init failed");
        return;
    }
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));

    /* Broadcast peer so we can send motor TELEMETRY back to the bridge. The peer's
     * ifidx MUST match the drone's ACTUAL active Wi-Fi interface or esp_now_send
     * returns ESP_ERR_ESPNOW_IF (0x306c) -- which is exactly what happened when we
     * hard-coded WIFI_IF_AP but the radio was actually on STA. Query the live mode
     * and pick the matching interface. channel 0 = "use the current channel". */
    wifi_mode_t wmode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&wmode);
    wifi_interface_t txif = (wmode == WIFI_MODE_AP || wmode == WIFI_MODE_APSTA)
                            ? WIFI_IF_AP : WIFI_IF_STA;
    esp_now_peer_info_t bpeer = {0};
    memcpy(bpeer.peer_addr, BCAST_ADDR, 6);
    bpeer.channel = 0;
    bpeer.ifidx   = txif;
    bpeer.encrypt = false;
    if (esp_now_add_peer(&bpeer) == ESP_OK) {
        s_bcast_ready = true;
        ESP_LOGI(TAG, "telemetry peer on %s (mode=%d)",
                 txif == WIFI_IF_AP ? "AP" : "STA", (int)wmode);
    } else {
        ESP_LOGW(TAG, "telemetry broadcast peer add failed (telemetry off)");
    }

    xTaskCreate(failsafe_task, "espnow_failsafe", 2048, NULL, 4, NULL);

    /* Report the AP MAC (what the bridge can unicast to) + the actual channel
     * the radio is parked on (must equal the bridge's ESPNOW_CHANNEL). */
    uint8_t apmac[6] = {0}, stamac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, apmac);
    esp_wifi_get_mac(WIFI_IF_STA, stamac);
    uint8_t actch = 0; wifi_second_chan_t sc;
    esp_wifi_get_channel(&actch, &sc);
    ESP_LOGI(TAG, "ESP-NOW control up (cfg ch %d, ACTUAL ch %d)", ESPNOW_CHANNEL, actch);
    ESP_LOGI(TAG, "  AP  MAC %02x:%02x:%02x:%02x:%02x:%02x (unicast target / broadcast also works)",
             apmac[0], apmac[1], apmac[2], apmac[3], apmac[4], apmac[5]);
    ESP_LOGI(TAG, "  STA MAC %02x:%02x:%02x:%02x:%02x:%02x",
             stamac[0], stamac[1], stamac[2], stamac[3], stamac[4], stamac[5]);
}
