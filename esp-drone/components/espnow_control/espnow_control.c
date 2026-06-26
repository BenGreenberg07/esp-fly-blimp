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
#define FAILSAFE_MS 400         /* no packet this long -> zero the motors */

static espnow_setpoint_cb_t s_cb = NULL;
static espnow_mocap_cb_t s_mocap_cb = NULL;
static espnow_gains_cb_t s_gains_cb = NULL;
static volatile uint32_t s_last_ms = 0;
static volatile bool s_active = false;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    /* 56-byte frame = live GAIN tuning (14 float32). Checked first; never drives
     * motors, just updates gains. Validated by the handler. */
    if (len >= 56 && s_gains_cb) {
        float g[14];
        memcpy(g, data, sizeof(g));
        s_gains_cb(g);
        ESP_LOGI(TAG, "gains rx (kpZ=%.0f kpYaw=%.2f kpFwd=%.2f arriveR=%.2f)",
                 g[0], g[5], g[7], g[9]);
        return;
    }
    /* 32-byte frame = AUTONOMOUS mocap pose (8 float32: cx,cy,cz,cyaw,tx,ty,tz,tyaw).
     * Checked before the 16-byte manual case, so it is never mistaken for one.
     * Validated so noise/other devices can't engage autonomy. */
    if (len >= 32 && len < 56 && s_mocap_cb) {
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
                ESP_LOGW(TAG, "ESP-NOW timeout -> motors zeroed (holding)");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
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
