/*
 * ble_control.c — NimBLE GATT control link for the ESP-FLY blimp.
 * See ble_control.h for the protocol and design rationale.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_coexist.h"

#include "ble_control.h"

#define TAG "BLE_CTRL"
#define DEVICE_NAME "ESP-BLIMP"
#define FAILSAFE_MS 400   /* no control packet for this long -> zero the motors */

static ble_setpoint_cb_t s_cb = NULL;
static uint8_t s_addr_type;
static volatile uint32_t s_last_write_ms = 0;
static volatile bool s_active = false;     /* a setpoint is currently being held */

/* Nordic UART Service UUIDs, little-endian byte order for NimBLE. */
static const ble_uuid128_t svc_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t rx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t tx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_tx_val_handle;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void deliver(float roll, float pitch, float yaw, float thrust)
{
    if (s_cb) {
        s_cb(roll, pitch, yaw, thrust);
    }
}

/* GATT access: the host writes 16 bytes (4 little-endian floats) to RX. */
static int rx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 16) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    float v[4];
    int rc = ble_hs_mbuf_to_flat(ctxt->om, v, sizeof(v), NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    deliver(v[0], v[1], v[2], v[3]);
    s_last_write_ms = now_ms();
    s_active = true;
    static uint32_t n = 0;
    if ((n++ % 40) == 0) {
        ESP_LOGI(TAG, "rx #%u pitch=%.0f yaw=%.0f thr=%.0f", (unsigned)n, v[1], v[2], v[3]);
    }
    return 0;
}

static int tx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* TX is notify-only; nothing to do on read. */
    return 0;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &rx_uuid.u,
                .access_cb = rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &tx_uuid.u,
                .access_cb = tx_access_cb,
                .val_handle = &s_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }, /* terminator */
        },
    },
    { 0 }, /* terminator */
};

static void start_advertising(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connect %s", event->connect.status == 0 ? "OK" : "FAILED");
        if (event->connect.status != 0) {
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "disconnect reason=0x%02x; zero motors + re-advertise",
                 event->disconnect.reason);
        deliver(0, 0, 0, 0);     /* failsafe on link loss */
        s_active = false;
        start_advertising();
        break;
    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "conn update status=%d", event->conn_update.status);
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "mtu=%d", event->mtu.value);
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "subscribe notify=%d", event->subscribe.cur_notify);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    default:
        ESP_LOGI(TAG, "gap event type=%d", event->type);
        break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "advertising as '%s'", DEVICE_NAME);
    }
}

static void on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "infer_auto rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void ble_host_task(void *param)
{
    nimble_port_run();              /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* Failsafe: if the host stops sending control packets, zero the motors. */
static void failsafe_task(void *param)
{
    for (;;) {
        if (s_active && (now_ms() - s_last_write_ms) > FAILSAFE_MS) {
            deliver(0, 0, 0, 0);
            s_active = false;
            ESP_LOGW(TAG, "control timeout -> motors zeroed");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void bleControlSetHandler(ble_setpoint_cb_t cb)
{
    s_cb = cb;
}

void bleControlInit(void)
{
    int rc = nimble_port_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed rc=%d", rc);
        return;
    }

    /* Bias the shared-antenna Wi-Fi/BLE coexistence toward Bluetooth so the
     * control link stays alive while the SoftAP is beaconing. Without this the
     * Mac-side connection gets starved and drops a fraction of a second after
     * connecting. */
    esp_coex_preference_set(ESP_COEX_PREFER_BT);

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "count_cfg rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "add_svcs rc=%d", rc); return; }

    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) { ESP_LOGE(TAG, "name_set rc=%d", rc); }

    nimble_port_freertos_init(ble_host_task);
    xTaskCreate(failsafe_task, "ble_failsafe", 2048, NULL, 4, NULL);
    ESP_LOGI(TAG, "BLE control up");
}
