#include "ble_provisioning.h"
#include "webserver.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "os/os_mbuf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "mysystem.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_crt_bundle.h"

#ifdef __cplusplus
}
#endif

#define MSG_QUEUE_SIZE 10
#define MAX_MSG_LEN 128

static const char *TAG = "BLE_PROV";

// Canactive Industries device UUID
static const ble_uuid128_t PROV_SERVICE_UUIDx = BLE_UUID128_INIT(
    0x7e, 0xda, 0x88, 0x5b, // least significant bytes
    0x8a, 0xcd, 0x8e, 0x4d,
    0x84, 0x45, 0x24, 0x53,
    0x1e, 0x8a, 0x2a, 0x3e // most significant bytes
);

static const ble_uuid128_t PROV_SERVICE_UUID = BLE_UUID128_INIT(
    0x3e, 0x2a, 0x8a, 0x1e, // least significant bytes
    0x53, 0x24, 0x45, 0x84,
    0x8e, 0x4d, 0x8a, 0xcd,
    0x7e, 0xda, 0x88, 0x5b // most significant bytes
);

static const ble_uuid128_t SSID_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB1, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t PASS_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB2, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t REBOOT_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB3, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t MSG_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB5, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t WIFI_ENABLE_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xC1, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t WIFI_STATUS_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xC2, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t WIFI_TOGGLE_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xC3, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t DEVICE_ID_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xD0, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t OTA_URL_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB6, 0xAA, 0x00, 0x00}};

static const ble_uuid128_t OTA_STATUS_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB7, 0xAA, 0x00, 0x00}};

// --- Globals --- (unchanged)
static char g_full_device_name[32] = {0};
static TaskHandle_t ble_host_handle = NULL;
static char received_ssid[33] = {0};
static char received_pass[64] = {0};

static uint16_t ssid_handle = 0;
static uint16_t pass_handle = 0;
static uint16_t reboot_handle = 0;
static uint16_t msg_handle = 0;
static uint16_t wifi_enable_handle = 0;
static uint16_t wifi_status_handle = 0;
static uint16_t wifi_toggle_handle = 0;
static uint16_t device_id_handle = 0;
static uint16_t ota_url_handle = 0;
static uint16_t ota_status_handle = 0;

static QueueHandle_t notification_queue = NULL;

typedef struct
{
    uint16_t conn_handle;
    char msg[MAX_MSG_LEN];
} notification_msg_t;

static volatile uint16_t current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t own_addr_type;

static char pending_ota_url[256] = {0};
static volatile bool ota_in_progress = false;
static TaskHandle_t ota_task_handle = NULL;

// Forward declarations (unchanged)
static int ble_provisioning_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static int send_message_notification(uint16_t conn_handle, const char *msg);
static void notification_consumer_task(void *param);
static void ota_update_task(void *param);

static void format_addr(char *buf, const uint8_t *addr)
{
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection established; status=%d", event->connect.status);
        if (event->connect.status == 0)
        {
            current_conn_handle = event->connect.conn_handle;
            led_stop_flashing(BLUE_LED, 1);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Connection lost; reason=%d", event->disconnect.reason);
        current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        led_set_state(BLUE_LED, false);
        if (provisioning_sem != NULL && uxSemaphoreGetCount(provisioning_sem) == 0)
        {
            ESP_LOGI(TAG, "Restarting advertising after disconnect...");
            ble_prov_advertise();
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
        if (provisioning_sem != NULL && uxSemaphoreGetCount(provisioning_sem) == 0)
        {
            ESP_LOGI(TAG, "Restarting advertising after adv complete...");
            ble_prov_advertise();
        }
        break;

    default:
        break;
    }
    return 0;
}

static const struct ble_gatt_chr_def provisioning_chars[] = {
    {.uuid = &SSID_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &ssid_handle},
    {.uuid = &PASS_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &pass_handle},
    {.uuid = &REBOOT_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &reboot_handle},
    {.uuid = &MSG_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &msg_handle},
    {.uuid = &WIFI_ENABLE_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &wifi_enable_handle},
    {.uuid = &WIFI_STATUS_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_READ, .val_handle = &wifi_status_handle},
    {.uuid = &WIFI_TOGGLE_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_WRITE, .val_handle = &wifi_toggle_handle},
    {.uuid = &DEVICE_ID_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_READ, .val_handle = &device_id_handle},
    {.uuid = &OTA_URL_CHAR_UUID.u,
     .access_cb = ble_provisioning_access_cb,
     .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
     .val_handle = &ota_url_handle},
    {.uuid = &OTA_STATUS_CHAR_UUID.u, .access_cb = ble_provisioning_access_cb, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &ota_status_handle},
    {0},
};

static const struct ble_gatt_svc_def provisioning_service[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &PROV_SERVICE_UUID.u, .characteristics = provisioning_chars},
    {0},
};

static void gatt_svr_init(void)
{
    int rc = ble_gatts_count_cfg(provisioning_service);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error counting GATT cfg; rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(provisioning_service);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error adding GATT svcs; rc=%d", rc);
    }
}

static void ble_hs_sync_cb(void)
{
    int rc;

    // Ensure valid Bluetooth address
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to ensure address; rc=%d", rc);
        return;
    }

    // Infer address type
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type; rc=%d", rc);
        return;
    }

    // Get the actual BLE MAC address
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to copy address; rc=%d", rc);
        ble_prov_advertise();  // Start with default
        return;
    }

    // Log the address
    char addr_str[18];
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "Using BT identity address: %s (type %d)", addr_str, own_addr_type);

    // Create full unique name using correct byte order (standard MAC suffix)
    static char full_unique_name[32];  // Make static so pointer remains valid
    snprintf(full_unique_name, sizeof(full_unique_name), "%s-%02X%02X",
             device::TYPE, addr_val[1], addr_val[0]);  // e.g., "HUMD-3D4E"

    ESP_LOGI(TAG, "Full unique name prepared: %s", full_unique_name);

    // Set the GAP device name (used for GATT and fallback)
    rc = ble_svc_gap_device_name_set(full_unique_name);
    if (rc == 0) {
        ESP_LOGI(TAG, "GAP device name set to: %s", full_unique_name);
    } else {
        ESP_LOGW(TAG, "Failed to set GAP name (rc=%d), using default", rc);
        ble_svc_gap_device_name_set(device::TYPE);
    }

    // Store the full name for use in scan response (passed via global or static)
    // We'll use a static global for simplicity
    // Declare this static global near the top of your file:
    // static char g_full_device_name[32] = {0};

    strncpy(g_full_device_name, full_unique_name, sizeof(g_full_device_name) - 1);

    // Start advertising
    ble_prov_advertise();
}

static void ble_prov_set_scan_rsp_data(void)
{
    struct ble_hs_adv_fields fields = {0};

    // Put the FULL unique name in scan response
    if (g_full_device_name[0] != '\0') {
        fields.name = (uint8_t *)g_full_device_name;
        fields.name_len = strlen(g_full_device_name);
        fields.name_is_complete = 1;
    }

    // Include your service UUID
    fields.uuids128 = (ble_uuid128_t *)&PROV_SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan response data; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Scan response set with full name: %s", g_full_device_name);
    }
}

void ble_prov_advertise(void)
{
    struct ble_hs_adv_fields fields = {0};

    // Main advertising packet: only short fixed name + flags
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)device::TYPE;  // "HUMD" only — guaranteed to fit
    fields.name_len = strlen(device::TYPE);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // Full name + service UUID goes in scan response
    ble_prov_set_scan_rsp_data();

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
        return;
    }

    led_start_flashing(BLUE_LED, 500);
    ESP_LOGI(TAG, "BLE Provisioning Advertising Started");
}

static int send_message_notification(uint16_t conn_handle, const char *msg)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || msg_handle == 0)
        return -1;

    struct os_mbuf *mbuf = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!mbuf)
        return -1;

    int rc = ble_gatts_notify_custom(conn_handle, msg_handle, mbuf);
    if (rc != 0)
    {
        os_mbuf_free(mbuf);
    }
    else
    {
        ESP_LOGI(TAG, "Sent notification: %s", msg);
    }
    return rc;
}

static void notification_consumer_task(void *param)
{
    notification_msg_t queue_msg;
    ESP_LOGI(TAG, "Notification Consumer Task started");

    while (1)
    {
        if (xQueueReceive(notification_queue, &queue_msg, portMAX_DELAY) == pdTRUE)
        {
            if (current_conn_handle != BLE_HS_CONN_HANDLE_NONE)
            {
                send_message_notification(queue_msg.conn_handle, queue_msg.msg);
            }
        }
    }
    vTaskDelete(NULL);
}

void send_notification_safe(const char *msg)
{
    if (notification_queue == NULL)
        return;

    uint16_t conn_handle = current_conn_handle;
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
        return;

    notification_msg_t queue_msg;
    queue_msg.conn_handle = conn_handle;
    strncpy(queue_msg.msg, msg, sizeof(queue_msg.msg) - 1);
    queue_msg.msg[sizeof(queue_msg.msg) - 1] = '\0';

    if (xQueueSend(notification_queue, &queue_msg, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGW(TAG, "Notification queue full; dropped message");
    }
}

// Advanced OTA task with yielding and progress
static void ota_update_task(void *param)
{
    const char *url = (const char *)param;

    ota_in_progress = true;
    send_notification_safe("OTA: Starting...");

    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = 30000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_https_ota_handle_t https_ota_handle = NULL;
    esp_err_t err = ESP_OK;

    // Declare variables early to allow safe goto
    int total_read_prev = 0;

    err = esp_https_ota_begin(&ota_config, &https_ota_handle);
    if (err != ESP_OK || https_ota_handle == NULL)
    {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        send_notification_safe("OTA: Begin failed");
        goto ota_cleanup;
    }

    send_notification_safe("OTA: Downloading...");

    while (1)
    {
        err = esp_https_ota_perform(https_ota_handle);
        if (err == ESP_ERR_HTTPS_OTA_IN_PROGRESS)
        {
            int total_read = esp_https_ota_get_image_len_read(https_ota_handle);
            if (total_read > total_read_prev + 10240)
            { // Update every ~10KB
                total_read_prev = total_read;
                char prog_msg[64];
                snprintf(prog_msg, sizeof(prog_msg), "OTA: %u KB", (unsigned)(total_read / 1024));
                send_notification_safe(prog_msg);
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Yield for BLE responsiveness
        }
        else
        {
            break;
        }
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "OTA perform failed: %s", esp_err_to_name(err));
        send_notification_safe("OTA: Download failed");
        goto ota_cleanup;
    }

    if (esp_https_ota_is_complete_data_received(https_ota_handle))
    {
        err = esp_https_ota_finish(https_ota_handle);
        https_ota_handle = NULL; // Prevent double finish
        if (err == ESP_OK)
        {
            send_notification_safe("OTA: Success - Rebooting...");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }
        else
        {
            ESP_LOGE(TAG, "OTA verify failed: %s", esp_err_to_name(err));
            send_notification_safe("OTA: Verify failed");
        }
    }
    else
    {
        send_notification_safe("OTA: Incomplete");
    }

ota_cleanup:
    if (https_ota_handle)
    {
        esp_https_ota_finish(https_ota_handle);
    }
    ota_in_progress = false;
    ota_task_handle = NULL;
    vTaskDelete(NULL);
}

static int ble_provisioning_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
    {
        if (attr_handle == msg_handle)
        {
            const char *default_msg = "Ready";
            rc = os_mbuf_append(ctxt->om, default_msg, strlen(default_msg));
        }
        else if (attr_handle == wifi_status_handle)
        {
            uint8_t status = MyWiFi::isWiFiEnabled() ? 1 : 0;
            rc = os_mbuf_append(ctxt->om, &status, 1);
        }
        else if (attr_handle == device_id_handle)
        {
            const char *id = MySystem::get_device_id();
            rc = os_mbuf_append(ctxt->om, id, strlen(id));
        }
        else if (attr_handle == ota_status_handle)
        {
            const char *status = ota_in_progress ? "OTA in progress..."
                                                 : (pending_ota_url[0] != '\0' ? "OTA pending..." : "Ready for OTA");
            rc = os_mbuf_append(ctxt->om, status, strlen(status));
        }
        else
        {
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR)
    {
        size_t len = ctxt->om->om_len;
        uint8_t *data = ctxt->om->om_data;

        if (attr_handle == ssid_handle)
        {
            len = len < sizeof(received_ssid) - 1 ? len : sizeof(received_ssid) - 1;
            memcpy(received_ssid, data, len);
            received_ssid[len] = '\0';
            ESP_LOGI(TAG, "Received SSID: %s", received_ssid);
        }
        else if (attr_handle == pass_handle)
        {
            len = len < sizeof(received_pass) - 1 ? len : sizeof(received_pass) - 1;
            memcpy(received_pass, data, len);
            received_pass[len] = '\0';
            ESP_LOGI(TAG, "Received PASS: %s", received_pass);
        }
        else if (attr_handle == reboot_handle)
        {
            ESP_LOGI(TAG, "Reboot command received.");
            if (strlen(received_ssid) > 0 && strlen(received_pass) > 0)
            {
                nvs_save_credentials(received_ssid, received_pass);
            }
            if (provisioning_sem)
                xSemaphoreGive(provisioning_sem);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        else if (attr_handle == wifi_enable_handle)
        {
            if (len >= 1)
            {
                bool enable = (data[0] != 0);
                MyWiFi::setWiFiEnabled(enable);
                send_notification_safe(enable ? "WiFi: ON" : "WiFi: OFF");
            }
        }
        else if (attr_handle == wifi_toggle_handle)
        {
            MyWiFi::toggleWiFi();
            bool now_on = MyWiFi::isWiFiEnabled();
            send_notification_safe(now_on ? "WiFi: ON" : "WiFi: OFF");
        }
        else if (attr_handle == ota_url_handle)
        {
            if (ota_in_progress)
            {
                ESP_LOGW(TAG, "OTA already in progress");
                send_notification_safe("OTA: Already running");
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            // Create a buffer to hold the full URL
            char full_url[256]; // Adjust size as needed

            // Define the base path
            const char *base_path = "http://filesrv.canactiveindustries.com/firmware/humidity-sensor/";

            // Build the full URL by concatenating base path and filename
            snprintf(full_url, sizeof(full_url), "%s%s", base_path, data);

            strcpy(pending_ota_url, full_url);
            ESP_LOGI(TAG, "Received OTA URL: %s", pending_ota_url);

            // Create OTA task with lower priority (4) and large stack for better BLE coexistence
            if (xTaskCreate(ota_update_task, "ota_update", 16384, (void *)pending_ota_url, 4, &ota_task_handle) != pdPASS)
            {
                ESP_LOGE(TAG, "Failed to create OTA task");
                send_notification_safe("OTA: Task failed");
            }
            else
            {
                send_notification_safe("OTA: Started");
            }
        }
        else
        {
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    return 0;
}

static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    ESP_LOGI(TAG, "BLE Host Task stopped.");
    vTaskDelete(NULL);
}

void ble_provisioning_init(bool blocking)
{
    ESP_LOGI(TAG, "Starting BLE Provisioning (blocking=%d)", blocking);

    memset(received_ssid, 0, sizeof(received_ssid));
    memset(received_pass, 0, sizeof(received_pass));
    current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    pending_ota_url[0] = '\0';
    ota_in_progress = false;

    provisioning_sem = xSemaphoreCreateBinary();
    notification_queue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(notification_msg_t));

    nimble_port_init();
    gatt_svr_init();

    ble_hs_cfg.sync_cb = ble_hs_sync_cb;

    xTaskCreate(ble_host_task, "ble_host", 12288, NULL, 5, &ble_host_handle);
    xTaskCreate(notification_consumer_task, "notify_consumer", 4096, NULL, 4, NULL);

    if (blocking)
    {
        if (xSemaphoreTake(provisioning_sem, pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS)) == pdTRUE)
        {
            ESP_LOGI(TAG, "Provisioning completed");
        }
        else
        {
            ESP_LOGE(TAG, "Provisioning timed out");
        }
        vSemaphoreDelete(provisioning_sem);
        provisioning_sem = NULL;
    }
}

void ble_provisioning_deinit(void)
{
    if (ble_host_handle)
    {
        vTaskDelete(ble_host_handle);
        ble_host_handle = NULL;
    }
    if (notification_queue)
    {
        vQueueDelete(notification_queue);
        notification_queue = NULL;
    }
    if (provisioning_sem)
    {
        vSemaphoreDelete(provisioning_sem);
        provisioning_sem = NULL;
    }
    ESP_LOGI(TAG, "BLE Provisioning deinitialized.");
}