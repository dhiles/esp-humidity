#ifdef __cplusplus
extern "C"
{
#endif

#include "ble_provisioning.h"
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_hs_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "os/os_mbuf.h"          // Contains declaration for os_mbuf_free()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_system.h" // For esp_restart()

// NOTE: Ensure your header or main file provides a definition for:
// void nvs_save_credentials(const char *ssid, const char *pass);
// And PROVISIONING_TIMEOUT_MS

#define DEVICE_NAME "ESP32-PROV"
#define MSG_QUEUE_SIZE 10  // Adjust based on expected rate
#define MAX_MSG_LEN 128    // Max message size for queue

static const char *TAG = "BLE_PROV";

// --- Global NimBLE UUIDs (Static Byte-Reversed Definition) ---
// Service UUID: 0000AAB0-218F-4424-8A05-F417242F7B0B
static const ble_uuid128_t PROV_SERVICE_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB0, 0xAA, 0x00, 0x00}};

// SSID Characteristic UUID: 0000AAB1-218F-4424-8A05-F417242F7B0B
static const ble_uuid128_t SSID_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB1, 0xAA, 0x00, 0x00}};

// Password Characteristic UUID: 0000AAB2-218F-4424-8A05-F417242F7B0B
static const ble_uuid128_t PASS_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB2, 0xAA, 0x00, 0x00}};

// Reboot Characteristic UUID: 0000AAB3-218F-4424-8A05-F417242F7B0B
static const ble_uuid128_t REBOOT_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB3, 0xAA, 0x00, 0x00}};

// Message Notifications Characteristic UUID: 0000AAB5-218F-4424-8A05-F417242F7B0B (notify-enabled)
static const ble_uuid128_t MSG_CHAR_UUID = {
    .u = {.type = BLE_UUID_TYPE_128},
    .value = {0x0B, 0x7B, 0x2F, 0x24, 0x17, 0xF4, 0x05, 0x8A, 0x24, 0x44, 0x8F, 0x21, 0xB5, 0xAA, 0x00, 0x00}};
// --- End Global NimBLE UUIDs ---

// Global variables
SemaphoreHandle_t provisioning_sem = NULL;
static TaskHandle_t ble_host_handle = NULL;
static char received_ssid[33] = {0};
static char received_pass[64] = {0};

// GATT attribute handles
static uint16_t ssid_handle;
static uint16_t pass_handle;
static uint16_t reboot_handle;
static uint16_t msg_handle;  // Handle for message notifications

// Queue for safe notifications from other tasks
static QueueHandle_t notification_queue = NULL;

// Message structure for queue
typedef struct {
    uint16_t conn_handle;
    char msg[MAX_MSG_LEN];
} notification_msg_t;

// Global for current connection handle (for main task use; update in events)
static volatile uint16_t current_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Global for address type (or pass as param)
static uint8_t own_addr_type;

// Forward declarations
static int ble_provisioning_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_prov_advertise(void);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static int send_message_notification(uint16_t conn_handle, const char* msg);
static void notify_demo_task(void* param);

// Demo task handle (optional, for cleanup)
static TaskHandle_t notify_demo_task_handle = NULL;

static void format_addr(char *buf, const uint8_t *addr)
{
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x",
            addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

/**
 * @brief GAP event callback handler for BLE connections and advertising events.
 */
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection established; status=%d", event->connect.status);
        if (event->connect.status == 0) {
            current_conn_handle = event->connect.conn_handle;  // Update global
            // Start a demo notifier task
            xTaskCreate(notify_demo_task, "notify_demo", 8192, (void*)(uintptr_t)event->connect.conn_handle, 3, &notify_demo_task_handle);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Connection lost; reason=%d", event->disconnect.reason);
        current_conn_handle = BLE_HS_CONN_HANDLE_NONE;  // Clear global
        if (notify_demo_task_handle != NULL) {
            vTaskDelete(notify_demo_task_handle);
            notify_demo_task_handle = NULL;
        }
        // Restart advertising if provisioning is not yet complete
        if (provisioning_sem != NULL && uxSemaphoreGetCount(provisioning_sem) == 0) {
            ESP_LOGI(TAG, "Restarting advertising after disconnect...");
            ble_prov_advertise();
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
        // Restart advertising if provisioning is not yet complete
        if (provisioning_sem != NULL && uxSemaphoreGetCount(provisioning_sem) == 0) {
            ESP_LOGI(TAG, "Restarting advertising after adv complete...");
            ble_prov_advertise();
        }
        break;

    default:
        break;
    }

    return 0;
}

/**
 * @brief Definition of the custom provisioning GATT service and its characteristics.
 */
static const struct ble_gatt_chr_def provisioning_chars[] = {
    {
        // SSID Characteristic (AAB1)
        .uuid = &SSID_CHAR_UUID.u,
        .access_cb = ble_provisioning_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &ssid_handle,
    },
    {
        // Password Characteristic (AAB2)
        .uuid = &PASS_CHAR_UUID.u,
        .access_cb = ble_provisioning_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &pass_handle,
    },
    {
        // Reboot Characteristic (AAB3) - Write property to trigger final action
        .uuid = &REBOOT_CHAR_UUID.u,
        .access_cb = ble_provisioning_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &reboot_handle,
    },
    {
        // Message Notifications Characteristic (AAB5) - Read + Notify
        .uuid = &MSG_CHAR_UUID.u,
        .access_cb = ble_provisioning_access_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,  // Enable read & notify
        .val_handle = &msg_handle,
    },
    {0}, // Stop marker
};

static const struct ble_gatt_svc_def provisioning_service[] = {
    {
        // Provisioning Service (AAB0)
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &PROV_SERVICE_UUID.u,
        .characteristics = provisioning_chars,
    },
    {0}, // Stop marker
};

/**
 * @brief Registers the GATT service definitions.
 */
static void gatt_svr_init(void)
{
    int rc;

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Register the custom provisioning service
    rc = ble_gatts_count_cfg(provisioning_service);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error counting GATT cfg; rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(provisioning_service);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error adding GATT svcs; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "Custom Provisioning Service (AAB0) registered successfully. SSID: %d, PASS: %d, Reboot: %d, Msg: %d", ssid_handle, pass_handle, reboot_handle, msg_handle);
}

/**
 * @brief Host synchronization callback: start advertising.
 */
static void ble_hs_sync_cb(void)
{
    int rc;

    // Set device name
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting device name; rc=%d", rc);
        return;
    }

    // Ensure identity address is configured (public or static random)
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to ensure BT identity address; rc=%d", rc);
        return;
    }

    // Infer address type (public=0, random=1)
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type; rc=%d", rc);
        return;
    }

    // Copy address for logging (optional but useful)
    uint8_t addr_val[6];
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to copy address; rc=%d", rc);
        return;
    }
    char addr_str[18];
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "Using BT identity address: %s (type %d)", addr_str, own_addr_type);

    // Start advertising with configured address type
    ble_prov_advertise();
}

/**
 * @brief Builds and sets the Scan Response payload.
 */
static void ble_prov_set_scan_rsp_data(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Set the service UUIDs in the scan response (CRITICAL for discovery)
    fields.uuids128 = (ble_uuid128_t *) &PROV_SERVICE_UUID;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_rsp_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting scan response data; rc=%d", rc);
    }
}

/**
 * @brief Builds the advertising payload and starts advertising.
 */
static void ble_prov_advertise(void)
{
    // --- Advertisement Packet Data (AD) ---
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set COMPLETE local name in primary advertisement
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error setting advertisement data; rc=%d", rc);
        return;
    }

    // --- Scan Response Packet Data (SRD) --- (includes UUID)
    ble_prov_set_scan_rsp_data();

    // --- Start Advertising ---
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(
        own_addr_type, // Use the configured identity address type
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        ble_gap_event_cb,
        NULL);

    if (rc != 0)
    {
        ESP_LOGE(TAG, "Error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE Provisioning Advertising Started as '%s' with addr type %d", DEVICE_NAME, own_addr_type);
}

/**
 * @brief Send a notification to the connected client via the message characteristic.
 * @param conn_handle: BLE connection handle
 * @param msg: Null-terminated string to send (e.g., "Event: Button pressed")
 * @return 0 on success, error code otherwise
 */
static int send_message_notification(uint16_t conn_handle, const char* msg) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || msg_handle == 0) {
        ESP_LOGE(TAG, "Invalid conn_handle or msg_handle for notification");
        return -1;
    }

    // Use ble_hs_mbuf_from_flat to create the message buffer
    struct os_mbuf* mbuf = ble_hs_mbuf_from_flat(msg, strlen(msg));
    if (!mbuf) {
        ESP_LOGE(TAG, "Failed to create mbuf from flat data");
        return -1;
    }

    // Send notification (only if client subscribed; handled internally)
    int rc = ble_gatts_notify_custom(conn_handle, msg_handle, mbuf);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send notification; rc=%d", rc);
        // Note: mbuf ownership is transferred on success, but must be freed on error
        // FIX: Replaced ble_hs_mbuf_free with os_mbuf_free
        os_mbuf_free(mbuf);
    } else {
        ESP_LOGI(TAG, "Sent notification: %s", msg);
    }

    return rc;
}

/**
 * @brief Safe wrapper to send notifications from other tasks (posts to queue).
 * This function retrieves the connection handle from the global state.
 * @param msg: Null-terminated string to send
 */
void send_notification_safe(const char* msg) { 
    if (notification_queue == NULL) {
        ESP_LOGE(TAG, "Notification queue not initialized");
        return;
    }
    
    // Retrieve the handle from the global variable
    uint16_t conn_handle = current_conn_handle; 

    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGW(TAG, "Cannot send notification: No active connection.");
        return;
    }


    notification_msg_t queue_msg;
    queue_msg.conn_handle = conn_handle; // Use the retrieved handle
    strncpy(queue_msg.msg, msg, sizeof(queue_msg.msg) - 1);
    queue_msg.msg[sizeof(queue_msg.msg) - 1] = '\0';

    // Must check if calling from ISR for xQueueSend. Use xQueueSendFromISR if needed.
    if (xQueueSend(notification_queue, &queue_msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Notification queue full; dropped message: %s", msg);
    }
}

/**
 * @brief Demo task to send periodic notifications (for testing).
 */
static void notify_demo_task(void* param) {
  //  uint16_t conn_handle = (uint16_t)(uintptr_t)param;
    int counter = 0;
    while (1) {  // Run until disconnect (handled in gap_cb)
        
        char msg[50];
        snprintf(msg, sizeof(msg), "Demo event #%d: Hello from ESP32!", ++counter);
        
        // Use the safe queue wrapper to send the message from the non-BLE-host task
        send_notification_safe(msg);
        
        vTaskDelay(pdMS_TO_TICKS(5000));  // Every 5s
    }
    // Task is typically deleted by gap_cb on disconnect, but keep vTaskDelete(NULL) here for completeness if loop breaks
    vTaskDelete(NULL); 
}

/**
 * @brief GATT access callback: Handles reads/writes from the Flutter app.
 * Extended to support read for msg_handle (returns default msg).
 */
static int ble_provisioning_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int rc = 0;

    // Handle READ operations
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        if (attr_handle == msg_handle) {
            // Optional: Return a default message on read, e.g., "Ready for events"
            const char* default_msg = "Ready";
            ESP_LOGI(TAG, "Read request for message char on conn %d", conn_handle);
            rc = os_mbuf_append(ctxt->om, default_msg, strlen(default_msg));
            if (rc != 0) {
                ESP_LOGE(TAG, "Error appending default msg; rc=%d", rc);
                return rc;
            }
            return 0;
        } else {
            // For other handles (write-only), deny read
            ESP_LOGW(TAG, "Read attempted on write-only handle %d", attr_handle);
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }
    }

    // Handle WRITE operations (existing logic unchanged)
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (attr_handle == ssid_handle)
        {
            size_t len = ctxt->om->om_len;
            len = len < sizeof(received_ssid) - 1 ? len : sizeof(received_ssid) - 1;
            memcpy(received_ssid, ctxt->om->om_data, len);
            received_ssid[len] = '\0';
            ESP_LOGI(TAG, "Received SSID: %s len=%d", received_ssid, len);
        }
        else if (attr_handle == pass_handle)
        {
            size_t len = ctxt->om->om_len;
            len = len < sizeof(received_pass) - 1 ? len : sizeof(received_pass) - 1;
            memcpy(received_pass, ctxt->om->om_data, len);
            received_pass[len] = '\0';
            ESP_LOGI(TAG, "Received PASS: %s len=%d", received_pass, len);
        }
        else if (attr_handle == reboot_handle)
        {
            ESP_LOGI(TAG, "Reboot command received. Finalizing provisioning.");

            // Check if both credentials were received before saving
            if (strlen(received_ssid) > 0 && strlen(received_pass) > 0) {
                nvs_save_credentials(received_ssid, received_pass);
                ESP_LOGI(TAG, "Credentials saved. Signaling completion...");
            } else {
                ESP_LOGE(TAG, "Reboot requested but missing credentials!");
            }

            if (provisioning_sem != NULL)
            {
                xSemaphoreGive(provisioning_sem);
            }
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        else {
            ESP_LOGW(TAG, "Unexpected write to handle %d", attr_handle);
        }
    }

    return 0; // Success
}

/**
 * @brief The function executed when the BLE Host is ready.
 */
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started (Persistent Mode)");
    
    // The correct function to run the NimBLE event loop in ESP-IDF
    nimble_port_run();

    ESP_LOGI(TAG, "BLE Host Task stopped.");
    // Free the queue and semaphore if task exits unexpectedly
    if (notification_queue) { vQueueDelete(notification_queue); notification_queue = NULL; }
    if (provisioning_sem) { vSemaphoreDelete(provisioning_sem); provisioning_sem = NULL; }
    ble_host_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief Initializes and runs the NimBLE host task, optionally blocking until provisioning completes or times out.
 */
void ble_provisioning_init(bool blocking)
{
    ESP_LOGI(TAG, "Starting BLE Provisioning (blocking=%d).", blocking);

    // Initial cleanup/reset of globals
    ble_host_handle = NULL;
    provisioning_sem = NULL;
    notification_queue = NULL;
    memset(received_ssid, 0, sizeof(received_ssid));
    memset(received_pass, 0, sizeof(received_pass));
    notify_demo_task_handle = NULL;
    current_conn_handle = BLE_HS_CONN_HANDLE_NONE;

    // 1. Create synchronization objects
    provisioning_sem = xSemaphoreCreateBinary();
    if (provisioning_sem == NULL) {
        ESP_LOGE(TAG, "Failed to create provisioning semaphore");
        return;
    }
    notification_queue = xQueueCreate(MSG_QUEUE_SIZE, sizeof(notification_msg_t));
    if (notification_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create notification queue");
        vSemaphoreDelete(provisioning_sem);
        return;
    }

    // 2. Initialize NimBLE port (sets up controller and base host)
    nimble_port_init();

    // 3. Register GATT services (BEFORE host is synced)
    gatt_svr_init();

    // 4. Set host synchronization callback
    ble_hs_cfg.sync_cb = ble_hs_sync_cb;

    // 5. Start the BLE host task
    if (ble_host_handle == NULL) {
        // Use a static size or a config macro for stack size
        xTaskCreate(ble_host_task, "ble_host", 8192, NULL, 5, &ble_host_handle);
    }

    // 6. Blocking wait if requested
    if (blocking) {
        BaseType_t sem_result = xSemaphoreTake(provisioning_sem, pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS));
        if (sem_result == pdTRUE) {
            ESP_LOGI(TAG, "Provisioning completed successfully");
        } else {
            ESP_LOGE(TAG, "Provisioning timed out. BLE remains active.");
        }
        // Provisioning attempt is over; we can delete the semaphore
        vSemaphoreDelete(provisioning_sem);
        provisioning_sem = NULL;
    }
    // Else: Non-blocking, task runs forever
}

#ifdef __cplusplus
}
#endif
