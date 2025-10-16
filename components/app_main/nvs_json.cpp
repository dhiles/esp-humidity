#include "nvs_json.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_system.h"

static const char *TAG = "NVS_JSON";

// Structure to store namespace and its JSON object
typedef struct
{
    char *name;  // Short namespace name (e.g., "wifi" from "config.wifi")
    cJSON *json; // JSON object for the namespace
} namespace_json_t;

// Dynamic array to store namespace JSON objects
static namespace_json_t *namespace_list = NULL;
static size_t namespace_count = 0;

// Prefix for namespaces to process
static const char *NAMESPACE_PREFIX = "config.";
static const size_t PREFIX_LEN = 7;      // Length of "config."
static const size_t MAX_NAMESPACES = 10; // Maximum number of namespaces to process

// Helper function to check if namespace is a system namespace
static bool is_system_namespace(const char *ns)
{
    if (!ns)
        return false;

    return (strcmp(ns, "nvs.net80211") == 0 ||
            strcmp(ns, "phy") == 0);
}

// Helper function to check if a namespace starts with config. and extract short name
static bool is_config_namespace(const char *ns, char **short_name)
{
    if (!ns || strncmp(ns, NAMESPACE_PREFIX, PREFIX_LEN) != 0)
    {
        return false;
    }
    const char *short_part = ns + PREFIX_LEN;
    size_t short_len = strlen(short_part);
    if (short_len == 0 || short_len > 15 - PREFIX_LEN)
    {
        ESP_LOGW(TAG, "Invalid short namespace name after config.: %s", short_part);
        return false;
    }
    *short_name = strdup(short_part);
    if (!*short_name)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for short namespace name %s (free heap: %lu)", short_part, esp_get_free_heap_size());
        return false;
    }
    return true;
}

// Helper function to convert blob to hex string
static char *blob_to_hex(const uint8_t *blob, size_t len)
{
    char *hex_str = (char *)heap_caps_malloc(2 * len + 1, MALLOC_CAP_8BIT);
    if (!hex_str)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for hex string (%d bytes, free heap: %lu)", 2 * len + 1, esp_get_free_heap_size());
        return NULL;
    }
    for (size_t i = 0; i < len; i++)
    {
        sprintf(hex_str + 2 * i, "%02x", blob[i]);
    }
    hex_str[2 * len] = '\0';
    return hex_str;
}

// Debug function to list all namespaces and their keys/values
static void log_all_namespaces(void)
{
    ESP_LOGI(TAG, "Listing all namespaces and keys (free heap: %lu bytes)", esp_get_free_heap_size());
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No namespaces found in NVS partition");
        return;
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start NVS iterator: %s", esp_err_to_name(err));
        return;
    }

    char current_ns[16] = "";
    nvs_handle_t handle = 0;
    bool handle_open = false;

    while (err == ESP_OK && it != NULL)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        if (is_system_namespace(info.namespace_name))
        {
            err = nvs_entry_next(&it);
            continue;
        }

        // Open namespace if changed
        if (strcmp(current_ns, info.namespace_name) != 0)
        {
            if (handle_open)
            {
                nvs_close(handle);
                handle_open = false;
            }
            strncpy(current_ns, info.namespace_name, sizeof(current_ns) - 1);
            current_ns[sizeof(current_ns) - 1] = '\0';
            ESP_LOGI(TAG, "Namespace: %s", current_ns);
            err = nvs_open_from_partition("nvs", current_ns, NVS_READONLY, &handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to open namespace %s: %s", current_ns, esp_err_to_name(err));
                err = nvs_entry_next(&it);
                continue;
            }
            handle_open = true;
        }

        // Log key and value
        ESP_LOGI(TAG, "Key: %s, Type: %d", info.key, info.type);
        switch (info.type)
        {
        case NVS_TYPE_STR:
        {
            size_t length = 0;
            if (nvs_get_str(handle, info.key, NULL, &length) == ESP_OK)
            {
                char *value = (char *)heap_caps_malloc(length, MALLOC_CAP_8BIT);
                if (value && nvs_get_str(handle, info.key, value, &length) == ESP_OK)
                {
                    ESP_LOGI(TAG, "Value: %s (string)", value);
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to read string %s", info.key);
                }
                heap_caps_free(value);
            }
            break;
        }
        case NVS_TYPE_BLOB:
        {
            size_t length = 0;
            if (nvs_get_blob(handle, info.key, NULL, &length) == ESP_OK)
            {
                if (length > 1024)
                {
                    ESP_LOGW(TAG, "Skipping large blob %s (%d bytes)", info.key, length);
                    break;
                }
                uint8_t *value = (uint8_t *)heap_caps_malloc(length, MALLOC_CAP_8BIT);
                if (value && nvs_get_blob(handle, info.key, value, &length) == ESP_OK)
                {
                    char *hex_str = blob_to_hex(value, length);
                    if (hex_str)
                    {
                        ESP_LOGI(TAG, "Value: %s (blob as hex)", hex_str);
                        heap_caps_free(hex_str);
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to read blob %s", info.key);
                }
                heap_caps_free(value);
            }
            break;
        }
        case NVS_TYPE_U8:
        {
            uint8_t value;
            if (nvs_get_u8(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %u (uint8)", value);
            }
            break;
        }
        case NVS_TYPE_I8:
        {
            int8_t value;
            if (nvs_get_i8(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %d (int8)", value);
            }
            break;
        }
        case NVS_TYPE_U16:
        {
            uint16_t value;
            if (nvs_get_u16(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %u (uint16)", value);
            }
            break;
        }
        case NVS_TYPE_I16:
        {
            int16_t value;
            if (nvs_get_i16(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %d (int16)", value);
            }
            break;
        }
        case NVS_TYPE_U32:
        {
            uint32_t value;
            if (nvs_get_u32(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %lu (uint32)", value);
            }
            break;
        }
        case NVS_TYPE_I32:
        {
            int32_t value;
            if (nvs_get_i32(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %ld (int32)", value);
            }
            break;
        }
        case NVS_TYPE_U64:
        {
            uint64_t value;
            if (nvs_get_u64(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %llu (uint64)", value);
            }
            break;
        }
        case NVS_TYPE_I64:
        {
            int64_t value;
            if (nvs_get_i64(handle, info.key, &value) == ESP_OK)
            {
                ESP_LOGI(TAG, "Value: %lld (int64)", value);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unsupported type: %d for key %s", info.type, info.key);
            break;
        }

        err = nvs_entry_next(&it);
    }

    if (handle_open)
    {
        nvs_close(handle);
    }
    nvs_release_iterator(it);
    if (err != ESP_ERR_NVS_NOT_FOUND && err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error listing namespaces: %s", esp_err_to_name(err));
    }
}

// Load JSON for a single namespace
static esp_err_t load_namespace_json(const char *full_ns, cJSON **json_output)
{
    if (!full_ns || !json_output)
    {
        ESP_LOGE(TAG, "Invalid parameters for load_namespace_json");
        return ESP_ERR_INVALID_ARG;
    }

    // Open NVS handle
    nvs_handle_t handle;
    esp_err_t err = nvs_open_from_partition("nvs", full_ns, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open namespace '%s': %s", full_ns, esp_err_to_name(err));
        return err;
    }

    // Create JSON object
    *json_output = cJSON_CreateObject();
    if (!*json_output)
    {
        ESP_LOGE(TAG, "Failed to create JSON object for %s (free heap: %lu)", full_ns, esp_get_free_heap_size());
        nvs_close(handle);
        return ESP_ERR_NO_MEM;
    }

    // Iterate through NVS entries
    nvs_iterator_t it = NULL;
    err = nvs_entry_find("nvs", full_ns, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No keys found in namespace %s", full_ns);
        nvs_close(handle);
        return ESP_OK;
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start iterator for %s: %s", full_ns, esp_err_to_name(err));
        cJSON_Delete(*json_output);
        *json_output = NULL;
        nvs_close(handle);
        return err;
    }

    while (err == ESP_OK && it != NULL)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        ESP_LOGI(TAG, "Found key: %s, type: %d in namespace %s", info.key, info.type, full_ns);

        // Read value based on type
        switch (info.type)
        {
        case NVS_TYPE_U8:
        {
            uint8_t value;
            if ((err = nvs_get_u8(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %u (uint8)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_I8:
        {
            int8_t value;
            if ((err = nvs_get_i8(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %d (int8)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_U16:
        {
            uint16_t value;
            if ((err = nvs_get_u16(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %u (uint16)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_I16:
        {
            int16_t value;
            if ((err = nvs_get_i16(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %d (int16)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_U32:
        {
            uint32_t value;
            if ((err = nvs_get_u32(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %lu (uint32)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_I32:
        {
            int32_t value;
            if ((err = nvs_get_i32(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %ld (int32)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_U64:
        {
            uint64_t value;
            if ((err = nvs_get_u64(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %llu (uint64)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_I64:
        {
            int64_t value;
            if ((err = nvs_get_i64(handle, info.key, &value)) == ESP_OK)
            {
                cJSON_AddNumberToObject(*json_output, info.key, value);
                ESP_LOGI(TAG, "Added %s: %lld (int64)", info.key, value);
            }
            break;
        }
        case NVS_TYPE_STR:
        {
            size_t length = 0;
            if ((err = nvs_get_str(handle, info.key, NULL, &length)) == ESP_OK)
            {
                if (length > 1024)
                {
                    ESP_LOGW(TAG, "Skipping large string %s (%d bytes)", info.key, length);
                    break;
                }
                char *value = (char *)heap_caps_malloc(length, MALLOC_CAP_8BIT);
                if (value && (err = nvs_get_str(handle, info.key, value, &length)) == ESP_OK)
                {
                    cJSON_AddStringToObject(*json_output, info.key, value);
                    ESP_LOGI(TAG, "Added %s: %s (string)", info.key, value);
                }
                else
                {
                    err = value ? err : ESP_ERR_NO_MEM;
                }
                heap_caps_free(value);
            }
            break;
        }
        case NVS_TYPE_BLOB:
        {
            size_t length = 0;
            if ((err = nvs_get_blob(handle, info.key, NULL, &length)) == ESP_OK)
            {
                if (length > 1024)
                {
                    ESP_LOGW(TAG, "Skipping large blob %s (%d bytes)", info.key, length);
                    break;
                }
                uint8_t *value = (uint8_t *)heap_caps_malloc(length, MALLOC_CAP_8BIT);
                if (value && (err = nvs_get_blob(handle, info.key, value, &length)) == ESP_OK)
                {
                    char *hex_str = blob_to_hex(value, length);
                    if (hex_str)
                    {
                        cJSON_AddStringToObject(*json_output, info.key, hex_str);
                        ESP_LOGI(TAG, "Added %s: %s (blob as hex)", info.key, hex_str);
                        heap_caps_free(hex_str);
                    }
                    else
                    {
                        err = ESP_ERR_NO_MEM;
                    }
                }
                else
                {
                    err = value ? err : ESP_ERR_NO_MEM;
                }
                heap_caps_free(value);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "Unsupported type %d for key %s", info.type, info.key);
            break;
        }

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read %s in %s: %s", info.key, full_ns, esp_err_to_name(err));
        }

        err = nvs_entry_next(&it);
    }

    nvs_release_iterator(it);
    nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        err = ESP_OK;
    }

    if (err != ESP_OK)
    {
        cJSON_Delete(*json_output);
        *json_output = NULL;
        ESP_LOGE(TAG, "Failed to load JSON for %s: %s", full_ns, esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "JSON initialized for namespace %s", full_ns);
    }

    return err;
}

// Initialize JSON objects for config.* namespaces
esp_err_t init_nvs_json_all(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init_partition("nvs");
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS partition invalid, erasing and reinitializing: %s", esp_err_to_name(err));
        err = nvs_flash_erase_partition("nvs");
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to erase NVS partition: %s", esp_err_to_name(err));
            return err;
        }
        err = nvs_flash_init_partition("nvs");
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS partition: %s", esp_err_to_name(err));
        return err;
    }

    // Log all namespaces for debugging
    log_all_namespaces();

    // Free existing JSON objects
    free_nvs_json_all();

    // Count config.* namespaces
    nvs_iterator_t it = NULL;
    err = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No namespaces found in NVS partition");
        return ESP_OK;
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start NVS iterator: %s", esp_err_to_name(err));
        return err;
    }

    char current_ns[16] = "";
    size_t ns_count = 0;
    while (err == ESP_OK && it != NULL && ns_count < MAX_NAMESPACES)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        if (is_system_namespace(info.namespace_name))
        {
            err = nvs_entry_next(&it);
            continue;
        }
        
        if (strcmp(current_ns, info.namespace_name) != 0)
        {
            strncpy(current_ns, info.namespace_name, sizeof(current_ns) - 1);
            current_ns[sizeof(current_ns) - 1] = '\0';
            char *short_name = NULL;
            if (is_config_namespace(current_ns, &short_name))
            {
                ESP_LOGI(TAG, "Found config namespace: %s (short: %s)", current_ns, short_name);
                ns_count++;
                free(short_name);
            }
        }
        err = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    if (err != ESP_ERR_NVS_NOT_FOUND && err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error counting namespaces: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Number of config.* namespaces: %d (free heap: %lu bytes)", ns_count, esp_get_free_heap_size());

    // Allocate array for namespace JSON objects
    if (ns_count == 0)
    {
        ESP_LOGI(TAG, "No config.* namespaces to process");
        return ESP_OK;
    }

    namespace_list = (namespace_json_t *)heap_caps_calloc(ns_count, sizeof(namespace_json_t), MALLOC_CAP_8BIT);
    if (!namespace_list)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for namespace list (%d bytes, free heap: %lu)", ns_count * sizeof(namespace_json_t), esp_get_free_heap_size());
        return ESP_ERR_NO_MEM;
    }
    namespace_count = ns_count;

    // Load JSON for each config.* namespace
    bool success = false;
    size_t ns_index = 0;
    current_ns[0] = '\0';
    it = NULL;
    err = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY, &it);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Failed to start NVS iterator for loading: %s", esp_err_to_name(err));
        free_nvs_json_all();
        return err;
    }

    while (err == ESP_OK && it != NULL && ns_index < namespace_count)
    {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        if (strcmp(current_ns, info.namespace_name) != 0)
        {
            strncpy(current_ns, info.namespace_name, sizeof(current_ns) - 1);
            current_ns[sizeof(current_ns) - 1] = '\0';
            char *short_name = NULL;
            if (is_config_namespace(current_ns, &short_name))
            {
                namespace_list[ns_index].name = short_name;
                err = load_namespace_json(current_ns, &namespace_list[ns_index].json);
                if (err == ESP_OK && namespace_list[ns_index].json)
                {
                    success = true;
                    char *json_str = cJSON_PrintUnformatted(namespace_list[ns_index].json);
                    if (json_str)
                    {
                        ESP_LOGI(TAG, "%s JSON: %s", short_name, json_str);
                        heap_caps_free(json_str);
                    }
                    ns_index++;
                }
                else
                {
                    ESP_LOGW(TAG, "Skipping namespace %s due to error: %s", current_ns, esp_err_to_name(err));
                    free(namespace_list[ns_index].name);
                    namespace_list[ns_index].name = NULL;
                }
            }
        }

        err = nvs_entry_next(&it);
    }

    nvs_release_iterator(it);

    // Update namespace count
    namespace_count = ns_index;

    if (!success)
    {
        ESP_LOGW(TAG, "No config.* namespaces loaded successfully");
        free_nvs_json_all();
        return ESP_OK;
    }

    ESP_LOGI(TAG, "All NVS JSON objects initialized successfully (%d namespaces, free heap: %lu bytes)", namespace_count, esp_get_free_heap_size());
    return ESP_OK;
}

// Get the JSON object for a specific namespace (using short name)
cJSON *get_nvs_json(const char *ns)
{
    if (!ns)
    {
        ESP_LOGW(TAG, "Invalid namespace parameter");
        return NULL;
    }
    for (size_t i = 0; i < namespace_count; i++)
    {
        if (namespace_list[i].name && strcmp(namespace_list[i].name, ns) == 0)
        {
            return namespace_list[i].json;
        }
    }
    ESP_LOGW(TAG, "Unknown namespace: %s", ns);
    return NULL;
}

// Free all global JSON objects
void free_nvs_json_all(void)
{
    if (!namespace_list && namespace_count == 0)
    {
        ESP_LOGI(TAG, "No NVS JSON objects to free");
        return;
    }
    for (size_t i = 0; i < namespace_count; i++)
    {
        if (namespace_list[i].json)
        {
            ESP_LOGD(TAG, "Freeing JSON for namespace %s", namespace_list[i].name ? namespace_list[i].name : "unknown");
            cJSON_Delete(namespace_list[i].json);
            namespace_list[i].json = NULL;
        }
        if (namespace_list[i].name)
        {
            ESP_LOGD(TAG, "Freeing name %s", namespace_list[i].name);
            free(namespace_list[i].name);
            namespace_list[i].name = NULL;
        }
    }
    ESP_LOGI(TAG, "Freeing namespace_list (count: %d)", namespace_count);
    heap_caps_free(namespace_list);
    namespace_list = NULL;
    namespace_count = 0;
    ESP_LOGI(TAG, "All NVS JSON objects freed (free heap: %lu bytes)", esp_get_free_heap_size());
}