/*
 * Copyright 2026 Rob Meades
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** @file
 * @brief Implementation of a the spider launcher.  Note that
 * this application uses several of the front garden railway (FGR)
 * components (cut down and modified from the originals), for instance
 * for the debug LED, task handling, non-volatile storage and a
 * few general utilities.
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "errno.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/ble_sm.h"      // For security manager constants
#include "host/util/util.h"

#include "fgr_util.h"
#include "fgr_debug.h"
#include "fgr_nvs.h"
#include "fgr_task.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

 // Logging prefix.
 #define TAG "spider_launcher"

// UART buffer.
#define UART_RX_BUFFFER_SIZE 256

// The name that we appear under in Bluetooth.
#define BLE_DEVICE_NAME "spider_launcher"

// Service UUID for spider launcher control.
#define SERVICE_UUID 0xFFE0

// The name for the "auto enable" entry in non-volatile storage.
#define NVS_NAME_AUTO_ENABLE "auto_enable"

// The name for the "auto period" entry in non-volatile storage.
#define NVS_NAME_AUTO_PERIOD_SECONDS "auto_period_sec"

// The name for the "height max" entry in non-volatile storage.
#define NVS_NAME_HEIGHT_MAX_MM "height_max_mm"

// The name for the "speed" entry in non-volatile storage (truncated
// to keep within 15 characters).
#define NVS_NAME_SPEED_MM_PER_SECOND "speed_mm_per_se"

// The name for the "random enable" entry in non-volatile storage.
#define NVS_NAME_RANDOM_ENABLE "random_enable"

// Default value for "auto enable".
#define DEFAULT_AUTO_ENABLE false

// Default value for "auto period".
#define DEFAULT_AUTO_PERIOD_SECONDS 60

// Default value for "height max".
#define DEFAULT_HEIGHT_MAX_MM 4000

// Default value for "speed".
#define DEFAULT_SPEED_MM_PER_SECOND 500

// Default value for "random enable".
#define DEFAULT_RANDOM_ENABLE true

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// BLE characteristics for our service.
// Note that this must start with CHARACTERISTIC_LAUNCH_NOW_UUID
// and increase by one each time for the g_characteristic_to_name[]
// table to work.
typedef enum {
    CHARACTERISTIC_LAUNCH_NOW_UUID           = 0xFFE1, // (has no value)
    CHARACTERISTIC_UP_UUID                   = 0xFFE2, // (has no value)
    CHARACTERISTIC_DOWN_UUID                 = 0xFFE3, // (has no value)
    CHARACTERISTIC_THIS_IS_GROUND_LEVEL_UUID = 0xFFE4, // (has no value)
    CHARACTERISTIC_THIS_IS_HEIGHT_MAX_UUID   = 0xFFE5, // (has no value)
    CHARACTERISTIC_RESET_TO_DEFAULTS_UUID    = 0xFFE6, // (has no value)
    CHARACTERISTIC_AUTO_ENABLE_UUID          = 0xFFE7, // Boolean
    CHARACTERISTIC_AUTO_PERIOD_SECONDS_UUID  = 0xFFE8, // uint32_t
    CHARACTERISTIC_HEIGHT_MAX_MM_UUID        = 0xFFE9, // uint32_t
    CHARACTERISTIC_SPEED_MM_PER_SECOND_UUID  = 0xFFEA, // uint32_t
    CHARACTERISTIC_RANDOM_ENABLE_UUID        = 0xFFEB  // Boolean
} characteristic_t;

// The context for the spider launcher.
typedef struct {
    QueueHandle_t command_queue;
    TaskHandle_t command_task;
    ble_uuid16_t spider_launcher_service_uuid;
    struct ble_hs_adv_fields ble_adv_fields;
    uint16_t ble_connection_handle;
    bool running;
} context_t;

// Command queue contents.
typedef struct {
    characteristic_t characteristic;
    uint32_t value;
    bool read_not_write;
} command_t;

/* ----------------------------------------------------------------
 * VARIABLES (THERE ARE MORE FURTHER DOWN)
 * -------------------------------------------------------------- */

// Set up valid advertising parameters.
static const struct ble_gap_adv_params g_ble_adv_params = {
    .conn_mode = BLE_GAP_CONN_MODE_UND,   // Connectable undirected advertising
    .disc_mode = BLE_GAP_DISC_MODE_GEN,   // General discoverable mode
    // Values are in 0.625ms units
    // 160 = 100ms, 320 = 200ms
    .itvl_min = 160,                      // Minimum advertising interval (100 ms)
    .itvl_max = 320};                     // Maximum advertising interval (200 ms)

// Context for the whole application
static context_t g_context = {
    .command_queue = NULL,
    .command_task = NULL,
    .spider_launcher_service_uuid = BLE_UUID16_INIT(SERVICE_UUID),
    .ble_adv_fields = {0},
    .ble_connection_handle = BLE_HS_CONN_HANDLE_NONE,
    .running = true};

// Names for each characteristic, used for debug prints only.
// Entries are in the same order as the characteristic enum
static const char *g_characteristic_to_name[] = {"LAUNCH_NOW",
                                                 "UP",
                                                 "DOWN",
                                                 "THIS_IS_GROUND_LEVEL",
                                                 "THIS_IS_HEIGHT_MAX",
                                                 "RESET_TO_DEFAULTS",
                                                 "AUTO",
                                                 "AUTO_PERIOD_SECONDS",
                                                 "HEIGHT_MAX_MM",
                                                 "SPEED_MM_PER_SECOND",
                                                 "RANDOM"};

// THERE ARE MORE VARIABLES FURTHER DOWN

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: NVS RELATED
 * -------------------------------------------------------------- */

// Retrieve whether auto is enabled or not from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_auto_enable_get(bool *enable)
{
    int32_t err = -ESP_ERR_INVALID_ARG;
    uint32_t value = 0;

    if (enable) {
        err = fgr_nvs_get(NVS_NAME_AUTO_ENABLE, &value);
        if (err == ESP_OK) {
            *enable = (value != 0);
        } else {
            *enable = DEFAULT_AUTO_ENABLE;
        }
    }

    return err;
}

// Set whether auto is enabled or not in NVS.
static int32_t nvs_auto_enable_set(bool enable)
{
    return fgr_nvs_set(NVS_NAME_AUTO_ENABLE, enable);
}

// Retrieve the auto period from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_auto_period_seconds_get(uint32_t *auto_period_seconds)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (auto_period_seconds) {
        err = fgr_nvs_get(NVS_NAME_AUTO_PERIOD_SECONDS, auto_period_seconds);
        if (err != ESP_OK) {
            *auto_period_seconds = DEFAULT_AUTO_PERIOD_SECONDS;
        }
    }

    return err;
}

// Set the auto period in NVS.
static int32_t nvs_auto_period_seconds_set(uint32_t auto_period_seconds)
{
    return fgr_nvs_set(NVS_NAME_AUTO_PERIOD_SECONDS, auto_period_seconds);
}

// Retrieve the height max from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_height_max_mm_get(uint32_t *height_max_mm)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (height_max_mm) {
        err = fgr_nvs_get(NVS_NAME_HEIGHT_MAX_MM, height_max_mm);
        if (err != ESP_OK) {
            *height_max_mm = DEFAULT_HEIGHT_MAX_MM;
        }
    }

    return err;
}

// Set the height max in NVS.
static int32_t nvs_height_max_mm_set(uint32_t height_max_mm)
{
    return fgr_nvs_set(NVS_NAME_HEIGHT_MAX_MM, height_max_mm);
}

// Retrieve the speed in mm per second from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_speed_mm_per_second_get(uint32_t *speed_mm_per_second)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (speed_mm_per_second) {
        err = fgr_nvs_get(NVS_NAME_SPEED_MM_PER_SECOND, speed_mm_per_second);
        if (err != ESP_OK) {
            *speed_mm_per_second = DEFAULT_SPEED_MM_PER_SECOND;
        }
    }

    return err;
}

// Set the speed mm per second in NVS.
static int32_t nvs_speed_mm_per_second_set(uint32_t speed_mm_per_second)
{
    return fgr_nvs_set(NVS_NAME_SPEED_MM_PER_SECOND, speed_mm_per_second);
}

// Retrieve whether random is enabled or not from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_random_enable_get(bool *enable)
{
    int32_t err = -ESP_ERR_INVALID_ARG;
    uint32_t value = 0;

    if (enable) {
        err = fgr_nvs_get(NVS_NAME_RANDOM_ENABLE, &value);
        if (err == ESP_OK) {
            *enable = (value != 0);
        } else {
            *enable = DEFAULT_RANDOM_ENABLE;
        }
    }

    return err;
}

// Set whether random is enabled or not in NVS.
static int32_t nvs_random_enable_set(bool enable)
{
    return fgr_nvs_set(NVS_NAME_RANDOM_ENABLE, enable);
}

// Populate NVS with defaults if required.
static void nvs_populate()
{
    bool enable;
    uint32_t value;

    // Since the NVS functions all return a default value
    // on error, we can write that value back to set up
    // NVS when it is blank
    if (nvs_auto_enable_get(&enable) != ESP_OK) {
        nvs_auto_enable_set(enable);
    }
    if (nvs_auto_period_seconds_get(&value) != ESP_OK) {
        nvs_auto_period_seconds_set(value);
    }
    if (nvs_height_max_mm_get(&value) != ESP_OK) {
        nvs_height_max_mm_set(value);
    }
    if (nvs_speed_mm_per_second_get(&value) != ESP_OK) {
        nvs_speed_mm_per_second_set(value);
    }
    if (nvs_random_enable_get(&enable) != ESP_OK) {
        nvs_random_enable_set(enable);
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTION PROTOTYPE: NEEDED FOR BLE CALLBACKS
 * -------------------------------------------------------------- */

static int ble_gap_event_callback(struct ble_gap_event *event, void *arg);

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: BLE
 * -------------------------------------------------------------- */

// Read up to four bytes from an mbuf, returning a
// int32_t made from them (with no endian changes,
// so the source must be litte-endian to work correctly).
static uint32_t mbuf_read(const struct os_mbuf *om)
{
    uint32_t value = 0;

    uint16_t len = os_mbuf_len(om);
    uint16_t read_len = sizeof(value);
    if (read_len > len) {
        read_len = len;
    }
    os_mbuf_copydata(om, 0, read_len, &value);
    ESP_LOGI(TAG, "BLE got data buffer length %d byte(s), extracting value %d.",
             len, value);

    return value;
}

// Start BLE advertising
static int32_t ble_start_advertising(context_t *context)
{
    ESP_LOGI(TAG, "BLE (re)starting advertising.");

    // CRITICAL: Zero out the advertising fields structure before populating
    memset(&context->ble_adv_fields, 0, sizeof(context->ble_adv_fields));

    // Set flags
    context->ble_adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set service UUID
    context->ble_adv_fields.uuids16 = &context->spider_launcher_service_uuid;
    context->ble_adv_fields.num_uuids16 = 1;
    context->ble_adv_fields.uuids16_is_complete = 1;

    // Optional: Set device name
    context->ble_adv_fields.name = (uint8_t *)BLE_DEVICE_NAME;
    context->ble_adv_fields.name_len = strlen(BLE_DEVICE_NAME);
    context->ble_adv_fields.name_is_complete = 1;

    int32_t err = ble_gap_adv_set_fields(&context->ble_adv_fields);
    if (err == 0) {
        err = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                &g_ble_adv_params, ble_gap_event_callback,
                                context);
        if (err != 0) {
            if (err == BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "Advertising already in progress");
            } else {
                ESP_LOGE(TAG, "Failed to start advertising: %d", err);
            }
        }
    } else {
        ESP_LOGE(TAG, "BLE failed to set advertisement fields: %d!", err);
    }

    return err;
}

// GAP event callback, handles connections.
static int ble_gap_event_callback(struct ble_gap_event *event, void *arg)
{
    context_t *context = (context_t *) arg;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
        {
            ESP_LOGI(TAG, "BLE_GAP_EVENT_CONNECT, status: %d.", event->connect.status);
            if (event->connect.status == 0) {
                // Small delay before requesting parameter update
                vTaskDelay(pdMS_TO_TICKS(100));

                // Propose connection parameters with a keep-alive
                struct ble_gap_upd_params params;
                params.itvl_min = 100;   // Connection interval min: 100 * 1.25ms = 125ms
                params.itvl_max = 100;   // Connection interval max (use same value for fixed interval)
                params.latency = 0;      // Slave latency: 0 = must respond to every connection event
                params.supervision_timeout = 400;  // Supervision timeout: 400 * 10ms = 4 seconds
                params.min_ce_len = 0;
                params.max_ce_len = 0;

                // Request connection parameter update
                int32_t err = ble_gap_update_params(event->connect.conn_handle, &params);
                if (err != 0) {
                    // There's nothing we can do if this fails, so continue
                    ESP_LOGW(TAG, "BLE failed to update connection params: %d", err);
                }

                context->ble_connection_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "BLE device connected successfully.");
            } else {
                ESP_LOGE(TAG, "BLE connection failed with status: %d!", event->connect.status);

                // A small delay before restarting advertising,
                // without this, NimBLE can get stuck in a bad state
                vTaskDelay(pdMS_TO_TICKS(200));
                ble_start_advertising(context);
            }
        }
        break;
        case BLE_GAP_EVENT_DISCONNECT:
        {
            ESP_LOGI(TAG, "BLE_GAP_EVENT_DISCONNECT, reason: %d.", event->disconnect.reason);
            context->ble_connection_handle = BLE_HS_CONN_HANDLE_NONE;

            // A small delay before restarting advertising,
            // without this, NimBLE can get stuck in a bad state
            vTaskDelay(pdMS_TO_TICKS(200));
            ble_start_advertising(context);
        }
        break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
        {
            ESP_LOGI(TAG, "BLE_GAP_EVENT_ADV_COMPLETE.");
        }
        break;

        case BLE_GAP_EVENT_CONN_UPDATE:
         {
            ESP_LOGI(TAG, "BLE_GAP_EVENT_CONN_UPDATE.");
                    // Accept connection parameter updates
            if (event->conn_update.status == 0) {
                struct ble_gap_conn_desc desc = {0};
                if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0) {
                    ESP_LOGI(TAG, "BLE new connection parameters:");
                    ESP_LOGI(TAG, "BLE   interval: %d", desc.conn_itvl);
                    ESP_LOGI(TAG, "BLE   latency: %d", desc.conn_latency);
                    ESP_LOGI(TAG, "BLE   supervision Timeout: %d", desc.supervision_timeout);
                }
            }
        }
        break;
        case BLE_GAP_EVENT_TERM_FAILURE:
        {
            ESP_LOGI(TAG, "BLE_GAP_EVENT_TERM_FAILURE, reason: %d", event->term_failure.status);
            // Restart advertising
            vTaskDelay(pdMS_TO_TICKS(200));
            ble_start_advertising(context);
        }
        break;
        default:
        break;
    }

    return 0;
}

// Called by BLE task when it has sorted itself out.
static void ble_on_sync_callback(void)
{
    int32_t err = ble_hs_util_ensure_addr(0);  // Use public address (0 = public)
    if (err == 0) {
        // Start BLE advertising
        err = ble_start_advertising(&g_context);
        if (err == 0) {
            // Print MAC address for info
            uint8_t ble_address_type;
            int32_t err = ble_hs_id_infer_auto(0, &ble_address_type);
            if (err == 0) {
                uint8_t address[6] = {0};
                err = ble_hs_id_copy_addr(ble_address_type, address, NULL);
                if (err == 0) {
                    ESP_LOGI(TAG, "BLE MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
                            address[0], address[1], address[2], address[3], address[4], address[5]);
                }
            }

            ESP_LOGI(TAG, "BLE ready, device name \"%s\".", BLE_DEVICE_NAME);
            ESP_LOGI(TAG, "BLE pair with device and connect using any Bluetooth terminal app.");
        }
    } else {
        ESP_LOGE(TAG, "BLE failed to ensure public address: %d", err);
    }
}

// Called by BLE task on a reset.
static void ble_on_reset_callback(int reason)
{
    ESP_LOGW(TAG, "\nBLE resetting state (%d).", reason);
}

// Wot it says.
static void ble_task(void *param)
{
    ESP_LOGI(TAG, "BLE host task started.");

    // This function will return only when nimble_port_stop() is executed
    nimble_port_run();

    nimble_port_freertos_deinit();
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: COMMAND HANDLING
 * -------------------------------------------------------------- */

// Callback that should be run as a task to handle BLE commands.
static void command_cb(void *handle, void *arg)
{
    (void) handle;

    context_t * context = (context_t *) arg;
    command_t command;

    if (xQueueReceive(context->command_queue, &command, pdMS_TO_TICKS(100))) {
        const char *characteristic_name = "UNKNOWN";
        if (command.characteristic - CHARACTERISTIC_LAUNCH_NOW_UUID < FGR_UTIL_ARRAY_LENGTH(g_characteristic_to_name)) {
            characteristic_name = g_characteristic_to_name[command.characteristic - CHARACTERISTIC_LAUNCH_NOW_UUID];
        }
        switch (command.characteristic) {
            case CHARACTERISTIC_LAUNCH_NOW_UUID:
            case CHARACTERISTIC_UP_UUID:
            case CHARACTERISTIC_DOWN_UUID:
            case CHARACTERISTIC_THIS_IS_GROUND_LEVEL_UUID:
            case CHARACTERISTIC_THIS_IS_HEIGHT_MAX_UUID:
            case CHARACTERISTIC_RESET_TO_DEFAULTS_UUID:
                ESP_LOGI(TAG, "BLE command (0x%04x).", command.characteristic);
                // TODO
            break;
            case CHARACTERISTIC_AUTO_ENABLE_UUID:
            case CHARACTERISTIC_RANDOM_ENABLE_UUID:
                if (command.read_not_write) {
                    ESP_LOGI(TAG, "BLE command %s (0x%04x), read: %s.",
                             characteristic_name, command.characteristic,
                             command.value ? "enabled" : "disabled");
                } else {
                    ESP_LOGI(TAG, "BLE command %s (0x%04x), write: %s.",
                             characteristic_name, command.characteristic,
                             command.value ? "enable" : "disable");
                }
                // TODO
            break;
            case CHARACTERISTIC_AUTO_PERIOD_SECONDS_UUID:
            case CHARACTERISTIC_HEIGHT_MAX_MM_UUID:
            case CHARACTERISTIC_SPEED_MM_PER_SECOND_UUID:
                if (command.read_not_write) {
                    ESP_LOGI(TAG, "BLE command %s (0x%04x), read: %d.",
                             characteristic_name, command.characteristic,
                             command.value);
                } else {
                    ESP_LOGI(TAG, "BLE command %s (0x%04x), write: %d.",
                             characteristic_name, command.characteristic,
                             command.value);
                }
                // TODO
            break;
            default:
                ESP_LOGE(TAG, "Unknown BLE characteristic 0x%04x.", command.characteristic);
            break;
            }
    }
}

// Send a command to the BLE command queue.
static int queue_command(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg,
                         command_t *command)
{
    context_t *context = (context_t *) arg;
    int return_code = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            // Queue the command (non-blocking)
            xQueueSendFromISR(context->command_queue, command, NULL);

            // Send empty success response
            os_mbuf_free_chain(ctxt->om);
            ctxt->om = ble_hs_mbuf_from_flat("", 0);
            return_code = 0;
        }
        break;
        case BLE_GATT_ACCESS_OP_READ_CHR:
        {
            // Queue the command (non-blocking), purely for information
            xQueueSendFromISR(context->command_queue, command, NULL);

            // Send the reading
            os_mbuf_append(ctxt->om, &command->value, sizeof(command->value));
            return_code = 0;
        }
        break;
        default:
        {
            const char *characteristic_name = "UNKNOWN";
            if (command->characteristic - CHARACTERISTIC_LAUNCH_NOW_UUID < FGR_UTIL_ARRAY_LENGTH(g_characteristic_to_name)) {
                characteristic_name = g_characteristic_to_name[command->characteristic - CHARACTERISTIC_LAUNCH_NOW_UUID];
            }
            ESP_LOGE(TAG, "BLE command %s (0x%04x) received unknown operation: %d (0x%02x).",
                     characteristic_name, command->characteristic, ctxt->op, ctxt->op);
        }
        break;
    }

    return return_code;
}

// "Launch now" callback.
static int characteristic_launch_now_cb(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_LAUNCH_NOW_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "Up" callback.
static int characteristic_up_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_UP_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "Down" callback.
static int characteristic_down_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_DOWN_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "This is ground level" callback.
static int characteristic_this_is_ground_level_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_THIS_IS_GROUND_LEVEL_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "This is height max" callback.
static int characteristic_this_is_height_max_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_THIS_IS_HEIGHT_MAX_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "Reset to defaults" callback.
static int characteristic_reset_to_defaults_cb(uint16_t conn_handle, uint16_t attr_handle,
                                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_t command = {.characteristic = CHARACTERISTIC_RESET_TO_DEFAULTS_UUID};
        return_code = queue_command(conn_handle, attr_handle, ctxt, arg, &command);
    }

    return return_code;
}

// "Auto enable" callback.
static int characteristic_auto_enable_cb(uint16_t conn_handle, uint16_t attr_handle,
                                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    command_t command = {.characteristic = CHARACTERISTIC_AUTO_ENABLE_UUID,
                         .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command.read_not_write) {
        // Read the current setting
        bool reading = false;
        nvs_auto_enable_get(&reading);
        command.value = (uint32_t) reading;
    } else {
        // Read the value sent with the BLE command
        command.value = mbuf_read(ctxt->om);
    }
    return queue_command(conn_handle, attr_handle, ctxt, arg, &command);
}

// "Auto period" callback.
static int characteristic_auto_period_seconds_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    command_t command = {.characteristic = CHARACTERISTIC_AUTO_PERIOD_SECONDS_UUID,
                         .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command.read_not_write) {
        // Read the current setting
        nvs_auto_period_seconds_get(&command.value);
    } else {
        // Read the value sent with the BLE command
        command.value = mbuf_read(ctxt->om);
    }
    return queue_command(conn_handle, attr_handle, ctxt, arg, &command);
}

// "Height max" callback.
static int characteristic_height_max_mm_cb(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    command_t command = {.characteristic = CHARACTERISTIC_HEIGHT_MAX_MM_UUID,
                         .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command.read_not_write) {
        // Read the current setting
        nvs_height_max_mm_get(&command.value);
    } else {
        // Read the value sent with the BLE command
        command.value = mbuf_read(ctxt->om);
    }
    return queue_command(conn_handle, attr_handle, ctxt, arg, &command);
}

// "Speed" callback.
static int characteristic_speed_mm_per_second_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    command_t command = {.characteristic = CHARACTERISTIC_SPEED_MM_PER_SECOND_UUID,
                         .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command.read_not_write) {
        // Read the current setting
        nvs_speed_mm_per_second_get(&command.value);
    } else {
        // Read the value sent with the BLE command
        command.value = mbuf_read(ctxt->om);
    }
    return queue_command(conn_handle, attr_handle, ctxt, arg, &command);
}

// "Random enable" callback.
static int characteristic_random_enable_cb(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    command_t command = {.characteristic = CHARACTERISTIC_RANDOM_ENABLE_UUID,
                         .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command.read_not_write) {
        // Read the current setting
        bool reading = false;
        nvs_random_enable_get(&reading);
        command.value = (uint32_t) reading;
    } else {
        // Read the value sent with the BLE command
        command.value = mbuf_read(ctxt->om);
    }
    return queue_command(conn_handle, attr_handle, ctxt, arg, &command);
}

/* ----------------------------------------------------------------
 * VARIABLES FOR BLE
 * -------------------------------------------------------------- */

// BLE service setup.
static const struct ble_gatt_svc_def g_ble_spider_launcher_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                // "Launch now" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_LAUNCH_NOW_UUID),
                .access_cb = characteristic_launch_now_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "Up" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_UP_UUID),
                .access_cb = characteristic_up_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "Down" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_DOWN_UUID),
                .access_cb = characteristic_down_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "This is ground level" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_THIS_IS_GROUND_LEVEL_UUID),
                .access_cb = characteristic_this_is_ground_level_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "This is height max" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_THIS_IS_HEIGHT_MAX_UUID),
                .access_cb = characteristic_this_is_height_max_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "Reset to defaults" characteristic (WRITE only)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_RESET_TO_DEFAULTS_UUID),
                .access_cb = characteristic_reset_to_defaults_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE
            },
            {
                // "Auto enable" characteristic (READ/WRITE)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_AUTO_ENABLE_UUID),
                .access_cb = characteristic_auto_enable_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ
            },
            {
                // "Auto period" characteristic (READ/WRITE)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_AUTO_PERIOD_SECONDS_UUID),
                .access_cb = characteristic_auto_period_seconds_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ
            },
            {
                // "Height max" characteristic (READ/WRITE)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_HEIGHT_MAX_MM_UUID),
                .access_cb = characteristic_height_max_mm_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ
            },
            {
                // "Speed" characteristic (READ/WRITE)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_SPEED_MM_PER_SECOND_UUID),
                .access_cb = characteristic_speed_mm_per_second_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ
            },
            {
                // "Random enable" characteristic (READ/WRITE)
                .uuid = BLE_UUID16_DECLARE(CHARACTERISTIC_RANDOM_ENABLE_UUID),
                .access_cb = characteristic_random_enable_cb,
                .arg = &g_context,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ
            },
            {0}  // Terminator
        },
    },
    {0}  // Service terminator
};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: INITIALISATION
 * -------------------------------------------------------------- */

// Initialisation.
static esp_err_t init(context_t *context)
{
    // Allow us to feed the watchdog
    esp_task_wdt_add(NULL);

    // Initialise tasking, FGR style
    esp_err_t  err = fgr_task_init();

    // Configure WS2812 LED operations needed for debug, FGR style
    if (err == ESP_OK) {
        err = fgr_ws2812_init();
    }

    // Initialize NVS, FGR style
    if (err == ESP_OK) {
        err = fgr_nvs_init();
    }

    // Configure the debug LED, FGR style
    if (err == ESP_OK) {
        err = fgr_debug_init(NULL, NULL);
    }

    // Make sure that NVS is populated (in case it is blank)
    if (err == ESP_OK) {
        nvs_populate();
    }

    if (err == ESP_OK) {
        err = -ESP_ERR_NO_MEM;
        // RTOS stuff needed for command handling
        context->command_queue = xQueueCreate(10, sizeof(command_t));
        if (context->command_queue) {
            err = fgr_task_create(&command_cb, context, "command", 4096, 3, &context->command_task);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Unable to create command task (%s).", esp_err_to_name(-err));
            }
        } else {
            ESP_LOGE(TAG, "Unable to create command queue.");
        }
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BLE starting.");
        // Initialize NimBLE (ESP32-S3's BLE stack)
        err = nimble_port_init();
        if (err == 0) {
            // Set advertising TX power to maximum (+9 dBm)
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
            // Also set connection and scan power
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
            esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
        } else {
            ESP_LOGE(TAG, "BLE failed to initialise NimBLE: %d!", err);
        }
    }

    if (err == 0) {
        // Initialise the NimBLE host configuration, noting that
        // ble_hs_cfg is a magic local variable exported by host/ble_hs.h
        ble_hs_cfg.sync_cb = ble_on_sync_callback;
        ble_hs_cfg.reset_cb = ble_on_reset_callback;

        // Disable bonding since we don't need it
        ble_hs_cfg.sm_bonding = 0;  // Important: Set to 0
        ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // No UI
        ble_hs_cfg.sm_mitm = 0;
        ble_hs_cfg.sm_sc = 0;

        // Distribute BOTH Encryption and Identity keys
        ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
        ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

        // Initialise the mandatory Generic Access Profile service (0x1800)
        ble_svc_gap_init();
        // Initialise the mandatory Generic ATTribute service (0x1801)
        ble_svc_gatt_init();
        // [optional] add our device name
        err = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
        if (err != 0) {
            ESP_LOGE(TAG, "BLE failed to initialise device name (\"%s\"): %d!",
                     BLE_DEVICE_NAME, err);
        }
    }

    if (err == 0) {
        // Set up _our_ Generic ATTribute service
        err = ble_gatts_count_cfg(g_ble_spider_launcher_svcs);
        if (err == 0) {
            err = ble_gatts_add_svcs(g_ble_spider_launcher_svcs);
            if (err != 0) {
                ESP_LOGE(TAG, "BLE failed to add our GATT service: %d!", err);
            }
        } else {
            ESP_LOGE(TAG, "BLE failed to configure GATT count: %d!", err);
        }
    }

    if (err == 0) {
        // Start the BLE task
        nimble_port_freertos_init(ble_task);
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Entry point
void app_main(void)
{
    context_t *context = &g_context;

    ESP_LOGI(TAG, "Spider launcher app_main start");

    esp_err_t err = init(context);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Initialization complete.");
        ESP_LOGI(TAG, "Waiting for BLE connections/commands.");
        while (context->running) {
            // Let BLE commands do their thing
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
        }
        esp_task_wdt_delete(NULL);
    } else {
        ESP_LOGE(TAG, "Initialization failed, system cannot continue, will restart soonish.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    nimble_port_stop();
    fgr_debug_deinit();
    fgr_ws2812_deinit();
    fgr_task_deinit();
    if (context->command_queue){
        vQueueDelete(context->command_queue);
    }
    esp_restart();
}

// End of file
