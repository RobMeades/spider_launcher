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
 * for the debug LED, task handling, non-volatile storage, retained
 * RAM handling and a few general utilities.
 *
 * Note: part of this setup controls a DY-HV8F audio board which has
 * a set of DIP switches on it that aren't properly documented.  By
 * experiment:
 *
 *     DIP SWITCH
 *    1   2   3
 * 0   L   L   L     2. IO Combination Trigger Mode [i.e. IO pins are a binary pattern]
 * 1   H   L   L     x. As (2) but stops playing immediately when IO line is released
 * 2   L   H   L     1. Independent IO Trigger Mode [i.e. IO pins each trigger an individual .mp3 file]
 * 3   H   H   L        As x again
 * 4   L   L   H     y. Nothing plays when any IO pin is shorted to ground
 * 5   H   L   H     6. Power-on Auto-play Mode: Automatically play specified [all, one after the other, in numerical order] audio after power on
 * 6   L   H   H        As y again
 * 7   H   H   H        As y again
 *
 * One of the "y" modes should be a UART running at 9600 (input only
 * on IO line 0) with a command format given in the data sheet but,
 * try as I might, I could never, ever, get this to work, hence this
 * code also supports the IO combination mode (see the KConfig file
 * for details).
 */

#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "errno.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_random.h"
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
#include "fgr_rram.h"
#include "motor_brushless.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

 // Logging prefix.
 #define TAG "spider_launcher"

// UART buffer.
#define UART_RX_BUFFFER_SIZE 256

// An unused track number (if it gets played nothing will happen).
#define TRACK_NUMBER_UNUSED 0

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

// The name for the "track num skittering" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_SKITTERING "track_num_skit"

// The name for the "track num jumping" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_JUMPING "track_num_jump"

// The name for the "track num resetting" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_RESETTING "track_num_reset"

// The name for the "mute" entry in non-volatile storage.
#define NVS_NAME_MUTE "mute"

// The name for the "random enable" entry in non-volatile storage.
#define NVS_NAME_RANDOM_ENABLE "random_enable"

// Default value for "auto enable".
#define DEFAULT_AUTO_ENABLE false

// Default value for "auto period".
#define DEFAULT_AUTO_PERIOD_SECONDS 60

// Default value for "height max".
#define DEFAULT_HEIGHT_MAX_MM 4000

// Default value for "speed".
#define DEFAULT_SPEED_MM_PER_SECOND 4000

// Default value for "track num skittering".
#define DEFAULT_TRACK_NUMBER_SKITTERING 1

// Default value for "track num jumping".
#define DEFAULT_TRACK_NUMBER_JUMPING 2

// Default value for "track num resetting".
#define DEFAULT_TRACK_NUMBER_RESETTING TRACK_NUMBER_UNUSED

// Default value for "random enable".
#define DEFAULT_RANDOM_ENABLE true

// Default value for "mute".
#define DEFAULT_MUTE false

// An out of range value to represent an unknown step count
#define STEP_UNKNOWN 0x7FFFFFFF

// How many mm of height is represented by one step (i.e. the distance
// between two QRD1114 sensor pulses).
#define QRD1114_STEP_LENGTH_MM  40

// The speed in mm per second of winch movement that matches 100% PWM.
#define SPEED_MAX_MM_PER_SECOND 8000

// The minimum usable speed in mm per second.
#define SPEED_MIN_MM_PER_SECOND 2500

#if DEFAULT_SPEED_MM_PER_SECOND < SPEED_MIN_MM_PER_SECOND
#  error "DEFAULT_SPEED_MM_PER_SECOND must be at least SPEED_MIN_MM_PER_SECOND"
#endif

// How long to dangle for at the top of the run.
#define DANGLE_TIME_SECONDS 8

// Percentage speed reduction when resetting.
#define RESETTING_SPEED_MODIFIER_PERCENT 50

// How long to pause for when doing random jerky movements, in milliseconds.
#define RANDOM_PAUSE_TIME_MS 100

// How long to leave between jerky movements in milliseconds.
#define RANDOM_PAUSE_INTERVAL_MS 250

// How long to keep the IO lines at the signalled level when running
// the audio board in IO mode.
#define AUDIO_IO_WAIT_TIME_MS 100

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The types of command
typedef enum {
    COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
    COMMAND_TYPE_WRITE_ONLY_UINT32,
    COMMAND_TYPE_READ_WRITE_UINT32,
    COMMAND_TYPE_READ_WRITE_BOOLEAN,
    COMMAND_TYPE_READ_ONLY
} command_type_t;

// Commands to the launcher.
// If you modify this enum, you will need to modify
// g_command_data_list[] to match.
typedef enum {
    COMMAND_LAUNCH_NOW,              // write only, has no value
    COMMAND_UP,                      // write only, has no value
    COMMAND_DOWN,                    // write only, has no value
    COMMAND_THIS_IS_GROUND_LEVEL,    // write only, has no value
    COMMAND_THIS_IS_HEIGHT_MAX,      // write only, has no value
    COMMAND_RESET_TO_DEFAULTS,       // write only, has no value
    COMMAND_EMERGENCY_STOP,          // write only, has no value
    COMMAND_PLAY_TRACK,              // write only, uint32_t
    COMMAND_AUTO_ENABLE,             // read/write, Boolean
    COMMAND_AUTO_PERIOD_SECONDS,     // read/write, uint32_t
    COMMAND_HEIGHT_MAX_MM,           // read/write, uint32_t
    COMMAND_SPEED_MM_PER_SECOND,     // read/write, uint32_t
    COMMAND_TRACK_NUMBER_SKITTERING, // read/write, uint32_t
    COMMAND_TRACK_NUMBER_JUMPING,    // read/write, uint32_t
    COMMAND_TRACK_NUMBER_RESETTING,  // read/write, uint32_t
    COMMAND_RANDOM_ENABLE,           // read/write, Boolean
    COMMAND_MUTE,                    // read/write, Boolean
    COMMAND_STATE,                   // read only, uint32_t
    COMMAND_HEIGHT_CURRENT_MM,       // read only, uint32_t
    COMMAND_NUMBER_OF
} command_t;

// The states the launcher can be in.
// If you modify this enum, you will need to modify
// g_launcher_state_name[] to match.
typedef enum {
    LAUNCHER_STATE_NULL = 0,
    LAUNCHER_STATE_HEIGHT_UNKNOWN,
    LAUNCHER_STATE_READY,
    LAUNCHER_STATE_STEP_UP,
    LAUNCHER_STATE_STEP_DOWN,
    LAUNCHER_STATE_RUN,
    LAUNCHER_STATE_RUNNING_SKITTERING,
    LAUNCHER_STATE_RUNNING_JUMPING,
    LAUNCHER_STATE_RUNNING_DANGLING,
    LAUNCHER_STATE_RUNNING_RESETTING,
    LAUNCHER_STATE_HALT,
    LAUNCHER_STATE_NUMBER_OF
} launcher_state_t;

// The launcher context
typedef struct {
    QueueHandle_t queue;
    TaskHandle_t task;
    launcher_state_t state;
    int32_t step_target;
    bool direction_up_not_down;
    int64_t timer_a_start_us;
    int64_t timer_b_start_us;
    bool random_enable;
    bool temporary_stop;
    void *motor;
} launcher_t;

// The top-level context.
typedef struct {
    SemaphoreHandle_t lock;
    QueueHandle_t command_queue;
    TaskHandle_t command_task;
    ble_uuid16_t service_uuid;
    struct ble_hs_adv_fields ble_adv_fields;
    uint16_t ble_connection_handle;
    launcher_t launcher;
} context_t;

// Command queue contents.
typedef struct {
    command_t command;
    uint32_t value;
    bool read_not_write;
} command_contents_t;

// Retained RAM storage.
typedef struct {
    int32_t step_ground_level;
    int32_t step_current;
} retained_ram_t;

// Function prototype for a characteristic callback (which BLE calls).
typedef int (*characteristic_cb_t) (uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);

// Function prototype for an command handler (which ultimate does the work).
typedef void (*command_handler_t) (launcher_t *launcher, uint32_t value);

// Properties of a command we advertise as a BLE characteristic;
// this is the basis of g_command_data_list[], which is populated way
// down below.
typedef struct {
    command_type_t type;
    uint16_t uuid;
    char *name;
    characteristic_cb_t characteristic_cb;
    ble_gatt_chr_flags flags;
    command_handler_t handler;
} command_data_t;

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

// Context for the whole application (fields not mentioned will be zeroed).
static context_t g_context = {
    .service_uuid = BLE_UUID16_INIT(SERVICE_UUID),
    .ble_connection_handle = BLE_HS_CONN_HANDLE_NONE,
    .launcher.step_target = STEP_UNKNOWN};

// Retained RAM storage
FGR_RRAM_DEFINE(retained_ram_t, retained_ram);

// The names of the launcher states for debug prints;
// entries are in the same order as launcher_state_t
// and there must be the same number of entries
static const char *g_launcher_state_name[] = {"NULL",
                                              "HEIGHT_UNKNOWN",
                                              "READY",
                                              "STEP_UP",
                                              "STEP_DOWN",
                                              "RUN",
                                              "RUNNING_SKITTERING",
                                              "RUNNING_JUMPING",
                                              "RUNNING_DANGLING",
                                              "RUNNING_RESETTING",
                                              "HALT"};

// Do some checking
_Static_assert (FGR_UTIL_ARRAY_LENGTH(g_launcher_state_name) == LAUNCHER_STATE_NUMBER_OF,
                "the number of g_launcher_state_name[] entries does not match the number of launcher states!");

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

// Retrieve the skittering track number from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_track_number_skittering_get(uint32_t *track_number)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (track_number) {
        err = fgr_nvs_get(NVS_NAME_TRACK_NUMBER_SKITTERING, track_number);
        if (err != ESP_OK) {
            *track_number = DEFAULT_TRACK_NUMBER_SKITTERING;
        }
    }

    return err;
}

// Set the track num skittering in NVS.
static int32_t nvs_track_number_skittering_set(uint32_t track_number)
{
    return fgr_nvs_set(NVS_NAME_TRACK_NUMBER_SKITTERING, track_number);
}

// Retrieve the jumping track number from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_track_number_jumping_get(uint32_t *track_number)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (track_number) {
        err = fgr_nvs_get(NVS_NAME_TRACK_NUMBER_JUMPING, track_number);
        if (err != ESP_OK) {
            *track_number = DEFAULT_TRACK_NUMBER_JUMPING;
        }
    }

    return err;
}

// Set the track num jumping in NVS.
static int32_t nvs_track_number_jumping_set(uint32_t track_number)
{
    return fgr_nvs_set(NVS_NAME_TRACK_NUMBER_JUMPING, track_number);
}

// Retrieve the jumping track resetting from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_track_number_resetting_get(uint32_t *track_number)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (track_number) {
        err = fgr_nvs_get(NVS_NAME_TRACK_NUMBER_RESETTING, track_number);
        if (err != ESP_OK) {
            *track_number = DEFAULT_TRACK_NUMBER_RESETTING;
        }
    }

    return err;
}

// Set the track num resetting in NVS.
static int32_t nvs_track_number_resetting_set(uint32_t track_number)
{
    return fgr_nvs_set(NVS_NAME_TRACK_NUMBER_RESETTING, track_number);
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

// Retrieve whether audio muting is enabled or not from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_mute_get(bool *enable)
{
    int32_t err = -ESP_ERR_INVALID_ARG;
    uint32_t value = 0;

    if (enable) {
        err = fgr_nvs_get(NVS_NAME_MUTE, &value);
        if (err == ESP_OK) {
            *enable = (value != 0);
        } else {
            *enable = DEFAULT_MUTE;
        }
    }

    return err;
}

// Set whether audio muting is enabled or not in NVS.
static int32_t nvs_mute_set(bool enable)
{
    return fgr_nvs_set(NVS_NAME_MUTE, enable);
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
    if (nvs_track_number_skittering_get(&value) != ESP_OK) {
        nvs_track_number_skittering_set(value);
    }
    if (nvs_track_number_jumping_get(&value) != ESP_OK) {
        nvs_track_number_jumping_set(value);
    }
    if (nvs_track_number_resetting_get(&value) != ESP_OK) {
        nvs_track_number_resetting_set(value);
    }
    if (nvs_random_enable_get(&enable) != ESP_OK) {
        nvs_random_enable_set(enable);
    }
    if (nvs_mute_get(&enable) != ESP_OK) {
        nvs_mute_set(enable);
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: DY-HV8F AUDIO BOARD RELATED
 * -------------------------------------------------------------- */

#if !defined(CONFIG_SPIDER_LAUNCHER_UART_NUM) || (CONFIG_SPIDER_LAUNCHER_UART_NUM < 0)
static uint8_t audio_play_set_pin(uint32_t track_number, int32_t bit_mask, int32_t pin, bool highNotLow)
{
    uint8_t resulting_bit_pattern = track_number & bit_mask;

    if (!highNotLow) {
        resulting_bit_pattern = ~resulting_bit_pattern;
    }
    gpio_set_level(pin, track_number & bit_mask ? highNotLow : !highNotLow);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);

    return resulting_bit_pattern;
}
#endif

// Play the given audio track.
static void audio_play_track(uint32_t track_number)
{
#if defined(CONFIG_SPIDER_LAUNCHER_UART_NUM) && (CONFIG_SPIDER_LAUNCHER_UART_NUM >= 0)
    // Command format is:
    //
    // 0xAA 0x04 0x03 high_byte low_byte checksum
    //
    // ...where checksum = 0xAA + data length + command byte + parameter byte
    //
    // For example, to play track 1:
    //
    // 0xAA 0x04 0x03 0x00 0x01 checksum

    uint8_t buffer[] = {0xaa, 0x04, 0x03,  (track_number >> 8) & 0xFF, track_number & 0xFF, 0};
    //uint8_t buffer[] = {0xaa, 0x07, 0x02,  (track_number >> 8) & 0xFF, track_number & 0xFF, 0};
    //uint8_t buffer[] = {0xaa, 0x02, 0x00, 0};

    uint8_t checksum = 0;
    for (size_t x = 0; x < FGR_UTIL_ARRAY_LENGTH(buffer) - 1; x++) {
        checksum += buffer[x];
    }
    buffer[FGR_UTIL_ARRAY_LENGTH(buffer) - 1] = checksum;

    int32_t x = uart_write_bytes(CONFIG_SPIDER_LAUNCHER_UART_NUM, buffer, sizeof(buffer));
    ESP_LOGI("DEBUG", "To play audio track %d sent %d byte(s) {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x} on UART %d, TXD on pin %d.",
             track_number, x, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], CONFIG_SPIDER_LAUNCHER_UART_NUM,
             CONFIG_SPIDER_LAUNCHER_AUDIO_UART_TXD_PIN);

    // Wait for transmission to complete
    uart_wait_tx_done(CONFIG_SPIDER_LAUNCHER_UART_NUM, pdMS_TO_TICKS(100));
#else
    // In GPIO control mode: each GPIO line is one bit in an up-to-four-bit
    // binary word which is set low to indicate a 1, high to indicate a 0.
    // For this to work all three DIP switches on a DY-HV8F board should be
    // in the low position
    uint8_t bit_pattern = 0xFF;
    size_t bit_count = 0;
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x01, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0, false);
    bit_count++;
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x02, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1, false);
    bit_count++;
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x04, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, false);
    bit_count++;
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x08, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, false);
    bit_count++;
#  endif

    ESP_LOGI(TAG, "To play audio track %d, setting IO lines (%d line(s)) to 0x%1x for %d ms.",
             track_number, bit_count, bit_pattern, AUDIO_IO_WAIT_TIME_MS);

    vTaskDelay(pdMS_TO_TICKS(AUDIO_IO_WAIT_TIME_MS));

#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0 >= 0)
    audio_play_set_pin(0xF, 0x01, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1 >= 0)
    audio_play_set_pin(0xF, 0x02, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2 >= 0)
    audio_play_set_pin(0xF, 0x04, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3 >= 0)
    audio_play_set_pin(0xF, 0x08, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, true);
#  endif
#endif
}

/* ----------------------------------------------------------------
 * STATIC FUNCTION PROTOTYPE: NEEDED FOR BLE CALLBACKS
 * -------------------------------------------------------------- */

static int ble_gap_event_callback(struct ble_gap_event *event, void *arg);

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: BLE
 * -------------------------------------------------------------- */

// Read up to four bytes from an mbuf, returning an
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
    int32_t err;
    ESP_LOGI(TAG, "BLE (re)starting advertising.");

    CONTEXT_LOCK(context->lock, "ble_start_advertising");

    // Zero out the advertising fields structure before populating
    memset(&context->ble_adv_fields, 0, sizeof(context->ble_adv_fields));

    // Set flags
    context->ble_adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set service UUID
    context->ble_adv_fields.uuids16 = &context->service_uuid;
    context->ble_adv_fields.num_uuids16 = 1;
    context->ble_adv_fields.uuids16_is_complete = 1;

    // Optional: Set device name
    context->ble_adv_fields.name = (uint8_t *)BLE_DEVICE_NAME;
    context->ble_adv_fields.name_len = strlen(BLE_DEVICE_NAME);
    context->ble_adv_fields.name_is_complete = 1;

    err = ble_gap_adv_set_fields(&context->ble_adv_fields);
    if (err == 0) {
        err = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                                &g_ble_adv_params, ble_gap_event_callback,
                                context);
        if (err != 0) {
            if (err == BLE_HS_EALREADY) {
                ESP_LOGW(TAG, "advertising already in progress.");
            } else {
                ESP_LOGE(TAG, "failed to start advertising: %d!", err);
            }
        }
    } else {
        ESP_LOGE(TAG, "BLE failed to set advertisement fields: %d!", err);
    }

    CONTEXT_UNLOCK(context->lock, "ble_start_advertising");

    return err;
}

// GAP event callback, handles connections.
static int ble_gap_event_callback(struct ble_gap_event *event, void *arg)
{
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "ble_gap_event_callback");

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

    CONTEXT_UNLOCK(context->lock, "ble_gap_event_callback");

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
        ESP_LOGE(TAG, "BLE failed to ensure public address: %d!", err);
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
 * STATIC FUNCTIONS: LAUNCHER (ACTUALLY DOING STUFF)
 * -------------------------------------------------------------- */

// Determine whether the launcher is in a state where the height
// related parameters can be reset.
static bool can_reset_height_parameters(launcher_state_t launcher_state)
{
    bool can_reset = false;

    if ((launcher_state == LAUNCHER_STATE_NULL) ||
        (launcher_state == LAUNCHER_STATE_HEIGHT_UNKNOWN) ||
        (launcher_state == LAUNCHER_STATE_READY)) {
        can_reset = true;
    }

    return can_reset;
}

// Print out the current step and height
static void print_step(int32_t step_current, int32_t step_ground_level)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), " (%d mm)", (int) (step_current - step_ground_level) * QRD1114_STEP_LENGTH_MM);
    ESP_LOGI(TAG, "step %d%s.", step_current,
                step_ground_level != STEP_UNKNOWN ? buffer : "");
}

// Start the motor speed in the given direction.
static void motor_start(void *motor, bool direction_up_not_down,
                        uint32_t speed_modifier_percent)
{
    uint32_t abs_speed_mm_per_second;
    int32_t err = nvs_speed_mm_per_second_get(&abs_speed_mm_per_second);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "unable to read speed from NVS (%s), using default (%d mm/s).",
                  esp_err_to_name(-err), abs_speed_mm_per_second);
    }

    int32_t speed_mm_per_second = (int32_t) abs_speed_mm_per_second * speed_modifier_percent / 100;
    if (!direction_up_not_down) {
        speed_mm_per_second = -speed_mm_per_second;
    }

    int32_t pwm = 100 * speed_mm_per_second / SPEED_MAX_MM_PER_SECOND;
    err = motor_brushless_set_speed(motor, pwm);
    if (err == ESP_OK) {
         ESP_LOGI(TAG, "speed set to %d mm/s (pwm %d%%).", speed_mm_per_second, pwm);
    } else {
         ESP_LOGE(TAG, "unable to set speed to %d mm/s (pwm %d%%) (%s).",
                  speed_mm_per_second, pwm, esp_err_to_name(-err));
    }
}

// Stop the motor.
static void motor_stop(void *motor)
{
    int32_t err = motor_brushless_set_speed(motor, 0);
    if (err == ESP_OK) {
         ESP_LOGI(TAG, "motor stopped.");
    } else {
         ESP_LOGE(TAG, "unable to stop motor.");
    }
}

// Advance state based on commanded state LAUNCHER_STATE_STEP_UP.
static launcher_state_t advance_state_commanded_step_up(launcher_t *launcher,
                                                        int32_t step_current,
                                                        int32_t step_max,
                                                        int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if (step_current == STEP_UNKNOWN) {
        // Got to have some sort of reference
        step_current = 0;
    }
    if ((step_max == STEP_UNKNOWN) || (step_current + 1 < step_max)) {
        launcher->step_target = step_current + 1;
        launcher->direction_up_not_down = true;
        ESP_LOGI(TAG, "step_current %d, step_target %d.", step_current, launcher->step_target);
        motor_start(launcher->motor, launcher->direction_up_not_down, 100);
        next_state = LAUNCHER_STATE_STEP_UP;
    } else {
        ESP_LOGW(TAG, "ignoring step up, already at limit (step %d (%d mm)).",
                 step_current, (step_ground_level - step_current) * QRD1114_STEP_LENGTH_MM);
    }

    return next_state;
}

// Advance state based on commanded state LAUNCHER_STATE_STEP_DOWN.
static launcher_state_t advance_state_commanded_step_down(launcher_t *launcher,
                                                          int32_t step_current,
                                                          int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if (step_current == STEP_UNKNOWN) {
        // Got to have some sort of reference
        step_current = 0;
    }
    if ((step_ground_level == STEP_UNKNOWN) || (step_current > step_ground_level)) {
        launcher->step_target = step_current - 1;
        launcher->direction_up_not_down = false;
        ESP_LOGI(TAG, "step_current %d, step_target %d.", step_current, launcher->step_target);
        motor_start(launcher->motor, launcher->direction_up_not_down, 100);
        next_state = LAUNCHER_STATE_STEP_DOWN;
    } else {
        ESP_LOGW(TAG, "ignoring step down, at ground level already (%d).", step_ground_level);
    }

    return next_state;
}

// Advance state based on commanded state LAUNCHER_STATE_RUN.
static launcher_state_t advance_state_commanded_run(launcher_t *launcher,
                                                    int32_t step_current,
                                                    int32_t step_max,
                                                    int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if ((step_max == STEP_UNKNOWN) || (step_current < step_max)) {
        // TODO: set up the skittering
        next_state = LAUNCHER_STATE_RUNNING_SKITTERING;
    } else {
        ESP_LOGW(TAG, "ignoring run, already at limit (step %d (%d mm)).",
                 step_current, (step_ground_level - step_current) * QRD1114_STEP_LENGTH_MM);
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_SKITTERING.
static launcher_state_t advance_state_skittering(launcher_t *launcher,
                                                 int32_t step_current,
                                                 int32_t step_max,
                                                 int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if ((step_max == STEP_UNKNOWN) || (step_current < step_max)) {
        // TODO: run the skittering
        // Prepare for randomness
        nvs_random_enable_get(&launcher->random_enable);
        launcher->timer_a_start_us = esp_timer_get_time();
        launcher->timer_b_start_us = launcher->timer_a_start_us;
        launcher->temporary_stop = false;
        // For now, go straight to jumping
        launcher->step_target = step_max;
        launcher->direction_up_not_down = true;
        ESP_LOGI(TAG, "step_current %d, step_target %d%s.", step_current, launcher->step_target,
                 launcher->random_enable ? ", (random is enabled)" : "");
        motor_start(launcher->motor, launcher->direction_up_not_down, 100);
        next_state = LAUNCHER_STATE_RUNNING_JUMPING;
    } else {
        ESP_LOGW(TAG, "ignoring step up, already at limit (step %d (%d mm)).",
                step_current, (step_ground_level - step_current) * QRD1114_STEP_LENGTH_MM);
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_JUMPING.
static launcher_state_t advance_state_running_jumping(launcher_t *launcher,
                                                      int32_t step_current,
                                                      int32_t step_max,
                                                      int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if (step_current != STEP_UNKNOWN) {
        if (step_current < launcher->step_target) {
            if (launcher->random_enable) {
                // Stop randomly to make the rise jerky
                if (!launcher->temporary_stop &&
                    (esp_timer_get_time() - launcher->timer_b_start_us > RANDOM_PAUSE_INTERVAL_MS * 1000)) {
                    if (esp_random() < UINT32_MAX / 2) {
                        ESP_LOGI(TAG, "random stop for %d ms.", RANDOM_PAUSE_TIME_MS);
                        launcher->timer_a_start_us = esp_timer_get_time();
                        motor_stop(launcher->motor);
                        launcher->temporary_stop = true;
                    } else {
                        ESP_LOGI(TAG, "won't stop randomly for %d ms.", RANDOM_PAUSE_INTERVAL_MS);
                        launcher->timer_b_start_us = esp_timer_get_time();
                    }
                }
                if (launcher->temporary_stop &&
                    (esp_timer_get_time() - launcher->timer_a_start_us > RANDOM_PAUSE_TIME_MS * 1000)) {
                    ESP_LOGI(TAG, "restarting after random stop, won't stop again for %d ms.", RANDOM_PAUSE_INTERVAL_MS);
                    motor_start(launcher->motor, launcher->direction_up_not_down, 100);
                    launcher->temporary_stop = false;
                    launcher->timer_b_start_us = esp_timer_get_time();
                }
            }
        } else {
            // Done.
            launcher->step_target = STEP_UNKNOWN;
            motor_stop(launcher->motor);
            print_step(step_current, step_ground_level);
            launcher->timer_a_start_us = esp_timer_get_time();
            ESP_LOGI(TAG, "dangling for %d second(s).", DANGLE_TIME_SECONDS);
            next_state = LAUNCHER_STATE_RUNNING_DANGLING;
        }
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_DANGLING.
static launcher_state_t advance_state_running_dangling(launcher_t *launcher,
                                                       int32_t step_current,
                                                       int32_t step_max,
                                                       int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    // Wait for the dangle time to pass
    if (esp_timer_get_time() - launcher->timer_a_start_us > DANGLE_TIME_SECONDS * 1000000) {
        // Done.
        ESP_LOGI(TAG, "resetting.");
        launcher->step_target = step_ground_level;
        launcher->direction_up_not_down = false;
        ESP_LOGI(TAG, "step_current %d, step_target %d.", step_current, launcher->step_target);
        motor_start(launcher->motor, launcher->direction_up_not_down, RESETTING_SPEED_MODIFIER_PERCENT);
        next_state = LAUNCHER_STATE_RUNNING_RESETTING;
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_RESETTING.
static launcher_state_t advance_state_running_resetting(launcher_t *launcher,
                                                        int32_t step_current,
                                                        int32_t step_max,
                                                        int32_t step_ground_level)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if ((step_current != STEP_UNKNOWN) && (step_current <= launcher->step_target)) {
        // Done.
        launcher->step_target = STEP_UNKNOWN;
        motor_stop(launcher->motor);
        print_step(step_current, step_ground_level);
        next_state = LAUNCHER_STATE_HEIGHT_UNKNOWN;
    }

    return next_state;
}

// Advance state as appropriate; only allowed states should be passed
// in via commanded_state.
// This function _returns_ the next state, it does not change the
// current state, it is up to the caller to do that.
// IMPORTANT: the context should be locked before this is called.
static launcher_state_t advance_state(launcher_t *launcher, launcher_state_t commanded_state)
{
    // Use a signed value here so that negative indicates not known
    int32_t step_current = STEP_UNKNOWN;
    int32_t step_ground_level = STEP_UNKNOWN;
    retained_ram_t retained_ram;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        step_current = (int32_t) retained_ram.step_current;
        step_ground_level = (int32_t) retained_ram.step_ground_level;
    }
    int32_t step_max = STEP_UNKNOWN;
    if (step_ground_level != STEP_UNKNOWN) {
        uint32_t height_max_mm = 0;
        nvs_height_max_mm_get(&height_max_mm);
        step_max = step_ground_level + ((int32_t) height_max_mm / QRD1114_STEP_LENGTH_MM);
    }

    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    if (commanded_state == LAUNCHER_STATE_HALT) {
        // A command to halt overrides everything else
        motor_stop(launcher->motor);
        next_state = commanded_state;
    } else {
        // Handle the current state
        switch(launcher->state) {
            case LAUNCHER_STATE_NULL:
                next_state = LAUNCHER_STATE_HEIGHT_UNKNOWN;
            break;
            case LAUNCHER_STATE_HEIGHT_UNKNOWN:
                // If we know where the ground is we can transition
                // to ready state
                if (step_ground_level != STEP_UNKNOWN) {
                    next_state = LAUNCHER_STATE_READY;
                } else {
                    // From height unknown state we can only do steps
                    switch (commanded_state) {
                        case LAUNCHER_STATE_STEP_UP:
                            next_state = advance_state_commanded_step_up(launcher,
                                                                         step_current,
                                                                         STEP_UNKNOWN,
                                                                         step_ground_level);
                        break;
                        case LAUNCHER_STATE_STEP_DOWN:
                            next_state = advance_state_commanded_step_down(launcher,
                                                                           step_current,
                                                                           STEP_UNKNOWN);
                        break;
                        case LAUNCHER_STATE_NULL:
                            // Nothing to do
                        break;
                        default:
                            ESP_LOGW(TAG, "ignoring command to enter state %d from state %d.",
                                     commanded_state, launcher->state);
                        break;
                    }
                }
            break;
            case LAUNCHER_STATE_READY:
                // From ready state we can step and run
                switch (commanded_state) {
                    case LAUNCHER_STATE_STEP_UP:
                        next_state = advance_state_commanded_step_up(launcher,
                                                                     step_current,
                                                                     step_max,
                                                                     step_ground_level);
                    break;
                    case LAUNCHER_STATE_STEP_DOWN:
                        next_state = advance_state_commanded_step_down(launcher,
                                                                       step_current,
                                                                       step_ground_level);
                    break;
                    case LAUNCHER_STATE_RUN:
                        // Run away
                        next_state = commanded_state;
                    break;
                    case LAUNCHER_STATE_NULL:
                        // Nothing to do
                    break;
                    default:
                        ESP_LOGW(TAG, "ignoring command to enter state %d from state %d.",
                                 commanded_state, launcher->state);
                    break;
                }
            break;
            case LAUNCHER_STATE_STEP_UP:
                if ((step_current != STEP_UNKNOWN) && (step_current >= launcher->step_target)) {
                    // Done.
                    launcher->step_target = STEP_UNKNOWN;
                    motor_stop(launcher->motor);
                    print_step(step_current, step_ground_level);
                    next_state = LAUNCHER_STATE_HEIGHT_UNKNOWN;
                }
            break;
            case LAUNCHER_STATE_STEP_DOWN:
                if ((step_current != STEP_UNKNOWN) && (step_current <= launcher->step_target)) {
                    // Done.
                    launcher->step_target = STEP_UNKNOWN;
                    motor_stop(launcher->motor);
                    print_step(step_current, step_ground_level);
                    next_state = LAUNCHER_STATE_HEIGHT_UNKNOWN;
                }
            break;
            case LAUNCHER_STATE_RUN:
                next_state = advance_state_commanded_run(launcher,
                                                         step_current,
                                                         step_max,
                                                         step_ground_level);
            break;
            case LAUNCHER_STATE_RUNNING_SKITTERING:
                next_state = advance_state_skittering(launcher,
                                                      step_current,
                                                      step_max,
                                                      step_ground_level);
            break;
            case LAUNCHER_STATE_RUNNING_JUMPING:
                next_state = advance_state_running_jumping(launcher,
                                                           step_current,
                                                           step_max,
                                                           step_ground_level);
            break;
            case LAUNCHER_STATE_RUNNING_DANGLING:
                next_state = advance_state_running_dangling(launcher,
                                                            step_current,
                                                            step_max,
                                                            step_ground_level);
            break;
            case LAUNCHER_STATE_RUNNING_RESETTING:
                next_state = advance_state_running_resetting(launcher,
                                                            step_current,
                                                            step_max,
                                                            step_ground_level);
            break;
            case LAUNCHER_STATE_HALT:
                next_state = LAUNCHER_STATE_HEIGHT_UNKNOWN;
            break;
            default:
                ESP_LOGE(TAG, "current state is unknown (%d)!", launcher->state);
            break;
        }
    }

    // Return the next state (which will be LAUNCHER_STATE_NULL if there is no change)
    return next_state;
}

// Callback that should be run as a task to operate the launcher.
static void launcher_cb(void *handle, void *arg)
{
    context_t *context = (context_t *) arg;
    launcher_t *launcher = &context->launcher;

    (void) handle;

    CONTEXT_LOCK(context->lock, "launcher_cb");

    // Get a commanded state from the queue
    launcher_state_t commanded_state = LAUNCHER_STATE_NULL;
    while (xQueueReceive(launcher->queue, &commanded_state, 0) == pdTRUE) {

        const char *name = "UNKNOWN";
        if (commanded_state < FGR_UTIL_ARRAY_LENGTH(g_launcher_state_name)) {
            name = g_launcher_state_name[commanded_state];
        }

        // Check if the commanded state is allowed
        switch (commanded_state) {
            case LAUNCHER_STATE_NULL:
            case LAUNCHER_STATE_HEIGHT_UNKNOWN:
            case LAUNCHER_STATE_READY:
            case LAUNCHER_STATE_RUNNING_SKITTERING:
            case LAUNCHER_STATE_RUNNING_JUMPING:
            case LAUNCHER_STATE_RUNNING_RESETTING:
                // These states can only be entered through internal
                // advancement, they cannot be commanded by the user
                ESP_LOGW(TAG, "ignoring command to enter state %s (%d).", name, commanded_state);
                commanded_state = LAUNCHER_STATE_NULL;
            break;
            case LAUNCHER_STATE_STEP_UP:
            case LAUNCHER_STATE_STEP_DOWN:
                // Can do these if we don't know our height or are ready
                if ((launcher->state == LAUNCHER_STATE_READY) ||
                    (launcher->state == LAUNCHER_STATE_HEIGHT_UNKNOWN)) {
                    ESP_LOGI(TAG, "starting %s.", name);
                } else {
                    ESP_LOGW(TAG, "cannot get to state %s from state %s (only from states %s and %s).",
                             name, g_launcher_state_name[launcher->state],
                             g_launcher_state_name[LAUNCHER_STATE_HEIGHT_UNKNOWN],
                             g_launcher_state_name[LAUNCHER_STATE_READY]);
                    commanded_state = LAUNCHER_STATE_NULL;
                }
            break;
            case LAUNCHER_STATE_RUN:
                // Can only do this  if we're ready
                if (launcher->state == LAUNCHER_STATE_READY) {
                    ESP_LOGI(TAG, "starting %s.", name);
                } else {
                    ESP_LOGW(TAG, "cannot get to state %s from state %s (only from state %s).",
                             name, g_launcher_state_name[launcher->state],
                             g_launcher_state_name[LAUNCHER_STATE_READY]);
                    commanded_state = LAUNCHER_STATE_NULL;
                }
            break;
            case LAUNCHER_STATE_HALT:
                 // Always handle this
                ESP_LOGI(TAG, "received %s.", name);
            break;
            default:
                ESP_LOGE(TAG, "commanded to enter unknown state (%d)!", commanded_state);
                commanded_state = LAUNCHER_STATE_NULL;
            break;
        }

        if (commanded_state != LAUNCHER_STATE_NULL) {
            // Move state on based on internal events or the new commanded_state
            launcher_state_t next_state = advance_state(launcher, commanded_state);
            if (next_state != LAUNCHER_STATE_NULL) {
                ESP_LOGI(TAG, "state transition due to commanded state %s, %s -> %s.", name,
                         g_launcher_state_name[launcher->state], g_launcher_state_name[next_state]);
                launcher->state = next_state;
            }
        }
    }

    // Make sure we advance state at least once in case there are any internal changes
    launcher_state_t next_state = advance_state(launcher, LAUNCHER_STATE_NULL);
    if (next_state != LAUNCHER_STATE_NULL) {
        ESP_LOGI(TAG, "internal state transition %s -> %s.",
                 g_launcher_state_name[launcher->state], g_launcher_state_name[next_state]);
        launcher->state = next_state;
    }

    CONTEXT_UNLOCK(context->lock, "launcher_cb");
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: COMMAND HANDLERS
 * -------------------------------------------------------------- */

// Handle the "launch now" command.
static void handler_launch_now(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Queue the command to the launcher to switch to running state
    launcher_state_t state = LAUNCHER_STATE_RUN;
    xQueueSend(launcher->queue, &state, portMAX_DELAY);
}

// Handle the "up" command.
static void handler_up(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Queue the command to the launcher to switch to step-up state
    launcher_state_t state = LAUNCHER_STATE_STEP_UP;
    xQueueSend(launcher->queue, &state, portMAX_DELAY);
}

// Handle the "down" command.
static void handler_down(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Queue the command to the launcher to switch to step-down state
    launcher_state_t state = LAUNCHER_STATE_STEP_DOWN;
    xQueueSend(launcher->queue, &state, portMAX_DELAY);
}

// Handle the "this is ground level" command.
static void handler_this_is_ground_level(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Set the groud level step to the current step
    retained_ram_t retained_ram;
    if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
        (retained_ram.step_current != STEP_UNKNOWN)) {
        retained_ram.step_ground_level = retained_ram.step_current;
        FGR_RRAM_SET(retained_ram);
    }
}

// Handle the "this is height max" command.
static void handler_this_is_height_max(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Get the current height
    retained_ram_t retained_ram;
    if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
        (retained_ram.step_ground_level != STEP_UNKNOWN) &&
        (retained_ram.step_current != STEP_UNKNOWN)) {
        // Set the "height max" value in NVS
        nvs_height_max_mm_set((retained_ram.step_current - retained_ram.step_ground_level) * QRD1114_STEP_LENGTH_MM);
    }
}

// Handle the "reset to defaults" command.
static void handler_reset_to_defaults(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Write the default settings to NVS
    nvs_auto_enable_set(DEFAULT_AUTO_ENABLE);
    nvs_auto_period_seconds_set(DEFAULT_AUTO_PERIOD_SECONDS);
    nvs_height_max_mm_set(DEFAULT_HEIGHT_MAX_MM);
    nvs_speed_mm_per_second_set(DEFAULT_SPEED_MM_PER_SECOND);
    nvs_track_number_skittering_set(DEFAULT_TRACK_NUMBER_SKITTERING);
    nvs_track_number_jumping_set(DEFAULT_TRACK_NUMBER_JUMPING);
    nvs_track_number_resetting_set(DEFAULT_TRACK_NUMBER_RESETTING);
    nvs_random_enable_set(DEFAULT_RANDOM_ENABLE);
    nvs_mute_set(DEFAULT_MUTE);

    // Set the ground level and current step to unknown
    retained_ram_t retained_ram;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        retained_ram.step_current = STEP_UNKNOWN;
        retained_ram.step_ground_level = STEP_UNKNOWN;
        FGR_RRAM_SET(retained_ram);
    }

    // Set the launcher state back to NULL
    launcher->state = LAUNCHER_STATE_NULL;

    // Anything else TODO?
}

// Handle the "emergency stop" command.
static void handler_emergency_stop(launcher_t *launcher, uint32_t unused)
{
    (void) unused;

    // Queue the command to the launcher to halt
    launcher_state_t state = LAUNCHER_STATE_HALT;
    xQueueSend(launcher->queue, &state, portMAX_DELAY);
}

// Handle the "play track" command.
static void handler_play_track(launcher_t *launcher, uint32_t track_number)
{
    // Play the track
    audio_play_track(track_number);
}

// Handle the "auto enable" command.
static void handler_auto_enable(launcher_t *launcher, uint32_t enable)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_auto_enable_set(enable ? true : false);

    // Anything else TODO?
}

// Handle the "auto period" command.
static void handler_auto_period_seconds(launcher_t *launcher, uint32_t period)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_auto_period_seconds_set(period);

    // Anything else TODO?
}

// Handle the "height max" command.
static void handler_height_max_mm(launcher_t *launcher, uint32_t height)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_height_max_mm_set(height);

    // Anything else TODO?
}

// Handle the "speed" command.
static void handler_speed_mm_per_second(launcher_t *launcher, uint32_t speed)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_speed_mm_per_second_set(speed);

    // Anything else TODO?
}

// Handle the "track num skittering" command.
static void handler_track_number_skittering(launcher_t *launcher, uint32_t track_number)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_track_number_skittering_set(track_number);

    // Anything else TODO?
}

// Handle the "track num jumping" command.
static void handler_track_number_jumping(launcher_t *launcher, uint32_t track_number)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_track_number_jumping_set(track_number);

    // Anything else TODO?
}

// Handle the "track num resetting" command.
static void handler_track_number_resetting(launcher_t *launcher, uint32_t track_number)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_track_number_resetting_set(track_number);

    // Anything else TODO?
}

// Handle the "random enable" command.
static void handler_random_enable(launcher_t *launcher, uint32_t enable)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_random_enable_set(enable ? true : false);

    // Anything else TODO?
}

// Handle the "mute" command.
static void handler_mute(launcher_t *launcher, uint32_t enable)
{
    (void) launcher;

    // Write the new setting to NVS
    nvs_mute_set(enable ? true : false);

    // Anything else TODO?
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: CHARACTERISTICS CALLBACKS (CALLED BY BLE)
 * -------------------------------------------------------------- */

// Send a command received over BLE to the command queue.
// IMPORTANT: the context should be locked before this is called.
static int queue_command(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, context_t *context,
                         command_contents_t *command_contents)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;

    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
        {
            // Queue the command (non-blocking)
            xQueueSendFromISR(context->command_queue, command_contents, NULL);

            // Send empty success response
            os_mbuf_free_chain(ctxt->om);
            ctxt->om = ble_hs_mbuf_from_flat("", 0);
            return_code = 0;
        }
        break;
        case BLE_GATT_ACCESS_OP_READ_CHR:
        {
            // Queue the command (non-blocking), purely for information
            xQueueSendFromISR(context->command_queue, command_contents, NULL);

            // Send the value
            os_mbuf_append(ctxt->om, &command_contents->value, sizeof(command_contents->value));
            return_code = 0;
        }
        break;
        default:
        {
            ESP_LOGE(TAG, "received unknown operation: %d (0x%02x)!", ctxt->op, ctxt->op);
        }
        break;
    }

    return return_code;
}

// "Launch now" characteristic callback.
static int characteristic_launch_now_cb(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_launch_now_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_contents_t command_contents = {.command = COMMAND_LAUNCH_NOW};
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_launch_now_cb");

    return return_code;
}

// "Up" characteristic callback.
static int characteristic_up_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_up_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_contents_t command_contents = {.command = COMMAND_UP};
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_up_cb");

    return return_code;
}

// "Down" characteristic callback.
static int characteristic_down_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_down_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_contents_t command_contents = {.command = COMMAND_DOWN};
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_down_cb");

    return return_code;
}

// "This is ground level" characteristic callback.
static int characteristic_this_is_ground_level_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_this_is_ground_level_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Only accept this command if the launcher is in the right state
        if (can_reset_height_parameters(context->launcher.state)) {
            command_contents_t command_contents = {.command = COMMAND_THIS_IS_GROUND_LEVEL};
            return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
        }
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_this_is_ground_level_cb");

    return return_code;
}

// "This is height max" characteristic callback.
static int characteristic_this_is_height_max_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_this_is_height_max_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Only accept this command if the launcher is in the right state
        if (can_reset_height_parameters(context->launcher.state)) {
            command_contents_t command_contents = {.command = COMMAND_THIS_IS_HEIGHT_MAX};
            return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
        }
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_this_is_height_max_cb");

    return return_code;
}

// "Reset to defaults" characteristic callback.
static int characteristic_reset_to_defaults_cb(uint16_t conn_handle, uint16_t attr_handle,
                                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_reset_to_defaults_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        // Only accept this command if the launcher is in the right state
        if (can_reset_height_parameters(context->launcher.state)) {
            command_contents_t command_contents = {.command = COMMAND_RESET_TO_DEFAULTS};
            return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
        }
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_reset_to_defaults_cb");

    return return_code;
}

// "Emergency stop" characteristic callback.
static int characteristic_emergency_stop_cb(uint16_t conn_handle, uint16_t attr_handle,
                                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_emergency_stop_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_contents_t command_contents = {.command = COMMAND_EMERGENCY_STOP};
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_emergency_stop_cb");

    return return_code;
}

// "Play track" characteristic callback.
static int characteristic_play_track_cb(uint16_t conn_handle, uint16_t attr_handle,
                                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_play_track_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        command_contents_t command_contents = {.command = COMMAND_PLAY_TRACK};
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_play_track_cb");

    return return_code;
}

// "Auto enable" characteristic callback.
static int characteristic_auto_enable_cb(uint16_t conn_handle, uint16_t attr_handle,
                                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_auto_enable_cb");

    command_contents_t command_contents = {.command = COMMAND_AUTO_ENABLE,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        bool reading = false;
        nvs_auto_enable_get(&reading);
        command_contents.value = (uint32_t) reading;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }

    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_auto_enable_cb");

    return return_code;
}

// "Auto period" characteristic callback.
static int characteristic_auto_period_seconds_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_auto_period_seconds_cb");

    command_contents_t command_contents = {.command = COMMAND_AUTO_PERIOD_SECONDS,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_auto_period_seconds_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_auto_period_seconds_cb");

    return return_code;
}

// "Height max" characteristic callback.
static int characteristic_height_max_mm_cb(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_height_max_mm_cb");

    command_contents_t command_contents = {.command = COMMAND_HEIGHT_MAX_MM,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_height_max_mm_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_height_max_mm_cb");

    return return_code;
}

// "Speed" characteristic callback.
static int characteristic_speed_mm_per_second_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                 struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_speed_mm_per_second_cb");

    command_contents_t command_contents = {.command = COMMAND_SPEED_MM_PER_SECOND,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_speed_mm_per_second_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
        if (command_contents.value < SPEED_MIN_MM_PER_SECOND) {
            command_contents.value = SPEED_MIN_MM_PER_SECOND;
        }
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_speed_mm_per_second_cb");

    return return_code;
}

// "Track num skittering" characteristic callback.
static int characteristic_track_number_skittering_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                     struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_skttering_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_SKITTERING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_track_number_skittering_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_skttering_cb");

    return return_code;
}

// "Track num jumping" characteristic callback.
static int characteristic_track_number_jumping_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_jumping_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_JUMPING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_track_number_jumping_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_jumping_cb");

    return return_code;
}

// "Track num resetting" characteristic callback.
static int characteristic_track_number_resetting_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_resetting_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_RESETTING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        nvs_track_number_resetting_get(&command_contents.value);
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_resetting_cb");

    return return_code;
}

// "Random enable" characteristic callback.
static int characteristic_random_enable_cb(uint16_t conn_handle, uint16_t attr_handle,
                                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_random_enable_cb");

    command_contents_t command_contents = {.command = COMMAND_RANDOM_ENABLE,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        bool reading = false;
        nvs_random_enable_get(&reading);
        command_contents.value = (uint32_t) reading;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_random_enable_cb");

    return return_code;
}

// "Mute" characteristic callback.
static int characteristic_mute_cb(uint16_t conn_handle, uint16_t attr_handle,
                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_mute_cb");

    command_contents_t command_contents = {.command = COMMAND_MUTE,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting
        bool reading = false;
        nvs_mute_get(&reading);
        command_contents.value = (uint32_t) reading;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_mute_cb");

    return return_code;
}

// "State" characteristic callback.
static int characteristic_state_cb(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_state_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        launcher_t *launcher = &context->launcher;
        command_contents_t command_contents = {.command = COMMAND_STATE,
                                               .value = launcher->state};
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_state_cb");

    return return_code;
}

// "Height current" characteristic callback.
static int characteristic_height_current_mm_cb(uint16_t conn_handle, uint16_t attr_handle,
                                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_height_current_mm_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        command_contents_t command_contents = {.command = COMMAND_HEIGHT_CURRENT_MM,
                                               .value = (uint32_t) -1};
        retained_ram_t retained_ram;
        if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
            (retained_ram.step_ground_level != STEP_UNKNOWN) &&
            (retained_ram.step_current != STEP_UNKNOWN)) {
            // Populate the current height if we have it
            command_contents.value = (retained_ram.step_current - retained_ram.step_ground_level) * QRD1114_STEP_LENGTH_MM;
        }
        return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_height_current_mm_cb");

    return return_code;
}

/* ----------------------------------------------------------------
 * VARIABLES FOR COMMAND_HANDLING
 * -------------------------------------------------------------- */

// Entries are in the same order as command_t, must
// have the same number of entries as command_t.
static const command_data_t g_command_data_list[] = {{.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE1,
                                                      .name = "LAUNCH_NOW",
                                                      .characteristic_cb = characteristic_launch_now_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_launch_now
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE2,
                                                      .name = "UP",
                                                      .characteristic_cb = characteristic_up_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_up
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE3,
                                                      .name = "DOWN",
                                                      .characteristic_cb = characteristic_down_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_down
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE4,
                                                      .name = "THIS_IS_GROUND_LEVEL",
                                                      .characteristic_cb = characteristic_this_is_ground_level_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                     .handler = handler_this_is_ground_level
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE5,
                                                      .name = "THIS_IS_HEIGHT_MAX",
                                                      .characteristic_cb = characteristic_this_is_height_max_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_this_is_height_max
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE6,
                                                      .name = "RESET_TO_DEFAULTS",
                                                      .characteristic_cb = characteristic_reset_to_defaults_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_reset_to_defaults
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE7,
                                                      .name = "EMERGENCY_STOP",
                                                      .characteristic_cb = characteristic_emergency_stop_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_emergency_stop
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_UINT32,
                                                      .uuid = 0xFFE8,
                                                      .name = "PLAY TRACK",
                                                      .characteristic_cb = characteristic_play_track_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE,
                                                      .handler = handler_play_track
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFE9,
                                                      .name = "AUTO",
                                                      .characteristic_cb = characteristic_auto_enable_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_auto_enable
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFEA,
                                                      .name = "AUTO_PERIOD_SECONDS",
                                                      .characteristic_cb = characteristic_auto_period_seconds_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_auto_period_seconds
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFEB,
                                                      .name = "HEIGHT_MAX_MM",
                                                      .characteristic_cb = characteristic_height_max_mm_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_height_max_mm
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFEC,
                                                      .name = "SPEED_MM_PER_SECOND",
                                                      .characteristic_cb = characteristic_speed_mm_per_second_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_speed_mm_per_second
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFED,
                                                      .name = "COMMAND_TRACK_NUMBER_SKITTERING",
                                                      .characteristic_cb = characteristic_track_number_skittering_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_track_number_skittering
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFEE,
                                                      .name = "COMMAND_TRACK_NUMBER_JUMPING",
                                                      .characteristic_cb = characteristic_track_number_jumping_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_track_number_jumping
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFEF,
                                                      .name = "COMMAND_TRACK_NUMBER_RELAXING",
                                                      .characteristic_cb = characteristic_track_number_resetting_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_track_number_resetting
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFF0,
                                                      .name = "RANDOM",
                                                      .characteristic_cb = characteristic_random_enable_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_random_enable
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFF1,
                                                      .name = "MUTE",
                                                      .characteristic_cb = characteristic_mute_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_mute
                                                     },
                                                     {.type = COMMAND_TYPE_READ_ONLY,
                                                      .uuid = 0xFFF2,
                                                      .name = "STATE",
                                                      .characteristic_cb = characteristic_state_cb,
                                                      .flags = BLE_GATT_CHR_F_READ
                                                      // No handler for this one
                                                     },
                                                     {.type = COMMAND_TYPE_READ_ONLY,
                                                      .uuid = 0xFFF3,
                                                      .name = "HEIGHT_CURRENT_MM",
                                                      .characteristic_cb = characteristic_height_current_mm_cb,
                                                      .flags = BLE_GATT_CHR_F_READ
                                                      // No handler for this one
                                                     }};

// Do some checking
_Static_assert (FGR_UTIL_ARRAY_LENGTH(g_command_data_list) == COMMAND_NUMBER_OF,
                "the number of g_command_data_list[] entries does not match the number of commands!");

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
                // "Launch now" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_LAUNCH_NOW].uuid),
                .access_cb = g_command_data_list[COMMAND_LAUNCH_NOW].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_LAUNCH_NOW].flags
            },
            {
                // "Up" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_UP].uuid),
                .access_cb = g_command_data_list[COMMAND_UP].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_UP].flags
            },
            {
                // "Down" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_DOWN].uuid),
                .access_cb = g_command_data_list[COMMAND_DOWN].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_DOWN].flags
            },
            {
                // "This is ground level" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_THIS_IS_GROUND_LEVEL].uuid),
                .access_cb = g_command_data_list[COMMAND_THIS_IS_GROUND_LEVEL].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_THIS_IS_GROUND_LEVEL].flags
            },
            {
                // "This is height max" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_THIS_IS_HEIGHT_MAX].uuid),
                .access_cb = g_command_data_list[COMMAND_THIS_IS_HEIGHT_MAX].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_THIS_IS_HEIGHT_MAX].flags
            },
            {
                // "Reset to defaults" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_RESET_TO_DEFAULTS].uuid),
                .access_cb = g_command_data_list[COMMAND_RESET_TO_DEFAULTS].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_RESET_TO_DEFAULTS].flags
            },
            {
                // "Emergency stop" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_EMERGENCY_STOP].uuid),
                .access_cb = g_command_data_list[COMMAND_EMERGENCY_STOP].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_EMERGENCY_STOP].flags
            },
            {
                // "Play track" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_PLAY_TRACK].uuid),
                .access_cb = g_command_data_list[COMMAND_PLAY_TRACK].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_PLAY_TRACK].flags
            },
            {
                // "Auto enable" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_AUTO_ENABLE].uuid),
                .access_cb = g_command_data_list[COMMAND_AUTO_ENABLE].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_AUTO_ENABLE].flags
            },
            {
                // "Auto period" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_AUTO_PERIOD_SECONDS].uuid),
                .access_cb = g_command_data_list[COMMAND_AUTO_PERIOD_SECONDS].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_AUTO_PERIOD_SECONDS].flags
            },
            {
                // "Height max" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_HEIGHT_MAX_MM].uuid),
                .access_cb = g_command_data_list[COMMAND_HEIGHT_MAX_MM].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_HEIGHT_MAX_MM].flags
            },
            {
                // "Speed" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_SPEED_MM_PER_SECOND].uuid),
                .access_cb = g_command_data_list[COMMAND_SPEED_MM_PER_SECOND].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_SPEED_MM_PER_SECOND].flags
            },
            {
                // "Track num skittering" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].flags
            },
            {
                // "Track num jumping" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].flags
            },
            {
                // "Track num resetting" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_RESETTING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_RESETTING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_RESETTING].flags
            },
            {
                // "Random enable" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_RANDOM_ENABLE].uuid),
                .access_cb = g_command_data_list[COMMAND_RANDOM_ENABLE].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_RANDOM_ENABLE].flags
            },
            {
                // "Mute" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_MUTE].uuid),
                .access_cb = g_command_data_list[COMMAND_MUTE].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_MUTE].flags
            },
            {
                // "State" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_STATE].uuid),
                .access_cb = g_command_data_list[COMMAND_STATE].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_STATE].flags
            },
            {
                // "Height current mm" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_HEIGHT_CURRENT_MM].uuid),
                .access_cb = g_command_data_list[COMMAND_HEIGHT_CURRENT_MM].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_HEIGHT_CURRENT_MM].flags
            },
            {0}  // Terminator
        },
    },
    {0}  // Service terminator
};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: COMMAND TASK
 * -------------------------------------------------------------- */

// Callback that should be run as a task to handle the command queue.
static void command_cb(void *handle, void *arg)
{
    context_t * context = (context_t *) arg;
    command_contents_t command_contents;

    (void) handle;

    CONTEXT_LOCK(context->lock, "command_cb");

    while (xQueueReceive(context->command_queue, &command_contents, 0) == pdTRUE) {
        command_t command = command_contents.command;
        if (command < FGR_UTIL_ARRAY_LENGTH(g_command_data_list)) {

            const char *name = g_command_data_list[command].name;
            uint16_t uuid = g_command_data_list[command].uuid;
            command_type_t type = g_command_data_list[command].type;
            command_handler_t handler = g_command_data_list[command].handler;
            uint32_t value = command_contents.value;

            // Print some debug
            switch (type) {
                case COMMAND_TYPE_WRITE_ONLY_NO_VALUE:
                    ESP_LOGI(TAG, "command %s (0x%04x).", name, uuid);
                break;
                case COMMAND_TYPE_WRITE_ONLY_UINT32:
                    ESP_LOGI(TAG, "command %s (0x%04x), write: %d.", name, uuid, value);
                break;
                case COMMAND_TYPE_READ_WRITE_BOOLEAN:
                    if (command_contents.read_not_write) {
                        ESP_LOGI(TAG, "command %s (0x%04x), read: %s.",
                                 name, uuid, value ? "enabled" : "disabled");
                    } else {
                        ESP_LOGI(TAG, "command %s (0x%04x), write: %s.",
                                 name, uuid, value ? "enable" : "disable");
                    }
                break;
                case COMMAND_TYPE_READ_WRITE_UINT32:
                    if (command_contents.read_not_write) {
                        ESP_LOGI(TAG, "command %s (0x%04x), read: %d.", name, uuid, value);
                    } else {
                        ESP_LOGI(TAG, "command %s (0x%04x), write: %d.", name, uuid, value);
                    }
                break;
                case COMMAND_TYPE_READ_ONLY:
                    ESP_LOGI(TAG, "command %s (0x%04x), read: %d.", name, uuid, value);
                break;
                default:
                    ESP_LOGE(TAG, "unknown command type (%d)!", type);
                break;
            }

            // Call the command handler
            if (handler) {
                handler(&context->launcher, value);
            }

        } else {
            ESP_LOGE(TAG, "unknown command (%d)!", command);
        }
    }

    CONTEXT_UNLOCK(context->lock, "command_cb");
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: MOTOR CALLBACK
 * -------------------------------------------------------------- */

// Callback called each time the QRD1114 sensor on the motor is triggered.
static void motor_callback(void *motor, void *param)
{
    launcher_t *launcher = (launcher_t *) param;

    (void) motor;

    retained_ram_t retained_ram = {.step_current = 0,
                                   .step_ground_level = STEP_UNKNOWN};
    FGR_RRAM_GET(retained_ram);

    if (retained_ram.step_current == STEP_UNKNOWN) {
        retained_ram.step_current = 0;
    } else {
        if (launcher->direction_up_not_down) {
            retained_ram.step_current++;
        } else {
            retained_ram.step_current--;
        }
    }

    FGR_RRAM_SET(retained_ram);
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: INITIALISATION
 * -------------------------------------------------------------- */

// Initialisation.
static esp_err_t init(context_t *context)
{
    launcher_t *launcher = &context->launcher;

    // Allow us to feed the watchdog
    esp_task_wdt_add(NULL);

    // Create mutex to guard the context
    esp_err_t err = -ESP_ERR_NO_MEM;
    context->lock = xSemaphoreCreateMutex();
    if (context->lock) {
        err = ESP_OK;
    }

    if (err == ESP_OK) {

        CONTEXT_LOCK(context->lock, "init");

        retained_ram_t retained_ram;
        if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
            (retained_ram.step_current != STEP_UNKNOWN) &&
            (retained_ram.step_ground_level != STEP_UNKNOWN)) {
            // There is retained RAM, which means we
            // were running, so set the initial
            // state to resetting
            int32_t height_steps = retained_ram.step_current - retained_ram.step_ground_level;
            ESP_LOGI(TAG, "warm start (current height is %d mm (%d step(s))).",
                     height_steps * QRD1114_STEP_LENGTH_MM, height_steps);
            launcher->state = LAUNCHER_STATE_RUNNING_RESETTING;
        }

        // Initialise a brushless motor
        if (err == ESP_OK) {
            err = motor_brushless_init();
            if (err == ESP_OK) {
                err = -ESP_ERR_NO_MEM;
                launcher->motor = motor_brushless_create(CONFIG_SPIDER_LAUNCHER_MOTOR_PWM_PIN,
                                                         CONFIG_SPIDER_LAUNCHER_MOTOR_DIRECTION_PIN,
                                                         CONFIG_SPIDER_LAUNCHER_QRD1114_PIN,
                                                         motor_callback, launcher);
                if (launcher->motor) {
                    err = ESP_OK;
                }
            }
        }

#if defined(CONFIG_SPIDER_LAUNCHER_UART_NUM) && (CONFIG_SPIDER_LAUNCHER_UART_NUM >= 0)
        // Initialise a UART to control the DY HV8F audio board
        if (err == ESP_OK) {
            const uart_config_t cfg = {
                .baud_rate = 9600,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
            };

            err = -uart_driver_install(CONFIG_SPIDER_LAUNCHER_UART_NUM, UART_RX_BUFFFER_SIZE, 0, 0, NULL, 0);
            if (err == ESP_OK) {
                err = -uart_param_config(CONFIG_SPIDER_LAUNCHER_UART_NUM, &cfg);
            }
            if (err == ESP_OK) {
                err = -uart_set_pin(CONFIG_SPIDER_LAUNCHER_UART_NUM,
                                    CONFIG_SPIDER_LAUNCHER_AUDIO_UART_TXD_PIN,
                                    CONFIG_SPIDER_LAUNCHER_AUDIO_UART_RXD_PIN,
                                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            }
        }
#else
    // Set the pins that control the audio track all high so that they can be pulled low
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0 >= 0)
    audio_play_set_pin(0xF, 0x01, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1 >= 0)
    audio_play_set_pin(0xF, 0x02, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2 >= 0)
    audio_play_set_pin(0xF, 0x04, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, true);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3 >= 0)
    audio_play_set_pin(0xF, 0x08, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, true);
#  endif
#endif

        // Initialise tasking, FGR style
        if (err == ESP_OK) {
            err = fgr_task_init();
        }

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
            err = fgr_debug_init();
        }

        // Make sure that NVS is populated (in case it is blank)
        if (err == ESP_OK) {
            nvs_populate();
        }

        if (err == ESP_OK) {
            err = -ESP_ERR_NO_MEM;
            // RTOS stuff needed for launcher operation
            launcher_t *launcher = &context->launcher;
            launcher->queue = xQueueCreate(10, sizeof(launcher_state_t));
            if (launcher->queue) {
                err = fgr_task_create(&launcher_cb, context, "launcher", 4096, 3, &launcher->task);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "unable to create launcher task (%s)!", esp_err_to_name(-err));
                }
            } else {
                ESP_LOGE(TAG, "unable to create launcher queue!");
            }
        }

        if (err == ESP_OK) {
            err = -ESP_ERR_NO_MEM;
            // RTOS stuff needed for command handling
            context->command_queue = xQueueCreate(10, sizeof(command_contents_t));
            if (context->command_queue) {
                err = fgr_task_create(&command_cb, context, "command", 4096, 3, &context->command_task);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "unable to create command task (%s)!", esp_err_to_name(-err));
                }
            } else {
                ESP_LOGE(TAG, "unable to create command queue!");
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

        CONTEXT_UNLOCK(context->lock, "init");
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

    ESP_LOGI(TAG, "app_main start");

    esp_err_t err = init(context);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "initialization complete.");
        ESP_LOGI(TAG, "waiting for BLE connections/commands.");
        uint32_t track_number = 1;
        bool mute = false;
        nvs_mute_get(&mute);
        if (mute) {
            ESP_LOGI(TAG, "muted, not playing an audio.");
        }
        while (1) {
            // Let BLE commands do their thing
            vTaskDelay(pdMS_TO_TICKS(3000));
            esp_task_wdt_reset();
            nvs_mute_get(&mute);
            if (!mute) {
                audio_play_track(track_number);
                track_number++;
                if (track_number > 4) {
                    track_number = 1;
                }
            }
        }
        esp_task_wdt_delete(NULL);
    } else {
        ESP_LOGE(TAG, "initialization failed, system cannot continue, will restart soonish.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    nimble_port_stop();
    fgr_debug_deinit();
    fgr_ws2812_deinit();
    fgr_task_deinit();
    if (context->command_queue){
        vQueueDelete(context->command_queue);
    }
    launcher_t *launcher = &context->launcher;
    if (launcher->queue){
        vQueueDelete(launcher->queue);
    }
    motor_brushless_deinit();
    if (context->lock) {
        vSemaphoreDelete(context->lock);
    }
    esp_restart();
}

// End of file
