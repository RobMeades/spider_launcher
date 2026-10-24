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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "esp_log_level.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
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
#include "sdkconfig.h"
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

#include "tmc5160.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

 // Logging prefix.
 #define TAG "spider_launcher"

// The version of the BLE interface: increment this if there is a
// breaking change to the BLE characteristic handling.
#define BLE_INTERFACE_VERSION 2

// UART buffer.
#define UART_RX_BUFFFER_SIZE 256

// The diameter of the winch in millimetres
#define WINCH_DIAMETER_MM 100

// TMC5160 stepper motor controller address.
#define TMC5160_ADDRESS 0

// The sense resisitor wired to the TMC5160, the BigTreeTech board
// uses 0.022 Ohms.
#define TMC5160_RSENSE_MOHM 22

// TBL time for the stepper motor, see tmc5160_configure_chopper().
#define TMC5160_TBL 1

// TOFF time for the stepper motor, see tmc5160_configure_chopper().
#define TMC5160_TOFF 3

// HSTRT time for the stepper motor, see tmc5160_configure_chopper().
#define TMC5160_HSTRT 4

// HEND time for the stepper motor, see tmc5160_configure_chopper().
#define TMC5160_HEND -2

// The step resolution for the motor.
#define TMC5160_STEPS_PER_REVOLUTION 200

// The microstep resolution for the motor (1 to 256 in powers of two).
#define TMC5160_MICROSTEP_RESOLUTION 256

// How much current to feed to the stepper motor when idle.
#define TMC5160_IDLE_CURRENT_MA 500

// How much current to feed to the stepper motor when running.
#define TMC5160_RUN_CURRENT_MA 1000

// The percentage to apply to the run current when holding.
#define TMC5160_HOLD_CURRENT_PERCENT 50

// Normal acceleration, for use with tmc5160_set_acceleration().
#define TMC5160_ACCELERATION_NORMAL 1000

// Abrupt acceleration, for use with tmc5160_set_acceleration.
#define TMC5160_ACCELERATION_ABRUPT 10000

// Normal deceleration, for use with tmc5160_set_deceleration().
#define TMC5160_DECELERATION_NORMAL 1000

// Abrupt deceleration, for use with tmc5160_set_deceleration().
#define TMC5160_DECELERATION_ABRUPT 10000

// Delay after stopping the motor to when we assume it has stopped
// (and can therefore safely switch current)
#define TMC5160_STOP_TIME_MS 1200

// The threshold at which we switch from Stealth Chop to
// Spread Cycle for extra oomph in millimetres per second.
#define TMC5160_SPREAD_CYCLE_THRESHOLD_MM_PER_SECOND 600

// An unused track number (if it gets played nothing will happen).
#define TRACK_NUMBER_UNUSED 0

// The name that we appear under in Bluetooth.
#define BLE_DEVICE_NAME "spider_launcher"

// Service UUID for spider launcher control.
#define SERVICE_UUID 0xFFE0

// The name for the "auto enable" entry in non-volatile storage.
#define NVS_NAME_AUTO_ENABLE "auto_enabled"

// The name for the "auto period" entry in non-volatile storage.
#define NVS_NAME_AUTO_PERIOD_SECONDS "auto_period_sec"

// The name for the "height max" entry in non-volatile storage.
#define NVS_NAME_HEIGHT_MAX_MM "height_max_mm"

// The name for the "speed" entry in non-volatile storage (truncated
// to keep within 15 characters).
#define NVS_NAME_SPEED_MM_PER_SECOND "speed_mm_per_se"

// The name for the "track number skittering" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_SKITTERING "track_num_skit"

// The name for the "track number jumping" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_JUMPING "track_num_jump"

// The name for the "track number dangling" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_DANGLING "track_num_dang"

// The name for the "track number resetting" entry in non-volatile
// storage (truncated to keep within 15 characters).
#define NVS_NAME_TRACK_NUMBER_RESETTING "track_num_reset"

// The name for the "mute" entry in non-volatile storage.
#define NVS_NAME_MUTE "mute"

// The name for the "random enable" entry in non-volatile storage.
#define NVS_NAME_RANDOM_ENABLE "random_enabled"

// Default value for "auto enable".
#define DEFAULT_AUTO_ENABLE false

// Default value for "auto period".
#define DEFAULT_AUTO_PERIOD_SECONDS 180

// Default value for "height max".
#define DEFAULT_HEIGHT_MAX_MM 2000

// Default value for "speed".
#define DEFAULT_SPEED_MM_PER_SECOND 500

// Default value for "track number skittering".
#define DEFAULT_TRACK_NUMBER_SKITTERING 5

// Default value for "track number jumping".
#define DEFAULT_TRACK_NUMBER_JUMPING 4

// Default value for "track number dangling".
#define DEFAULT_TRACK_NUMBER_DANGLING TRACK_NUMBER_UNUSED

// Default value for "track number resetting".
#define DEFAULT_TRACK_NUMBER_RESETTING TRACK_NUMBER_UNUSED

// Default value for "random enable".
#define DEFAULT_RANDOM_ENABLE true

// Default value for "mute".
#define DEFAULT_MUTE false

// An out of range value to represent an unknown step count
#define POSITION_UNKNOWN 0x7FFFFFFF

// A maximum speed.
#define SPEED_MAX_MM_PER_SECOND 1000

// The minimum usable speed in mm per second.
#define SPEED_MIN_MM_PER_SECOND 40

#if DEFAULT_SPEED_MM_PER_SECOND < SPEED_MIN_MM_PER_SECOND
#  error "DEFAULT_SPEED_MM_PER_SECOND must be at least SPEED_MIN_MM_PER_SECOND"
#endif

// The height of fall needed to trigger the spider's motion sensor.
#define WAKING_JIGGLE_HEIGHT_MM 150

// The number of falls to trigger the spider's motion sensor reliably.
#define WAKING_JIGGLE_CYCLES 1

// How fast to raise and drop the spider while waking it.
#define WAKING_SPEED_MM_PER_SECOND 1500

// A nice high acceleration for the wake-up.
#define WAKING_ACCELERATION 30000

// A high enough deceleration to switch on the eyes.
#define WAKING_DECELERATION 1500

// Pause for the poor thing to recover.
#define WAKING_SETTLE_TIME_MS 0

// How long to dangle for at the top of the run.
#define DANGLE_TIME_SECONDS 8

// Percentage speed reduction when resetting.
#define RESETTING_SPEED_MODIFIER_PERCENT 50

// Percentage speed reduction when just moving up or down.
#define MOVE_UP_DOWN_SPEED_MODIFIER_PERCENT 100

// The maximum duration for a move up or down in milliseconds
#define MOVE_UP_DOWN_DURATION_MAX_MS 500

// How long to pause for when doing random jerky movements, in milliseconds.
#define RANDOM_PAUSE_TIME_MS 100

// How long to leave between jerky movements in milliseconds.
#define RANDOM_PAUSE_INTERVAL_MS 250

// Put the spider down nice and gently to avoid turning its eyes on again.
#define RESETTING_DECELERATION 100

// How long to keep the IO lines at the signalled level when running
// the audio board in IO mode.
#define AUDIO_IO_WAIT_TIME_MS 100

// Retry time for auto mode if it was unable to run.
#define AUTO_MODE_RETRY_TIME_SECONDS 5

// Defaults for the retained_ram_t structure.
#define DEFAULTS_RETAINED_RAM {.ground_level_mm = POSITION_UNKNOWN,   \
                               .emergency_stopped = false}

// The colour for the debug LED if we are emergency stopped.
#define DEBUG_LED_COLOUR_EMERGENCY_STOPPED FGR_WS2812_LED_COLOUR_WHITE

// Pi approximated as 3141593 / 1000000.
#define PI_NUMERATOR        3141593LL
#define PI_DENOMINATOR      1000000LL

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// The types of command
typedef enum {
    COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
    COMMAND_TYPE_WRITE_ONLY_UINT32,
    COMMAND_TYPE_WRITE_ONLY_BOOLEAN,
    COMMAND_TYPE_READ_WRITE_UINT32,
    COMMAND_TYPE_READ_WRITE_BOOLEAN,
    COMMAND_TYPE_READ_ONLY
} command_type_t;

// Commands to the launcher.
// If you modify this enum, you will need to modify
// g_command_data_list[] to match.
typedef enum {
    COMMAND_LAUNCH_NOW,              // write only, has no value
    COMMAND_UP,                      // write only, Boolean
    COMMAND_DOWN,                    // write only, Boolean
    COMMAND_THIS_IS_GROUND_LEVEL,    // write only, has no value
    COMMAND_THIS_IS_HEIGHT_MAX,      // write only, has no value
    COMMAND_RESET_TO_DEFAULTS,       // write only, has no value
    COMMAND_EMERGENCY_STOP_ENABLE,   // read/write, Boolean
    COMMAND_PLAY_TRACK,              // write only, uint32_t
    COMMAND_AUTO_ENABLE,             // read/write, Boolean
    COMMAND_AUTO_PERIOD_SECONDS,     // read/write, uint32_t
    COMMAND_HEIGHT_MAX_MM,           // read/write, uint32_t
    COMMAND_SPEED_MM_PER_SECOND,     // read/write, uint32_t
    COMMAND_TRACK_NUMBER_SKITTERING, // read/write, uint32_t
    COMMAND_TRACK_NUMBER_JUMPING,    // read/write, uint32_t
    COMMAND_TRACK_NUMBER_DANGLING,   // read/write, uint32_t
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
    LAUNCHER_STATE_POSITION_UNKNOWN,
    LAUNCHER_STATE_READY,
    LAUNCHER_STATE_MOVE_UP,
    LAUNCHER_STATE_MOVE_DOWN,
    LAUNCHER_STATE_RUN,
    LAUNCHER_STATE_RUNNING_SKITTERING,
    LAUNCHER_STATE_RUNNING_WAKING,
    LAUNCHER_STATE_RUNNING_JUMPING,
    LAUNCHER_STATE_RUNNING_DANGLING,
    LAUNCHER_STATE_RUNNING_RESETTING,
    LAUNCHER_STATE_HALT,
    LAUNCHER_STATE_NUMBER_OF
} launcher_state_t;

// Shadow structure for NVS items: since BLE
// _also_ write to NVS, trying to avoid
// accessing it is a good thing to reduce contention.
typedef struct {
   bool auto_enabled;
   uint32_t auto_period_seconds;
   uint32_t height_max_mm;
   uint32_t speed_mm_per_second;
   uint32_t track_number_skittering;
   uint32_t track_number_jumping;
   uint32_t track_number_dangling;
   uint32_t track_number_resetting;
   bool random_enabled;
   bool mute;
} nvs_shadow_t;

// A timer: start time and duration.
typedef struct {
    int64_t start_us;
    int64_t duration_us;
} launcher_timer_t;

// Structure to hold motor data
typedef struct {
    stepper_t *handle;
    uint32_t last_current_setting_ma;
} launcher_motor_t;

// The launcher context
typedef struct {
    QueueHandle_t queue_state;
    TaskHandle_t task_state;
    launcher_state_t state;
    int32_t height_target_mm;
    bool direction_up_not_down;
    launcher_timer_t timer_a;
    launcher_timer_t timer_b;
    bool temporary_stop;
    uint32_t waking_cycles_completed;
    TaskHandle_t task_auto;
    uint32_t auto_period_seconds;
    launcher_timer_t timer_auto;
    nvs_shadow_t nvs_shadow;
    launcher_motor_t motor;
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

// Retained RAM storage.  If you update this structure, you
// also need to update DEFAULTS_RETAINED_RAM above.
typedef struct {
    int32_t ground_level_mm;
    bool emergency_stopped;
} retained_ram_t;

// Function prototype for a characteristic callback (which BLE calls).
typedef int (*characteristic_cb_t) (uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);

// Function prototype for a command handler (which ultimate does the work).
typedef void (*command_handler_t) (launcher_t *launcher, uint32_t value, bool read_not_write);

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
static context_t g_context = {.service_uuid = BLE_UUID16_INIT(SERVICE_UUID),
                              .ble_connection_handle = BLE_HS_CONN_HANDLE_NONE,
                              .launcher.height_target_mm = POSITION_UNKNOWN};

// Retained RAM storage
FGR_RRAM_DEFINE(retained_ram_t, retained_ram);

// The names of the launcher states for debug prints;
// entries are in the same order as launcher_state_t
// and there must be the same number of entries
static const char *g_launcher_state_name[] = {"NULL",
                                              "POSITION_UNKNOWN",
                                              "READY",
                                              "MOVE_UP",
                                              "MOVE_DOWN",
                                              "RUN",
                                              "RUNNING_SKITTERING",
                                              "RUNNING_WAKING",
                                              "RUNNING_JUMPING",
                                              "RUNNING_DANGLING",
                                              "RUNNING_RESETTING",
                                              "HALT"};

// Do some checking
_Static_assert (FGR_UTIL_ARRAY_LENGTH(g_launcher_state_name) == LAUNCHER_STATE_NUMBER_OF,
                "the number of g_launcher_state_name[] entries does not match the number of launcher states!");

// Table of launcher states to LED colours
static const fgr_ws2812_colour_t g_launcher_state_colour[] = {FGR_WS2812_LED_COLOUR_WHITE,      // NULL
                                                              FGR_WS2812_LED_COLOUR_CYAN,       // POSITION_UNKNOWN
                                                              FGR_WS2812_LED_COLOUR_GREEN,      // READY
                                                              FGR_WS2812_LED_COLOUR_YELLOW,     // MOVE_UP
                                                              FGR_WS2812_LED_COLOUR_YELLOW,     // MOVE_DOWN
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,    // RUN
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,    // RUNNING_SKITTERING
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,    // RUNNING_WAKING
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,    // RUNNING_JUMPING
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,    // RUNNING_DANGLING
                                                              FGR_WS2812_LED_COLOUR_MAGENTA,   // RUNNING_RESETTING
                                                              FGR_WS2812_LED_COLOUR_NONE};     // HALT

// Do some checking
_Static_assert (FGR_UTIL_ARRAY_LENGTH(g_launcher_state_colour) == LAUNCHER_STATE_NUMBER_OF,
                "the number of g_launcher_state_colour[] entries does not match the number of launcher states!");


// The durations of the known audio tracks, in order of track number
// noting that 0 is unused
static const uint32_t g_track_duration_ms[] = {0, 1000, 1000, 1000,
                                               14000, 10000, 1000, 1000,
                                               1000, 1000};

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
static int32_t nvs_auto_enable_set(nvs_shadow_t *nvs_shadow, bool enable)
{
    int32_t err = fgr_nvs_set(NVS_NAME_AUTO_ENABLE, enable);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->auto_enabled = enable;
    }

    return err;
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
static int32_t nvs_auto_period_seconds_set(nvs_shadow_t *nvs_shadow,
                                           uint32_t auto_period_seconds)
{
    int32_t err = fgr_nvs_set(NVS_NAME_AUTO_PERIOD_SECONDS, auto_period_seconds);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->auto_period_seconds = auto_period_seconds;
    }

    return err;
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
static int32_t nvs_height_max_mm_set(nvs_shadow_t *nvs_shadow,
                                     uint32_t height_max_mm)
{
    int32_t err = fgr_nvs_set(NVS_NAME_HEIGHT_MAX_MM, height_max_mm);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->height_max_mm = height_max_mm;
    }

    return err;
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
static int32_t nvs_speed_mm_per_second_set(nvs_shadow_t *nvs_shadow,
                                           uint32_t speed_mm_per_second)
{
    int32_t err = fgr_nvs_set(NVS_NAME_SPEED_MM_PER_SECOND, speed_mm_per_second);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->speed_mm_per_second = speed_mm_per_second;
    }

    return err;
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

// Set the track number for skittering in NVS.
static int32_t nvs_track_number_skittering_set(nvs_shadow_t *nvs_shadow,
                                               uint32_t track_number)
{
    int32_t err = fgr_nvs_set(NVS_NAME_TRACK_NUMBER_SKITTERING, track_number);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->track_number_skittering = track_number;
    }

    return err;
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

// Set the track number for jumping in NVS.
static int32_t nvs_track_number_jumping_set(nvs_shadow_t *nvs_shadow,
                                            uint32_t track_number)
{
    int32_t err = fgr_nvs_set(NVS_NAME_TRACK_NUMBER_JUMPING, track_number);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->track_number_jumping = track_number;
    }

    return err;
}

// Retrieve the dangling track number from NVS;
// this is guaranteed to return a useful answer, even on error.
static int32_t nvs_track_number_dangling_get(uint32_t *track_number)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (track_number) {
        err = fgr_nvs_get(NVS_NAME_TRACK_NUMBER_DANGLING, track_number);
        if (err != ESP_OK) {
            *track_number = DEFAULT_TRACK_NUMBER_DANGLING;
        }
    }

    return err;
}

// Set the track number for dangling in NVS.
static int32_t nvs_track_number_dangling_set(nvs_shadow_t *nvs_shadow,
                                             uint32_t track_number)
{
    int32_t err = fgr_nvs_set(NVS_NAME_TRACK_NUMBER_DANGLING, track_number);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->track_number_dangling = track_number;
    }

    return err;
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

// Set the track number for resetting in NVS.
static int32_t nvs_track_number_resetting_set(nvs_shadow_t *nvs_shadow,
                                              uint32_t track_number)
{
    int32_t err = fgr_nvs_set(NVS_NAME_TRACK_NUMBER_RESETTING, track_number);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->track_number_resetting = track_number;
    }

    return err;
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
static int32_t nvs_random_enable_set(nvs_shadow_t *nvs_shadow,
                                     bool enable)
{
    int32_t err = fgr_nvs_set(NVS_NAME_RANDOM_ENABLE, enable);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->random_enabled = enable;
    }

    return err;
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
static int32_t nvs_mute_set(nvs_shadow_t *nvs_shadow,
                            bool enable)
{
    int32_t err = fgr_nvs_set(NVS_NAME_MUTE, enable);

    if ((err == ESP_OK) && nvs_shadow) {
        nvs_shadow->mute = enable;
    }

    return err;
}

// Populate NVS with defaults if required.
static void nvs_populate(nvs_shadow_t *nvs_shadow)
{
    bool enable;
    uint32_t value;

    // Since the NVS functions all return a default value
    // on error, we can write that value back to set up
    // NVS when it is blank
    if (nvs_auto_enable_get(&enable) != ESP_OK) {
        nvs_auto_enable_set(NULL, enable);
    }
    nvs_shadow->auto_enabled = enable;
    if (nvs_auto_period_seconds_get(&value) != ESP_OK) {
        nvs_auto_period_seconds_set(NULL, value);
    }
    nvs_shadow->auto_period_seconds = value;
    if (nvs_height_max_mm_get(&value) != ESP_OK) {
        nvs_height_max_mm_set(NULL, value);
    }
    nvs_shadow->height_max_mm = value;
    if (nvs_speed_mm_per_second_get(&value) != ESP_OK) {
        nvs_speed_mm_per_second_set(NULL, value);
    }
    nvs_shadow->speed_mm_per_second = value;
    if (nvs_track_number_skittering_get(&value) != ESP_OK) {
        nvs_track_number_skittering_set(NULL, value);
    }
    nvs_shadow->track_number_skittering = value;
    if (nvs_track_number_jumping_get(&value) != ESP_OK) {
        nvs_track_number_jumping_set(NULL, value);
    }
    nvs_shadow->track_number_jumping = value;
    if (nvs_track_number_dangling_get(&value) != ESP_OK) {
        nvs_track_number_dangling_set(NULL, value);
    }
    nvs_shadow->track_number_dangling = value;
    if (nvs_track_number_resetting_get(&value) != ESP_OK) {
        nvs_track_number_resetting_set(NULL, value);
    }
    nvs_shadow->track_number_resetting = value;
    if (nvs_random_enable_get(&enable) != ESP_OK) {
        nvs_random_enable_set(NULL, enable);
    }
    nvs_shadow->random_enabled = enable;
    if (nvs_mute_get(&enable) != ESP_OK) {
        nvs_mute_set(NULL, enable);
    }
    nvs_shadow->mute = enable;
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: DY-HV8F AUDIO BOARD RELATED
 * -------------------------------------------------------------- */

#if !defined(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM) || (CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM < 0)
static uint8_t audio_play_set_pin(uint32_t track_number, int32_t bit_mask, int32_t pin, bool highNotLow)
{
    uint8_t resulting_bit_pattern = track_number & bit_mask;

    if (!highNotLow) {
        resulting_bit_pattern = ~resulting_bit_pattern;
    }
    gpio_set_level(pin, track_number & bit_mask ? highNotLow : !highNotLow);

    return resulting_bit_pattern;
}
#endif

// Play the given audio track.
static void audio_play_track(uint32_t track_number)
{
#if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM) && (CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM >= 0)
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

    int32_t x = uart_write_bytes(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM, buffer, sizeof(buffer));
    ESP_LOGI("DEBUG", "to play audio track %d sent %d byte(s) {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x} on UART %d, TXD on pin %d.",
             track_number, x, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM,
             CONFIG_SPIDER_LAUNCHER_AUDIO_UART_TXD_PIN);

    // Wait for transmission to complete
    uart_wait_tx_done(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM, pdMS_TO_TICKS(100));
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
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x04, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, false);
    bit_count++;
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3 >= 0)
    bit_pattern &= audio_play_set_pin(track_number, 0x08, CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, false);
    bit_count++;
#  endif

    ESP_LOGI(TAG, "to play audio track %d, setting IO lines (%d line(s)) to 0x%1x for %d ms.",
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
                params.itvl_min = 0x06;   // 7.5 ms (minimum)
                params.itvl_max = 0x06;   // 7.5 ms
                params.latency = 0;       // No slave latency
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
 * STATIC FUNCTIONS: MOTOR RELATED
 * -------------------------------------------------------------- */

// Convert height in mm to motor microsteps.
static int32_t height_to_microsteps(int32_t height_mm)
{
    int64_t numerator = (int64_t) height_mm *
                        TMC5160_STEPS_PER_REVOLUTION *
                        TMC5160_MICROSTEP_RESOLUTION *
                        PI_DENOMINATOR;
    int64_t denominator = (int64_t) WINCH_DIAMETER_MM * PI_NUMERATOR;

    // Round to nearest microstep, including negative values.
    return (int32_t) ((numerator +
                       (numerator >= 0 ? denominator / 2 : -denominator / 2)) /
                      denominator);
}

// Convert motor microsteps to height in mm.
static int32_t microsteps_to_mm(int32_t microsteps)
{
    int64_t numerator = (int64_t) microsteps *
                        WINCH_DIAMETER_MM *
                        PI_NUMERATOR;
    int64_t denominator = (int64_t) TMC5160_STEPS_PER_REVOLUTION *
                          TMC5160_MICROSTEP_RESOLUTION *
                          PI_DENOMINATOR;

    // Round to nearest mm, including negative values.
    return (int32_t) ((numerator +
                       (numerator >= 0 ? denominator / 2 : -denominator / 2)) /
                      denominator);
}

// Convert linear speed in mm/s to motor rotational speed in milliHertz.
static int32_t speed_mm_per_second_to_millihertz(int32_t speed_mm_per_second)
{
    int64_t numerator = (int64_t) speed_mm_per_second * 1000 * PI_DENOMINATOR;
    int64_t denominator = (int64_t) WINCH_DIAMETER_MM * PI_NUMERATOR;

    // Round to nearest milliHertz, including negative values.
    return (int32_t) ((numerator +
                       (numerator >= 0 ? denominator / 2 : -denominator / 2)) /
                      denominator);
}

// Set the motor current.
static void motor_set_current_ma(launcher_motor_t *motor, uint32_t current_ma)
{
    // Set the motor current
    int32_t err = tmc5160_set_current(motor->handle, TMC5160_RSENSE_MOHM,
                                      current_ma,
                                      TMC5160_HOLD_CURRENT_PERCENT);
    if (err >= 0) {
        ESP_LOGI(TAG, "motor current set to %d mA.", err);
        motor->last_current_setting_ma = current_ma;
        err = ESP_OK;
    } else {
        ESP_LOGW(TAG, "unable to set motor current to %d mA (%s)!",
                 current_ma, esp_err_to_name(-err));
    }
}

// Get the current position from the motor.
static int32_t motor_get_position_mm(launcher_motor_t *motor)
{
    int32_t position_mm = POSITION_UNKNOWN;

    // Read the current position from the TMC5160
    int32_t microsteps;
    int32_t err = tmc5160_get_position(motor->handle, &microsteps);
    if ((err == ESP_OK) && !tmc5160_has_reset(motor->handle)) {
        // Convert to a position in mm
        position_mm = microsteps_to_mm(microsteps);
    }

    return position_mm;
}

// Start the motor moving at the given speed.
static void motor_set_speed(launcher_motor_t *motor, bool direction_up_not_down,
                            uint32_t abs_speed_mm_per_second,
                            uint32_t speed_modifier_percent, uint32_t current_ma,
                            uint32_t acceleration, uint32_t deceleration)
{
    int32_t speed_mm_per_second = abs_speed_mm_per_second * speed_modifier_percent / 100;
    if (!direction_up_not_down) {
        speed_mm_per_second = -speed_mm_per_second;
    }

    // Make sure the motor is enabled
    motor_set_current_ma(motor, current_ma);
    int32_t err = tmc5160_motor_enable(motor->handle);
    if (err == ESP_OK) {
        err = tmc5160_set_acceleration(motor->handle, acceleration);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unable to set acceleration to %d (%s)!", acceleration,
                     esp_err_to_name(-err));
        }
        err = tmc5160_set_deceleration(motor->handle, deceleration);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unable to set deceleration to %d (%s)!", deceleration,
                     esp_err_to_name(-err));
        }
        int32_t millihertz = speed_mm_per_second_to_millihertz(speed_mm_per_second);
        int32_t err = tmc5160_set_velocity(motor->handle, millihertz);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "speed set to %d mm/s (%d milliHertz).", speed_mm_per_second, millihertz);
        } else {
            ESP_LOGE(TAG, "unable to set speed to %d mm/s (%d milliHertz) (%s)!",
                     speed_mm_per_second, millihertz, esp_err_to_name(-err));
        }
    } else {
        ESP_LOGE(TAG, "unable to enable motor (%s).", esp_err_to_name(-err));
    }
}

// Move the winch to the given height.
static void motor_set_height_mm(launcher_motor_t *motor, uint32_t height_mm,
                                uint32_t abs_speed_mm_per_second,
                                int32_t ground_level_mm, uint32_t current_ma,
                                uint32_t acceleration, uint32_t deceleration)
{
    int32_t position_mm = ground_level_mm + height_mm;
    int32_t microsteps = height_to_microsteps(position_mm);
    int32_t abs_speed_millihertz = speed_mm_per_second_to_millihertz(abs_speed_mm_per_second);

    motor_set_current_ma(motor, current_ma);
    // Make sure the motor is enabled
    int32_t err = tmc5160_motor_enable(motor->handle);
    if (err == ESP_OK) {
        err = tmc5160_set_acceleration(motor->handle, acceleration);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unable to set acceleration to %d (%s)!", acceleration,
                     esp_err_to_name(-err));
        }
        err = tmc5160_set_deceleration(motor->handle, deceleration);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "unable to set deceleration to %d (%s)!", deceleration,
                     esp_err_to_name(-err));
        }
        err = tmc5160_set_position(motor->handle, microsteps, abs_speed_millihertz);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "target height set to %d mm (%d microsteps),"
                    " max speed %d mm/second (%d milliHertz).", height_mm, microsteps,
                    abs_speed_mm_per_second, abs_speed_millihertz);
        } else {
            ESP_LOGE(TAG, "unable to set target height to %d mm (%d microsteps) (%s).",
                    height_mm, microsteps, esp_err_to_name(-err));
        }
    } else {
        ESP_LOGE(TAG, "unable to enable motor (%s).", esp_err_to_name(-err));
    }
}

// Wait for the motor to stop.
static bool motor_wait_until_stopped(launcher_motor_t *motor)
{
    bool motor_has_stopped = false;
    int32_t err = ESP_OK;
    int64_t start_us = esp_timer_get_time();

    while ((esp_timer_get_time() - start_us <  TMC5160_STOP_TIME_MS * 1000) && !motor_has_stopped && (err == ESP_OK)) {
        int32_t velocity = 0;
        err = tmc5160_get_velocity(motor->handle, &velocity);

        if (err == ESP_OK) {
            motor_has_stopped = (velocity == 0);
        } else {
            ESP_LOGW(TAG, "unable to read motor velocity (%s)!", esp_err_to_name(-err));
        }

        if (!motor_has_stopped && (err == ESP_OK)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (err == ESP_OK) {
        if (motor_has_stopped) {
            ESP_LOGI(TAG, "motor took %lld ms to stop.",  (esp_timer_get_time() - start_us) / 1000);
        } else {
            ESP_LOGW(TAG, "motor did not stop within %d ms!", TMC5160_STOP_TIME_MS);
        }
    }

    return motor_has_stopped;
}

// Stop the motor.
static void motor_stop(launcher_motor_t *motor, uint32_t current_ma, bool emergency)
{
    int32_t err = tmc5160_set_velocity(motor->handle, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "unable to stop the motor (%s).", esp_err_to_name(-err));
    }
    // If we're reducing the current, wait for the motor to stop
    // at the old current setting and then change it
    if (!emergency && (current_ma < motor->last_current_setting_ma)) {
        motor_wait_until_stopped(motor);
    }
    motor_set_current_ma(motor, current_ma);
    if (emergency) {
        err = tmc5160_motor_disable(motor->handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "unable to disable the motor (%s).", esp_err_to_name(-err));
        }
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: LAUNCHER (ACTUALLY DOING STUFF)
 * -------------------------------------------------------------- */

// Start a timer.
static void timer_start_us(launcher_timer_t *timer, int64_t duration_us)
{
    timer->start_us = esp_timer_get_time();
    timer->duration_us = duration_us;
}

// Check a timer for expiry.
static bool timer_has_expired(launcher_timer_t *timer)
{
    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_us = now_us - timer->start_us;
    return elapsed_us > timer->duration_us;
}

// Task callback to run auto mode.
static void auto_cb(void *handle, void *arg)
{
    launcher_t *launcher = (launcher_t *) arg;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    (void) handle;

    if (nvs_shadow->auto_enabled && timer_has_expired(&launcher->timer_auto)) {
        if (launcher->state == LAUNCHER_STATE_READY) {
            // Start the next run
            launcher_state_t state = LAUNCHER_STATE_RUN;
            xQueueSend(launcher->queue_state, &state, portMAX_DELAY);
            launcher->auto_period_seconds = nvs_shadow->auto_period_seconds;
            ESP_LOGI(TAG, "### Auto mode timer expired (next expiry in %d second(s)) ###",
                     launcher->auto_period_seconds);
        } else {
            // Couldn't start, try again in a shorter time
            launcher->auto_period_seconds = AUTO_MODE_RETRY_TIME_SECONDS;
            ESP_LOGI(TAG, "Unable to auto-run in state %s, retrying in %d second(s).",
                     g_launcher_state_name[launcher->state], launcher->auto_period_seconds);
        }
        // Set the next timeout
        timer_start_us(&launcher->timer_auto, ((int64_t) launcher->auto_period_seconds) * 1000000);
    }
}

// Print out the current height.
static void print_height(int32_t height_mm)
{
    char buffer[32] = "unknown";
    if (height_mm != POSITION_UNKNOWN) {
        snprintf(buffer, sizeof(buffer), "%d mm", (int) height_mm);
    }
    ESP_LOGI(TAG, "height %s.", buffer);
}

// Print out the current height based on the current position and the ground level.
static void print_height_from_position(int32_t position_mm, int32_t ground_level_mm)
{
    char buffer_position[32] = "unknown";
    char buffer_ground_level[32] = "unknown";

    if (position_mm != POSITION_UNKNOWN) {
        snprintf(buffer_position, sizeof(buffer_position), "%d mm", (int) position_mm);
        if (ground_level_mm != POSITION_UNKNOWN) {
            snprintf(buffer_ground_level, sizeof(buffer_ground_level), "%d mm", (int) (position_mm - ground_level_mm));
        }
    }
    ESP_LOGI(TAG, "position %s, height %s.", buffer_position, buffer_ground_level);
}

// Set the LED breathe colour according to its new state
static void set_led_breathe_colour(launcher_state_t state, bool emergency_stopped)
{
    if (emergency_stopped) {
        fgr_debug_led_breathe_set(DEBUG_LED_COLOUR_EMERGENCY_STOPPED);
    } else {
        if (state < FGR_UTIL_ARRAY_LENGTH((g_launcher_state_colour))) {
            fgr_debug_led_breathe_set(g_launcher_state_colour[state]);
        }
    }
}

// Advance state based on commanded state LAUNCHER_STATE_MOVE_UP.
static launcher_state_t advance_state_commanded_move_up(launcher_t *launcher,
                                                        int32_t height_mm,
                                                        int32_t height_max_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    if ((height_mm == POSITION_UNKNOWN) || (height_max_mm == POSITION_UNKNOWN) ||
        (height_mm < height_max_mm)) {
        // Set a timeout for our move
        timer_start_us(&launcher->timer_a, MOVE_UP_DOWN_DURATION_MAX_MS * 1000);
        launcher->direction_up_not_down = true;
        launcher->height_target_mm = POSITION_UNKNOWN;

        char buffer[32] = "unknown";
        if (height_mm != POSITION_UNKNOWN) {
            snprintf(buffer, sizeof(buffer), "%d mm", (int) height_mm);
        }
        ESP_LOGI(TAG, "height_mm %s, timeout %d ms.", buffer, MOVE_UP_DOWN_DURATION_MAX_MS);

        // Move up
        motor_set_speed(&launcher->motor, launcher->direction_up_not_down,
                        nvs_shadow->speed_mm_per_second, MOVE_UP_DOWN_SPEED_MODIFIER_PERCENT,
                        TMC5160_RUN_CURRENT_MA, TMC5160_ACCELERATION_NORMAL, TMC5160_DECELERATION_NORMAL);
        next_state = LAUNCHER_STATE_MOVE_UP;
    } else {
        ESP_LOGW(TAG, "ignoring move up, already at limit (step %d (%d mm)).",
                 height_mm, height_max_mm);
    }

    return next_state;
}

// Advance state based on commanded state LAUNCHER_STATE_MOVE_DOWN.
static launcher_state_t advance_state_commanded_move_down(launcher_t *launcher,
                                                          int32_t height_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Note: stepping down should work irrespective of ground level as this might
    // be an operation to reset where the ground level is
    timer_start_us(&launcher->timer_a, MOVE_UP_DOWN_DURATION_MAX_MS * 1000);
    launcher->direction_up_not_down = false;
    launcher->height_target_mm = POSITION_UNKNOWN;

    char buffer[32] = "unknown";
    if (height_mm != POSITION_UNKNOWN) {
        snprintf(buffer, sizeof(buffer), "%d mm", (int) height_mm);
    }
    ESP_LOGI(TAG, "height_mm %s, timeout %d ms.", buffer, MOVE_UP_DOWN_DURATION_MAX_MS);

    motor_set_speed(&launcher->motor, launcher->direction_up_not_down,
                    nvs_shadow->speed_mm_per_second, MOVE_UP_DOWN_SPEED_MODIFIER_PERCENT,
                    TMC5160_RUN_CURRENT_MA, TMC5160_ACCELERATION_NORMAL, TMC5160_DECELERATION_NORMAL);
    next_state = LAUNCHER_STATE_MOVE_DOWN;

    return next_state;
}

// Advance state based on commanded state LAUNCHER_STATE_RUN.
static launcher_state_t advance_state_commanded_run(launcher_t *launcher,
                                                    int32_t height_mm,
                                                    int32_t height_max_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;


    if (height_mm < height_max_mm) {
        // Set up the skittering audio for a duration
        uint32_t track_number = nvs_shadow->track_number_skittering;
        int64_t duration_us = 0;
        if (track_number < FGR_UTIL_ARRAY_LENGTH(g_track_duration_ms)) {
            duration_us = ((int64_t) g_track_duration_ms[track_number]) * 1000;
        }
        timer_start_us(&launcher->timer_a, duration_us);

        // Start audio
        if (!nvs_shadow->mute) {
            audio_play_track(track_number);
        }
        ESP_LOGI(TAG, "playing skittering noise for %lld second(s).", duration_us / 1000000);
        next_state = LAUNCHER_STATE_RUNNING_SKITTERING;
    } else {
        ESP_LOGW(TAG, "ignoring run, already at limit (height %d (max %d mm)).",
                 height_mm, height_max_mm);
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_SKITTERING.
static launcher_state_t advance_state_running_skittering(launcher_t *launcher,
                                                         int32_t height_mm,
                                                         int32_t height_max_mm,
                                                         int32_t ground_level_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Wait for the skittering time to pass
    if (timer_has_expired(&launcher->timer_a)) {
        // Prepare for waking
        launcher->temporary_stop = false;
        launcher->direction_up_not_down = true;
        launcher->waking_cycles_completed = 0;
        // Start waking with a sharp lift
        launcher->height_target_mm = WAKING_JIGGLE_HEIGHT_MM;
        if (launcher->height_target_mm > height_max_mm) {
            launcher->height_target_mm = height_max_mm;
        }

        // Enable sg_stop.
        uint32_t data;
        if (tmc5160_read(launcher->motor.handle, 0x34, &data) == ESP_OK) {
            data |= 1UL << 10;
            if (tmc5160_write(launcher->motor.handle, 0x34, data) == ESP_OK) {
               ESP_LOGI(TAG, "sg_stop ENABLED");
            }
        }

        ESP_LOGI(TAG, "waking: lifting to %d mm.", launcher->height_target_mm);
        motor_set_height_mm(&launcher->motor, launcher->height_target_mm,
                            nvs_shadow->speed_mm_per_second, ground_level_mm,
                            TMC5160_RUN_CURRENT_MA, TMC5160_ACCELERATION_NORMAL, TMC5160_DECELERATION_NORMAL);
        next_state = LAUNCHER_STATE_RUNNING_WAKING;
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_WAKING.
static launcher_state_t advance_state_running_waking(launcher_t *launcher,
                                                     int32_t height_mm,
                                                     int32_t height_max_mm,
                                                     int32_t ground_level_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    (void) height_max_mm;

    if (launcher->direction_up_not_down) {
        // Currently lifting the spider.
        if (height_mm >= launcher->height_target_mm) {
            // Reached the top: return to ground level.
            launcher->direction_up_not_down = false;
            launcher->height_target_mm = 0;
            ESP_LOGI(TAG, "waking: lowering to ground.");
            motor_set_height_mm(&launcher->motor,
                                0,
                                WAKING_SPEED_MM_PER_SECOND,
                                ground_level_mm,
                                TMC5160_RUN_CURRENT_MA,
                                WAKING_ACCELERATION,
                                WAKING_DECELERATION);
        }
    } else {
        // Currently lowering the spider.
        if (height_mm <= launcher->height_target_mm) {
            // Reached ground level: one complete jiggle.
            if (!launcher->temporary_stop) {
                launcher->waking_cycles_completed++;
                ESP_LOGI(TAG, "waking: completed jiggle %d of %d.",
                        launcher->waking_cycles_completed, WAKING_JIGGLE_CYCLES);
            }
            if (launcher->waking_cycles_completed < WAKING_JIGGLE_CYCLES) {
                if (launcher->temporary_stop) {
                    if (timer_has_expired(&launcher->timer_a)) {
                        // Another jiggle: lift again.
                        launcher->temporary_stop = false;
                        launcher->direction_up_not_down = true;
                        launcher->height_target_mm = WAKING_JIGGLE_HEIGHT_MM;
                        if (launcher->height_target_mm > height_max_mm) {
                            launcher->height_target_mm = height_max_mm;
                        }
                        ESP_LOGI(TAG, "waking: lifting to %d mm.",
                                launcher->height_target_mm);
                        motor_set_height_mm(&launcher->motor,
                                            launcher->height_target_mm,
                                            WAKING_SPEED_MM_PER_SECOND,
                                            ground_level_mm,
                                            TMC5160_RUN_CURRENT_MA,
                                            WAKING_ACCELERATION,
                                            WAKING_DECELERATION);
                    }
                } else {
                    timer_start_us(&launcher->timer_a,
                                   WAKING_SETTLE_TIME_MS * 1000);
                    launcher->temporary_stop = true;
                }
            } else {
                // Waking complete: prepare for random movements.
                timer_start_us(&launcher->timer_a,
                               RANDOM_PAUSE_INTERVAL_MS * 1000);
                timer_start_us(&launcher->timer_b,
                               RANDOM_PAUSE_TIME_MS * 1000);
                launcher->temporary_stop = false;
                // Start jumping.
                launcher->height_target_mm = height_max_mm;
                launcher->direction_up_not_down = true;
                // Start audio
                if (!nvs_shadow->mute) {
                    audio_play_track(nvs_shadow->track_number_jumping);
                }
                ESP_LOGI(TAG, "height_mm %d, height_target_mm %d%s.",
                         height_mm, launcher->height_target_mm,
                         nvs_shadow->random_enabled ?
                         " (random is enabled)" : "");
                motor_set_height_mm(&launcher->motor,
                                    launcher->height_target_mm,
                                    nvs_shadow->speed_mm_per_second,
                                    ground_level_mm,
                                    TMC5160_RUN_CURRENT_MA,
                                    TMC5160_ACCELERATION_ABRUPT,
                                    TMC5160_DECELERATION_ABRUPT);
                next_state = LAUNCHER_STATE_RUNNING_JUMPING;
            }
        }
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_JUMPING.
static launcher_state_t advance_state_running_jumping(launcher_t *launcher,
                                                      int32_t height_mm,
                                                      int32_t height_max_mm,
                                                      int32_t ground_level_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    if (height_mm < launcher->height_target_mm) {
        if (nvs_shadow->random_enabled) {
            // Stop randomly to make the rise jerky
            if (!launcher->temporary_stop && timer_has_expired(&launcher->timer_b)) {
                if (esp_random() < UINT32_MAX / 2) {
                    ESP_LOGI(TAG, "random stop for %d ms.", RANDOM_PAUSE_TIME_MS);
                    timer_start_us(&launcher->timer_a, RANDOM_PAUSE_TIME_MS * 1000);
                    motor_stop(&launcher->motor, TMC5160_RUN_CURRENT_MA, false);
                    launcher->temporary_stop = true;
                } else {
                    ESP_LOGI(TAG, "won't stop randomly for %d ms.", RANDOM_PAUSE_INTERVAL_MS);
                    ESP_LOGI(TAG, "height_mm %d, height_target_mm %d.", height_mm, launcher->height_target_mm);
                    timer_start_us(&launcher->timer_b, RANDOM_PAUSE_INTERVAL_MS * 1000);
                }
            }
            if (launcher->temporary_stop && timer_has_expired(&launcher->timer_a)) {
                ESP_LOGI(TAG, "restarting after random stop, won't stop again for %d ms.", RANDOM_PAUSE_INTERVAL_MS);
                ESP_LOGI(TAG, "height_mm %d, height_target_mm %d.", height_mm, launcher->height_target_mm);
                motor_set_height_mm(&launcher->motor, launcher->height_target_mm,
                                    nvs_shadow->speed_mm_per_second, ground_level_mm,
                                    TMC5160_RUN_CURRENT_MA, TMC5160_ACCELERATION_ABRUPT, TMC5160_DECELERATION_ABRUPT);
                timer_start_us(&launcher->timer_b, RANDOM_PAUSE_INTERVAL_MS * 1000);
                launcher->temporary_stop = false;
            }
        }
    } else {
        // Done.
        launcher->height_target_mm = POSITION_UNKNOWN;
        motor_stop(&launcher->motor, TMC5160_RUN_CURRENT_MA,  false);
        print_height(height_mm);
        timer_start_us(&launcher->timer_a, DANGLE_TIME_SECONDS * 1000000);
        // Start audio
        if (!nvs_shadow->mute) {
            audio_play_track(nvs_shadow->track_number_dangling);
        }
        ESP_LOGI(TAG, "dangling for %d second(s).", DANGLE_TIME_SECONDS);
        next_state = LAUNCHER_STATE_RUNNING_DANGLING;
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_DANGLING.
static launcher_state_t advance_state_running_dangling(launcher_t *launcher,
                                                       int32_t height_mm,
                                                       int32_t ground_level_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Wait for the dangle time to pass
    if (timer_has_expired(&launcher->timer_a)) {
        // Done.
        ESP_LOGI(TAG, "resetting.");
        // Start audio
        if (!nvs_shadow->mute) {
            audio_play_track(nvs_shadow->track_number_resetting);
        }
        launcher->height_target_mm = 0;
        launcher->direction_up_not_down = false;
        ESP_LOGI(TAG, "height_mm %d, height_target_mm %d.", height_mm, launcher->height_target_mm);
        motor_set_height_mm(&launcher->motor, launcher->height_target_mm,
                            nvs_shadow->speed_mm_per_second * RESETTING_SPEED_MODIFIER_PERCENT / 100,
                            ground_level_mm, TMC5160_RUN_CURRENT_MA, TMC5160_ACCELERATION_NORMAL, RESETTING_DECELERATION);
        next_state = LAUNCHER_STATE_RUNNING_RESETTING;
    }

    return next_state;
}

// Advance state based on state LAUNCHER_STATE_RUNNING_RESETTING.
static launcher_state_t advance_state_running_resetting(launcher_t *launcher,
                                                        int32_t height_mm)
{
    launcher_state_t next_state = LAUNCHER_STATE_NULL;

    if ((height_mm != POSITION_UNKNOWN) && (height_mm <= launcher->height_target_mm)) {
        // Done.
        launcher->height_target_mm = POSITION_UNKNOWN;
        motor_stop(&launcher->motor, TMC5160_IDLE_CURRENT_MA, false);
        print_height(height_mm);
        next_state = LAUNCHER_STATE_POSITION_UNKNOWN;
    }

    return next_state;
}

// Advance state as appropriate; only allowed states should be passed
// in via commanded_state.
// This function _returns_ the next state, it does not change the
// current state, it is up to the caller to do that.
// IMPORTANT: the context should be locked before this is called.
static launcher_state_t advance_state(launcher_t *launcher, launcher_state_t commanded_state,
                                      retained_ram_t *retained_ram)
{
    launcher_motor_t *motor = &launcher->motor;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Use a signed value here so that negative indicates not known
    int32_t position_mm = motor_get_position_mm(motor);
    int32_t height_mm = POSITION_UNKNOWN;
    int32_t ground_level_mm = POSITION_UNKNOWN;
    if (retained_ram && (position_mm != POSITION_UNKNOWN)) {
        ground_level_mm = (int32_t) retained_ram->ground_level_mm;
        if (ground_level_mm != POSITION_UNKNOWN) {
            height_mm = position_mm - ground_level_mm;
        }
    }
    int32_t height_max_mm = nvs_shadow->height_max_mm;
    // When an up or down move, or a halt, is over, reduce the
    // current if we're at ground level
    uint32_t up_down_final_current_ma = TMC5160_RUN_CURRENT_MA;
    if ((height_mm != POSITION_UNKNOWN) && (height_mm <= 0)) {
        up_down_final_current_ma = TMC5160_IDLE_CURRENT_MA;
    }

    launcher_state_t next_state = LAUNCHER_STATE_NULL;
    if (commanded_state == LAUNCHER_STATE_HALT) {
        // A command to halt overrides everything else
        motor_stop(&launcher->motor, up_down_final_current_ma, retained_ram->emergency_stopped);
        next_state = commanded_state;
    } else {
        // Handle the current state
        switch(launcher->state) {
            case LAUNCHER_STATE_NULL:
                next_state = LAUNCHER_STATE_POSITION_UNKNOWN;
            break;
            case LAUNCHER_STATE_POSITION_UNKNOWN:
                // If we know where the ground is we can transition
                // to ready state
                if ((ground_level_mm != POSITION_UNKNOWN) && (height_mm != POSITION_UNKNOWN)) {
                    next_state = LAUNCHER_STATE_READY;
                } else {
                    // From height unknown state we can only do steps
                    switch (commanded_state) {
                        case LAUNCHER_STATE_MOVE_UP:
                            next_state = advance_state_commanded_move_up(launcher,
                                                                         height_mm,
                                                                         height_max_mm);
                        break;
                        case LAUNCHER_STATE_MOVE_DOWN:
                            next_state = advance_state_commanded_move_down(launcher, height_mm);
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
                    case LAUNCHER_STATE_MOVE_UP:
                        next_state = advance_state_commanded_move_up(launcher,
                                                                     height_mm,
                                                                     height_max_mm);
                    break;
                    case LAUNCHER_STATE_MOVE_DOWN:
                        next_state = advance_state_commanded_move_down(launcher, height_mm);
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
            case LAUNCHER_STATE_MOVE_UP:
                if (commanded_state == LAUNCHER_STATE_MOVE_UP) {
                    // We've had another move up command, increase the timer
                    timer_start_us(&launcher->timer_a, MOVE_UP_DOWN_DURATION_MAX_MS * 1000);
                }
                if (timer_has_expired(&launcher->timer_a) ||
                    ((height_mm != POSITION_UNKNOWN) && (height_max_mm != POSITION_UNKNOWN) &&
                    (height_mm >= height_max_mm))) {
                    // Done.
                    launcher->height_target_mm = POSITION_UNKNOWN;
                    motor_stop(&launcher->motor, up_down_final_current_ma, false);
                    print_height_from_position(position_mm, ground_level_mm);
                    next_state = LAUNCHER_STATE_POSITION_UNKNOWN;
                }
            break;
            case LAUNCHER_STATE_MOVE_DOWN:
                if (commanded_state == LAUNCHER_STATE_MOVE_DOWN) {
                    // We've had another move down command, increase the timer
                    timer_start_us(&launcher->timer_a, MOVE_UP_DOWN_DURATION_MAX_MS * 1000);
                }
                if (timer_has_expired(&launcher->timer_a)) {
                    // Done.
                    // Note: stepping down should work irrespective of ground level as this might
                    // be an operation to reset where the ground level is
                    launcher->height_target_mm = POSITION_UNKNOWN;
                    motor_stop(&launcher->motor, up_down_final_current_ma, false);
                    print_height_from_position(position_mm, ground_level_mm);
                    next_state = LAUNCHER_STATE_POSITION_UNKNOWN;
                }
            break;
            case LAUNCHER_STATE_RUN:
                next_state = advance_state_commanded_run(launcher,
                                                         height_mm,
                                                         height_max_mm);
            break;
            case LAUNCHER_STATE_RUNNING_SKITTERING:
                next_state = advance_state_running_skittering(launcher,
                                                              height_mm,
                                                              height_max_mm,
                                                              ground_level_mm);
            break;
            case LAUNCHER_STATE_RUNNING_WAKING:
                next_state = advance_state_running_waking(launcher,
                                                          height_mm,
                                                          height_max_mm,
                                                          ground_level_mm);
            break;
            case LAUNCHER_STATE_RUNNING_JUMPING:
                next_state = advance_state_running_jumping(launcher,
                                                           height_mm,
                                                           height_max_mm,
                                                           ground_level_mm);
            break;
            case LAUNCHER_STATE_RUNNING_DANGLING:
                next_state = advance_state_running_dangling(launcher,
                                                            height_mm,
                                                            ground_level_mm);
            break;
            case LAUNCHER_STATE_RUNNING_RESETTING:
                next_state = advance_state_running_resetting(launcher, height_mm);
            break;
            case LAUNCHER_STATE_HALT:
                next_state = LAUNCHER_STATE_POSITION_UNKNOWN;
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

    bool emergency_stopped = false;
    retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        emergency_stopped = retained_ram.emergency_stopped;
    }

    // Get a commanded state from the queue
    launcher_state_t commanded_state = LAUNCHER_STATE_NULL;
    while (xQueueReceive(launcher->queue_state, &commanded_state, 0) == pdTRUE) {

        const char *name = "UNKNOWN";
        if (commanded_state < FGR_UTIL_ARRAY_LENGTH(g_launcher_state_name)) {
            name = g_launcher_state_name[commanded_state];
        }

        if (!emergency_stopped || (commanded_state == LAUNCHER_STATE_HALT)) {
            // Check if the commanded state is allowed
            switch (commanded_state) {
                case LAUNCHER_STATE_NULL:
                case LAUNCHER_STATE_POSITION_UNKNOWN:
                case LAUNCHER_STATE_READY:
                case LAUNCHER_STATE_RUNNING_SKITTERING:
                case LAUNCHER_STATE_RUNNING_WAKING:
                case LAUNCHER_STATE_RUNNING_JUMPING:
                case LAUNCHER_STATE_RUNNING_RESETTING:
                    // These states can only be entered through internal
                    // advancement, they cannot be commanded by the user
                    ESP_LOGW(TAG, "ignoring command to enter state %s (%d).", name, commanded_state);
                    commanded_state = LAUNCHER_STATE_NULL;
                    fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
                break;
                case LAUNCHER_STATE_MOVE_UP:
                case LAUNCHER_STATE_MOVE_DOWN:
                    // Can do these if we don't know our height or are ready
                    if ((launcher->state == LAUNCHER_STATE_READY) ||
                        (launcher->state == LAUNCHER_STATE_POSITION_UNKNOWN)) {
                        ESP_LOGI(TAG, "got %s.", name);
                    } else {
                        // Can also do these again if we were already doing them
                        if (launcher->state == commanded_state) {
                            ESP_LOGI(TAG, "continuing %s.", name);
                        } else {
                            ESP_LOGW(TAG, "cannot get to state %s from state %s (only from states %s and %s).",
                                     name, g_launcher_state_name[launcher->state],
                                     g_launcher_state_name[LAUNCHER_STATE_POSITION_UNKNOWN],
                                     g_launcher_state_name[LAUNCHER_STATE_READY]);
                            commanded_state = LAUNCHER_STATE_NULL;
                            fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
                        }
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
                        fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
                    }
                break;
                case LAUNCHER_STATE_HALT:
                    // Always handle this
                    ESP_LOGI(TAG, "received %s.", name);
                break;
                default:
                    ESP_LOGE(TAG, "commanded to enter unknown state (%d)!", commanded_state);
                    commanded_state = LAUNCHER_STATE_NULL;
                    fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
                break;
            }
        } else {
            ESP_LOGI(TAG, "ignoring command to enter state %s as we are emergency stopped.", name);
            commanded_state = LAUNCHER_STATE_NULL;
            fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
        }
        if (commanded_state != LAUNCHER_STATE_NULL) {
            // Move state on based on internal events or the new commanded_state
            launcher_state_t next_state = advance_state(launcher, commanded_state, &retained_ram);
            if (next_state != LAUNCHER_STATE_NULL) {
                ESP_LOGI(TAG, "state transition due to commanded state %s, %s -> %s.", name,
                         g_launcher_state_name[launcher->state], g_launcher_state_name[next_state]);
                set_led_breathe_colour(next_state, emergency_stopped);
                launcher->state = next_state;
            }
        }
    }

    // Make sure we advance state at least once in case there are any internal changes
    launcher_state_t next_state = advance_state(launcher, LAUNCHER_STATE_NULL, &retained_ram);
    if (next_state != LAUNCHER_STATE_NULL) {
        ESP_LOGI(TAG, "internal state transition %s -> %s.",
                 g_launcher_state_name[launcher->state], g_launcher_state_name[next_state]);
        set_led_breathe_colour(next_state, emergency_stopped);
        launcher->state = next_state;
    }

    CONTEXT_UNLOCK(context->lock, "launcher_cb");
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: COMMAND HANDLERS
 * -------------------------------------------------------------- */

// Handle the "launch now" command.
static void handler_launch_now(launcher_t *launcher, uint32_t unused, bool read_not_write)
{
    (void) unused;
    (void) read_not_write;

    // Queue the command to the launcher to switch to running state
    launcher_state_t state = LAUNCHER_STATE_RUN;
    xQueueSend(launcher->queue_state, &state, portMAX_DELAY);
}

// Handle the "up" command.
static void handler_up(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    launcher_state_t state = LAUNCHER_STATE_HALT;

    (void) read_not_write;

    if (enable) {
        state = LAUNCHER_STATE_MOVE_UP;
    }
    // Queue the command to the launcher
    xQueueSend(launcher->queue_state, &state, portMAX_DELAY);
}

// Handle the "down" command.
static void handler_down(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    launcher_state_t state = LAUNCHER_STATE_HALT;

    (void) read_not_write;

    if (enable) {
        state = LAUNCHER_STATE_MOVE_DOWN;
    }
    // Queue the command to the launcher
    xQueueSend(launcher->queue_state, &state, portMAX_DELAY);

}

// Handle the "this is ground level" command.
static void handler_this_is_ground_level(launcher_t *launcher, uint32_t unused, bool read_not_write)
{
    launcher_motor_t *motor = &launcher->motor;

    (void) unused;
    (void) read_not_write;

    // Set the groud level step to the current position of the TMC5160
    retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        // We are now establishing height according to the
        // TMC5160 so acknowledge the reset flag over there, just
        // in case
        int32_t err = tmc5160_acknowledge_reset(motor->handle);
        if (err == ESP_OK) {
            int32_t microsteps;
            err = tmc5160_get_position(motor->handle, &microsteps);
            if (err == ESP_OK) {
                retained_ram.ground_level_mm = microsteps_to_mm(microsteps);
                FGR_RRAM_SET(retained_ram);
            } else {
                ESP_LOGW(TAG, "failed to get position from TMC5160, cannot set ground level.");
            }
        } else {
            ESP_LOGW(TAG, "unable to acknowledge reset in TMC5160, cannot set ground level.");
        }
    }
}

// Handle the "this is height max" command.
static void handler_this_is_height_max(launcher_t *launcher, uint32_t unused, bool read_not_write)
{
    launcher_motor_t *motor = &launcher->motor;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    (void) unused;
    (void) read_not_write;

    // Get the current position from the TMC5160
    retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
    if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
        (retained_ram.ground_level_mm != POSITION_UNKNOWN)) {
        int32_t microsteps;
        int32_t err = tmc5160_get_position(motor->handle, &microsteps);
        if ((err == ESP_OK) && !tmc5160_has_reset(motor->handle)) {
            // Convert to a height in mm
            int32_t height_mm = microsteps_to_mm(microsteps);
            if ((height_mm < POSITION_UNKNOWN) && (height_mm > retained_ram.ground_level_mm)) {
                // Set the "height max" value in NVS
                nvs_height_max_mm_set(nvs_shadow,
                                    (height_mm - retained_ram.ground_level_mm));
            } else {
                ESP_LOGW(TAG, "not setting height max, height returned by TMC5160()"
                         " (%d mm, %d microsteps, with ground at %d mm) out of range.",
                         height_mm, microsteps, retained_ram.ground_level_mm);
            }
        } else {
            ESP_LOGW(TAG, "failed to get position from TMC5160, cannot set height.");
        }
    }
}

// Handle the "reset to defaults" command.
static void handler_reset_to_defaults(launcher_t *launcher, uint32_t unused, bool read_not_write)
{
    (void) unused;
    (void) read_not_write;

    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Write the default settings to NVS
    nvs_auto_enable_set(nvs_shadow, DEFAULT_AUTO_ENABLE);
    nvs_auto_period_seconds_set(nvs_shadow, DEFAULT_AUTO_PERIOD_SECONDS);
    nvs_height_max_mm_set(nvs_shadow, DEFAULT_HEIGHT_MAX_MM);
    nvs_speed_mm_per_second_set(nvs_shadow, DEFAULT_SPEED_MM_PER_SECOND);
    nvs_track_number_skittering_set(nvs_shadow, DEFAULT_TRACK_NUMBER_SKITTERING);
    nvs_track_number_jumping_set(nvs_shadow, DEFAULT_TRACK_NUMBER_JUMPING);
    nvs_track_number_dangling_set(nvs_shadow, DEFAULT_TRACK_NUMBER_DANGLING);
    nvs_track_number_resetting_set(nvs_shadow, DEFAULT_TRACK_NUMBER_RESETTING);
    nvs_random_enable_set(nvs_shadow, DEFAULT_RANDOM_ENABLE);
    nvs_mute_set(nvs_shadow, DEFAULT_MUTE);

    // Reset the auto timer directly
    launcher->auto_period_seconds = DEFAULT_AUTO_PERIOD_SECONDS;
    timer_start_us(&launcher->timer_auto, ((int64_t) launcher->auto_period_seconds) * 1000000);

    // Reset the current step and set ground level to unknown
    // but leave emergency stopped alone
    retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        retained_ram.ground_level_mm = POSITION_UNKNOWN;
        FGR_RRAM_SET(retained_ram);
    }

    // Set the launcher state back to NULL
    launcher->state = LAUNCHER_STATE_NULL;
}

// Handle the "emergency stop enable" command.
static void handler_emergency_stop_enable(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    // Set the state in retained RAM
    retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
    bool previous_stopped_state = false;
    if (FGR_RRAM_GET(retained_ram) == ESP_OK) {
        previous_stopped_state = retained_ram.emergency_stopped;
        retained_ram.emergency_stopped = enable;
        // The motor will be off after an emergency stop so
        // the spider may sag, ground level is no longer known
        if (enable) {
            retained_ram.ground_level_mm = POSITION_UNKNOWN;
        }
        FGR_RRAM_SET(retained_ram);
    }
    // Either kick the state machine back into life or halt it
    if (enable) {
        FGR_RRAM_SET(retained_ram);
        launcher_state_t state = LAUNCHER_STATE_HALT;
        xQueueSend(launcher->queue_state, &state, portMAX_DELAY);
    } else {
        if (previous_stopped_state == true) {
            // Directly set the launcher back into NULL state
            launcher->state = LAUNCHER_STATE_NULL;
        }
    }
}

// Handle the "play track" command.
static void handler_play_track(launcher_t *launcher, uint32_t track_number, bool read_not_write)
{
    (void) launcher;
    (void) read_not_write;

    // Play the track
    audio_play_track(track_number);
}

// Handle the "auto enable" command.
static void handler_auto_enable(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_auto_enable_set(&launcher->nvs_shadow, enable ? true : false);
        if (enable) {
            // Set the timer to a short period so that the change takes effect quickly
            launcher->auto_period_seconds = AUTO_MODE_RETRY_TIME_SECONDS;
            timer_start_us(&launcher->timer_auto, ((int64_t) launcher->auto_period_seconds) * 1000000);
        }
    }
}

// Handle the "auto period" command.
static void handler_auto_period_seconds(launcher_t *launcher, uint32_t period, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_auto_period_seconds_set(&launcher->nvs_shadow, period);
        // Reset the timer
        launcher->auto_period_seconds = period;
        timer_start_us(&launcher->timer_auto, ((int64_t) launcher->auto_period_seconds) * 1000000);
    }
}

// Handle the "height max" command.
static void handler_height_max_mm(launcher_t *launcher, uint32_t height, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_height_max_mm_set(&launcher->nvs_shadow, height);
    }
}

// Handle the "speed" command.
static void handler_speed_mm_per_second(launcher_t *launcher, uint32_t speed, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_speed_mm_per_second_set(&launcher->nvs_shadow, speed);
    }
}

// Handle the "track number skittering" command.
static void handler_track_number_skittering(launcher_t *launcher, uint32_t track_number, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_track_number_skittering_set(&launcher->nvs_shadow, track_number);
    }
}

// Handle the "track number jumping" command.
static void handler_track_number_jumping(launcher_t *launcher, uint32_t track_number, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_track_number_jumping_set(&launcher->nvs_shadow, track_number);
    }
}

// Handle the "track number dangling" command.
static void handler_track_number_dangling(launcher_t *launcher, uint32_t track_number, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_track_number_dangling_set(&launcher->nvs_shadow, track_number);
    }
}

// Handle the "track number resetting" command.
static void handler_track_number_resetting(launcher_t *launcher, uint32_t track_number, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_track_number_resetting_set(&launcher->nvs_shadow, track_number);
    }
}

// Handle the "random enable" command.
static void handler_random_enable(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_random_enable_set(&launcher->nvs_shadow, enable ? true : false);
    }
}

// Handle the "mute" command.
static void handler_mute(launcher_t *launcher, uint32_t enable, bool read_not_write)
{
    if (!read_not_write) {
        // Write the new setting to NVS
        nvs_mute_set(&launcher->nvs_shadow, enable ? true : false);
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: CHARACTERISTICS CALLBACKS (CALLED BY BLE)
 * -------------------------------------------------------------- */

// ##################################################################
// The characteristic callbacks here are for BLE_INTERFACE_VERSION 2.
// Should any breaking changes be made, please incrememnt the version
// number and make any accompanyying changes in the Android
// application.
// ##################################################################

// Determine whether the launcher is in a state where the height
// related parameters can be reset.
static bool can_reset_height_parameters(launcher_state_t launcher_state)
{
    bool can_reset = false;

    if ((launcher_state == LAUNCHER_STATE_NULL) ||
        (launcher_state == LAUNCHER_STATE_POSITION_UNKNOWN) ||
        (launcher_state == LAUNCHER_STATE_READY)) {
        can_reset = true;
    }

    return can_reset;
}

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
            // Queue the command
            xQueueSend(context->command_queue, command_contents, portMAX_DELAY);

            // Send empty success response
            os_mbuf_free_chain(ctxt->om);
            ctxt->om = ble_hs_mbuf_from_flat("", 0);
            return_code = 0;
        }
        break;
        case BLE_GATT_ACCESS_OP_READ_CHR:
        {
            // Queue the command, purely for information
            xQueueSend(context->command_queue, command_contents, portMAX_DELAY);

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

    if (return_code == 0) {
        fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_WS2812_LED_COLOUR_BLUE);
    } else {
        fgr_debug_led_flash(FGR_DEBUG_LED_SHORT_MS, FGR_DEBUG_LED_COLOUR_BAD);
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
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
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
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
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

// "Emergency stop enable" characteristic callback.
static int characteristic_emergency_stop_enable_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code = BLE_ATT_ERR_UNLIKELY;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_emergency_stop_enable_cb");

    command_contents_t command_contents = {.command = COMMAND_EMERGENCY_STOP_ENABLE,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting from retained RAM
        retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
        FGR_RRAM_GET(retained_ram);
        command_contents.value = retained_ram.emergency_stopped ? 1 : 0;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }

    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_emergency_stop_enable_cb");

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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->auto_enabled ? 1 : 0;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->auto_period_seconds;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->height_max_mm;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->speed_mm_per_second;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
        if (command_contents.value < SPEED_MIN_MM_PER_SECOND) {
            command_contents.value = SPEED_MIN_MM_PER_SECOND;
        }
        if (command_contents.value > SPEED_MAX_MM_PER_SECOND) {
            command_contents.value = SPEED_MAX_MM_PER_SECOND;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->track_number_skittering;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_skttering_cb");

    return return_code;
}

// "Track number jumping" characteristic callback.
static int characteristic_track_number_jumping_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                  struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_jumping_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_JUMPING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->track_number_jumping;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_jumping_cb");

    return return_code;
}

// "Track number dangling" characteristic callback.
static int characteristic_track_number_dangling_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                   struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_dangling_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_DANGLING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->track_number_dangling;
    } else {
        // Read the value sent with the BLE command
        command_contents.value = mbuf_read(ctxt->om);
    }
    return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);

    CONTEXT_UNLOCK(context->lock, "characteristic_track_number_dangling_cb");

    return return_code;
}

// "Track number resetting" characteristic callback.
static int characteristic_track_number_resetting_cb(uint16_t conn_handle, uint16_t attr_handle,
                                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int return_code;
    context_t *context = (context_t *) arg;

    CONTEXT_LOCK(context->lock, "characteristic_track_number_resetting_cb");

    command_contents_t command_contents = {.command = COMMAND_TRACK_NUMBER_RESETTING,
                                           .read_not_write = (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)};
    if (command_contents.read_not_write) {
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->track_number_resetting;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->random_enabled ? 1 : 0;
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
        // Read the current setting from the shadow
        launcher_t *launcher = &context->launcher;
        nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;
        command_contents.value = nvs_shadow->mute ? 1 : 0;
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
    launcher_t *launcher = &context->launcher;
    launcher_motor_t *motor = &launcher->motor;

    CONTEXT_LOCK(context->lock, "characteristic_height_current_mm_cb");

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        command_contents_t command_contents = {.command = COMMAND_HEIGHT_CURRENT_MM,
                                               .value = (uint32_t) -1};
        // Read the current position from the TMC5160
        int32_t microsteps;
        int32_t err = tmc5160_get_position(motor->handle, &microsteps);
        if ((err == ESP_OK) && !tmc5160_has_reset(motor->handle)) {
            // Convert to a height in mm
            int32_t height_mm = microsteps_to_mm(microsteps);
            retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
            if ((FGR_RRAM_GET(retained_ram) == ESP_OK) &&
                (retained_ram.ground_level_mm!= POSITION_UNKNOWN)) {
                // Populate the current height if we know where the ground is
                command_contents.value = height_mm - retained_ram.ground_level_mm;
            }
            return_code = queue_command(conn_handle, attr_handle, ctxt, context, &command_contents);
        }
    }

    CONTEXT_UNLOCK(context->lock, "characteristic_height_current_mm_cb");

    return return_code;
}

/* ----------------------------------------------------------------
 * VARIABLES FOR COMMAND_HANDLING
 * -------------------------------------------------------------- */

// ##################################################################
// The characteristics here are for BLE_INTERFACE_VERSION 2.
// Should any breaking changes be made, please incrememnt the version
// number and make any accompanyying changes in the Android
// application.
// ##################################################################

// Entries are in the same order as command_t, must
// have the same number of entries as command_t.
static const command_data_t g_command_data_list[] = {{.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE1,
                                                      .name = "LAUNCH_NOW",
                                                      .characteristic_cb = characteristic_launch_now_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                      .handler = handler_launch_now
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_BOOLEAN,
                                                      .uuid = 0xFFE2,
                                                      .name = "UP",
                                                      .characteristic_cb = characteristic_up_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                      .handler = handler_up
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_BOOLEAN,
                                                      .uuid = 0xFFE3,
                                                      .name = "DOWN",
                                                      .characteristic_cb = characteristic_down_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                      .handler = handler_down
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE4,
                                                      .name = "THIS_IS_GROUND_LEVEL",
                                                      .characteristic_cb = characteristic_this_is_ground_level_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                     .handler = handler_this_is_ground_level
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE5,
                                                      .name = "THIS_IS_HEIGHT_MAX",
                                                      .characteristic_cb = characteristic_this_is_height_max_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                      .handler = handler_this_is_height_max
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_NO_VALUE,
                                                      .uuid = 0xFFE6,
                                                      .name = "RESET_TO_DEFAULTS",
                                                      .characteristic_cb = characteristic_reset_to_defaults_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
                                                      .handler = handler_reset_to_defaults
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFE7,
                                                      .name = "EMERGENCY_STOP_ENABLE",
                                                      .characteristic_cb = characteristic_emergency_stop_enable_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_emergency_stop_enable
                                                     },
                                                     {.type = COMMAND_TYPE_WRITE_ONLY_UINT32,
                                                      .uuid = 0xFFE8,
                                                      .name = "PLAY TRACK",
                                                      .characteristic_cb = characteristic_play_track_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE_NO_RSP,
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
                                                      .name = "COMMAND_TRACK_NUMBER_DANGLING",
                                                      .characteristic_cb = characteristic_track_number_dangling_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_track_number_dangling
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_UINT32,
                                                      .uuid = 0xFFF0,
                                                      .name = "COMMAND_TRACK_NUMBER_RESETTING",
                                                      .characteristic_cb = characteristic_track_number_resetting_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_track_number_resetting
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFF1,
                                                      .name = "RANDOM",
                                                      .characteristic_cb = characteristic_random_enable_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_random_enable
                                                     },
                                                     {.type = COMMAND_TYPE_READ_WRITE_BOOLEAN,
                                                      .uuid = 0xFFF2,
                                                      .name = "MUTE",
                                                      .characteristic_cb = characteristic_mute_cb,
                                                      .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_READ,
                                                      .handler = handler_mute
                                                     },
                                                     {.type = COMMAND_TYPE_READ_ONLY,
                                                      .uuid = 0xFFF3,
                                                      .name = "STATE",
                                                      .characteristic_cb = characteristic_state_cb,
                                                      .flags = BLE_GATT_CHR_F_READ
                                                      // No handler for this one
                                                     },
                                                     {.type = COMMAND_TYPE_READ_ONLY,
                                                      .uuid = 0xFFF4,
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
                // "Emergency stop enable" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_EMERGENCY_STOP_ENABLE].uuid),
                .access_cb = g_command_data_list[COMMAND_EMERGENCY_STOP_ENABLE].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_EMERGENCY_STOP_ENABLE].flags
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
                // "Track number skittering" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_SKITTERING].flags
            },
            {
                // "Track number jumping" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_JUMPING].flags
            },
            {
                // "Track number dangling" characteristic
                .uuid = BLE_UUID16_DECLARE(g_command_data_list[COMMAND_TRACK_NUMBER_DANGLING].uuid),
                .access_cb = g_command_data_list[COMMAND_TRACK_NUMBER_DANGLING].characteristic_cb,
                .arg = &g_context,
                .flags = g_command_data_list[COMMAND_TRACK_NUMBER_DANGLING].flags
            },
            {
                // "Track number resetting" characteristic
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
                case COMMAND_TYPE_WRITE_ONLY_BOOLEAN:
                    ESP_LOGI(TAG, "command %s (0x%04x), write: %s.",
                                name, uuid, value ? "enable" : "disable");
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
                handler(&context->launcher, value, command_contents.read_not_write);
            }

        } else {
            ESP_LOGE(TAG, "unknown command (%d)!", command);
        }
    }

    CONTEXT_UNLOCK(context->lock, "command_cb");
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: INITIALISATION
 * -------------------------------------------------------------- */

// Initialisation.
static int32_t init(context_t *context)
{
    launcher_t *launcher = &context->launcher;
    launcher_motor_t *motor = &launcher->motor;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    // Allow us to feed the watchdog
    esp_task_wdt_add(NULL);

    // Create mutex to guard the context
    int32_t err = -ESP_ERR_NO_MEM;
    context->lock = xSemaphoreCreateMutex();
    if (context->lock) {
        err = ESP_OK;
    }

    if (err == ESP_OK) {

        CONTEXT_LOCK(context->lock, "init");

        // Initialise tasking, FGR style
        if (err == ESP_OK) {
            err = fgr_task_init();
        }

        // Initialise the TMC5160 stepper motor controller
        if (err == ESP_OK) {
            err = tmc5160_init(CONFIG_TMC5160_SPI_NUM, CONFIG_TMC5160_SPI_SCK_PIN,
                               CONFIG_TMC5160_SPI_MOSI_PIN, CONFIG_TMC5160_SPI_MISO_PIN,
                               CONFIG_TMC5160_SPI_CS_PIN, CONFIG_TMC5160_SPI_CLOCK_HZ);
            if (err == ESP_OK) {
                err = -ESP_ERR_NO_MEM;
                motor->handle = tmc5160_create(TMC5160_ADDRESS,
                                               CONFIG_TMC5160_ENABLE_PIN);
                if (motor->handle) {
                    // Set the chopper configuration
                    err = tmc5160_configure_chopper(motor->handle,
                                                    TMC5160_TBL,
                                                    TMC5160_TOFF,
                                                    TMC5160_HSTRT,
                                                    TMC5160_HEND);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "unable to configure chopper (%s)!", esp_err_to_name(-err));
                    }
                    if (err == ESP_OK) {
                        // Set the microstep resolution
                        err = tmc5160_set_microstep_resolution(motor->handle, TMC5160_MICROSTEP_RESOLUTION);
                        if (err >= 0) {
                            ESP_LOGI(TAG, "microstep resolution set to %d.", TMC5160_MICROSTEP_RESOLUTION);
                            err = ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "unable to set microstep resolution to %d!", TMC5160_MICROSTEP_RESOLUTION);
                        }
                    } else {
                        ESP_LOGE(TAG, "unable to configure chopper (%s)!", esp_err_to_name(-err));
                    }
                    if (err == ESP_OK) {
                        // Engage stealth mods
                        err = tmc5160_set_chopper_mode(motor->handle, TMC5160_CHOPPER_STEALTH_CHOP);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "chopper mode set to TMC5160_CHOPPER_STEALTH_CHOP.");
                            err = ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "unable to set chopper mode to TMC5160_CHOPPER_STEALTH_CHOP!",
                                     esp_err_to_name(-err));
                        }
                    }
                    if (err == ESP_OK) {
                        // Set a threshold to switch to spread cycle
                        err = tmc5160_set_spread_cycle_threshold(motor->handle,
                                                                 speed_mm_per_second_to_millihertz(TMC5160_SPREAD_CYCLE_THRESHOLD_MM_PER_SECOND));
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "spread cycle threshold set to %d mm/second.",
                                     TMC5160_SPREAD_CYCLE_THRESHOLD_MM_PER_SECOND);
                            err = ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "unable to set spread cycle threshold to %d mm/second!",
                                     TMC5160_SPREAD_CYCLE_THRESHOLD_MM_PER_SECOND,
                                     esp_err_to_name(-err));
                        }
                    }
                    if (err == ESP_OK) {
                        // Set the motor current
                        err = tmc5160_set_current(motor->handle, TMC5160_RSENSE_MOHM,
                                                  TMC5160_RUN_CURRENT_MA,
                                                  TMC5160_HOLD_CURRENT_PERCENT);
                        if (err >= 0) {
                            ESP_LOGI(TAG, "motor current set to %d mA.", err);
                            motor->last_current_setting_ma = TMC5160_RUN_CURRENT_MA;
                            err = ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "unable to set motor current to %d mA (%s)!",
                                     TMC5160_RUN_CURRENT_MA, esp_err_to_name(-err));
                        }
                    }
                    if (err == ESP_OK) {
                        // Finally enable the motor
                        err = tmc5160_motor_enable(motor->handle);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "motor is enabled.");
                            err = ESP_OK;
                        } else {
                            ESP_LOGE(TAG, "unable to enable the motor (%s)!", esp_err_to_name(-err));
                        }
                    }
                }
            }
        }

        // Sort retained RAM if we can, irrespective of error state.
        retained_ram_t retained_ram = DEFAULTS_RETAINED_RAM;
        if (FGR_RRAM_GET(retained_ram) != ESP_OK) {
            // Retained RAM unavailable: ground is unknown.
            retained_ram.ground_level_mm = POSITION_UNKNOWN;
            retained_ram.emergency_stopped = false;
            FGR_RRAM_SET(retained_ram);
        }

        if (motor->handle && (err == ESP_OK)) {
            bool reset_detected = tmc5160_has_reset(motor->handle);

            if (reset_detected) {
                // The TMC5160 has reset (or its status could not be read).
                // Its position can no longer be trusted.
                ESP_LOGW(TAG, "TMC5160 reset detected; invalidating ground level.");

                retained_ram.ground_level_mm = POSITION_UNKNOWN;
            }

            // Persist any invalidation before acknowledging the reset.
            // If this fails, leave the reset indication uncleared.
            if (!reset_detected ||
                (FGR_RRAM_SET(retained_ram) == ESP_OK)) {

                int32_t ack_err = tmc5160_acknowledge_reset(motor->handle);

                if (ack_err != ESP_OK) {
                    ESP_LOGE(TAG, "unable to acknowledge TMC5160 reset (%s).",
                             esp_err_to_name(-ack_err));
                }

                // Attempt warm-start recovery only if the TMC5160 has
                // not reset and we have a known ground reference.
                if (!reset_detected &&
                    (ack_err == ESP_OK) &&
                    (retained_ram.ground_level_mm != POSITION_UNKNOWN)) {

                    int32_t microsteps = 0;

                    if (tmc5160_get_position(motor->handle, &microsteps) == ESP_OK) {
                        int32_t height_mm = microsteps_to_mm(microsteps) - retained_ram.ground_level_mm;

                        ESP_LOGI(TAG, "warm start (height %d mm, %d microsteps, %s).",
                                (int) height_mm,
                                (int) microsteps,
                                retained_ram.emergency_stopped ? "EMERGENCY STOPPED" : "not emergency stopped");

                        if (!retained_ram.emergency_stopped) {
                            launcher->height_target_mm = 0;
                            launcher->direction_up_not_down = false;

                            motor_set_height_mm(motor,
                                                launcher->height_target_mm,
                                                nvs_shadow->speed_mm_per_second *
                                                RESETTING_SPEED_MODIFIER_PERCENT / 100,
                                                retained_ram.ground_level_mm,
                                                TMC5160_RUN_CURRENT_MA,
                                                TMC5160_ACCELERATION_NORMAL,
                                                TMC5160_DECELERATION_NORMAL);

                            launcher->state = LAUNCHER_STATE_RUNNING_RESETTING;
                        }
                    }
                }
            } else {
                ESP_LOGE(TAG, "unable to invalidate retained ground level; "
                         "TMC5160 reset not acknowledged.");
            }
        }

#if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM) && (CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM >= 0)
        // Initialise a UART to control the DY HV8F audio board
        if (err == ESP_OK) {
            const uart_config_t cfg = {
                .baud_rate = SPIDER_LAUNCHER_AUDIO_UART_BAUD_RATE,
                .data_bits = UART_DATA_8_BITS,
                .parity = UART_PARITY_DISABLE,
                .stop_bits = UART_STOP_BITS_1,
                .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .source_clk = UART_SCLK_DEFAULT,
            };

            err = -uart_driver_install(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM, UART_RX_BUFFFER_SIZE, 0, 0, NULL, 0);
            if (err == ESP_OK) {
                err = -uart_param_config(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM, &cfg);
            }
            if (err == ESP_OK) {
                err = -uart_set_pin(CONFIG_SPIDER_LAUNCHER_AUDIO_UART_NUM,
                                    CONFIG_SPIDER_LAUNCHER_AUDIO_UART_TXD_PIN,
                                    CONFIG_SPIDER_LAUNCHER_AUDIO_UART_RXD_PIN,
                                    UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            }
        }
#else
    // Set the pins that control the audio track all high so that they can be pulled low
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0 >= 0)
    gpio_set_level(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0, 1);
    gpio_set_direction(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_0, GPIO_MODE_OUTPUT);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1 >= 0)
    gpio_set_level(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1, 1);
    gpio_set_direction(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_1, GPIO_MODE_OUTPUT);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2 >= 0)
    gpio_set_level(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, 1);
    gpio_set_direction(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_2, GPIO_MODE_OUTPUT);
#  endif
#  if defined(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3) && (CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3 >= 0)
    gpio_set_level(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, 1);
    gpio_set_direction(CONFIG_SPIDER_LAUNCHER_AUDIO_PIN_3, GPIO_MODE_OUTPUT);
#  endif
#endif

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
            nvs_populate(nvs_shadow);
        }

        if (err == ESP_OK) {
            err = -ESP_ERR_NO_MEM;
            // RTOS stuff needed for launcher operation
            launcher_t *launcher = &context->launcher;
            launcher->queue_state = xQueueCreate(10, sizeof(launcher_state_t));
            if (launcher->queue_state) {
                err = fgr_task_create(&launcher_cb, context, "launcher", 4096, 3, &launcher->task_state);
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
            // Stuff needed for automatic mode
            launcher->auto_period_seconds = nvs_shadow->auto_period_seconds;
            timer_start_us(&launcher->timer_auto, ((int64_t) launcher->auto_period_seconds) * 1000000);
            err = fgr_task_create(&auto_cb, launcher, "auto", 4096, 3, &launcher->task_auto);
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
    launcher_t *launcher = &context->launcher;
    launcher_motor_t *motor = &launcher->motor;
    nvs_shadow_t *nvs_shadow = &launcher->nvs_shadow;

    ESP_LOGI(TAG, "app_main start");

    int32_t err = init(context);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "initialization complete.");
        ESP_LOGI(TAG, "waiting for BLE connections/commands.");
        if (nvs_shadow->mute) {
            ESP_LOGI(TAG, "muted, not playing any audio.");
        }
        while (err == ESP_OK) {
            // Let BLE commands do their thing
            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_task_wdt_reset();
        }
        esp_task_wdt_delete(NULL);
        ESP_LOGE(TAG, "dropped out with error (%s)!", esp_err_to_name(-err));
    } else {
        ESP_LOGE(TAG, "initialization failed, system cannot continue, will restart soonish.");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));

    nimble_port_stop();
    fgr_debug_deinit();
    fgr_ws2812_deinit();
    if (motor->handle) {
        tmc5160_set_velocity(motor->handle, 0);
        tmc5160_motor_disable(motor->handle);
    }
    tmc5160_deinit();
    fgr_task_deinit();
    if (context->command_queue){
        vQueueDelete(context->command_queue);
    }
    if (launcher->queue_state){
        vQueueDelete(launcher->queue_state);
    }

    if (context->lock) {
        vSemaphoreDelete(context->lock);
    }
    esp_restart();
}

// End of file
