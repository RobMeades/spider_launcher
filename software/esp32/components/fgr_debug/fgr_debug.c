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
 * @brief Debug utilities for a node of the front garden railway.
 */

// Ensure we are compiling with maximum debug, can then be trimmed
// at run-time by fgr_log
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "errno.h"
#include "ctype.h"
#include "esp_mac.h"
#include "driver/gpio.h"
#include "esp_core_dump.h"
#include "esp_partition.h"
#include "esp_app_desc.h"
#include "mbedtls/base64.h"

#include "fgr_util.h"
#include "fgr_ws2812.h"
#include "fgr_task.h"
#include "fgr_nvs.h"

#include "fgr_debug.h"

// Forward declaration of the abstracted panic info structure from ESP-IDF
void __real_esp_panic_handler(void *info);
// Make sure the linker doesn't optimize-out our wrapper
void __wrap_esp_panic_handler(void *info) __attribute__((used));

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "debug"

#ifdef CONFIG_FGR_DEBUG_LED_WS2812_GRB
#  define FGR_DEBUG_LED_WS2812_GRB 1
#else
#  define FGR_DEBUG_LED_WS2812_GRB 0
#endif

#ifndef NVS_NAME_LED_MASKED
// A name for the field that masks the LED off in NV storage.
#  define NVS_NAME_LED_MASKED "led_masked"
#endif

#ifndef NVS_NAME_LED_BREATHE_ENABLED
// A name for the field that enables LED "breathing" in NV storage.
// Note: not "led_breathe_enabled" as that turns out to be too long.
#  define NVS_NAME_LED_BREATHE_ENABLED "led_breathe_on"
#endif

#ifndef CORE_DUMP_BASE64_CHUNK_LENGTH
// The maximum length of chunk of a core dump to base64 encode; the
// raw chunk length before base64 encoding will be 3/4 of this size.
#  define CORE_DUMP_BASE64_CHUNK_LENGTH 512
#endif

// The amount of core dump raw data that can be base64 encoded
// into CORE_DUMP_BASE64_CHUNK_LENGTH.
#define CORE_DUMP_CHUNK_LENGTH (CORE_DUMP_BASE64_CHUNK_LENGTH * 3 / 4)

// The base64 chunk length must be a multiple of four to
// accommodate base64 cleanly.
#if CORE_DUMP_BASE64_CHUNK_LENGTH % 4 != 0
#  error CORE_DUMP_CHUNK_LENGTH must be such that CORE_DUMP_BASE64_CHUNK_LENGTH is a multiple of four
#endif

// mbedTLS requires room for a null terminator in the buffer
#define CORE_DUMP_BASE64_CHUNK_LENGTH_MBEDTLS (CORE_DUMP_BASE64_CHUNK_LENGTH + 1)

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    bool initialised;
    void *ws2812_handle;
    bool led_masked_off;
    bool breathe_enabled;
} context_t;

// Storage for a backtrace.
typedef struct {
    uint8_t hash[32];
    uint8_t length;
    uint32_t address_list[FGR_DEBUG_BACKTRACE_DEPTH_MAX];
} backtrace_t;

// Storage for an overflowing task name.
// Note: name _must_ be at the start of the structure,
// see vApplicationStackOverflowHook() for why.
typedef struct {
    char name[FGR_UTIL_TASK_NAME_MAX_LENGTH];
} stack_overflow_task_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

// The names of the ESP32 reset reasons, in the order of
// esp_reset_reason_t, for easy display.
static const char *g_esp_reset_reason[]  = {"UNKNOWN",      // ESP_RST_UNKNOWN
                                            "POWERON",      // ESP_RST_POWERON
                                            "EXT",          // ESP_RST_EXT
                                            "SW",           // ESP_RST_SW
                                            "PANIC",        // ESP_RST_PANIC
                                            "INT_WDT",      // ESP_RST_INT_WDT
                                            "TASK_WDT",     // ESP_RST_TASK_WDT
                                            "WDT",          // ESP_RST_WDT
                                            "DEEPSLEEP",    // ESP_RST_DEEPSLEEP
                                            "BROWNOUT",     // ESP_RST_BROWNOUT
                                            "SDIO",         // ESP_RST_SDIO
                                            "USB",          // ESP_RST_USB
                                            "JTAG",         // ESP_RST_JTAG
                                            "EFUSE",        // ESP_RST_EFUSE
                                            "PWR_GLITCH",   // ESP_RST_PWR_GLITCH
                                            "CPU_LOCKUP"};  // ESP_RST_CPU_LOCKUP

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: NVS RELATED
 * -------------------------------------------------------------- */

// Retrieve whether the LED is masked off or not from NVS.
static int32_t nvs_led_masked_get(bool *masked)
{
    int32_t err = -ESP_ERR_INVALID_ARG;
    uint32_t value = 0;

    if (masked) {
        err = fgr_nvs_get(NVS_NAME_LED_MASKED, &value);
        if (err == ESP_OK) {
            *masked = (value != 0);
        }
    }

    return err;
}

// Set whether LED is masked off or not in NVS.
static int32_t nvs_led_masked_set(bool masked)
{
    return fgr_nvs_set(NVS_NAME_LED_MASKED, masked);
}

// Retrieve whether LED "breathing" is enabled from NVS.
static int32_t nvs_led_breathe_enabled_get(bool *enabled)
{
    int32_t err = -ESP_ERR_INVALID_ARG;
    uint32_t value = 0;

    if (enabled) {
        err = fgr_nvs_get(NVS_NAME_LED_BREATHE_ENABLED, &value);
        if (err == ESP_OK) {
            *enabled = (value != 0);
        }
    }

    return err;
}

// Set whether LED "breathing" is enabled in NVS.
static int32_t nvs_led_breathe_enabled_set(bool enabled)
{
    return fgr_nvs_set(NVS_NAME_LED_BREATHE_ENABLED, enabled);
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: MISC
 * -------------------------------------------------------------- */

// Set whether the LED is masked off or not.
static void led_masked_off(context_t *context, bool masked)
{
    if (context->lock) {

        CONTEXT_LOCK(context->lock, "led_masked_off()");
        context->led_masked_off = masked;
        nvs_led_masked_set(masked);
        if (masked) {
            fgr_ws2812_led_mask_off(context->ws2812_handle, 0);
        } else {
            fgr_ws2812_led_mask_on(context->ws2812_handle, 0);
        }
        CONTEXT_UNLOCK(context->lock, "led_masked_off()");
    }
}

// Set whether LED "breathing" is enabled.
static void led_breathe_enabled(context_t *context, bool enabled)
{
    if (context->lock) {

        CONTEXT_LOCK(context->lock, "led_breathe_enabled()");

        context->breathe_enabled = enabled;
        nvs_led_breathe_enabled_set(enabled);
        if (!enabled) {
            // The desired effect is that the LED is off but flashes
            // can still occur.  The static colour will be none in any
            // case (because this code never sets it), so all we need
            // to do is not set the LED colour based on the callback
            fgr_ws2812_led_set(context->ws2812_handle, 0, FGR_DEBUG_LED_COLOUR_NONE);
            fgr_ws2812_led_set_cb(g_context.ws2812_handle, 0,
                                    NULL, NULL);
        }

        CONTEXT_UNLOCK(context->lock, "led_breathe_enabled()");
    }
}

// Format a 32-byte hash into a null-terminated string.
static void hash_str(char *buffer_str, const uint8_t *hash)
{
    if (buffer_str && hash) {
        // Convert the raw 32-byte binary SHA256 into a readable hex string
        for (size_t x = 0; x < FGR_DEBUG_HASH_LENGTH; x++) {
            buffer_str += sprintf(buffer_str, "%02x", *(hash + x));
        }
        *buffer_str = '\0'; // null-terminate
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: INITIALISE/DEINITIALISE
 * -------------------------------------------------------------- */

// Initialise debug stuff.
int32_t fgr_debug_init()
{
    int32_t err = -ESP_ERR_NO_MEM;

    if (!g_context.lock) {
        g_context.lock = xSemaphoreCreateMutex();
    }

    if (g_context.lock) {
        err = ESP_OK;
        if (!g_context.initialised) {

            CONTEXT_LOCK(g_context.lock, "fgr_debug_init()");

            g_context.led_masked_off = false;
            g_context.breathe_enabled = true;

            // Read values from non-volatile storage and,
            // if not present, write the default value back
            if (nvs_led_masked_get(&g_context.led_masked_off) != ESP_OK) {
                nvs_led_masked_set(g_context.led_masked_off);
            }
            if (nvs_led_breathe_enabled_get(&g_context.breathe_enabled) != ESP_OK) {
                nvs_led_breathe_enabled_set(g_context.breathe_enabled);
            }

#if defined(CONFIG_FGR_DEBUG_LED_PIN) && (CONFIG_FGR_DEBUG_LED_PIN >= 0)
#  if defined(CONFIG_FGR_DEBUG_LED_SPI_NUM) && (CONFIG_FGR_DEBUG_LED_SPI_NUM > 1) // SPIs 0 and 1 are used internally
            // Create a WS2812 LED chain for the debug LED
            err = fgr_ws2812_chain_init(CONFIG_FGR_DEBUG_LED_SPI_NUM, -1,
                                        CONFIG_FGR_DEBUG_LED_PIN, 1,
                                        FGR_DEBUG_LED_WS2812_GRB,
                                        &g_context.ws2812_handle);
#  else
            // Configure our single colour debug LED
            err = gpio_set_level(CONFIG_FGR_DEBUG_LED_PIN, 1);
            if (err == ESP_OK) {
                err = gpio_set_direction(CONFIG_FGR_DEBUG_LED_PIN, GPIO_MODE_OUTPUT);
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "Using a single colour debug LED on pin %d.",
                             CONFIG_FGR_DEBUG_LED_PIN);
                } else {
                    ESP_LOGE(TAG, "gpio_set_direction() on pin %d failed (%s)!",
                             CONFIG_FGR_DEBUG_LED_PIN, esp_err_to_name(err));
                }
            } else {
                ESP_LOGE(TAG, "gpio_set_level() on pin %d failed (%s)!",
                         CONFIG_FGR_DEBUG_LED_PIN, esp_err_to_name(err));
            }
            // Return ESP_OK or negative error code from esp_err_t
            err = -err;
#  endif  // #if defined(CONFIG_FGR_DEBUG_LED_PIN) && (CONFIG_FGR_DEBUG_LED_PIN >= 0)
#endif    // #  if defined(CONFIG_FGR_DEBUG_LED_SPI_NUM) && (CONFIG_FGR_DEBUG_LED_SPI_NUM > 1)

            if (err == ESP_OK) {
                g_context.initialised = true;
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_debug_init()");

            if (err == ESP_OK) {
                // Flash the LED so that we know it can be active
                fgr_debug_led_flash(FGR_DEBUG_LED_LONG_MS, FGR_DEBUG_LED_COLOUR_BOOT);
            }
        }
    }

    return err;
}

// Deinitialise debug stuff.
void fgr_debug_deinit()
{
    if (g_context.lock) {

        ESP_LOGI(TAG, "Stopping debug.");

        CONTEXT_LOCK(g_context.lock, "fgr_debug_deinit()");

        if (g_context.ws2812_handle) {
            fgr_ws2812_led_set(g_context.ws2812_handle, 0, FGR_DEBUG_LED_COLOUR_NONE);
            vTaskDelay(pdMS_TO_TICKS(100));
            fgr_ws2812_chain_deinit(g_context.ws2812_handle);
            g_context.ws2812_handle = NULL;
        }

        g_context.initialised = false;

        CONTEXT_UNLOCK(g_context.lock, "fgr_debug_deinit()");
        // The semaphore will be re-used
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: LED RELATED
 * -------------------------------------------------------------- */

// Flash the debug LED.
void fgr_debug_led_flash(int32_t duration_ms, fgr_ws2812_colour_t colour)
{
    if (g_context.lock) {

        CONTEXT_LOCK(g_context.lock, "fgr_debug_led_flash()");

        if (!g_context.led_masked_off) {
            if (g_context.ws2812_handle) {
                // WS2812 LED
                fgr_ws2812_led_flash(g_context.ws2812_handle, 0,
                                     duration_ms, colour);
            } else {
                // Single colour LED
                gpio_set_level(CONFIG_FGR_DEBUG_LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(duration_ms));
                gpio_set_level(CONFIG_FGR_DEBUG_LED_PIN, 1);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_debug_led_flash()");
    }
}

// Set the LED "breathing" a specific colour.
void fgr_debug_led_breathe_set(fgr_ws2812_colour_t colour)
{
    if (g_context.lock) {

        CONTEXT_LOCK(g_context.lock, "fgr_debug_set_breathe()");

        if (!g_context.led_masked_off && g_context.breathe_enabled) {
            if (fgr_ws2812_led_set(g_context.ws2812_handle, 0, colour) == ESP_OK) {
                // The callback mechanism overrides manually set breathing,
                // so need to switch it off
                fgr_ws2812_led_set_cb(g_context.ws2812_handle, 0, NULL, NULL);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_debug_set_breathe()");
    }
}

// Turn the LED "breathe" effect off.
void fgr_debug_led_breathe_off(void)
{
    led_breathe_enabled(&g_context, false);
}

// Turn the LED "breathe" effect on.
void fgr_debug_led_breathe_on(void)
{
    led_breathe_enabled(&g_context, true);
}

// Turn all debug LEDs off.
void fgr_debug_led_off(void)
{
    led_masked_off(&g_context, true);
}

// Allow all debug LEDs to operate.
void fgr_debug_led_on(void)
{
    led_masked_off(&g_context, false);
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: MISC
 * -------------------------------------------------------------- */

// Log the ESP32 reset reason.
void fgr_debug_reset_reason_log()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    const char *name = "DONT_KNOW_THIS_ONE";

    if ((reset_reason >= 0) && (reset_reason < FGR_UTIL_ARRAY_LENGTH(g_esp_reset_reason))) {
        name = g_esp_reset_reason[reset_reason];
    }
    ESP_LOGI(TAG, "Last reset reason was ESP_RESET_%s (%d)", name, reset_reason);
}

// Print out our MAC address.
void fgr_debug_print_mac_address()
{
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        ESP_LOGI(TAG, "MAC address %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4],
                 mac[5]);
    }
}

// Get the hash of the current firmware version as a string.
void fgr_debug_get_hash(char *buffer)
{
    // Get description of the currently running app
    const esp_app_desc_t *app_desc = esp_app_get_description();
    hash_str(buffer, app_desc->app_elf_sha256);
}

// Create a hex dump of data in a provided buffer
// (written by DeepSeek 'cos I couldn't find my own
// hex print routine and got lazy).
int32_t fgr_debug_hex_dump_to_buffer(const void *data, size_t data_size,
                                     char *output, size_t output_size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    char *out_ptr = output;
    size_t remaining = output_size;
    int32_t total_written = 0;

    if (output_size == 0) {
        return -1;
    }

    for (size_t i = 0; i < data_size; i += 16) {
        int32_t line_written;

        // Print offset (8 hex digits + space)
        line_written = snprintf(out_ptr, remaining, "%08zx  ", i);
        if (line_written < 0 || (size_t)line_written >= remaining) {
            output[output_size - 1] = '\0';
            return -1;
        }
        out_ptr += line_written;
        remaining -= line_written;
        total_written += line_written;

        // Print hex bytes
        for (size_t j = 0; j < 16; j++) {
            if (i + j < data_size) {
                line_written = snprintf(out_ptr, remaining, "%02x ", bytes[i + j]);
            } else {
                line_written = snprintf(out_ptr, remaining, "   ");
            }

            if (line_written < 0 || (size_t)line_written >= remaining) {
                output[output_size - 1] = '\0';
                return -1;
            }
            out_ptr += line_written;
            remaining -= line_written;
            total_written += line_written;

            // Add extra space in the middle
            if (j == 7) {
                if (remaining < 1) {
                    output[output_size - 1] = '\0';
                    return -1;
                }
                *out_ptr++ = ' ';
                remaining--;
                total_written++;
            }
        }

        // Print ASCII representation
        line_written = snprintf(out_ptr, remaining, " |");
        if (line_written < 0 || (size_t)line_written >= remaining) {
            output[output_size - 1] = '\0';
            return -1;
        }
        out_ptr += line_written;
        remaining -= line_written;
        total_written += line_written;

        for (size_t j = 0; j < 16 && i + j < data_size; j++) {
            unsigned char c = bytes[i + j];
            if (remaining < 1) {
                output[output_size - 1] = '\0';
                return -1;
            }
            *out_ptr++ = isprint(c) ? c : '.';
            remaining--;
            total_written++;
        }

        // Close ASCII section and add newline
        line_written = snprintf(out_ptr, remaining, "|\n");
        if (line_written < 0 || (size_t)line_written >= remaining) {
            output[output_size - 1] = '\0';
            return -1;
        }
        out_ptr += line_written;
        remaining -= line_written;
        total_written += line_written;
    }

    // Ensure null termination
    if (remaining > 0) {
        *out_ptr = '\0';
    } else {
        output[output_size - 1] = '\0';
        return -1;
    }

    return total_written;
}

// End of file
