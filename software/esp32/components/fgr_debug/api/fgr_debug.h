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

#ifndef _FGR_DEBUG_H_
#define _FGR_DEBUG_H_

/** @file
 * @brief The debug utilities API for a node of the front garden railway.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Required for fgr_ws2812_colour_t.
#include "fgr_ws2812.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef FGR_DEBUG_LED_SHORT_MS
// Standard short duration for an LED lash.
#  define FGR_DEBUG_LED_SHORT_MS 400
#endif

#ifndef FGR_DEBUG_LED_LONG_MS
// Standard short duration for a long LED flash.
#  define FGR_DEBUG_LED_LONG_MS 1000
#endif

#ifndef FGR_DEBUG_LED_COLOUR_NONE
// Debug LED off
#  define FGR_DEBUG_LED_COLOUR_NONE FGR_WS2812_LED_COLOUR_NONE
#endif

#ifndef FGR_DEBUG_LED_COLOUR_BOOT
// Standardised boot colour.
#  define FGR_DEBUG_LED_COLOUR_BOOT FGR_WS2812_LED_COLOUR_WHITE
#endif

#ifndef FGR_DEBUG_LED_COLOUR_NEEDS_CFG
// Standardised colour, primarily for breathe, when waiting for configuration.
#  define FGR_DEBUG_LED_COLOUR_NEEDS_CFG FGR_WS2812_LED_COLOUR_CYAN
#endif

#ifndef FGR_DEBUG_LED_COLOUR_STOPPED
// Standardised colour, primarily for breathe, when stopped.
#  define FGR_DEBUG_LED_COLOUR_STOPPED FGR_WS2812_LED_COLOUR_MAGENTA
#endif

#ifndef FGR_DEBUG_LED_COLOUR_ALARM
// Standardised alarm colour: the only one that is bright.
#  define FGR_DEBUG_LED_COLOUR_ALARM FGR_WS2812_LED_COLOUR_BRIGHT_RED
#endif

#ifndef FGR_DEBUG_LED_COLOUR_BAD
// Standardised negative colour.
#  define FGR_DEBUG_LED_COLOUR_BAD FGR_WS2812_LED_COLOUR_RED
#endif

#ifndef FGR_DEBUG_LED_COLOUR_GOOD
// Standardised positive colour.
#  define FGR_DEBUG_LED_COLOUR_GOOD FGR_WS2812_LED_COLOUR_GREEN
#endif

#ifndef FGR_DEBUG_LED_COLOUR_NOTIFY
// Standardised neutral notification colour.
#  define FGR_DEBUG_LED_COLOUR_NOTIFY FGR_WS2812_LED_COLOUR_YELLOW
#endif

#ifndef FGR_DEBUG_LED_COLOUR_MSG_SENT
// Standardised message sent colour.
#  define FGR_DEBUG_LED_COLOUR_MSG_SENT FGR_WS2812_LED_COLOUR_BLUE
#endif

#ifndef FGR_DEBUG_BACKTRACE_DEPTH_MAX
// The maximim depth of a backtrace.
#  define FGR_DEBUG_BACKTRACE_DEPTH_MAX 32
#endif

#ifndef FGR_DEBUG_BACKTRACE_FORMAT_STRING
// The printf() format string for an item in a backtrace.
#  define FGR_DEBUG_BACKTRACE_FORMAT_STRING "0x%08x "
#endif

#ifndef FGR_DEBUG_BACKTRACE_NUMBER_LENGTH
// The length of buffer required for the format string
// FGR_DEBUG_BACKTRACE_FORMAT_STRING **WITHOUT** a terminator.
#  define FGR_DEBUG_BACKTRACE_NUMBER_LENGTH 11
#endif

// The length of buffer required to encode the maximum length
// backtrace, including room for a null terminator, enough space
// for FGR_DEBUG_BACKTRACE_BUFFER_NUMBER_LENGTH characters N times
// plus the terminator.
#define FGR_DEBUG_BACKTRACE_BUFFER_LENGTH ((FGR_DEBUG_BACKTRACE_DEPTH_MAX * FGR_DEBUG_BACKTRACE_NUMBER_LENGTH) + 1)

// The length of the hash of an image file.
#define FGR_DEBUG_HASH_LENGTH 32

// The amount of storage needed for the hash of the current
// binary as a null-terminated human-readable string; used
// by fgr_debug_get_hash().
#define FGR_DEBUG_HASH_BUFFER_LENGTH ((FGR_DEBUG_HASH_LENGTH * 2) + 1)

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * FUNCTIONS: INITIALISE/DEINITIALISE
 * -------------------------------------------------------------- */

/** Initialise debug.  If CONFIG_FGR_DEBUG_LED_SPI_NUM and the cb
 * parameter below is populated then fgr_ws2812_init() must have
 * been called first.  It is always safe to call this at any time:
 * if already initialised it will do nothing and return success.
 *
 * Note: this will create a semaphore that is never destroyed.
 *
 * @return           ESP_OK on success, else a negative value from
 *                   esp_err_t.
 */
int32_t fgr_debug_init();

/** Deinitialise debug.  This will also free any remaining tasks,
 * and do so in a coooperative way, waiting for any task callbacks
 * to return, no crowbars.  It is always safe to call this at any time.
 */
void fgr_debug_deinit();

/* ----------------------------------------------------------------
 * FUNCTIONS: LED RELATED
 * -------------------------------------------------------------- */

/** Flash the debug LED.
 *
 * @param duration_ms how long to flash the LED for (e.g.
 *                    FGR_DEBUG_LED_SHORT_MS or FGR_DEBUG_LED_LONG_MS).
 * @param colour      the LED colour, ignored if
 *                    CONFIG_FGR_DEBUG_LED_SPI_NUM is not defined.
 */
void fgr_debug_led_flash(int32_t duration_ms, fgr_ws2812_colour_t colour);

/** Turn the LED "breathe" effect off.  If fgr_nvs_init() or
 * fgs_ota_init() have been called then the setting will persist
 * across boot cycles.  The breathe effect only runs if you are using
 * a WS2812 debug LED (i.e. CONFIG_FGR_DEBUG_LED_SPI_NUM is defined).
 */
void fgr_debug_led_breathe_off(void);

/** Turn the LED "breathe" effect on (i.e. continue to operate
 * as it did before fgr_debug_led_breathe_off()).  If fgr_nvs_init()
 * or fgs_ota_init() have been called then the setting will persist
 * across boot cycles.  The breathe effect only runs if you are using
 * a WS2812 debug LED (i.e. CONFIG_FGR_DEBUG_LED_SPI_NUM is defined).
 */
void fgr_debug_led_breathe_on(void);

/** Turn all debug LEDs off.  If fgr_nvs_init() or fgs_ota_init()
 * have been called then the setting will persist across boot cycles.
 */
void fgr_debug_led_off(void);

/** Turn all debug LEDs on (i.e. continue to operate as they
 * did before fgr_debug_led_off()).  If fgr_nvs_init() or
 * fgs_ota_init() have been called then the setting will persist
 * across boot cycles.
 */
void fgr_debug_led_on(void);

/** Set the LED "breathing" manually: you would not normally call
 * this, just let the debug function operate the breathe effect
 * based on the state callback passed to fgr_debug_init().  To
 * return to normal operation after manually setting a breathe
 * effect, pass in all zeros for the colours.  The breathe
 * effect only runs if you are using a WS2812 debug LED
 * (i.e. CONFIG_FGR_DEBUG_LED_SPI_NUM is defined).
 *
 * Note: LED breathing is by default on but this will do nothing
 * if fgr_debug_led_breathe_off() was previously called, or if
 * non-volatile storage is in play and fgr_debug_led_breathe_off()
 * was called in a previous boot.
 *
 * @param colour the LED colour.
 */
void fgr_debug_led_breathe_set(fgr_ws2812_colour_t colour);

/* ----------------------------------------------------------------
 * FUNCTIONS: PANIC
 * -------------------------------------------------------------- */

/** Function to obtain any backtrace captured after a panic.
 * For this to work, the main project CMakeLists.txt file
 * must have:
 *
 * idf_build_set_property(LINK_OPTIONS "-Wl,--wrap=esp_panic_handler" APPEND)
 *
 * ...which will wrap calls to esp_panic_handler().
 *
 * You might call this function on boot and log the result so that,
 * if you have the ELF file, the function call tree leading
 * to the panic can be printed using the Espressif ESP-IDF tools:
 *
 * xtensa-esp-elf-addr2line -pfiaC -e my_binary.elf <backtrace in hex>
 *
 * e.g.:
 *
 * xtensa-esp-elf-addr2line -pfiaC -e test.elf 0x400D1234 0x400D5678 ...
 *
 *
 * See also fgr_debug_panic_str_get() ,fgr_debug_panic_str_get()
 * and fgr_debug_panic_log().
 *
 * @param backtrace_copy  a pointer to storage for
 *                        FGR_DEBUG_BACKTRACE_DEPTH_MAX uint32_t
 *                        values that are the backtrace copied
 *                        from retained RAM; may be NULL, in which
 *                        case the backtrace is retained and you might
 *                        use the return value to size your storage
 *                        before calling this function again.  If
 *                        non-NULL the backtrace storage is emptied
 *                        on return.
 * @param hash            a pointer to at least FGR_DEBUG_HASH_BUFFER_LENGTH
 *                        bytes of storage for the hash of the software
 *                        version at the time of the panic; a human-
 *                        readable string, null-terminated will be
 *                        placed here.  May be NULL.
 * @return                if there have been one or more panics since
 *                        power-on, the number of uint32_t values
 *                        that would be populated in backtrace if it
 *                        were non-NULL, ESP_OK if there have been no
 *                        panics.
 */
int32_t fgr_debug_panic_get(uint32_t *backtrace_copy, char *hash);

/** As fgr_debug_panic_get() but populates a buffer with
 * a string that can be passed straight to xtensa-esp-elf-addr2line,
 * e.g. "0x400D1234 0x400D5678..."
 *
 * See also fgr_debug_panic_str_get();
 *
 * @param backtrace_str a pointer to storage for up to
 *                      FGR_DEBUG_BACKTRACE_BUFFER_LENGTH that will be
 *                      populated with the null-termkinated backtrace
 *                      string; may be NULL, in which case the backtrace
 *                      is retained and you might use the return value
 *                      to size your storage before calling this function
 *                      again.  If non-NULL the backtrace storage is
 *                      emptied on return.
 * @param hash          a pointer to at least FGR_DEBUG_HASH_BUFFER_LENGTH
 *                      bytes of storage for the hash of the software
 *                      version at the time of the panic; a human-
 *                      readable string, null-terminated, will be
 *                      placed here.  May be NULL.
 * @return              if there have been one or more panics since
 *                      power-on, the number of characters that would be
 *                      populated in buffer (i.e. what strlen() would
 *                      return) if it were not NULL, ESP_OK if there have
 *                      been no panics.
 */
int32_t fgr_debug_panic_str_get(char *backtrace_str, char *hash);

/** As fgr_debug_panic_str_get() but instead of returning a
 * string, logs the backtrace string (if present) to two ESP_LOGx()
 * macros with the given ESP-IDF log level: the first will contain
 * the hash of the software version when the panic occurred, the
 * second will contain the backtrace.
 *
 * @param tag    the tag to apply to the log messages; may be NULL
 *               in which case whatever is the default tag for
 *               debug messages will be employed.
 * @param prefix a prefix to put in front of the backtrace string;
 *               may be NULL.
 * @param level  the log level to log the string as.
 * @return       1 if there was a panic, ESP_OK if not, else a
 *               negative error code from esp_err_t.
 */
int32_t fgr_debug_panic_log(const char *tag, const char *prefix,
                            esp_log_level_t level);

/* ----------------------------------------------------------------
 * FUNCTIONS: STACK OVERFLOW
 * -------------------------------------------------------------- */

/** Get the name of a task that had a stack overflow; you might call
 * this at boot to see if the boot was actually a reboot resulting
 * from a stack overflow.
 *
 * @param buffer a pointer to storage for up to
 *               FGR_UTIL_TASK_NAME_MAX_LENGTH characters that will
 *               be populated with the task name; may be NULL,
 *               in which case the task name is retained and you
 *               might use the return value to size your storage
 *               before calling this function again.  If non-NULL
 *               the task name is emptied on return.
 * @return       if a stack overflow occurred, the number
 *               characters that would be populated in buffer
 *               (i.e. what strlen() would return) if it were
 *               non-NULL, ESP_OK if there was no stack overflow.
 */
int32_t fgr_debug_stack_overflow_get(char *buffer);

/** As fgr_debug_stack_overflow_get() but instead of returning
 * the task name string, logs the task name string (if present) to
 * an ESP_LOGx() macro with the given ESP-IDF log level.
 *
 * @param tag    the tag to apply to the log message; may be NULL
 *               in which case whatever is the default tag for
 *               debug messages will be employed.
 * @param prefix a prefix to put in front of the backtrace string;
 *               may be NULL.
 * @param level  the log level to log the string as.
 * @return       1 if there was a stack overflow, ESP_OK if not,
 *               else a negative error code from esp_err_t.
 */
int32_t fgr_debug_stack_overflow_log(const char *tag, const char *prefix,
                                     esp_log_level_t level);

/* ----------------------------------------------------------------
 * FUNCTIONS: CORE DUMP
 * -------------------------------------------------------------- */

/** Send a core dump to logging; you might call this at boot to see
 * if there is a core dump stored to flash.  The core dump will be sent
 * in ESP_LOGx() messages, base64 encoded.
 *
 * For this to work, you must have a flash partition of at least
 * 64 kbytes dedicated for core dumps, e.g. like this:
 *
 * coredump,   data, coredump,0x3E0000, 64K
 *
 * ...and you must have set CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=y
 * and likely CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF=y in your sdkconfig
 * file.  With this configuration, core dumps are automatically sent
 * by ESP-IDF when a crash occurs.  You will need the hash of the
 * software version that was running at the time to decode it: this
 * can be obtained from the panic backtrace that will have been saved
 * at the same time as the core dump.
 *
 * @param tag    the tag to apply to the log message; may be NULL
 *               in which case whatever is the default tag for
 *               debug messages will be employed.
 * @param level  the log level to log the string as.
 * @return       1 if a core dump as present, ESP_OK if not,
 *               else a negative error code from esp_err_t.
 */
int32_t fgr_debug_core_dump_get(const char *tag, esp_log_level_t level);

/* ----------------------------------------------------------------
 * FUNCTIONS: MISC
 * -------------------------------------------------------------- */

/** Log the ESP32 reset reason.
 */
void fgr_debug_reset_reason_log();

/** Print out our MAC address if possible.
 */
void fgr_debug_print_mac_address();

/** Get the hash of the current firmware version as a
 * human-readable string.
 *
 * @param buffer a pointer to at least
 *               FGR_DEBUG_HASH_BUFFER_LENGTH bytes of
 *               storage for the hash; the string
 *               written to buffer will be null-terminated.
 */
void fgr_debug_get_hash(char *buffer);

/** Create a hex dump of data in a provided buffer.
 *
 * @param data          input data to dump
 * @param data_size     size of input data in bytes
 * @param output        output buffer to fill with hex dump
 * @param output_size   size of output buffer
 * @return              number of characters written (excluding
 *                      null terminator), or -1 if output
 *                      buffer is too small
 */
int32_t fgr_debug_hex_dump_to_buffer(const void *data, size_t data_size,
                                     char *output, size_t output_size);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // _FGR_DEBUG_H_

// End of file
