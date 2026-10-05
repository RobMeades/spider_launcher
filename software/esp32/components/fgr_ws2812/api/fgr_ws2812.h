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

#ifndef _FGR_WS2812_H_
#define _FGR_WS2812_H_

/** @file
 * @brief Drive a [chain of] WS2812 LEDs.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef FGR_WS2812_LED_INTENSITY_LOW
// How bright to shine the LED for low intensity: these LEDS are very bright!
#  define FGR_WS2812_LED_INTENSITY_LOW 64
#endif

#ifndef FGR_WS2812_LED_INTENSITY_HIGH
// How bright to shine the LED for high intensity.
#  define FGR_WS2812_LED_INTENSITY_HIGH 128
#endif

#ifndef FGR_WS2812_BREATHE_MIN_INTENSITY
// Lowest intensity during a breath, 0-255.  Set high enough that
// (colour * this / 255) is still visibly lit for your LED intensity.
// For INTENSITY_LOW = 16, 96 gives a trough PWM of ~6; for
// INTENSITY_LOW = 64, 96 gives a trough PWM of ~24.
#  define FGR_WS2812_BREATHE_MIN_INTENSITY 32
#endif

#ifndef FGR_WS2812_BREATHE_MAX_INTENSITY
// Highest intensity during a breath, 0-255.
#  define FGR_WS2812_BREATHE_MAX_INTENSITY 255
#endif

#ifndef FGR_WS2812_BREATHE_FADE_MS
// How long to crossfade when the breathe colour changes.
#  define FGR_WS2812_BREATHE_FADE_MS 300
#endif

#define FGR_WS2812_LED_COLOUR_NONE ((fgr_ws2812_colour_t) {0, 0, 0})

#ifndef FGR_WS2812_LED_COLOUR_RED
#  define FGR_WS2812_LED_COLOUR_RED ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_LOW, 0, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_RED
#  define FGR_WS2812_LED_COLOUR_BRIGHT_RED ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_HIGH, 0, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BLUE
#  define FGR_WS2812_LED_COLOUR_BLUE ((fgr_ws2812_colour_t) {0, 0, FGR_WS2812_LED_INTENSITY_LOW})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_BLUE
#  define FGR_WS2812_LED_COLOUR_BRIGHT_BLUE ((fgr_ws2812_colour_t) {0, 0, FGR_WS2812_LED_INTENSITY_HIGH})
#endif

#ifndef FGR_WS2812_LED_COLOUR_GREEN
#  define FGR_WS2812_LED_COLOUR_GREEN ((fgr_ws2812_colour_t) {0, FGR_WS2812_LED_INTENSITY_LOW, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_GREEN
#  define FGR_WS2812_LED_COLOUR_BRIGHT_GREEN ((fgr_ws2812_colour_t) {0, FGR_WS2812_LED_INTENSITY_HIGH, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_YELLOW
#  define FGR_WS2812_LED_COLOUR_YELLOW ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_LOW, FGR_WS2812_LED_INTENSITY_LOW, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_YELLOW
#  define FGR_WS2812_LED_COLOUR_BRIGHT_YELLOW ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_HIGH, FGR_WS2812_LED_INTENSITY_HIGH, 0})
#endif

#ifndef FGR_WS2812_LED_COLOUR_CYAN
#  define FGR_WS2812_LED_COLOUR_CYAN ((fgr_ws2812_colour_t) {0, FGR_WS2812_LED_INTENSITY_LOW, FGR_WS2812_LED_INTENSITY_LOW})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_CYAN
#  define FGR_WS2812_LED_COLOUR_BRIGHT_CYAN ((fgr_ws2812_colour_t) {0, FGR_WS2812_LED_INTENSITY_HIGH, FGR_WS2812_LED_INTENSITY_HIGH})
#endif

#ifndef FGR_WS2812_LED_COLOUR_MAGENTA
#  define FGR_WS2812_LED_COLOUR_MAGENTA ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_LOW, 0, FGR_WS2812_LED_INTENSITY_LOW})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_MAGENTA
#  define FGR_WS2812_LED_COLOUR_BRIGHT_MAGENTA ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_HIGH, 0, FGR_WS2812_LED_INTENSITY_HIGH})
#endif

#ifndef FGR_WS2812_LED_COLOUR_WHITE
#  define FGR_WS2812_LED_COLOUR_WHITE ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_LOW, FGR_WS2812_LED_INTENSITY_LOW, FGR_WS2812_LED_INTENSITY_LOW})
#endif

#ifndef FGR_WS2812_LED_COLOUR_BRIGHT_WHITE
#  define FGR_WS2812_LED_COLOUR_BRIGHT_WHITE ((fgr_ws2812_colour_t) {FGR_WS2812_LED_INTENSITY_HIGH, FGR_WS2812_LED_INTENSITY_HIGH, FGR_WS2812_LED_INTENSITY_HIGH})
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** WS2812 LED colour.
 */
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} fgr_ws2812_colour_t;

/** Function to call to set an LED colour; used by fgr_ws2812_led_set_cb().
 *
 * @param colour a pointer to the current LED colour that may be
 *               modified by the caller; will not be NULL.
 * @param param  cb_param as passed to fgr_ws2812_led_set_cb().
 */
typedef void (*fgr_ws2812_led_set_cb_t) (fgr_ws2812_colour_t *colour,
                                         void *param);

/* ----------------------------------------------------------------
 * FUNCTIONS: INITIALISE/DEINITIALISE
 * -------------------------------------------------------------- */

/** Initialise WS2812 LED operations.  Needs a task so fgr_task_init()
 * must have been called first.  It is always safe to call this at any
 * time: if already initialised it will do nothing and return success.
 *
 * Note: this will create a semaphore that is never destroyed.
 *
 * @return ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_init();

/** Deinitialise WS2812 LED operations.  Any WS2812 LED chains that have
 * been initialised will be deinitialised with calls to
 * fgr_ws2812_chain_deinit().  It is always safe to call this function
 * at any time.
 */
void fgr_ws2812_deinit();

/** Create a WS2812 LED chain.
 *
 * @param spi_num      the SPI HW block to use for the chain; do not use 0
 *                     or 1 as those are used internally for flash memory
 *                     within the ESP32.
 * @param cs           the chip select for the LED chain, -1 if there is none.
 * @param pin          the GPIO pin that the LED chain is connected to.
 * @param count        the number of LEDs in the chain.
 * @param grb_not_rgb  set this to true if the WS2812 LEDs in the chain
 *                     require colours in order GRB rather than RGB.
 * @param handle       a pointer to a place to put the handle for the chain;
 *                     cannot be NULL.
 * @return             ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_chain_init(int32_t spi_num, int32_t cs, int32_t pin,
                              size_t count, bool grb_not_rgb, void **handle);

/** Destroy a WS2812 LED chain; this does not have to be called,
 * fgr_ws2812_deinit() will destroy any chains that have been created
 * automatically.  Note that the LED colour is not affected by this:
 * if you want the LEDs to be switched off you must do that explicitly
 * first.
 *
 * @param handle  a pointer to the handle that was returned by
 *                fgr_ws2812_chain_init().
 */
void fgr_ws2812_chain_deinit(void *handle);

/* ----------------------------------------------------------------
 * FUNCTIONS: SETTING LEDS
 * -------------------------------------------------------------- */

/** Set a WS2812 LED to the given colour.
 *
 * @param handle a pointer to the handle that was returned by
 *               fgr_ws2812_chain_init(); cannot be NULL.
 * @param led    the number of the LED in the chain to set, counting
 *               from 0.  Use -1 to set all of the LEDs in the chain.
 * @param colour the colour to set the LED(s) to.
 * @return       ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_set(void *handle, int32_t led,
                           fgr_ws2812_colour_t colour);

/** Set a WS2812 LED through a callback.  This will override any colour
 * set via fgr_ws2812_led_set().
 *
 * @param handle   a pointer to the handle that was returned by
 *                 fgr_ws2812_chain_init(); cannot be NULL.
 * @param led      the number of the LED in the chain to set, counting
 *                 from 0.  Use -1 to set all of the LEDs in the chain.
 * @param cb       a callback that will be called to set the LED; use
 *                 NULL to clear any callback and return to the colour
 *                 set via fgr_ws2812_led_set().
 * @param cb_param a parameter that will be passed to cb() when it is called.
 * @return         ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_set_cb(void *handle, int32_t led,
                              fgr_ws2812_led_set_cb_t cb, void *cb_param);

/** Flash the given LED.
 *
 * @param handle      a pointer to the handle that was returned by
 *                    fgr_ws2812_chain_init(); cannot be NULL.
 * @param led         the number of the LED in the chain to flash, counting
 *                    from 0.  Use -1 to flash all of the LEDs in the chain.
 * @param duration_ms how long to flash the LED for.
 * @param colour      the colour to flash.
 * @return            ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_flash(void *handle, int32_t led,
                             int32_t duration_ms, fgr_ws2812_colour_t colour);

/** Make the given LED "breathe".
 *
 * @param handle  a pointer to the handle that was returned by
 *                fgr_ws2812_chain_init(); cannot be NULL.
 * @param led     the number of the LED in the chain to set to breathing,
 *                counting from 0.  Use -1 to set all of the LEDs
 *                in the chain to breathing.
 * @return        ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_breathe_on(void *handle, int32_t led);

/** Turn the "breathe" effect off for the given LED.
 *
 * @param handle  a pointer to the handle that was returned by
 *                fgr_ws2812_chain_init(); cannot be NULL.
 * @param led     the number of the LED in the chain to stop breathing,
 *                counting from 0.  Use -1 to stop all of the LEDs
 *                in the chain breathing.
 * @return        ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_breathe_off(void *handle, int32_t led);

/** Mask the given LED to off.
 *
 * @param handle  a pointer to the handle that was returned by
 *                fgr_ws2812_chain_init(); cannot be NULL.
 * @param led     the number of the LED in the chain to switch off,
 *                counting from 0.  Use -1 to switch off all of the LEDs.
 * @return        ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_mask_off(void *handle, int32_t led);

/** Turn the given LED back to on.
 *
 * @param handle  a pointer to the handle that was returned by
 *                fgr_ws2812_chain_init(); cannot be NULL.
 * @param led     the number of the LED in the chain to switch on,
 *                counting from 0.  Use -1 to switch off all of the LEDs.
 * @return        ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_ws2812_led_mask_on(void *handle, int32_t led);

#ifdef __cplusplus
}
#endif

/** @} */

#endif // _FGR_WS2812_H_

// End of file
