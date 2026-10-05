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
#include "freertos/task.h"
#include "freertos/queue.h"
#include "errno.h"
#include "ctype.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "sys/queue.h"

#include "fgr_util.h"
#include "fgr_task.h"

#include "fgr_ws2812.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "ws2812"

#ifndef FGR_WS2812_TASK_STACK_SIZE
// Stack size for the task that controls the LEDs.
#  define FGR_WS2812_TASK_STACK_SIZE (1024 * 4)
#endif

#ifndef LED_STEP_DURATION_MS
// How often the LED is reprogrammed: 20 ms is 50 Hz.
#  define LED_STEP_DURATION_MS 20
#endif

#ifndef LED_UPDATE_BREATHE_PERIOD_STEPS
// The number of steps, of duration LED_STEP_DURATION_MS, that
// constitute a "breath", i.e. one cycle of breating.
// 200 steps * 20ms = 4000ms (4 seconds)
#  define LED_UPDATE_BREATHE_PERIOD_STEPS 200
#endif

#ifndef FLASH_INTENSITY_BOOST_NUMERATOR
// Flash must be at least this much brighter than breath peak
// Value is numerator of a fraction with denominator 100 (e.g., 50 = 50% brighter)
#  define FLASH_INTENSITY_BOOST_NUMERATOR 50
#endif

#ifndef FLASH_MIN_INTENSITY
// Minimum intensity for any flash when breathing (0-255)
#  define FLASH_MIN_INTENSITY 128
#endif

#ifndef FGR_WS2812_FLASH_MIN_STEPS
// Minimum number of task ticks a flash should last, so that the
// ease curve is traversed with enough resolution to look like a
// smooth fade in/out rather than a spike.
#  define FGR_WS2812_FLASH_MIN_STEPS 20
#endif

// The WS2812 tri-colour LED, see datasheet here:
//
// https://www.normandled.com/upload/201607/WS2812B%20Mini%203535%20LED%20Datasheet.pdf
//
// ...is driven over a single wire as follows:
//
// Zero bit high for 400 ns +/- 150 ns
// Zero bit low for 850 ns +/- 150 ns
// One bit high for 800 ns +/- 150 ns
// One bit low for 450 ns +/- 150 ns
// Meaning of bits is 8 bits red then 8 bits green then 8 bits blue
// or 8 bits green then 8 bits red then 8 bits blue if CONFIG_FGR_DEBUG_LED_WS2812_GRB
// Order of transmission is MSB first, as is SPI
// End of group timing is to go low for > 50000 ns
//
//
// Therefore the frequency of transmission is 800 kHz
// (1000000000 / (400 + 850)) but we need to obey the specific
// timings for each bit, hence we run at 8 MHz, so 10 SPI bits
// per WS2812 bit, and send more 1's and 0's to meet the timings.
#define SPI_SPEED_HZ 8000000

// Given an SPI speed of 8 MHz, each SPI bit is 125 ns, so a
// good number of SPI bits high to represent a zero (400 ns)
// is 3 (3 * 125 ns = 375 ns).  The remaining bits, are zero
// so that is 11100000 (given SPI transmits MSB first) plus
// two bits of 00 to give a total of 10 making 1250 ns.
#define WS2812_ZERO 0x380

// Similarly a good number of SPI bits high to represent a one
// (800 ns) is 6 (6 * 125 ns = 750 ns), so 11111100 plus two
// bits of 00.
#define WS2812_ONE 0x3f0

// The number of SPI bits per WS2812 bit
#define SPI_BITS_PER_WS2812_BIT 10

// The number of SPI bits per WS2812 byte
#define SPI_BITS_PER_WS2812_BYTE (SPI_BITS_PER_WS2812_BIT * 8)

// Bytes needed for one LED (no reset): 30 SPI bytes.
#define SPI_BYTES_PER_WS2812_LED (SPI_BITS_PER_WS2812_BYTE * 3 / 8)

// The low time to add on the end to signal end of group
#define WS2812_END_OF_GROUP_SPI_BITS_LOW (51000 / (1000000000 / SPI_SPEED_HZ))

// Bytes needed for the whole chain plus the final reset.
#define SPI_TRANSACTION_BUFFER_LENGTH_BYTES(count)  ((SPI_BYTES_PER_WS2812_LED * (count)) + (WS2812_END_OF_GROUP_SPI_BITS_LOW / 8) + 1)

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// LED command type.
typedef enum {
    FGR_LED_CMD_TYPE_COLOUR,
    FGR_LED_CMD_TYPE_BREATHE,
    FGR_LED_CMD_TYPE_FLASH
} led_cmd_type_t;

// The LED command contents for command type FGR_LED_CMD_TYPE_COLOUR.
typedef struct {
    fgr_ws2812_colour_t colour;
} led_cmd_type_colour_t;

// The LED command contents for command type FGR_LED_CMD_TYPE_BREATHE.
typedef struct {
    size_t period_steps;  // Period in steps (each step = LED_STEP_DURATION_MS)
} led_cmd_type_breathe_t;

// The LED command contents for command type FGR_LED_CMD_TYPE_FLASH.
typedef struct {
    fgr_ws2812_colour_t colour;
    size_t duration_steps;  // Duration in steps (each step = LED_STEP_DURATION_MS)
} led_cmd_type_flash_t;

// Breathe state.
typedef struct {
    fgr_ws2812_colour_t colour_from;    // Colour we are fading from
    fgr_ws2812_colour_t colour_to;      // Colour we are fading to
    size_t transition_steps;            // 0 = no crossfade in progress
    size_t transition_counter;
    size_t period_steps;                // Period in steps (each step = LED_STEP_DURATION_MS)
    size_t step_counter;                // Current step in breathing cycle (0 to period_steps-1)
    uint8_t intensity;                  // Current breathing intensity (0-255)
} breathe_state_t;

// Flash state.
typedef struct {
    fgr_ws2812_colour_t colour;
    size_t duration_steps;   // Duration in steps (each step = LED_STEP_DURATION_MS)
    size_t step_counter;     // Current step (0 to total_steps-1)
} flash_state_t;

// Structure defining the state of an LED.
typedef struct led_state_t {
    fgr_ws2812_colour_t colour;
    fgr_ws2812_led_set_cb_t cb;
    void *cb_param;
    bool led_masked_off;
    breathe_state_t *breathe_state;
    flash_state_t *flash_state;
} led_state_t;

// Structure defining an LED chain, designed to be used as part of
// a linked list.
typedef struct chain_t {
    int32_t spi_num;
    int32_t cs;
    spi_device_handle_t spi;
    bool grb_not_rgb;
    size_t led_count;
    int32_t last_update_ms;
    led_state_t *led_state;
    uint8_t *spi_buffer;
    SLIST_ENTRY(chain_t) next;
} chain_t;

// Chain list head.
SLIST_HEAD(chain_list_t, chain_t);

// Union of different command contents.
typedef union {
    led_cmd_type_colour_t colour;
    led_cmd_type_breathe_t breathe;
    led_cmd_type_flash_t flash;
} led_cmd_contents_t;

// The command structure to be transferred on a queue to the LED task.
typedef struct {
    chain_t *chain;
    int32_t led;
    led_cmd_type_t type;
    led_cmd_contents_t contents;
} led_cmd_t;

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    TaskHandle_t task_handle;
    QueueHandle_t queue_handle;
    struct chain_list_t chain_list;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

// Sine lookup table: values from 0 to 255.
static const uint8_t g_sine_table[] = {
    128, 140, 153, 165, 177, 188, 199, 209,
    218, 226, 234, 240, 245, 250, 253, 255,
    255, 255, 253, 250, 245, 240, 234, 226,
    218, 209, 199, 188, 177, 165, 153, 140,
    128, 115, 102,  90,  78,  67,  56,  46,
    37,  29,  21,  15,  10,   5,   2,   0,
    0,   0,   2,   5,  10,  15,  21,  29,
    37,  46,  56,  67,  78,  90, 102, 115
};

// Flash ease table: a smooth hump from 0 to 255 and back to 0,
// generated with a smoothstep curve so the attack and decay are
// both gentle.  Peak at indices 31/32 (the midpoint), symmetric
// about that point.  64 entries.
static const uint8_t g_flash_ease_table[] = {
      0,   1,   3,   7,  12,  18,  25,  33,
     42,  52,  62,  73,  85,  97, 109, 121,
    134, 146, 158, 170, 182, 193, 203, 213,
    222, 230, 237, 243, 248, 252, 254, 255,
    255, 254, 252, 248, 243, 237, 230, 222,
    213, 203, 193, 182, 170, 158, 146, 134,
    121, 109,  97,  85,  73,  62,  52,  42,
     33,  25,  18,  12,   7,   3,   1,   0
};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: SPI RELATED
 * -------------------------------------------------------------- */

// Encode one WS2812 bit (10 SPI bits) into the buffer, MSB first.
// Advances *buffer_ptr and *bit_offset as needed.
static inline void ws2812_encode_bit(bool oneNotZero,
                                     uint8_t **buffer_ptr, size_t *bit_offset)
{
    uint32_t pattern = oneNotZero ? WS2812_ONE : WS2812_ZERO;

    for (int32_t i = SPI_BITS_PER_WS2812_BIT - 1; i >= 0; i--) {
        if (*bit_offset == 0) {
            **buffer_ptr = 0;
        }
        if ((pattern >> i) & 1) {
            **buffer_ptr |= 0x80 >> *bit_offset;
        }
        (*bit_offset)++;
        if (*bit_offset == 8) {
            *bit_offset = 0;
            (*buffer_ptr)++;
        }
    }
}

// Encode one WS2812 byte (8 bits, MSB first).
static inline void ws2812_encode_byte(uint8_t byte_ws2812,
                                      uint8_t **buffer_ptr, size_t *bit_offset)
{
    for (int32_t x = 7; x >= 0; x--) {
        ws2812_encode_bit(byte_ws2812 & (1 << x), buffer_ptr, bit_offset);
    }
}

// Encode one LED's 24 bits (30 SPI bytes) with no reset pulse.
static size_t ws2812_encode_led(fgr_ws2812_colour_t *colour,
                                bool grb_not_rgb,
                                uint8_t *buffer, size_t length)
{
    size_t encoded_length_bits = 0;

    if (colour && buffer && (length >= SPI_BYTES_PER_WS2812_LED)) {
        uint8_t *buffer_ptr = buffer;
        size_t bit_offset = 0;

        if (grb_not_rgb) {
            ws2812_encode_byte(colour->green, &buffer_ptr, &bit_offset);
            ws2812_encode_byte(colour->red,   &buffer_ptr, &bit_offset);
        } else {
            ws2812_encode_byte(colour->red,   &buffer_ptr, &bit_offset);
            ws2812_encode_byte(colour->green, &buffer_ptr, &bit_offset);
        }
        ws2812_encode_byte(colour->blue, &buffer_ptr, &bit_offset);
        encoded_length_bits = SPI_BITS_PER_WS2812_BYTE * 3;
    }

    return encoded_length_bits;
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: COLOUR RELATED
 * -------------------------------------------------------------- */

// Apply intensity to colour.
static fgr_ws2812_colour_t apply_intensity(fgr_ws2812_colour_t colour,
                                           uint8_t intensity)
{
    fgr_ws2812_colour_t result;

    // (colour * intensity) / 255 - all integer math
    result.red = (uint16_t)colour.red * intensity / 255;
    result.green = (uint16_t)colour.green * intensity / 255;
    result.blue = (uint16_t)colour.blue * intensity / 255;

    return result;
}

// Linear interpolation between two 8-bit values; t is 0..255.
static inline uint8_t lerp8(uint8_t a, uint8_t b, uint32_t t)
{
    return (uint8_t) (((uint32_t) a * (255 - t) + (uint32_t) b * t) / 255);
}

// Return the breathe colour currently being displayed, taking any
// in-flight crossfade into account.
static fgr_ws2812_colour_t current_breathe_colour(breathe_state_t *breathe_state)
{
    fgr_ws2812_colour_t result = breathe_state->colour_to;

    if (breathe_state->transition_steps > 0) {
        uint32_t t = (breathe_state->transition_counter * 255) /
                     breathe_state->transition_steps;
        result.red   = lerp8(breathe_state->colour_from.red,
                             breathe_state->colour_to.red,   t);
        result.green = lerp8(breathe_state->colour_from.green,
                             breathe_state->colour_to.green, t);
        result.blue  = lerp8(breathe_state->colour_from.blue,
                             breathe_state->colour_to.blue,  t);
    }

    return result;
}

// Calculate boosted flash colour to ensure it is noticeably brighter than breath.
static void boost_flash_intensity(fgr_ws2812_colour_t *flash_colour,
                                  fgr_ws2812_colour_t breathe_colour,
                                  uint8_t breathe_intensity)
{

    if (flash_colour) {
        fgr_ws2812_colour_t result = {0, 0, 0};

        // Calculate the current maximum brightness of any channel in the breath
        uint16_t breathe_peak = 0;
        uint16_t breathe_red = ((uint16_t) breathe_colour.red) * breathe_intensity / 255;
        uint16_t breathe_green = ((uint16_t) breathe_colour.green) * breathe_intensity / 255;
        uint16_t breathe_blue = ((uint16_t) breathe_colour.blue) * breathe_intensity / 255;

        if (breathe_red > breathe_peak) {
            breathe_peak = breathe_red;
        }
        if (breathe_green > breathe_peak) {
            breathe_peak = breathe_green;
        }
        if (breathe_blue > breathe_peak) {
            breathe_peak = breathe_blue;
        }

        // Calculate required minimum intensity for flash: breath_peak * (100 + boost) / 100
        uint16_t required_intensity = breathe_peak * (100 + FLASH_INTENSITY_BOOST_NUMERATOR) / 100;
        if (required_intensity > 255) {
            required_intensity = 255;
        }
        if (required_intensity < FLASH_MIN_INTENSITY) {
            required_intensity = FLASH_MIN_INTENSITY;
        }

        // Find the maximum channel value in the requested flash colour
        uint16_t flash_max = 0;
        if (flash_colour->red > flash_max) {
            flash_max = flash_colour->red;
        }
        if (flash_colour->green > flash_max) {
            flash_max = flash_colour->green;
        }
        if (flash_colour->blue > flash_max) {
            flash_max = flash_colour->blue;
        }

        if (flash_max > 0) {
            // Scale all channels proportionally: (colour * required) / flash_max
            result.red = ((uint16_t) flash_colour->red) * required_intensity / flash_max;
            result.green = ((uint16_t) flash_colour->green) * required_intensity / flash_max;
            result.blue = ((uint16_t) flash_colour->blue) * required_intensity / flash_max;
        } else {
            // Flash colour is black - use pure white at required intensity
            result.red = required_intensity;
            result.green = required_intensity;
            result.blue = required_intensity;
        }

        *flash_colour = result;
    }
}

// Update breathing intensity using lookup table, remapped into
// [FGR_WS2812_BREATHE_MIN_INTENSITY .. FGR_WS2812_BREATHE_MAX_INTENSITY]
// so the LED never sits in the perceptually-dark region of the PWM range.
// IMPORTANT: the context should be locked before this is called.
static void update_breathe_intensity(breathe_state_t *breathe_state)
{
    if (breathe_state && (breathe_state->period_steps != 0)) {
        // Map step_counter to sine table index
        // We want a full sine wave over the period
        uint32_t index = (breathe_state->step_counter * FGR_UTIL_ARRAY_LENGTH(
                          g_sine_table)) / breathe_state->period_steps;
        index &= FGR_UTIL_ARRAY_LENGTH(g_sine_table) - 1;  // Ensure within bounds
        uint8_t sine = g_sine_table[index];

        // Remap [0..255] -> [MIN..MAX]
        breathe_state->intensity = FGR_WS2812_BREATHE_MIN_INTENSITY +
                                   ((uint16_t) (FGR_WS2812_BREATHE_MAX_INTENSITY -
                                   FGR_WS2812_BREATHE_MIN_INTENSITY) * sine) / 255;

        // Advance step counter
        breathe_state->step_counter++;
        if (breathe_state->step_counter >= breathe_state->period_steps) {
            breathe_state->step_counter = 0;
        }
    }
}

// Update flash intensity using lookup table
// IMPORTANT: the context should be locked before this is called.
static void update_flash_intensity(flash_state_t *flash_state,
                                   fgr_ws2812_colour_t *colour)
{
    if (flash_state && (flash_state->duration_steps > 0)) {

        // Traverse the ease table fully from index 0 to the last
        // entry, regardless of duration, so the flash always fades
        // in and out rather than being truncated.
        uint32_t denom = (flash_state->duration_steps > 1) ?
                         (flash_state->duration_steps - 1) : 1;
        uint32_t index = (flash_state->step_counter *
                          (FGR_UTIL_ARRAY_LENGTH(g_flash_ease_table) - 1)) / denom;
        uint8_t intensity = g_flash_ease_table[index];
        *colour = apply_intensity(flash_state->colour, intensity);

        flash_state->step_counter++;
        if (flash_state->step_counter >= flash_state->duration_steps) {
            flash_state->duration_steps = 0;
        }
    }
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: TASK RELATED
 * -------------------------------------------------------------- */

// Update physical WS2812 LED chain.
// IMPORTANT: the context should be locked before this is called.
static void update_chain(chain_t *chain)
{
    if (chain->spi) {
        spi_transaction_t transaction = {0};
        uint8_t *write = chain->spi_buffer;
        led_state_t *led_state = chain->led_state;
        for (size_t x = 0; x < chain->led_count; x++) {
            // Run through all of the LEDs and build up a buffer
            // that will be the SPI transaction
            breathe_state_t *breathe_state = led_state->breathe_state;
            flash_state_t *flash_state = led_state->flash_state;

            if (led_state->cb) {
                // Update LED colour from callback.  The callback may
                // change the colour; if it does, and the LED is
                // breathing, kick off a crossfade so the change is
                // smooth rather than a snap.
                fgr_ws2812_colour_t new_colour = led_state->colour;
                led_state->cb(&new_colour, led_state->cb_param);

                if ((new_colour.red   != led_state->colour.red) ||
                    (new_colour.green != led_state->colour.green) ||
                    (new_colour.blue  != led_state->colour.blue)) {

                    if (breathe_state && (breathe_state->period_steps > 0)) {
                        // Breathing: start a crossfade from whatever is
                        // currently displayed to the new colour.
                        breathe_state->colour_from =
                            current_breathe_colour(breathe_state);
                        breathe_state->colour_to = new_colour;
                        breathe_state->transition_steps =
                            (FGR_WS2812_BREATHE_FADE_MS + LED_STEP_DURATION_MS - 1) /
                            LED_STEP_DURATION_MS;
                        breathe_state->transition_counter = 0;
                    }

                    led_state->colour = new_colour;
                }
            }

            // Update the breathe intensity
            update_breathe_intensity(breathe_state);

            fgr_ws2812_colour_t final_colour = led_state->colour;
            fgr_ws2812_colour_t temp_colour;

            // Priority order: Mask > Flash > Breathe
            if (led_state->led_masked_off) {
                final_colour = FGR_WS2812_LED_COLOUR_NONE;
            } else if (flash_state && (flash_state->duration_steps > 0)) {
                temp_colour = flash_state->colour;
                update_flash_intensity(flash_state, &temp_colour);
                final_colour = temp_colour;
            } else if (breathe_state && (breathe_state->period_steps > 0)) {
                fgr_ws2812_colour_t breathe_colour =
                    current_breathe_colour(breathe_state);
                final_colour = apply_intensity(breathe_colour, breathe_state->intensity);

                // Advance any in-flight crossfade
                if (breathe_state->transition_steps > 0) {
                    breathe_state->transition_counter++;
                    if (breathe_state->transition_counter >=
                        breathe_state->transition_steps) {
                        breathe_state->transition_steps = 0;
                        breathe_state->colour_from = breathe_state->colour_to;
                    }
                }
            }

            // Add this transaction to the buffer
            ws2812_encode_led(&final_colour, chain->grb_not_rgb, write, SPI_BYTES_PER_WS2812_LED);
            write += SPI_BYTES_PER_WS2812_LED;
            transaction.length += SPI_BITS_PER_WS2812_BYTE * 3;

            // Next...
            led_state++;
        }

        // The reset/end-of-group low pulse goes once, after the last LED in the chain.
        memset(write, 0, WS2812_END_OF_GROUP_SPI_BITS_LOW / 8);
        transaction.length += WS2812_END_OF_GROUP_SPI_BITS_LOW;
        // Perform the SPI transaction
        transaction.tx_buffer = chain->spi_buffer;
        spi_device_transmit(chain->spi, &transaction);
    }
}

// Set the colour of an LED; called by process_cmd().
static void cmd_set_colour(led_state_t *led_state, led_cmd_type_colour_t *cmd)
{
    breathe_state_t *breathe_state = led_state->breathe_state;

    if (breathe_state && (breathe_state->period_steps > 0)) {
        // Breathing: start a crossfade from whatever is currently
        // displayed to the new colour, so the change is smooth.
        breathe_state->colour_from = current_breathe_colour(breathe_state);
        breathe_state->colour_to = cmd->colour;
        breathe_state->transition_steps = (FGR_WS2812_BREATHE_FADE_MS + LED_STEP_DURATION_MS - 1) /
                                          LED_STEP_DURATION_MS;
        breathe_state->transition_counter = 0;
    }

    // Always update the base colour too
    led_state->colour = cmd->colour;
}

// Set flashing of an LED; called by process_cmd().
static void cmd_set_flash(led_state_t *led_state, led_cmd_type_flash_t *cmd)
{
    flash_state_t *flash_state = led_state->flash_state;

    if (flash_state) {
        flash_state->duration_steps = cmd->duration_steps;
        flash_state->colour = cmd->colour;
        flash_state->step_counter = 0;
        if (led_state->breathe_state &&
            (led_state->breathe_state->period_steps > 0)) {
            // Boost against the breathe peak, not the current
            // instant, so the flash is always sized to beat the
            // brightest the breathe ever gets.
            boost_flash_intensity(&flash_state->colour,
                                  led_state->colour,
                                  255);
        }
    }
}

// Set breathing of an LED; called by process_cmd().
static void cmd_set_breathe(led_state_t *led_state, led_cmd_type_breathe_t *cmd)
{
    breathe_state_t *breathe_state = led_state->breathe_state;

    if (breathe_state) {
        breathe_state->period_steps = cmd->period_steps;
        breathe_state->step_counter = 0;
        if (breathe_state->transition_steps == 0) {
            // No crossfade in flight: seed both ends with the
            // current base colour so the first breath is correct.
            breathe_state->colour_from = led_state->colour;
            breathe_state->colour_to   = led_state->colour;
        }
    }
}

// Process a command directed at one LED; called by process_cmd_n().
static void process_cmd(led_state_t *led_state, led_cmd_type_t type,
                        led_cmd_contents_t *contents)
{
    switch (type) {
        case FGR_LED_CMD_TYPE_COLOUR:
            cmd_set_colour(led_state, &contents->colour);
            break;
        case FGR_LED_CMD_TYPE_FLASH:
            cmd_set_flash(led_state, &contents->flash);
            break;
        case FGR_LED_CMD_TYPE_BREATHE:
            cmd_set_breathe(led_state, &contents->breathe);
            break;
        default:
            break;
    }
}

// Process a command directed at one or all LEDs; called by task_ws2812_cb().
static void process_cmd_n(chain_t *chain, int32_t led, led_cmd_type_t type,
                          led_cmd_contents_t *contents)
{
    led_state_t *led_state = chain->led_state;

    if (led >= 0) {
        if (led < chain->led_count) {
            led_state += led;
            process_cmd(led_state, type, contents);
        } else {
            ESP_LOGE(TAG, "led %d out of range (max %u)!", led, chain->led_count);
        }
    } else {
        for (size_t x = 0; x < chain->led_count; x++) {
            process_cmd(led_state, type, contents);
            led_state++;
        }
    }
}

// WS2812 LED task callback
static void task_ws2812_cb(void *handle, void *param)
{
    context_t *context = (context_t *) param;
    led_cmd_t cmd;

    (void) handle;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    CONTEXT_LOCK(context->lock, "task_ws2812_cb()");

    // Process all pending messages (non-blocking)
    while (xQueueReceive(context->queue_handle, &cmd, 0) == pdTRUE) {
        // For any command type, check that the chain still exists
        chain_t *iter = NULL;
        SLIST_FOREACH(iter, &context->chain_list, next) {
            if (iter == cmd.chain) {
                break;
            }
        }
        if (iter != NULL) {
            // It is now safe to use cmd.chain
            process_cmd_n(cmd.chain, cmd.led, cmd.type, &cmd.contents);
        }
    }

    // Run through all chains
    chain_t *chain = NULL;
    SLIST_FOREACH(chain, &context->chain_list, next) {
        // Update chains at consistent intervals
        if ((now_ms - chain->last_update_ms) >= LED_STEP_DURATION_MS) {
            update_chain(chain);
            chain->last_update_ms = now_ms;
        }
    }

    CONTEXT_UNLOCK(context->lock, "task_ws2812_cb()");
}

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS: MISC
 * -------------------------------------------------------------- */

// Mallloc()/free() memory for LED breathe state.
static int32_t led_breathe_state_malloc_free(led_state_t *led_state, bool breathe)
{
    int32_t err = -ESP_ERR_NOT_FOUND;

    if (led_state) {
        err = ESP_OK;
        if (breathe) {
            if (led_state->breathe_state == NULL) {
                err = -ESP_ERR_NO_MEM;
                led_state->breathe_state = (breathe_state_t *) malloc(sizeof(*led_state->breathe_state));
                if (led_state->breathe_state) {
                    memset(led_state->breathe_state, 0, sizeof(*led_state->breathe_state));
                    err = ESP_OK;
                }
            }
        } else {
            free(led_state->breathe_state);
            led_state->breathe_state = NULL;
        }
    }

    return err;
}

// Mallloc()/free() memory for LED flash state.
static int32_t led_flash_state_malloc_free(led_state_t *led_state, bool flash)
{
    int32_t err = -ESP_ERR_NOT_FOUND;

    if (led_state) {
        err = ESP_OK;
        if (flash) {
            if (led_state->flash_state == NULL) {
                err = -ESP_ERR_NO_MEM;
                led_state->flash_state = (flash_state_t *) malloc(sizeof(*led_state->flash_state));
                if (led_state->flash_state) {
                    memset(led_state->flash_state, 0, sizeof(*led_state->flash_state));
                    err = ESP_OK;
                }
            }
        } else {
            free(led_state->flash_state);
            led_state->flash_state = NULL;
        }
    }

    return err;
}

// Free an LED chain.
static void chain_free(chain_t *chain)
{
    if (chain) {
        led_state_t *led_state = chain->led_state;
        for (size_t x = 0; x < chain->led_count; x++) {
            led_breathe_state_malloc_free(led_state, false);
            led_flash_state_malloc_free(led_state, false);
            led_state++;
        }
        heap_caps_free(chain->spi_buffer);
        free(chain);
    }
}

// Destroy a WS2812 LED chain.
// IMPORTANT: the context should be locked before this is called.
static void chain_deinit(chain_t *chain)
{
    int32_t spi_num = -1;

    chain_t *iter;
    chain_t *prev = NULL;
    SLIST_FOREACH(iter, &g_context.chain_list, next) {
        if (iter == chain) {
            // Found it: remove it from the list
            if (prev == NULL) {
                // Removing the first element
                SLIST_REMOVE_HEAD(&g_context.chain_list, next);
            } else {
                // Removing a middle element
                SLIST_REMOVE_AFTER(prev, next);
            }
            // Remove the SPI device and free memory, but
            // remember the SPI number for the following check
            spi_bus_remove_device(iter->spi);
            spi_num = iter->spi_num;
            chain_free(iter);
            // Done; MUST break after an insertion or removal as
            // otherwise SLIST_FOREACH will go bang as it
            // relies on pointers still being valid.
            break;
        }
    }

    if (spi_num >= 0) {
        // Check if the SPI bus is still in use
        chain_t *iter = NULL;
        SLIST_FOREACH(iter, &g_context.chain_list, next) {
            if (iter->spi_num == spi_num) {
                break;
            }
        }
        if (iter == NULL) {
            // Not found: free the bus also
            spi_bus_free(spi_num);
        }
    }
}

// Get an LED from a chain.
static led_state_t *get_led_state(chain_t *chain, int32_t led)
{
    led_state_t *led_state = NULL;

    if (chain && (led < chain->led_count)) {
        led_state = (chain->led_state + led);
    }

    return led_state;
}

// Have the given LED breathe (or not).
static int32_t led_breathe_on(void *handle, int32_t led, bool breathe)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "led_breathe_on()");

        err = -ESP_ERR_INVALID_ARG;
        if (handle) {
            chain_t *chain = (chain_t *) handle;
            if (led >= 0) {
                err = -ESP_ERR_NOT_FOUND;
                led_state_t *led_state = get_led_state(chain, led);
                if (led_state) {
                    if (breathe) {
                        err = led_breathe_state_malloc_free(led_state, true);
                    } else {
                        err = ESP_OK;
                    }
                }
            } else {
                err = ESP_OK;
                led_state_t *led_state = chain->led_state;
                if (breathe) {
                    for (size_t x = 0; (x < chain->led_count) && (err == ESP_OK); x++) {
                        err = led_breathe_state_malloc_free(led_state, true);
                        led_state++;
                    }
                }
            }
            if (err == ESP_OK) {
                // Signal the change to the LED task
                led_cmd_t cmd = {
                    .chain = chain,
                    .led = led,
                    .type = FGR_LED_CMD_TYPE_BREATHE,
                    .contents.breathe.period_steps = breathe ? LED_UPDATE_BREATHE_PERIOD_STEPS : 0
                };
                xQueueSend(g_context.queue_handle, &cmd, portMAX_DELAY);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "led_breathe_on()");
    }

    return err;
}

// Mask (or not) the given LED.
static int32_t led_masked_off(void *handle, int32_t led, bool masked)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "led_masked_off()");

        err = -ESP_ERR_INVALID_ARG;
        if (handle) {
            chain_t *chain = (chain_t *) handle;
            if (led >= 0) {
                err = -ESP_ERR_NOT_FOUND;
                led_state_t *led_state = get_led_state(chain, led);
                if (led_state) {
                    led_state->led_masked_off = masked;
                    err = ESP_OK;
                }
            } else {
                err = ESP_OK;
                led_state_t *led_state = chain->led_state;
                for (size_t x = 0; x < chain->led_count; x++) {
                    led_state->led_masked_off = masked;
                    led_state++;
                }
            }
            // Nothing to signal here, all done statically above.
        }

        CONTEXT_UNLOCK(g_context.lock, "led_masked_off()");
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: INITIALISATION/DEINITIALISATION
 * -------------------------------------------------------------- */

// Initialise WS2812 stuff.
int32_t fgr_ws2812_init()
{
    esp_err_t err = ESP_ERR_NO_MEM;

    if (!g_context.lock) {
        g_context.lock = xSemaphoreCreateMutex();
    }

    if (g_context.lock) {
        err = ESP_OK;

        if (!g_context.queue_handle) {

            CONTEXT_LOCK(g_context.lock, "fgr_ws2812_init()");

            g_context.queue_handle = xQueueCreate(10, sizeof(led_cmd_t));
            if (g_context.queue_handle) {
                err = fgr_task_create(&task_ws2812_cb, &g_context, "ws2812",
                                      FGR_WS2812_TASK_STACK_SIZE,
                                      3, &g_context.task_handle);
                if (err != ESP_OK) {
                    vQueueDelete(g_context.queue_handle);
                    g_context.queue_handle = NULL;
                }
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_init()");
        }
    }

    // Returns ESP_OK or negative error code from esp_err_t
    return (int32_t) -err;
}

// Deinitialise WS2812 stuff.
void fgr_ws2812_deinit()
{
    if (g_context.queue_handle) {

        // Need to do this before taking the lock or we
        // will lock-up the task exit
        fgr_task_destroy(g_context.task_handle);
        g_context.task_handle = NULL;

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_deinit()");

        vQueueDelete(g_context.queue_handle);
        g_context.queue_handle = NULL;

        // Deinitialise all of the LED chains
        while (!SLIST_EMPTY(&g_context.chain_list)) {
            chain_t *chain = SLIST_FIRST(&g_context.chain_list);
            chain_deinit(chain);
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_deinit()");
        // The semaphore will be re-used
    }
}

// Create a WS2812 LED chain.
int32_t fgr_ws2812_chain_init(int32_t spi_num, int32_t cs, int32_t pin,
                              size_t count, bool grb_not_rgb, void **handle)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_chain_init()");

        err = -ESP_ERR_INVALID_ARG;
        bool spi_already_initialised = false;
        if (handle && (spi_num > 0) && (count > 0)) { // SPIs 0 is used internally
            // Check if the SPI/cs combination is already in use
            chain_t *iter;
            err = ESP_OK;
            SLIST_FOREACH(iter, &g_context.chain_list, next) {
                if (iter->spi_num == spi_num) {
                    spi_already_initialised = true;
                    if ((iter->cs == -1) || (cs == -1) || (iter->cs == cs)) {
                        // Already have this SPI/cs combination
                        err = -ESP_ERR_NOT_ALLOWED;
                        break;
                    }
                }
            }
        }

        if (err == ESP_OK) {
            // Create the chain
            err = -ESP_ERR_NO_MEM;
            // Allocate memory for the chain and the LEDs all at once
            chain_t *chain = (chain_t *) malloc(sizeof(*chain) + (sizeof(*chain->led_state) * count));
            if (chain) {
                memset(chain, 0, sizeof(*chain));
                chain->spi_num = spi_num;
                chain->cs = cs;
                chain->grb_not_rgb = grb_not_rgb;
                chain->led_count = count;
                // Point memory for LED state to just after the chain bit out of all that we malloc()ed
                chain->led_state = (led_state_t *) ((char *) chain + sizeof(*chain));
                memset(chain->led_state, 0, sizeof(*chain->led_state) * count);
                // Now allocate the buffer we will need for the SPI transaction
                chain->spi_buffer = (uint8_t *) heap_caps_malloc(SPI_TRANSACTION_BUFFER_LENGTH_BYTES(count),
                                                                 MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
                if (chain->spi_buffer) {
                    err = ESP_OK;
                }
            }

            if (err == ESP_OK) {
                if (!spi_already_initialised) {
                    // Initialise the SPI bus
                    spi_bus_config_t bus_cfg = {
                        .mosi_io_num = pin,
                        .miso_io_num = -1,
                        // We don't need an SCLK pin, however for SPI 1 (which shares HW with the
                        // SPI 0 stuff), ESP-IDF performs extra validaton and _requires_ there to
                        // be an SCLK pin in all cases.  Here this is assigned to GPIO 42, which
                        // should be well out of the way of anything.
                        .sclk_io_num = (spi_num == 1) ? 42 : -1,
                        .quadwp_io_num = -1,
                        .quadhd_io_num = -1
                    };
                    err = spi_bus_initialize(spi_num, &bus_cfg, SPI_DMA_CH_AUTO);
                }

                if (err == ESP_OK) {
                    // Initialise the SPI device.
                    spi_device_interface_config_t dev_cfg = {
                        .clock_speed_hz = SPI_SPEED_HZ,
                        .mode = 0,
                        .spics_io_num = cs,
                        .queue_size = 7
                    };
                    err = spi_bus_add_device(spi_num, &dev_cfg, &chain->spi);
                    if (err == ESP_OK) {
                        // All is good, add the chain to the list and return
                        // the chain in handle
                        SLIST_INSERT_HEAD(&g_context.chain_list, chain, next);
                        *handle = chain;
                    } else {
                        if (!spi_already_initialised) {
                            // Free the SPI bus again if no-one else was using it
                            spi_bus_free(spi_num);
                        }
                        ESP_LOGE(TAG, "spi_bus_add_device() to SPI %d failed (%s)!",
                                 spi_num, esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGE(TAG, "spi_bus_initialize() on SPI %d failed (%s)!",
                            spi_num, esp_err_to_name(err));
                }
                // Return ESP_OK or negative error code from esp_err_t
                err = -err;
            }

            if (err != ESP_OK) {
                // Release memory on error
                chain_free(chain);
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_chain_init()");
        }
    }

    return err;
}

// Destroy a WS2812 LED chain.
void fgr_ws2812_chain_deinit(void *handle)
{
    if (g_context.queue_handle && handle) {

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_chain_deinit()");

        chain_deinit((chain_t *) handle);

        CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_chain_deinit()");
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: SETTING LEDS
 * -------------------------------------------------------------- */

// Set a WS2812 LED to the given colour.
int32_t fgr_ws2812_led_set(void *handle, int32_t led,
                           fgr_ws2812_colour_t colour)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_led_set()");

        err = -ESP_ERR_INVALID_ARG;
        if (handle) {
            chain_t *chain = (chain_t *) handle;
            if (led >= 0) {
                err = -ESP_ERR_NOT_FOUND;
                led_state_t *led_state = get_led_state(chain, led);
                if (led_state) {
                    err = ESP_OK;
                    led_state->colour = colour;
                }
            } else {
                err = ESP_OK;
                led_state_t *led_state = chain->led_state;
                for (size_t x = 0; x < chain->led_count; x++) {
                    led_state->colour = colour;
                    led_state++;
                }
            }
            if (err == ESP_OK) {
                // Signal the change to the LED task
                led_cmd_t cmd = {
                    .chain = chain,
                    .led = led,
                    .type = FGR_LED_CMD_TYPE_COLOUR,
                    .contents.colour.colour = colour
                };
                xQueueSend(g_context.queue_handle, &cmd, portMAX_DELAY);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_led_set()");
    }

    return err;
}

// Set a WS2812 LED through a callback.
int32_t fgr_ws2812_led_set_cb(void *handle, int32_t led,
                              fgr_ws2812_led_set_cb_t cb,
                              void *cb_param)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_led_set_cb()");

        err = -ESP_ERR_INVALID_ARG;
        if (handle) {
            chain_t *chain = (chain_t *) handle;
            if (led >= 0) {
                err = -ESP_ERR_NOT_FOUND;
                led_state_t *led_state = get_led_state(chain, led);
                if (led_state) {
                    err = ESP_OK;
                    led_state->cb = cb;
                    led_state->cb_param = cb_param;
                }
            } else {
                err = ESP_OK;
                led_state_t *led_state = chain->led_state;
                for (size_t x = 0; x < chain->led_count; x++) {
                    led_state->cb = cb;
                    led_state->cb_param = cb_param;
                    led_state++;
                }
            }
            // Nothing to signal here, all done statically above.
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_led_set_cb()");
    }

    return err;
}

// Flash the given LED.
int32_t fgr_ws2812_led_flash(void *handle, int32_t led,
                             int32_t duration_ms, fgr_ws2812_colour_t colour)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.queue_handle) {

        CONTEXT_LOCK(g_context.lock, "fgr_ws2812_led_flash()");

        err = -ESP_ERR_INVALID_ARG;
        if (handle) {
            // Convert milliseconds to steps (rounded up), enforcing
            // a minimum so the ease curve is traversed smoothly.
            uint32_t steps = (duration_ms + LED_STEP_DURATION_MS - 1) / LED_STEP_DURATION_MS;
            if (steps < FGR_WS2812_FLASH_MIN_STEPS) {
                steps = FGR_WS2812_FLASH_MIN_STEPS;
            }
            chain_t *chain = (chain_t *) handle;
            // Allocate memory if necessary
            if (led >= 0) {
                led_state_t *led_state = get_led_state(chain, led);
                err = led_flash_state_malloc_free(led_state, true);
            } else {
                err = ESP_OK;
                led_state_t *led_state = chain->led_state;
                for (size_t x = 0; (x < chain->led_count) && (err == ESP_OK); x++) {
                    err = led_flash_state_malloc_free(led_state, true);
                    led_state++;
                }
            }
            if (err == ESP_OK) {
                // Signal the change to the LED task
                led_cmd_t cmd = {
                    .chain = chain,
                    .led = led,
                    .type = FGR_LED_CMD_TYPE_FLASH,
                    .contents.flash.colour = colour,
                    .contents.flash.duration_steps = steps
                };
                xQueueSend(g_context.queue_handle, &cmd, portMAX_DELAY);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_ws2812_led_flash()");
    }

    return err;
}

// Make the given LED "breathe".
int32_t fgr_ws2812_led_breathe_on(void *handle, int32_t led)
{
    return led_breathe_on(handle, led, true);
}

// Turn the "breathe" effect off for the given LED.
int32_t fgr_ws2812_led_breathe_off(void *handle, int32_t led)
{
    return led_breathe_on(handle, led, false);
}

// Mask the given LED to off.
int32_t fgr_ws2812_led_mask_off(void *handle, int32_t led)
{
    return led_masked_off(handle, led, true);
}

// Remove the mask for a given LED.
int32_t fgr_ws2812_led_mask_on(void *handle, int32_t led)
{
    return led_masked_off(handle, led, false);
}

// End of file
