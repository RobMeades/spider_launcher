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
 * @brief Task related functions for a node of the front garden railway.
 */

// Ensure we are compiling with maximum debug, can then be trimmed
// at run-time by fgr_log
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "sys/queue.h"

#include "fgr_util.h"

#include "motor_brushless.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "brushless_motor"

// Set the timer resolution for PWM: with a value of 10000000 we
// get 10 MHz, 1 tick = 0.1us
#define TIMER_RESOLUTION_HERTZ 100000000

// Set the period of the timer for PWM: a value of 500
// with a timer resolution of 1 MHz gives 50us, 20 KHz
#define TIMER_PERIOD_TICKS 500

// How often to sample a sensor pin in milliseconds
#define SENSOR_SAMPLE_MS 1

// The maximum confidence value
#define SENSOR_CONFIDENCE_MAX 8   // max confidence value

// The confidence needed to assert that the sensor is triggered
#define SENSOR_THRESHOLD 5

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// Structure to keep track of all the elements of an MCPWM motor
// driver, designed to be used as part of a linked list.
typedef struct motor_t {
    int32_t pwm_pin;
    int32_t dir_pin;
    int32_t sensor_pin;
    brushless_motor_cb_t cb;
    void *cb_param;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    int32_t sensor_confidence;
    bool sensor_state;
    esp_timer_handle_t sensor_timer;
    SLIST_ENTRY(motor_t) next;
} motor_t;

// Motor list head.
SLIST_HEAD(motor_list_t, motor_t);

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    struct motor_list_t motor_list;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Sample the sensor
static void sensor_sample(void *arg)
{
    motor_t *motor = (motor_t *) arg;
    bool raw = (gpio_get_level(motor->sensor_pin) == 0);

    if (raw) {
        if (motor->sensor_confidence < SENSOR_CONFIDENCE_MAX) {
            motor->sensor_confidence++;
        }
    } else {
        if (motor->sensor_confidence > 0) {
            motor->sensor_confidence--;
        }
    }

    // Hysteresis on the confidence value
    if (!motor->sensor_state && (motor->sensor_confidence >= SENSOR_THRESHOLD)) {
        motor->sensor_state = true;
        if (motor->cb) {
            motor->cb(motor, motor->cb_param);
        }
    } else if (motor->sensor_state && (motor->sensor_confidence == 0)) {
        motor->sensor_state = false;
    }
}

// Clean up a motor.
// IMPORTANT: the context must be locked before this is called.
static void clean_up(motor_t *motor)
{
    if (motor) {
        if (motor->generator) {
            mcpwm_del_generator(motor->generator);
        }
        if (motor->comparator) {
            mcpwm_del_comparator(motor->comparator);
        }
        if (motor->operator) {
            mcpwm_del_operator(motor->operator);
        }
        if (motor->timer) {
            mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_STOP_EMPTY);
            mcpwm_timer_disable(motor->timer);
            mcpwm_del_timer(motor->timer);
        }
        if (motor->sensor_timer) {
            esp_timer_stop(motor->sensor_timer);
            esp_timer_delete(motor->sensor_timer);
        }
        gpio_reset_pin(motor->pwm_pin);
        gpio_reset_pin(motor->dir_pin);
        if (motor->sensor_pin >= 0) {
            gpio_reset_pin(motor->sensor_pin);
        }

        // Remove the motor from the list, if present
        motor_t *iter;
        motor_t *prev = NULL;
        SLIST_FOREACH(iter, &g_context.motor_list, next) {
            if (iter == motor) {
                if (prev == NULL) {
                    // Removing the first element
                    SLIST_REMOVE_HEAD(&g_context.motor_list, next);
                } else {
                    // Removing a middle element
                    SLIST_REMOVE_AFTER(prev, next);
                }
                // Done; MUST break after an insertion or removal as
                // otherwise SLIST_FOREACH will go bang as it
                // relies on pointers still being valid.
                break;
            }
            prev = iter;
        }
        free(motor);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise the brushless motor module.
int32_t motor_brushless_init()
{
    int32_t err = ESP_OK;

    if (!g_context.lock) {
        // Create mutex
        err = -ESP_ERR_NO_MEM;
        g_context.lock = xSemaphoreCreateMutex();
        if (g_context.lock) {
            err = ESP_OK;
            SLIST_INIT(&g_context.motor_list);
        }
    }

    return err;
}

// Deinitialise the brushless motor module.
void motor_brushless_deinit()
{
    CONTEXT_LOCK(g_context.lock, "motor_brushless_deinit()");

    while (!SLIST_EMPTY(&g_context.motor_list)) {
        motor_t *p = SLIST_FIRST(&g_context.motor_list);
        clean_up(p);
    }

    CONTEXT_UNLOCK(g_context.lock, "motor_brushless_deinit()");
    // The semaphore will be re-used
}

// Create a brushless motor.
void *motor_brushless_create(int32_t pwm_pin, int32_t dir_pin,
                             int32_t sensor_pin,
                             brushless_motor_cb_t cb, void *cb_param)
{
    motor_t *motor = malloc(sizeof(*motor));

    if (motor) {

        CONTEXT_LOCK(g_context.lock, "motor_brushless_create()");

        memset(motor, 0, sizeof(*motor));

        motor->pwm_pin = pwm_pin;
        motor->dir_pin = dir_pin;
        motor->sensor_pin = sensor_pin;
        motor->cb = cb;
        motor->cb_param = cb_param;

        // Create an MCPWM timer
        mcpwm_timer_config_t cfg = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = TIMER_RESOLUTION_HERTZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = TIMER_PERIOD_TICKS
        };
        esp_err_t err = mcpwm_new_timer(&cfg, &motor->timer);

        // Create an MCPWM operator
        if (err == ESP_OK) {
            mcpwm_operator_config_t cfg = {.group_id = 0};
            err = mcpwm_new_operator(&cfg, &motor->operator);
        }

        // Connect the operator to the timer
        if (err == ESP_OK) {
            err = mcpwm_operator_connect_timer(motor->operator, motor->timer);
        }

        // Create a comparator for the PWM pin
        if (err == ESP_OK) {
            mcpwm_comparator_config_t cfg = {.flags.update_cmp_on_tez = true};
            err = mcpwm_new_comparator(motor->operator, &cfg, &motor->comparator);
            if (err == ESP_OK) {
                // PWM off for now
                mcpwm_comparator_set_compare_value(motor->comparator, 0);
            }
        }

        // Create a generator for the PWM pin
        if (err == ESP_OK) {
            mcpwm_generator_config_t cfg = {.gen_gpio_num = pwm_pin};
            err = mcpwm_new_generator(motor->operator, &cfg, &motor->generator);
        }

        // Set actions for the generator
        if (err == ESP_OK) {
            // Set output high at the start of each period
            mcpwm_generator_set_action_on_timer_event(motor->generator,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                             MCPWM_TIMER_EVENT_EMPTY,
                                             MCPWM_GEN_ACTION_HIGH));

            // Set output low when the comparator matches
            mcpwm_generator_set_action_on_compare_event(motor->generator,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                               motor->comparator,
                                               MCPWM_GEN_ACTION_LOW));
        }

        // Enable the timer
        if (err == ESP_OK) {
            err = mcpwm_timer_enable(motor->timer);
        }

        // Start the timer running continuously
        if (err == ESP_OK) {
            err = mcpwm_timer_start_stop(motor->timer, MCPWM_TIMER_START_NO_STOP);
        }

        // Set the direction pin to be an output
        if (err == ESP_OK) {
            gpio_set_level(dir_pin, 0);
            gpio_set_direction(dir_pin, GPIO_MODE_OUTPUT);
        }

        // Deal with the sensor pin
        if (sensor_pin >= 0) {
            gpio_config_t cfg = {
                .pin_bit_mask = 1ULL << sensor_pin,
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
            };
            err = gpio_config(&cfg);

            if (err == ESP_OK) {
                esp_timer_create_args_t args = {
                    .callback = sensor_sample,
                    .arg = motor,
                    .dispatch_method = ESP_TIMER_TASK,
                    .name = "sensor_sample",
                };
                err = esp_timer_create(&args, &motor->sensor_timer);
            }

            if (err == ESP_OK) {
                err = esp_timer_start_periodic(motor->sensor_timer, SENSOR_SAMPLE_MS * 1000);
            }
        }

        if (err == ESP_OK) {
            SLIST_INSERT_HEAD(&g_context.motor_list, motor, next);
        } else {
            clean_up(motor);
            motor = NULL;
        }

        CONTEXT_UNLOCK(g_context.lock, "motor_brushless_create()");
    }

    return motor;
}

// Free a brushless motor.
void motor_brushless_destroy(void *motor)
{
    CONTEXT_LOCK(g_context.lock, "motor_brushless_destroy()");
    clean_up(motor);
    CONTEXT_UNLOCK(g_context.lock, "motor_brushless_destroy()");
}

// Set the speed and direction of motor rotation.
int32_t motor_brushless_set_speed(void *motor, int32_t pwm_rate_percent)
{
    int32_t err = ESP_ERR_INVALID_ARG;

    if (motor) {
        err = ESP_OK;
        if (pwm_rate_percent > 100) {
            pwm_rate_percent = 100;
        }
        if (pwm_rate_percent < -100) {
            pwm_rate_percent = -100;
        }
        if (pwm_rate_percent == 0) {
            // Timer off
            err = mcpwm_comparator_set_compare_value(((motor_t *) motor)->comparator, 0);
        } else {
            // Set direction
            gpio_set_level(((motor_t *) motor)->dir_pin, (pwm_rate_percent < 0) ? 1 : 0);

            // Set duty
            int32_t abs_pwm_rate_percent = pwm_rate_percent;
            if (abs_pwm_rate_percent < 0) {
                abs_pwm_rate_percent = -abs_pwm_rate_percent;
            }
            uint32_t duty_ticks = (abs_pwm_rate_percent * TIMER_PERIOD_TICKS) / 100;
            err = mcpwm_comparator_set_compare_value(((motor_t *) motor)->comparator, duty_ticks);
        }
    }

    // Return negative value from esp_err_t.
    return -err;
}

// End of file
