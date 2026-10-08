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

#ifndef _MOTOR_BRUSHLESS_H_
#define _MOTOR_BRUSHLESS_H_

/** @file
 * @brief Brushless motor API.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** A callback to be called on every trigger of a sensor associated
 * with a brushless motor.
 *
 * @param motor     the handle that was returned by motor_brushless_create().
 * @param param     cb_param as passed to motor_brushless_create().
 */
typedef void (*brushless_motor_cb_t) (void *motor, void *param);

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the brushless motor module.  This function may be
 * called at any time; if it has already been called it will do
 * nothing and return success.  This creates a semaphore that will
 * never be destroyed.
 *
 * @return ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t motor_brushless_init();

/** Deinitialise the brushless motor module.  When called all
 * motors will be stopped and released by calling
 * motor_brushless_destroy().
 *
 * @return ESP_OK on success, else a negative value from esp_err_t.
 */
void motor_brushless_deinit();

/** Create a brushless motor.
 *
 * @param pwm_pin             the GPIO output pin to which a PWM signal
 *                            should be supplied to control the speed of the
 *                            motor.
 * @param dir_pin             the GPIO output pin to which a direction signal
 *                            should be supplied to set the motor direction.
 * @param sensor_pin          the GPIO input pin that is connected to a
 *                            a rotation sensor; use -1 if there is none.
 *                            A triggering of the sensor should pull
 *                            sensor_pin low.
 * @param cb                  a callback to be called every time
 *                            sensor_pin is triggered; ignored if
 *                            sensor_pin is not supplied, may be NULL.
 * @param cb_param            a user parameter that will be passsed
 *                            to cb when it is called; ignored if cb is NULL.
 * @return                    a handle for the motor, else NULL.
 */
void *motor_brushless_create(int32_t pwm_pin, int32_t dir_pin,
                             int32_t sensor_pin,
                             brushless_motor_cb_t cb, void *cb_param);

/** Delete a brushless motor.  It is always safe to call this
 * function.  The motor is stopped and all resources released.  Once
 * this function has returned any callback passed to motor_brushless_create()
 * will no longer be called.
 *
 * @param motor the handle that was returned by motor_brushless_create().
 */
void motor_brushless_destroy(void *motor);

/** Set the speed and direction of motor rotation.
 *
 * @param motor            the handle that was returned by
 *                         motor_brushless_create().
 * @param pwm_rate_percent a signed PWM percentage, 0 for zero speed,
 *                         -100 or 100 for maximum speed in either
 *                         direction.  A negative PWM value will cause
 *                         the dir_pin to be high, a zero or positive
 *                         PWM value low.
 * @return                 ESP_OK on success, else a negative value from
 *                         esp_err_t.
 */
int32_t motor_brushless_set_speed(void *motor, int32_t pwm_rate_percent);

/** Get the resolution of the capture timer, in Hz.  Divide a
 * difference of the capture_value's retuned by brushless_motor_cb_t
 * by this to get a time in seconds.
 *
 * @param motor the handle that was returned by motor_brushless_create().
 * @return      the capture timer resolution in Hz, or 0 if the motor
 *              was created without a sensor.
 */
uint32_t motor_brushless_get_capture_resolution(void *motor);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _MOTOR_BRUSHLESS_H_

// End of file
