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

#ifndef _TMC5160_H_
#define _TMC5160_H_

/** @file
 * @brief The TMC5160 stepper motor driver API.
 */

#ifdef __cplusplus
extern "C" {
#endif

// Needed for gpio_isr_t.
#include "driver/gpio.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/** Masks for the line states of each pin of a TMC5160,
 * use on the return value of tmc5160_read_lines().
 */
#define TMC5160_LINE_MASK_REFL_STEP        0x0001
#define TMC5160_LINE_MASK_REFR_DIR         0x0002
#define TMC5160_LINE_MASK_ENCB_DCEN_CFG4   0x0004
#define TMC5160_LINE_MASK_ENCA_DCIN_CFG5   0x0008
#define TMC5160_LINE_MASK_DRV_ENN          0x0010
#define TMC5160_LINE_MASK_ENC_N_DCO_CFG6   0x0020
#define TMC5160_LINE_MASK_SD_MODE          0x0040
#define TMC5160_LINE_MASK_SWCOMP_IN        0x0080

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* Stepper motor handle.
 */
typedef struct stepper stepper_t;

/** Chopper operating mode. */
typedef enum {
    TMC5160_CHOPPER_SPREAD_CYCLE,
    TMC5160_CHOPPER_STEALTH_CHOP
} tmc5160_chopper_mode_t;

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise the interface to the TMC5160.  Note that tmc5160_create()
 * still needs to be called before a TMC5160 will respond to
 * read or write requests.
 *
 * @param spi               SPI host, e.g. SPI2_HOST.
 * @param sck_pin           GPIO for SPI clock.
 * @param mosi_pin          GPIO for SPI MOSI (TMC SDI).
 * @param miso_pin          GPIO for SPI MISO (TMC SDO).
 * @param cs_pin            GPIO for SPI chip select (TMC CSN).
 * @param clock_hz          SPI clock frequency in Hz.
 * @return                  ESP_OK on success, else a negative value
 *                          from esp_err_t.
 */
int32_t tmc5160_init(int32_t spi, int32_t sck_pin, int32_t mosi_pin,
                     int32_t miso_pin, int32_t cs_pin, int32_t clock_hz);

/** Create a TMC5160 instance.  Note that if a pin_motor_enable is
 * provided to this function it will be held high (i.e. motor is off)
 * and tmc5160_motor_enable() must be called to enable the motor.
 *
 * @param address    legacy device identifier (unused for SPI).
 * @param enable_pin GPIO pin connected to the EN pin of a TMC5160
 *                   that must be pulled low to enable the motor;
 *                   use -1 if there is no such pin.
 * @return           the stepper motor instance, NULL on error.
 */
stepper_t *tmc5160_create(int32_t address, int32_t enable_pin);

/** Destroy a TMC5160 instance.
 *
 * Note: if you have setup an interrupt handler by calling
 * tmc5160_init_stallguard() it is up to you to call
 * tmc5160_deinit_stallguard() to remove it, this function will
 * not do so.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 */
void tmc5160_destroy(stepper_t *stepper);

/** Deinitialise the interface to the TMC5160.  Any TMC5160
 * instances will be removed as if tmc5160_destroy() had been
 * called on them.
 *
 * Note: if you have setup an interrupt handler by calling
 * tmc5160_init_stallguard() it is up to you to call
 * tmc5160_deinit_stallguard() to remove it, this function will
 * not do so.
 */
void tmc5160_deinit();

/** Call this to determine if the TMC5160 has reset
 * since tmc5160_acknowledge_reset() was last called.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        true if this function has reset since
 *                tmc5160_acknowledge_reset() was last
 *                called.
 */
bool tmc5160_has_reset(stepper_t *stepper);

/** Acknowledge the reset bit; after this is callled
 * tmc5160_has_reset() will return false until the TMC5160
 * next resets.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        zero on success else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_acknowledge_reset(stepper_t *stepper);

/** Enable the motor of a TMC5160; only needs to be called if
 * a pin_motor_enable was provided to tmc5160_start().
 *
 * In stealthChop mode this call first commands zero velocity and waits
 * one second after enabling the outputs, to initialise PWM regulation.
 * Repeated calls while enabled do not repeat the delay.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        zero on success else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_motor_enable(stepper_t *stepper);

/** Disable the motor of a TMC5160.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        zero on success else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_motor_disable(stepper_t *stepper);

/** Write a buffer of data to the given register of a TMC5160.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @param reg     the register to write to, range 0 to 127.
 * @param data    the data to send.
 * @return        zero on sucess else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_write(stepper_t *stepper, int32_t reg, uint32_t data);

/** Read a buffer of data from the given register of a TMC5160.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @param reg     the register to write to, range 0 to 127.
 * @param data    a pointer to a uint32_t into which the received
 *                data will be written; may be NULL.
 * @return        zero on sucess else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_read(stepper_t *stepper, int32_t reg, uint32_t *data);

/** Read the state of all of a TMC5160's lines.  The return
 * value, if non-negative, may be masked with one or more
 * TMC5160_LINE_MASK_* values (see above) to get the state
 * of a given line.  The top byte of the response contains the
 * version number of the IC (normally 0x21).
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        the lines state as a bit-map, else negative
 *                error code from esp_err_t.
 */
int32_t tmc5160_read_lines(stepper_t *stepper);

/** Set the microstep resolution of a TMC5160.
 *
 * @param stepper    the stepper instance that was returned by
 *                   tmc5160_create().
 * @param resolution the power of 2 value that the microstep
 *                   resolution should be set, from 1 (i.e.
 *                   one step is one step of the motor)
 *                   to 256 (so it would take 256 steps to move
 *                   the motor by one step); if not a power
 *                   of two the value is rounded down.
 * @return           the power of two set, else negative
 *                   error code from esp_err_t.
 */
 int32_t tmc5160_set_microstep_resolution(stepper_t *stepper,
                                          int32_t resolution);

/** Get the microstep resolution of the stepper motor
 * attached to a TMC5160 device.
 *
 * @param stepper the stepper instance that was returned by
 *                tmc5160_create().
 * @return        the microstep resolution, else negative
 *                error code from esp_err_t.
 */
int32_t tmc5160_get_microstep_resolution(stepper_t *stepper);

/** Set the current supplied to the stepper motor by a
 * TMC5160 device.
 *
 * @param stepper              the stepper instance that
 *                             was returned by tmc5160_create().
 * @param r_sense_mohm         the value of the sense
 *                             resistors connected to
 *                             the TMC5160 in milliOhms.
 * @param run_current_ma       the desired current in
 *                             milliAmps
 * @param hold_current_percent the hold current as a
 *                             percentage of the run
 *                             current; 50% is a good
 *                             value
 * @return                     the run current set in
 *                             milliAmps, else negative
 *                             error code from esp_err_t.
 */
int32_t tmc5160_set_current(stepper_t *stepper,
                            uint32_t r_sense_mohm,
                            uint32_t run_current_ma,
                            uint32_t hold_current_percent);

/** Set the acceleration/deceleration of the stepper motor
 * attached to a TMC5160 device.
 *
 * In velocity mode this controls both acceleration and
 * deceleration.  Higher values give more abrupt starts
 * and stops.
 *
 * The default value set by tmc5160_create() is 1000.
 *
 * @param stepper      the stepper instance that was returned
 *                     by tmc5160_create().
 * @param acceleration the value to write to the AMAX register,
 *                     range 1 to 65535.
 * @return             zero on success else negative error
 *                     code from esp_err_t.
 */
int32_t tmc5160_set_acceleration(stepper_t *stepper,
                                 uint32_t acceleration);

/** Set the velocity of the stepper motor attached to a
 * TMC5160 device.
 *
 * IMPORTANT: if tmc5160_motor_enable() has been called
 * this will drive the steps of the stepper motor from
 * its own internal step generator, i.e. it WILL START
 * MOVING IMMEDIATELY.
 *
 * Note: for operation at low speeds you might want to
 * look at tmc5160_set_stealth_chop_threshold().
 *
 * @param stepper     the stepper instance that was returned
 *                    by tmc5160_create().
 * @param millihertz  the step rate in millihertz, i.e.
 *                    a value of 1000 would be one step per
 *                    second.
 * @return            zero on success else negative error
 *                    code from esp_err_t.
 */
int32_t tmc5160_set_velocity(stepper_t *stepper,
                             int32_t millihertz);

/** Get the velocity of the stepper motor attached to a
 * TMC5160 device.
 *
 * @param stepper     the stepper instance that was returned
 *                    by tmc5160_create().
 * @param millihertz  a place to put the step rate in millihertz;
 *                    may be NULL.
 * @return            zero on success else negative error
 *                    code from esp_err_t.
 */
int32_t tmc5160_get_velocity(stepper_t *stepper, int32_t *millihertz);

/** Command the TMC5160 to move to a position.
 *
 * @param stepper                  the stepper instance that was returned
 *                                 by tmc5160_create().
 * @param microsteps               the position to move to, in micro steps.
 * @param max_velocity_millihertz  the maximum speed at which to move
 * @return                         zero on success else negatie errp
 *                                 code from esp_err_t.
 */
int32_t tmc5160_set_position(stepper_t *stepper, int32_t microsteps,
                             uint32_t max_velocity_millihertz);

/** Read the microstep counter of a TMC5160.
 *
 * @param stepper    the stepper instance that was returned
 *                   by tmc5160_create().
 * @param microsteps the current micro step count.
 * @return           the count, else negative error code
 *                   from esp_err_t.
 */
 int32_t tmc5160_get_position(stepper_t *stepper, int32_t *microsteps);

/** Select spreadCycle or stealthChop operation.
 *
 * Call while the motor is disabled.  In stealthChop mode, the next
 * tmc5160_motor_enable() holds the motor stationary for one second to
 * initialise automatic PWM regulation.  The motor must have a physical
 * enable GPIO for this automatic startup sequence.
 *
 * @param stepper the stepper instance.
 * @param mode    the desired chopper mode.
 * @return        zero on success, else a negative esp_err_t value.
 */
int32_t tmc5160_set_chopper_mode(stepper_t *stepper,
                                 tmc5160_chopper_mode_t mode);

/** Set the TSTEP value below which the TMC5160 will leave
 * leave SpreadCycle mode and switch to StealthChop mode,
 * which is quiter and more efficient at higher speeds.
 * When TSTEP is low (higher speeds), StealthChop is better
 * because it is quieter and more efficient, however it can
 * cause motor stalls because of the clever current control.
 *
 * Note: if you find you have to use SpreadCycle mode,
 * you may want to look at tmc5160_stop_that_bloody_racket().
 *
 * @param stepper    the stepper instance that was returned
 *                   by tmc5160_create().
 * @param threshold  the value of TSTEP below which
 *                   SpreadCycle should be abandoned for
 *                   StealthChop mode.  A special value of
 *                   0 means "always stay in StealthChop mode",
 *                   use UINT32_MAX to disable switching
 *                   to StealthChop entirely.
 * @return           zero on success else negative error
 *                   code from esp_err_t.
 */
int32_t tmc5160_set_stealth_chop_threshold(stepper_t *stepper,
                                           uint32_t threshold);

/** Configure the chopper in a TMC5160, usually to stop
 * a motor sounding like a cricket in SpreadCycle mode,
 * see the data sheet for an explanation of the parameters.
 *
 * @param stepper the stepper instance that was returned
 *                by tmc5160_create().
 * @param tbl     set the blank time.
 * @param toff    set the off time.
 * @param hstrt   set the hysteresis low value.
 * @param hend    set the hysteresis value added to hstrt,
 *                hend + hstart must be 16 or less.
 * @return        zero on success else negative error
 *                code from esp_err_t.
 */
int32_t tmc5160_configure_chopper(stepper_t *stepper,
                                  uint8_t tbl,
                                  uint8_t toff,
                                  uint8_t hstrt,
                                  int8_t hend);

/** Get the value of TSTEP from a TMC5160 device; this
 * value may be required when setting stallguard
 * operation with tmc5160_set_stallguard().
 *
 * @param stepper the stepper instance that was returned
 *                by tmc5160_create().
 * @return        the TSTEP value, else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_get_tstep(stepper_t *stepper);

/** Get the SG_RESULT value from a TMC5160 device; this
 * value may be required when setting stallguard
 * operation with tmc5160_set_stallguard().  Lower
 * values mean that there is a higher load.
 *
 * @param stepper the stepper instance that was returned
 *                by tmc5160_create().
 * @return        the SG_RESULT value, else negative error code
 *                from esp_err_t.
 */
int32_t tmc5160_get_sg_result(stepper_t *stepper);

/** Set the operation of StallGuard in a TMC5160.
 * This allows the detection of a stall condition which
 * will pulse the DIAG pin of the TMC5160 that may be
 * connected to a pin of this microcontroller and cause
 * an interrupt. See the TMC5160 data sheet for how to
 * derive the TCOOLTHRS and SGT values but basically
 * TCOOLTHRS has to be at least as large as TSTEP for
 * StallGuard to be employed while SGT is a signed
 * value, +/-63, that adjusts when a stall occurs,
 * higher values making StallGuard less sensitive
 * (i.e. requiring more torque to indicate a stall).
 *
 * Note: the value of TSTEP, which is relevant because
 * TCOOLTHRS is compared with it, changes with velocity,
 * therefore you should call tmc5160_set_stallguard()
 * if the velocity is changed.
 *
 * Note: if you set an interrupt handler, make sure to call
 * tmc5160_deinit_stallguard() when done, otherwise your
 * interrupt handler may be called at any time.
 *
 * Note: this will make sure that gpio_install_isr_service()
 * is installed.
 *
 * @param stepper     the stepper instance that was returned
 *                    by tmc5160_create().
 * @param tcoolthrs   the value of TCOOLTHRS to set in the
 *                    TMC5160.  Use a negative value to have
 *                    this code set a default value that
 *                    is the same as the TSTEP value in the
 *                    chip, meaning that stallguard should
 *                    always be active.
 * @param sgt         the value of SGT to set in the
 *                    TMC5160, range +/-63, higher values
 *                    requiring more torque to indicate a
 *                    stall.
 * @param pin         the GPIO pin of this microcontroller
 *                    that the DIAG pin of the TMC5160
 *                    is connected to; use a negative value
 *                    if there is no such connection.
 * @param handler     the interrupt handler to call when
 *                    pin changes state; must be non-NULL
 *                    if pin is non-negative.  THIS HANDLER
 *                    WILL BE CALLED IN INTERRUPT CONTEXT
 *                    so do very little in it, e.g. queue
 *                    an event or give a semaphore.
 *                    Ignored if pin is negative.
 * @param handler_arg user-defined argument that will be
 *                    passed to handler when it is called;
 *                    may be NULL.
 * @return            zero on success, else negative error
 *                    code from esp_err_t.
 */
int32_t tmc5160_init_stallguard(stepper_t *stepper,
                                int32_t tcoolthrs,
                                int8_t sgt,
                                int32_t pin,
                                gpio_isr_t handler,
                                void *handler_arg);

/** Set the StallGuard registers in a TMC5160.  If you
 * have connected the DIAG output of the TMC5160 to a
 * pin of this microcontroller you should call
 * tmc5160_init_stallguard() first to set up an
 * interrupt handler.  See that function also for a
 * more detailed description of the TCOOLTHRS and SGT
 * parameters.
 *
 * @param stepper     the stepper instance that was returned
 *                    by tmc5160_create().
 * @param tcoolthrs   the value of TCOOLTHRS to set in the
 *                    TMC5160.  Use a negative value to have
 *                    this code set a default value that
 *                    is the same as the TSTEP value in the
 *                    chip, meaning that stallguard should
 *                    always be active.  Use a value of
 *                    UINT32_MAX to disable StallGuard
 * @param sgt         the value of SGT to set in the
 *                    TMC5160, range +/-63, higher values
 *                    requiring more torque to indicate a
 *                    stall.
 * @return            zero on success, else negative error
 *                    code from esp_err_t.
 */
int32_t tmc5160_set_stallguard(stepper_t *stepper,
                               int32_t tcoolthrs,
                               int8_t sgt);

/** Call this to remove the interrupt handling that was
 * set up by tmc5160_init_stallguard().  After this function
 * has returned your interrupt handler will no longer be
 * called.  There is no need to call this if you did not
 * pass a GPIO pin to tmc5160_init_stallguard().
 *
 * Note: this will not uninstall gpio_install_isr_service().
 *
 * @param pin     the GPIO pin that was passed to
 *                tmc5160_init_stallguard().
 */
void tmc5160_deinit_stallguard(int32_t pin);

/** Set the CoolStep threshold registers in a TMC5160,
 * see section 12 of the data sheet for an explanation.
 * To disable CoolStep, set seimin and semax both to 0.
 *
 * @param stepper the stepper instance that was returned
 *                by tmc5160_create().
 * @param seimin  the minimum current in CoolStep mode:
 *                use 0 for half of the normal setting
 *                (good if IRUN >= 10), use 1 for 1/4 of
 *                of the normal setting (good if IRUN >= 20).
 * @param semin   if SGRESULT goes below this value the
 *                current to the motor is increased.
 * @param semax   if SGRESULT goes above this value often
 *                enough the current to the motor is
 *                decreased.
 * @return        zero on success, else negative error
 *                code from esp_err_t.
 */
int32_t tmc5160_set_coolstep(stepper_t *stepper, uint8_t seimin,
                             uint8_t semin, uint8_t semax);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _TMC5160_H_

// End of file
