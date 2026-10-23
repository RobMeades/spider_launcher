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
 * @brief Implementation of the TMC5160 stepper motor driver API.
 */

#include <string.h>
#include <inttypes.h>
#include "api/tmc5160.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "string.h"
#include "sys/queue.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "fgr_util.h"

#include "tmc5160.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "tmc5160"

// The expected chip version (checked during tmc5160_create()).
#define TMC5160_VERSION 0x30

// The TMC5160 clock.
#define TMC5160_CLOCK_HZ 12000000ULL

// The number of steps per revolution.
#define MOTOR_STEPS_PER_REV 200ULL

// The maximum value of IRUN or IHOLD, which are 5-bit values.
#define IRUN_OR_IHOLD_MAX 31

// Default value for the VSTART register
#define VSTART_DEFAULT 0

// Default value for the A1 register
#define A1_DEFAULT 1000

// Default value for the V1 register
#define V1_DEFAULT 0

// Default value for the V1 register
#define V1_DEFAULT 0

// Default value for the AMAX register
#define AMAX_DEFAULT 1000

// Default value for the DMAX register
#define DMAX_DEFAULT 1000

// Default value for the D1 register
#define D1_DEFAULT 1000

// Default value for the VSTOP register
#define VSTOP_DEFAULT 10

// Default values for Stealth Chop
#define STEALTH_CHOP_PWMCONF 0xC40C001EU
#define STEALTH_CHOP_CALIBRATION_MS 1000

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// Structure to keep track of everything we need for a stepper motor.
struct stepper {
    int32_t address; // Legacy identifier; SPI uses chip select instead.
    int32_t enable_pin;
    bool motor_enabled;
    tmc5160_chopper_mode_t chopper_mode;
    SLIST_ENTRY(stepper) next;
};

// Motor list head.
SLIST_HEAD(stepper_list_t, stepper);

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    spi_device_handle_t spi;
    bool bus_initialised;
    spi_host_device_t spi_host;
    struct stepper_list_t stepper_list;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// The context.
static context_t g_context = {0};

// Table of permitted microstep values: order is important, the index
// into the array corresponds to the MRES value in the CHOPCONF
// register i.e. 256 is an MRES value of 0, 1 is an MRES value of 8.
static const int32_t g_microstep_table[] = {256, 128, 64, 32, 16, 8, 4, 2, 1};

// Table of SPI status word bit-meanings in order of bit position.
//
// From section 4.1.2 of the TMC5160 data sheet:
//
// bit 7 status_stop_r RAMP_STAT[1] stop right switch status
// bit 6 status_stop_l RAMP_STAT[0] stop left switch status
// bit 5 position_reached RAMP_STAT[9] target position reached
// bit 4 velocity_reached RAMP_STAT[8] target velocity reached
// bit 3 standstill DRV_STATUS[31] motor stand still
// bit 2 sg2 DRV_STATUS[24] StallGuard flag active
// bit 1 driver_error GSTAT[1] driver error (clear GSTAT to reset)
// bit 0 reset_flag GSTAT[0] a reset has occurred (clear GSTAT to reset)
static const char *g_spi_status_bit_meaning[] = {"reset",    // bit 0
                                                 "error",    // bit 1
                                                 "SG",       // bit 2
                                                 "still",    // bit 3
                                                 "at speed", // bit 4
                                                 "at pos",   // bit 5
                                                 "left",     // bit 6
                                                 "right"};   // bit 7

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// Clean-up a stepper.
// IMPORTANT the context should be locked before this is called.
static void clean_up(stepper_t *stepper)
{
    if (stepper) {
        if (stepper->enable_pin >= 0) {
            gpio_set_level(stepper->enable_pin, 1);
        }

        // Remove the stepper from the list, if present
        stepper_t *iter;
        stepper_t *prev = NULL;
        SLIST_FOREACH(iter, &g_context.stepper_list, next) {
            if (iter == stepper) {
                if (prev == NULL) {
                    // Removing the first element
                    SLIST_REMOVE_HEAD(&g_context.stepper_list, next);
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
        free(stepper);
    }
}

// Print an individual part of the SPI status word, should
// be called in increasing bit-position for the comma thing
// to work
static int32_t print_status_item(uint8_t spi_status, uint8_t mask,
                                 char *buffer, char const *string)
{
    char *p = buffer;

    if (spi_status & mask) {
        p += sprintf(p, string);
        // If any of the bits above this mask position are set,
        // add a barrier
        if (spi_status && (-mask)) {
            p += sprintf(p, "|");
        }
    }

    return p - buffer;
}

// Print the SPI status word, always the first returned.
static void print_spi_status(uint8_t spi_status)
{
    char buffer[128] = {0};
    char *p = buffer;

    for (size_t x = 0; x < FGR_UTIL_ARRAY_LENGTH(g_spi_status_bit_meaning); x++) {
        p += print_status_item(spi_status, 1 << x, p, g_spi_status_bit_meaning[x]);
    }

    ESP_LOGD(TAG, "SPI status: %s", buffer);
}

// Perform one 40-bit SPI transaction.  The response byte zero is SPI_STATUS.
// Note that this function performs an endian conversion to little endian,
// which is the ESP32 endianness (whereas the TMC5160 has a bit endian API).
// IMPORTANT: the context mutex must be held by the caller.
static int32_t transfer(int32_t reg, bool write_access, uint32_t value,
                        uint32_t *received)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (g_context.spi && (reg >= 0) && (reg <= 0x7f)) {
        uint8_t tx[5] = {(uint8_t) (reg | (write_access ? 0x80 : 0)),
                         (uint8_t) (value >> 24), (uint8_t) (value >> 16),
                         (uint8_t) (value >> 8), (uint8_t) value};
        uint8_t rx[5] = {0};
        spi_transaction_t transaction = {
            .length = 40,
            .tx_buffer = tx,
            .rx_buffer = rx
        };

        err = -spi_device_polling_transmit(g_context.spi, &transaction);
        if (err == ESP_OK) {
            if (received) {
                *received = ((uint32_t) rx[1] << 24) | ((uint32_t) rx[2] << 16) |
                            ((uint32_t) rx[3] << 8) | rx[4];
            }
            ESP_LOGD(TAG, "SPI reg 0x%02lx: RX %02x %02x %02x %02x %02x",
                    (unsigned long) reg, rx[0], rx[1], rx[2], rx[3], rx[4]);
            // Print the status byte
            print_spi_status(rx[0]);
        } else {
            ESP_LOGE(TAG, "SPI transaction failed (%s).", esp_err_to_name(-err));
        }
    }

    return err;
}

// TMC5160 SPI reads are pipelined: data arrives in the next transaction.
// IMPORTANT: the context mutex must be held across both transactions.
static int32_t read(stepper_t *stepper, int32_t reg, uint32_t *data)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (stepper && (reg >= 0) && (reg <= 0x7f)) {
        err = transfer(reg, false, 0, NULL);
        if (err == ESP_OK) {
            err = transfer(reg, false, 0, data);
        }
        if (err == ESP_OK) {
            err = sizeof(uint32_t);
        }
    }

    return err;
}

// Write to a TMC5160.
// IMPORTANT: the context mutex must be held across both transactions.
static int32_t write(stepper_t *stepper, int32_t reg, uint32_t *data)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (stepper && data && (reg >= 0) && (reg <= 0x7f)) {
        err = transfer(reg, true, *data, NULL);
        if (err == ESP_OK) {
            err = sizeof(uint32_t);
        }
    }

    return err;
}

// Do a generic register write.
// IMPORTANT the context should be locked before this is called.
static int32_t write_reg(stepper_t *stepper, int32_t reg, uint32_t data)
{
    int32_t err = write(stepper, reg, &data);

    if (err == sizeof(data)) {
        err = ESP_OK;
    } else {
        if (err >= 0) {
            err = -ESP_ERR_INVALID_RESPONSE;
        }
    }

    return err;
}

// Do a generic register read.
// IMPORTANT the context should be locked before this is called.
static int32_t read_reg(stepper_t *stepper, int32_t reg, uint32_t *data)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (data) {
        err = read(stepper, reg, data);
        if (err == sizeof(*data)) {
            err = ESP_OK;
        } else {
            if (err >= 0) {
                err = -ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    return err;
}

// Set the stallguard registers.
// IMPORTANT the context should be locked before this is called.
static int32_t set_stallguard(stepper_t *stepper, int32_t tcoolthrs,
                              int8_t sgt)
{
    int32_t err = ESP_OK;
    uint32_t data;

    ESP_LOGI(TAG, "configuring StallGuard, TCOOLTHRS %d, SGT %d.",
             tcoolthrs, sgt);
    if (tcoolthrs < 0) {
        // Read the TSTEP register so that we
        // can use it as TCOOLTHRS
        err = read_reg(stepper, 0x12, &data);
        if (err == ESP_OK) {
            tcoolthrs = data;
            ESP_LOGI(TAG, "TCOOLTHRS will be TSTEP which is %d.", tcoolthrs);
        }
    }
    if (err == ESP_OK) {
        // Write to the TCOOLTHRS register (0x14)
        err = write_reg(stepper, 0x14, tcoolthrs);
    }
    if (err == ESP_OK) {
        // The SGT is in bits 16 to 22 of the COOLCNF register (0x6d)
        err = read_reg(stepper, 0x6d, &data);
        if (err == ESP_OK) {
            data &= 0xff80ffff;
            data |= ((uint32_t) sgt) << 16;
            // Write it back again
            err = write_reg(stepper, 0x6d, data);
        }
    }

    return err;
}

// Return the RMS current in milliAmps for a given i_run and
// r_sense_mohm.
static uint32_t rms_current_milliamps(uint32_t i_run,
                                      uint32_t r_sense_mohm)
{
    // From section 9 of the data sheet, with GLOBALSCALER
    // left at its default of 256, the RMS current delivered is:
    //
    // ((IRUN + 1)  / 32)) * (v_full_scale_millivolts / r_sense_mohm) *  (1 / root-2)
    //
    // ...where v_full_scale_millivolts is VSRTL (325 mV),
    // and the root-2 is to give the RMS current.

    // Do this in uint64_t just in case
    uint64_t numerator = (uint64_t) (i_run + 1) * 325 * 1000000;
    uint64_t denominator = 32ULL * 1414 * r_sense_mohm;

    return (uint32_t) (numerator / denominator);
}

// Get the chip version.
// IMPORTANT the context should be locked before this is called.
static int32_t get_version(stepper_t *stepper)
{
    // Read the IOIN register (0x04)
    uint32_t data;
    int32_t err = read_reg(stepper, 0x04, &data);
    if (err == ESP_OK) {
        // The chip version is in bits 24 to 31
        err = data >> 24;
    }

    return err;
}

// Set the enable pin of a motor.
static int32_t enable_pin_set(stepper_t *stepper, bool enable_not_disable)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (stepper && (stepper->enable_pin >= 0)) {
        // Pin is low for enable (true), high for disable (false)
        err = -gpio_set_level(stepper->enable_pin, !enable_not_disable);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "STEPPER %s by pin %d.",
                     enable_not_disable ? "ENABLED": "DISABLED",
                     stepper->enable_pin);
        }
    } else {
        ESP_LOGE(TAG, "Not %sing stepper, no enable GPIO pin was set.",
                    enable_not_disable ? "enabl": "disabl");
    }

    return err;
}

// Get the microstep resolution.
// IMPORTANT the context should be locked before this is called.
static int32_t get_microstep_resolution(stepper_t *stepper,
                                        int32_t *microstep_resolution)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (stepper && microstep_resolution) {
        // Read the CHOPCONF register (0x6c).
        uint32_t data;
        err = read_reg(stepper, 0x6c, &data);
        if (err == ESP_OK) {
            err = -ESP_ERR_INVALID_RESPONSE;
            // The MRES value is in bits 24 to 27
            data = (data >> 24) & 0x0f;
            // Use this as an index into g_microstep_table
            // to get the power-of-two value
            if (data < FGR_UTIL_ARRAY_LENGTH(g_microstep_table)) {
                *microstep_resolution = g_microstep_table[data];
                err = ESP_OK;
            }
        }
    }

    return err;
}

// Get the VMAX value for a given velocity
// IMPORTANT the context should be locked before this is called.
static int32_t millihertz_to_vmax(stepper_t *stepper, int32_t millihertz, uint32_t *vmax)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (vmax) {
        // Read the microstep resolution
        int32_t microstep_resolution;
        err = get_microstep_resolution(stepper, &microstep_resolution);
        if (err == ESP_OK) {
            // VMAX takes an unsigned magnitude; RAMPMODE determines direction.
            uint64_t velocity = (millihertz < 0) ?
                                (uint32_t) (-(int64_t) millihertz) :
                                (uint32_t) millihertz;
            uint64_t _vmax = (velocity *
                              MOTOR_STEPS_PER_REV * microstep_resolution *
                              (1ULL << 24)) / (1000ULL * TMC5160_CLOCK_HZ);

            if (_vmax <= 0x007fffffULL) {
                *vmax = (uint32_t) _vmax;
            } else {
                err = -ESP_ERR_INVALID_ARG;
            }
        }
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Initialise SPI2 for the TMC5160.  The device owns its SPI bus.
int32_t tmc5160_init(int32_t spi, int32_t sck_pin, int32_t mosi_pin,
                     int32_t miso_pin, int32_t cs_pin, int32_t clock_hz)
{
    int32_t err = ESP_OK;
    if (!g_context.lock) {
        g_context.lock = xSemaphoreCreateMutex();
        if (g_context.lock) {
            SLIST_INIT(&g_context.stepper_list);
        } else {
            err = -ESP_ERR_NO_MEM;
        }
    }

    if (err == ESP_OK) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_init()");

        if (!g_context.spi) {
            ESP_LOGI(TAG, "starting SPI %d at %d Hz with SCK pin %d,"
                     " MOSI pin %d, MISO pin %d, CS pin %d.",
                     spi, clock_hz, sck_pin, mosi_pin, miso_pin, cs_pin);
           spi_bus_config_t bus = {
                .sclk_io_num = sck_pin, .mosi_io_num = mosi_pin,
                .miso_io_num = miso_pin, .quadwp_io_num = -1,
                .quadhd_io_num = -1, .max_transfer_sz = 5
            };
            esp_err_t result = spi_bus_initialize((spi_host_device_t) spi, &bus, SPI_DMA_DISABLED);
            if (result == ESP_OK) {
                g_context.bus_initialised = true;
                spi_device_interface_config_t device = {
                    .clock_speed_hz = clock_hz, .mode = 3,
                    .spics_io_num = cs_pin, .queue_size = 1
                };
                result = spi_bus_add_device((spi_host_device_t)spi, &device, &g_context.spi);
                if (result != ESP_OK) {
                    spi_bus_free((spi_host_device_t)spi);
                    g_context.bus_initialised = false;
                }
            }
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "SPI initialisation failed (%s).", esp_err_to_name(result));
                err = -result;
            } else {
                g_context.spi_host = (spi_host_device_t)spi;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_init()");

    }

    return err;
}

// Create a TMC5160 instance.
stepper_t *tmc5160_create(int32_t address, int32_t enable_pin)
{
    stepper_t *stepper = NULL;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_create()");

        int32_t err = ESP_ERR_NO_MEM;
        stepper = malloc(sizeof(*stepper));
        if (stepper) {
            memset(stepper, 0, sizeof(*stepper));
            err = ESP_OK;
            stepper->address = address;
            stepper->enable_pin = -1;
            stepper->chopper_mode = TMC5160_CHOPPER_SPREAD_CYCLE;
            char buffer[50] = "no enable pin";
            if (enable_pin >= 0) {
                snprintf(buffer, sizeof(buffer), ", motor enable on pin %d", (int) enable_pin);
            }
            if (enable_pin >= 0) {
                err = gpio_set_level(enable_pin, 1);
                if (err == ESP_OK) {
                    err = gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
                    if (err == ESP_OK) {
                        stepper->enable_pin = enable_pin;
                        // Disable the motor to begin with
                        enable_pin_set(stepper, false);
                    }
                }
            }
            ESP_LOGI(TAG, "starting TMC5160%s.", buffer);
            if (err == ESP_OK) {
                // Read the chip version
                err = get_version(stepper);
                if (err >= 0) {
                    if (err == TMC5160_VERSION) {
                        // Read the GCONF register to prove it is there
                        uint32_t data;
                        err = read_reg(stepper, 0, &data);
                        if (err == ESP_OK) {
                            ESP_LOGI(TAG, "TMC5160 started, GCONF is 0x%08x.", data);
                        } else {
                            ESP_LOGE(TAG, "failed to read GCONF register (%s)!", esp_err_to_name(-err));
                            err = -ESP_ERR_NOT_FOUND;
                        }
                        if (err == ESP_OK) {
                            // Configure the positioning ramp generator registers
                            // VSTART (0x23)
                            err = write_reg(stepper, 0x23, VSTART_DEFAULT);
                            if (err == ESP_OK) {
                                // A1 (0x24)
                                err = write_reg(stepper, 0x24, A1_DEFAULT);
                            }
                            if (err == ESP_OK) {
                                // V1 (0x25)
                                err = write_reg(stepper, 0x25, V1_DEFAULT);
                            }
                            if (err == ESP_OK) {
                                // AMAX (0x26)
                                err = write_reg(stepper, 0x26, AMAX_DEFAULT);
                            }
                            if (err == ESP_OK) {
                                // DMAX (0x28)
                                err = write_reg(stepper, 0x28, DMAX_DEFAULT);
                            }
                            if (err == ESP_OK) {
                                // D1 (0x2A)
                                err = write_reg(stepper, 0x2a, D1_DEFAULT);
                            }
                            if (err == ESP_OK) {
                                // VSTOP (0x2b)
                                err = write_reg(stepper, 0x2b, VSTOP_DEFAULT);
                            }
                        }
                    } else {
                        ESP_LOGE(TAG, "TMC5160 returned version 0x%02x when 0x%02x was expected!",
                                 err, TMC5160_VERSION);
                        err = -ESP_ERR_NOT_SUPPORTED;
                    }
                } else {
                    ESP_LOGE(TAG, "failed to read TMC5160 (%s)!", esp_err_to_name(-err));
                }
            }
        }

        if (err == ESP_OK) {
            SLIST_INSERT_HEAD(&g_context.stepper_list, stepper, next);
        } else {
            clean_up(stepper);
            stepper = NULL;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_create()");

    }

    return stepper;
}

// Destroy a TMC5160 instance.
void tmc5160_destroy(stepper_t *stepper)
{
    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_destroy()");

        clean_up(stepper);

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_destroy()");

    }
}

// Deinitialise the TMC5160 driver.
void tmc5160_deinit()
{
    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_deinit()");

        while (!SLIST_EMPTY(&g_context.stepper_list)) {
            stepper_t *p = SLIST_FIRST(&g_context.stepper_list);
            clean_up(p);
        }

        spi_bus_remove_device(g_context.spi);
        g_context.spi = NULL;
        if (g_context.bus_initialised) {
            spi_bus_free(g_context.spi_host);
            g_context.bus_initialised = false;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_deinit()");

    }

    // The semaphore will be re-used
}

// Determine if the TMC5160 has reset.
bool tmc5160_has_reset(stepper_t *stepper)
{
    bool has_reset = true;  // Failure to read a register treated as a reset

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_has_reset()");

        // Read GSTAT (0x01)
        uint32_t data;
        int32_t err = read_reg(stepper, 0x01, &data);
        if (err == ESP_OK) {
            // Bit 0 indicates reset
            has_reset = (data & 0x01) > 0;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_has_reset()");
    }

    return has_reset;
}

// Acknowledge the reset bit
int32_t tmc5160_acknowledge_reset(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_acknowledge_reset()");

        // Reset the reset bit (bit 0) in GSTAT (0x01)
        err = write_reg(stepper, 0x01, 0x01);
        if (err < 0) {
            ESP_LOGE(TAG, "failed to write GSTAT (%s)!", esp_err_to_name(-err));
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_acknowledge_reset()");
    }

    return err;
}

// Set the motor enable pin low to enable motors.
int32_t tmc5160_motor_enable(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_motor_enable()");

        err = -ESP_ERR_INVALID_ARG;
        if (stepper) {
            err = ESP_OK;
            if (!stepper->motor_enabled) {
                if (stepper->chopper_mode == TMC5160_CHOPPER_STEALTH_CHOP) {
                    // Set VMAX to zero and velocity mode before enabling.
                    err = write_reg(stepper, 0x27, 0);
                    if (err == ESP_OK) {
                        err = write_reg(stepper, 0x20, 1);
                    }
                }
                if (err == ESP_OK) {
                    err = enable_pin_set(stepper, true);
                }
                if (err == ESP_OK) {
                    stepper->motor_enabled = true;
                    if (stepper->chopper_mode == TMC5160_CHOPPER_STEALTH_CHOP) {
                        vTaskDelay(pdMS_TO_TICKS(STEALTH_CHOP_CALIBRATION_MS));
                    }
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_motor_enable()");
    }

    return err;
}

// Set the motor enable pin high to disable motors.
int32_t tmc5160_motor_disable(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_motor_disable()");

        err = enable_pin_set(stepper, false);
        if ((err == ESP_OK) && stepper) {
            stepper->motor_enabled = false;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_motor_disable()");

    }

    return err;
}

// Write a buffer of data to the given register of the TMC5160 at
// the given address.
int32_t tmc5160_write(stepper_t *stepper, int32_t reg, uint32_t data)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_write()");

        err = write_reg(stepper, reg, data);

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_write()");
    }

    return err;
}

// Read a buffer of data from the given register of the TMC5160
// at the given address
int32_t tmc5160_read(stepper_t *stepper, int32_t reg, uint32_t *data)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_read()");

        err = read_reg(stepper, reg, data);

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_read()");
    }

    return err;
}

// Read the state of all of a TMC5160's lines.
int32_t tmc5160_read_lines(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_read_lines()");

        // Read the IOIN register (0x04)
        uint32_t data;
        err = read_reg(stepper, 0x04, &data);
        if (err == ESP_OK) {
            err = data;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_read_lines()");
    }

    return err;
}

// Set the microstep resolution of a TMC5160.
 int32_t tmc5160_set_microstep_resolution(stepper_t *stepper, int32_t resolution)
 {
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_microstep_resolution()");

        err = -ESP_ERR_INVALID_ARG;
        int32_t index;
        // Find the entry in g_microstep_table which is less than or
        // equal to microsteps
        for (index = 0;
            (index < FGR_UTIL_ARRAY_LENGTH(g_microstep_table)) &&
            (g_microstep_table[index] > resolution);
            index++) {
        }

        if (index < FGR_UTIL_ARRAY_LENGTH(g_microstep_table)) {
            // index is now the value to write to the MRES value in the
            // CHOPCONF register (0x6c).  Since there are other things in
            // there we need to do a read-modify-write
            uint32_t data;
            err = read_reg(stepper, 0x6c, &data);
            if (err == ESP_OK) {
                // The MRES value is in bits 24 to 27
                data &= 0xf0ffffff;
                data |= ((uint32_t) index) << 24;
                // Write it back again
                err = write_reg(stepper, 0x6c, data);
                if (err == ESP_OK) {
                    // Return the value set
                    err = g_microstep_table[index];
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_microstep_resolution()");
    }

    return err;
}

// Get the microstep resolution of a TMC5160.
int32_t tmc5160_get_microstep_resolution(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_get_microstep_resolution()");

        // Read the CHOPCONF register (0x6c).
        uint32_t data;
        err = read_reg(stepper, 0x6c, &data);
        if (err == ESP_OK) {
            err = -ESP_ERR_INVALID_RESPONSE;
            // The MRES value is in bits 24 to 27
            data = (data >> 24) & 0x0f;
            // Use this as an index into g_microstep_table
            // to get the power-of-two value
            if (data < FGR_UTIL_ARRAY_LENGTH(g_microstep_table)) {
                err = g_microstep_table[data];
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_get_microstep_resolution()");
    }

    return err;
}

// Set the current supplied to the stepper motor.
int32_t tmc5160_set_current(stepper_t *stepper,
                            uint32_t r_sense_mohm,
                            uint32_t run_current_ma,
                            uint32_t hold_current_percent)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_current()");

        err = -ESP_ERR_INVALID_ARG;
        if ((hold_current_percent <= 100) && (run_current_ma > 0)) {
            uint32_t max_current_ma = rms_current_milliamps(IRUN_OR_IHOLD_MAX,
                                                            r_sense_mohm);

            if (run_current_ma > max_current_ma) {
                run_current_ma = max_current_ma;
            }

            uint32_t run_steps = ((uint64_t) run_current_ma * 32 + max_current_ma / 2) / max_current_ma;
            if (run_steps < 1) {
                run_steps = 1;
            } else if (run_steps > 32) {
                run_steps = 32;
            }
            uint32_t i_run = run_steps - 1;

            uint32_t hold_steps = (run_steps * hold_current_percent + 50) / 100;
            if (hold_steps < 1) {
                hold_steps = 1;
            }
            uint32_t i_hold = hold_steps - 1;

            ESP_LOGI(TAG, "IRUN %d, IHOLD %d.", i_run, i_hold);

            // Set IRUN and IHOLD
            uint32_t data;
            err = read_reg(stepper, 0x10, &data);
            if (err == ESP_OK) {
                // IHOLD is the first 5 bits, IRUN bits 8 to 12
                data = (data & 0xffffe0e0) | (i_hold & 0x1f) | ((i_run & 0x1f) << 8);
                err = write_reg(stepper, 0x10, data);
            }

            if (err >= 0) {
                // Re-calculate the current to return
                err = (int32_t) rms_current_milliamps(i_run, r_sense_mohm);
                // Print CHOPCONF (0x6c) and GLOBAL_SCALER (0x0b) for information
                if (read_reg(stepper, 0x6c, &data) >= 0) {
                    ESP_LOGI(TAG, "CHOPCONF 0x%08x.", data);
                }
                if (read_reg(stepper, 0x0b, &data) >= 0) {
                    ESP_LOGI(TAG, "GLOBAL_SCALER 0x%08x.", data);
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_current()");
    }

    return err;
}

// Set the acceleration/deceleration of the stepper motor
// attached to a TMC5160 device.
int32_t tmc5160_set_acceleration(stepper_t *stepper,
                                 uint32_t acceleration)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_acceleration()");

        err = -ESP_ERR_INVALID_ARG;
        if (stepper && (acceleration > 0) &&
            (acceleration <= 0xFFFF)) {
            // Write to the AMAX register (0x26)
            err = write_reg(stepper, 0x26, acceleration);
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_acceleration()");
    }

    return err;
}

// Set the velocity of the stepper motor attached to a
// TMC5160 device.
int32_t tmc5160_set_velocity(stepper_t *stepper, int32_t millihertz)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_velocity()");

        uint32_t vmax;
        err =  millihertz_to_vmax(stepper, millihertz, &vmax);
        if (err == ESP_OK) {
            // Write to the VMAX register (0x27)
            err = write_reg(stepper, 0x27, vmax);
            if (err == ESP_OK) {
                // Write to the RAMPMODE register (0x20)
                err = write_reg(stepper, 0x20,
                                millihertz < 0 ? 2U : 1U);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_velocity()");
    }

    return err;
}

// Get the velocity of the stepper motor attached to a
// TMC5160 device.
int32_t tmc5160_get_velocity(stepper_t *stepper, int32_t *millihertz)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_get_velocity()");

        err = -ESP_ERR_INVALID_ARG;
        // read_reg() checks the validity of stepper
        if (millihertz) {
            // Read the microstep resolution
            int32_t microstep_resolution;
            err = get_microstep_resolution(stepper, &microstep_resolution);
            if (err == ESP_OK) {
                // Read the VACTUAL register
                uint32_t data;
                err = read_reg(stepper, 0x22, &data);
                if (err == ESP_OK) {
                    // Since VACTUAL is a signed value of length 24 bits,
                    // need to sign extend it manually
                    int32_t vactual = (int32_t) (data & 0x007fffffU) -
                                    (int32_t) (data & 0x00800000U);
                    int64_t numerator = ((int64_t) vactual) * TMC5160_CLOCK_HZ * 1000;
                    int64_t denominator = (1LL << 24) *
                                        MOTOR_STEPS_PER_REV *
                                        microstep_resolution;

                    *millihertz = numerator / denominator;
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_get_velocity()");
    }
    return err;
}

// Set the target position of a TMC5160, in microsteps.
int32_t tmc5160_set_position(stepper_t *stepper, int32_t microsteps,
                             uint32_t max_velocity_millihertz)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_position()");

        err = -ESP_ERR_INVALID_ARG;
        if (max_velocity_millihertz <= INT32_MAX) {
            uint32_t vmax;
            err =  millihertz_to_vmax(stepper, (int32_t) max_velocity_millihertz, &vmax);
            if (err == ESP_OK) {
                // Order is important here
                // Set target position in XTARGET (0x2d)
                err = write_reg(stepper, 0x2d, (uint32_t) microsteps);
                if (err == ESP_OK) {
                    // Write to the VMAX register (0x27)
                    err = write_reg(stepper, 0x27, vmax);
                }
                if (err == ESP_OK) {
                    // Select positioning mode in RAMPMODE (0x20)
                    err = write_reg(stepper, 0x20, 0);
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_position()");
    }

    return err;
}

// Read the position of a TMC5160, in microsteps.
int32_t tmc5160_get_position(stepper_t *stepper, int32_t *microsteps)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_get_position()");

        err = -ESP_ERR_INVALID_ARG;
        // read_reg() checks the validity of stepper
        if (microsteps) {
            uint32_t data;
            // Read XACTUAL (0x21)
            err = read_reg(stepper, 0x21, &data);
            if (err == ESP_OK) {
                *microsteps = (int32_t) data;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_get_position()");
    }

    return err;
}

// Select the chopper mode.  Changing mode is only allowed with
// the motor disabled so that the next enable can calibrate stealthChop.
int32_t tmc5160_set_chopper_mode(stepper_t *stepper,
                                 tmc5160_chopper_mode_t mode)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {
        CONTEXT_LOCK(g_context.lock, "tmc5160_set_chopper_mode()");

        err = -ESP_ERR_INVALID_ARG;
        if (stepper && ((mode == TMC5160_CHOPPER_SPREAD_CYCLE) ||
                        (mode == TMC5160_CHOPPER_STEALTH_CHOP))) {
            err = -ESP_ERR_INVALID_STATE;
            if (!stepper->motor_enabled) {
                uint32_t gconf;
                err = read_reg(stepper, 0x00, &gconf);
                if ((err == ESP_OK) &&
                    (mode == TMC5160_CHOPPER_STEALTH_CHOP)) {
                    // PWMCONF is write-only: do not attempt read-modify-write.
                    err = write_reg(stepper, 0x70, STEALTH_CHOP_PWMCONF);
                }
                if (err == ESP_OK) {
                    // en_pwm_mode is bit 2; preserve other GCONF bits.
                    if (mode == TMC5160_CHOPPER_STEALTH_CHOP) {
                        gconf |= (1U << 2);
                    } else {
                        gconf &= ~(1U << 2);
                    }
                    err = write_reg(stepper, 0x00, gconf);
                }
                if (err == ESP_OK) {
                    // 0 permits stealthChop at all velocities when enabled.
                    // It also leaves spreadCycle selected when en_pwm_mode=0.
                    err = write_reg(stepper, 0x13, 0);
                }
                if (err == ESP_OK) {
                    stepper->chopper_mode = mode;
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_chopper_mode()");
    }

    return err;
}

// Set the StealthChop threshold.
int32_t tmc5160_set_stealth_chop_threshold(stepper_t *stepper,
                                           uint32_t threshold)
{

    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_stealth_chop_threshold()");

        // Write to the TPWMTHRS register (0x13)
        err = write_reg(stepper, 0x13, threshold);

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_stealth_chop_threshold()");

    }

    return err;
}

// Configure the chopper in a TMC5160.
int32_t tmc5160_configure_chopper(stepper_t *stepper,
                                  uint8_t tbl,
                                  uint8_t toff,
                                  uint8_t hstrt,
                                  int8_t hend)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_configure_chopper()");

        err = -ESP_ERR_INVALID_ARG;
        if ((tbl <= 3) && (toff <= 15) && (hstrt <= 7) &&
            (hend >= -3) && (hend <= 12)) {

            uint32_t data;
            err = read_reg(stepper, 0x6c, &data);
            if (err == ESP_OK) {
                uint32_t hend_encoded = (uint32_t) (hend + 3);

                data = (data & 0xfffe7800U) |
                       (toff & 0x0fU) |
                       (((uint32_t) hstrt & 0x07U) << 4) |
                       ((hend_encoded & 0x0fU) << 7) |
                       (((uint32_t) tbl & 0x03U) << 15);

                err = write_reg(stepper, 0x6c, data);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_configure_chopper()");

    }

    return err;
}

// Get the value of TSTEP from a TMC5160.
int32_t tmc5160_get_tstep(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_get_tstep()");

        // Read the TSTEP register (0x12)
        uint32_t data;
        err = read_reg(stepper, 0x12, &data);
        if (err == ESP_OK) {
            err = data;
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_get_tstep()");
    }

    return err;
}

// Get the value of SG_RESULT from a TMC5160.
int32_t tmc5160_get_sg_result(stepper_t *stepper)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_get_sg_result()");

        // SG_RESULT is in bits 0 to 9 of the DRV_STATUS register (0x6f)
        uint32_t data;
        err = read_reg(stepper, 0x6f, &data);
        if (err == ESP_OK) {
            err = data & 0x000003ff;
        }
        CONTEXT_UNLOCK(g_context.lock, "tmc5160_get_sg_result()");

    }

    return err;
}

// Set the operation of stall-guard in a TMC5160.
int32_t tmc5160_init_stallguard(stepper_t *stepper,
                                int32_t tcoolthrs,
                                int8_t sgt,
                                int32_t pin,
                                gpio_isr_t handler,
                                void *handler_arg)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_init_stallguard()");

        err = -ESP_ERR_INVALID_ARG;
        if ((pin < 0) || (handler != NULL)) {
            err = set_stallguard(stepper, tcoolthrs, sgt);
            if ((err == ESP_OK) && (pin >= 0)) {
                ESP_LOGI(TAG, "Configuring interrupt pin %d.", pin);
                gpio_config_t cfg = {
                    .intr_type = GPIO_INTR_POSEDGE,
                    .mode = GPIO_MODE_INPUT,
                    .pin_bit_mask = (1ULL << pin),
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .pull_up_en = GPIO_PULLUP_ENABLE
                };
                err = -gpio_config(&cfg);
                if (err == ESP_OK) {
                    err = -gpio_install_isr_service(0);
                    // ESP_ERR_INVALID_STATE means the ISR service was already installed
                    if ((err == ESP_OK) || (err == ESP_ERR_INVALID_STATE)) {
                        err = -gpio_isr_handler_add((gpio_num_t) pin,
                                                    handler, handler_arg);
                    }
                }
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_init_stallguard()");

    }

    return err;
}

// Set the stall-guard registers in a TMC5160.
int32_t tmc5160_set_stallguard(stepper_t *stepper,
                               int32_t tcoolthrs,
                               int8_t sgt)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_stallguard()");

        err = set_stallguard(stepper, tcoolthrs, sgt);

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_stallguard()");

    }

    return err;
}

// Remove interrupt handling that was set up by
// tmc5160_init_stallguard().
void tmc5160_deinit_stallguard(int32_t pin)
{
    gpio_isr_handler_remove((gpio_num_t) pin);
}

// Set the CoolStep threshold registers.
int32_t tmc5160_set_coolstep(stepper_t *stepper, uint8_t seimin,
                             uint8_t semin, uint8_t semax)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.spi != NULL) {

        CONTEXT_LOCK(g_context.lock, "tmc5160_set_coolstep()");

        // Read the COOLCONF register (0x6d)
        uint32_t data;
        err = read_reg(stepper, 0x6d, &data);
        if (err == ESP_OK) {
            // SEMIN is in bits 0 to 3, SEMAX bits 8 to 11, SEIMIN bit 15
            data = (data & 0xffffe0f0) | (semin & 0x0f) | (((uint32_t) (semax & 0x0f)) << 8) | (((uint32_t) (seimin & 0x01)) << 15);
            // Write the new value back
            err = write_reg(stepper, 0x6d, data);
        }

        CONTEXT_UNLOCK(g_context.lock, "tmc5160_set_coolstep()");

    }

    return err;
}

// End of file

