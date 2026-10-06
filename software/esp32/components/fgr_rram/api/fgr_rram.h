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

#ifndef _FGR_RRAM_H_
#define _FGR_RRAM_H_

/** @file
 * @brief Retained RAM API for a node of the front garden railway.
 * Why is this necessary?  Well it turns out that the retained RAM
 * on an ESP32 is on the RTC peripheral bus and this does not behave
 * like normal RAM in subtlle ways.  The RTC is a peripheral that happens
 * to have RAM on it but, most importantly, all accesses MUST be
 * 32-bit aligned, otherwise strange things may happen; empty reads,
 * mysterious bus failures, crashes, etc.  Standard library functions
 * such as memcpy() and memset() may also cause crashes, as they expect,
 * to do optimised copy operations that assume alignments and access
 * mechanisms that simply do not work with this restricted hardware.
 *
 * Hence this API enforces the following rules for retained RAM variables:
 *
 * 1.  Hides the name of the retained RAM variable so that no references
 *     can be taken; it would be too easy to forget the restrictions below
 *     once the pointer type was passed on.
 * 2.  No standard library functions (e.g. memcpy(), memset(),
 *     strcpy(), or printf(%s)) are used on retained RAM variables.
 * 3.  Retained RAM variables CANNOT even be assigned to other variables
 *     (e.g. to make shadow copies), they can only be copied to other
 *     variables using the copy functions below; this is because,
 *     under the hood, the compiler may decide to use memcpy() when
 *     doing a block copy.
 *
 * In addition, the code/macros here are intended to work when the RTOS,
 * stack and heap memory may not be stable/available, the use case
 * being to write some data to retained RAM on a crash, reboot to ensure
 * stability and then read the data out again afterwards to determine
 * what happened.
 *
 * To store a value in retained RAM you should:
 *
 * - Use the macro FGR_RRAM_DEFINE() below to create the variable,
 *   e.g. FGR_RRAM_DEFINE(my_variable_t, my_variable); there is no
 *   need to use the "g_" prefix on what is a global variable name;
 *   the macro deals with all of that.
 * - To assign a value to the retained RAM variable, create the value
 *   in a shadow variable, which MUST be of the same name, then copy
 *   it into the retained RAM variable using the SET macro FGR_RRAM_SET().
 *   e.g. FGR_RRAM_SET(my_variable).
 * - To retrieve the value of the retained RAM variable, create a
 *   shadow variable, which MUST be of the same name, and copy the value
 *   into it using the GET macro FGR_RRAM_GET(), e.g. FGR_RRAM_GET(my_variable),
 *   then check that the return value of the macro is ESP_OK for success;
 *   if it is not then the retained RAM variable was never set.
 *
 * An example usage pattern might be, in your .c file:
 *
 * // Structure for the retained variable
 * typedef struct {
 *     int32_t length;
 *     uint8_t bean_diameter_mm[32];
 * } beans_t;
 *
 * // Create the retained RAM variable that stores beans_t; this
 * // will create a retained RAM global variable called something
 * // like g_beans_rr_container
 * FGR_RRAM_DEFINE(beans_t, beans);
 *
 * // Operate on the retained RAM variable.
 * void do_thing_with_beans()
 * {
 *     // Shadow variable in normal RAM, same as you gave to FGR_RRAM_DEFINE()
 *     beans_t beans;
 *
 *     // Try to read into the shadow variable from retained RAM
 *     int32_t err = FGR_RRAM_GET(beans);
 *
 *     if (err == ESP_OK) {
 *         // Have a populated shadow variable, do
 *         // something with it
 *
 *         printf("%d bean(s).\n", beans.length);
 *
 *         // Maybe zero the retained RAM variable to stop it
 *         // appearing on the next reboot
 *         FGR_RRAM_CLEAR(beans);
 *
 *     } else {
 *         // There was no retained value; remember a new value
 *         beans.length = 2;
 *         beans.bean_diameter_mm[0] = 5;
 *         beans.bean_diameter_mm[1] = 8;
 *
 *         // Set the retained RAM variable from the shadow variable
 *         FGR_RRAM_SET(beans);
 *     }
 * }
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Macro to declare a variable that is to be placed in retained RAM.
// Note: the wrapping here is to ensure there is no dead space between
// the payload and the CRC that might contain rubbish which could
// cause the CRC calculation to fail.
#define FGR_RRAM_DEFINE(type_name, variable_name)                                          \
    typedef struct __attribute__((aligned(4))) {                                           \
        type_name payload;                                                                 \
    } variable_name##_payload_wrapper_t;                                                   \
    typedef struct {                                                                       \
        variable_name##_payload_wrapper_t wrapped_payload;                                 \
        uint32_t crc;                                                                      \
    } variable_name##_rr_container_t;                                                      \
    RTC_NOINIT_ATTR __attribute__((aligned(4))) static variable_name##_rr_container_t g_##variable_name##_rr_container

// Macro to set a retained RAM variable from a shadow variable.
#define FGR_RRAM_SET(variable)                                                             \
    fgr_rram_set(&(variable), sizeof(variable), &(g_##variable##_rr_container), sizeof(g_##variable##_rr_container))

// Macro to get a retained RAM variable into a normal variable.
#define FGR_RRAM_GET(variable)                                                             \
    fgr_rram_get(&(g_##variable##_rr_container), sizeof(g_##variable##_rr_container), &(variable), sizeof(variable))

// Macro to clear a retained RAM variable.
// Note: the RAM variable is cleared as well as the retained RAM variable, only because
// that seems the most obvious behaviour to adopt.
#define FGR_RRAM_CLEAR(variable)                                                           \
    fgr_rram_clear(&(g_##variable##_rr_container), sizeof(g_##variable##_rr_container));   \
    memset(&variable, 0, sizeof(variable))

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * FUNCTIONS: MISC
 * -------------------------------------------------------------- */

/** Set a retained RAM variable from a shadow variable; rather than
 * calling this directly, use the macro FGR_RRAM_SET() above, which
 * sorts out the naming and size for you.
 *
 * @param variable           a pointer to the variable to copy, cannot
 *                           be NULL.
 * @param variable_size      sizeof(variable).
 * @param rram_variable      a pointer to the retained RAM variable
 *                           to copy into; the variable MUST have
 *                           been defined using the macro
 *                           FGR_RRAM_DEFINE(). Cannot be NULL.
 * @param rram_variable_size sizeof(rram_variable).
 * @return                   ESP_OK on success, else negative error
 *                           code from esp_err_t.
 */
int32_t fgr_rram_set(const void *variable, size_t variable_size,
                     void *rram_variable, size_t rram_variable_size);

/** Get a retained RAM variable into a normal variable; rather
 * than calling this directly, use the macro FGR_RRAM_GET()
 * above, which sorts out the naming and size for you.
 *
 * @param rram_variable      a pointer to the retained RAM variable
 *                           to copy from; the variable MUST have
 *                           been defined using the macro
 *                           FGR_RRAM_DEFINE().  Cannot be NULL.
 * @param rram_variable_size sizeof(rram_variable).
 * @param variable           a pointer to the normal variable to copy
 *                           into; cannot be NULL.
 * @param variable_size      sizeof(variable).
 * @return                   ESP_OK on success, else negative error
 *                           code from esp_err_t.
 */
int32_t fgr_rram_get(const void *rram_variable, size_t rram_variable_size,
                     void *variable, size_t variable_size);

/** Clear a retained RAM variable; rather than calling this directly,
 * use the macro FGR_RRAM_CLEAR() above.
 *
 * @param rram_variable      a pointer to the retained RAM variable.
 * @param rram_variable_size sizeof(rram_variable).
 */
void fgr_rram_clear(void *rram_variable, size_t rram_variable_size);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _FGR_RRAM_H_

// End of file
