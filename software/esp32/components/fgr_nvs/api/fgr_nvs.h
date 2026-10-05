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

#ifndef _FGR_NVS_H_
#define _FGR_NVS_H_

/** @file
 * @brief The NVS API for a node of the front garden railway: just
 * init really.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

#ifndef FGR_NVS_STORAGE_AREA
// The name of the default NVS storage area.
#  define FGR_NVS_STORAGE_AREA "nvs"
#endif

#ifndef FGR_NVS_COMMIT_GUARD_MS
// The guard time after calling nvs_commit() to wait for the
// write to flash to complete: without some time here the DMA
// may not complete if there is a reset and flash will come up
// as not having enough space.
#  define FGR_NVS_COMMIT_GUARD_MS 1000
#endif

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Initialise NVS. If NVS has already been initialised this will
 * do nothing and return success.
 *
 * IMPORTANT: if you are using fgr_ota do NOT call this: OTA
 * needs to initialise first and it will then, internally, call
 * fgr_nvs_init();
 *
 * @return  ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_nvs_init();

/** Retrieve a uint32_t value from NVS.
 *
 * @param name  a null-terminated string that is the name of the
 *              value to retrieve.
 * @param value a pointer to a place to put the returned value.
 * @return      ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_nvs_get(const char *name, uint32_t *value);

/** Store a uint32_t value to NVS.
 *
 * IMPORTANT: there will be a delay of at least FGR_NVS_COMMIT_GUARD_MS
 * before this function returns.
 *
 * @param name  a null-terminated string that is the name of the
 *              value to store.
 * @param value the value to store.
 * @return      ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_nvs_set(const char *name, uint32_t value);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _FGR_NVS_H_

// End of file
