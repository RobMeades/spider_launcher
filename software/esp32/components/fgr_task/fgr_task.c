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
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_log.h"
#include "sys/queue.h"

#include "fgr_util.h"
#include "fgr_task.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "task"

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

// Structure to allow monitoring of a task's state.
typedef struct {
    SemaphoreHandle_t lock;
    fgr_task_state_cb_t cb;
    void *cb_param;
} task_state_t;

// Structure that defines all the things we need for a task,
// designed to be used as part of a linked list.
typedef struct task_t {
    fgr_task_cb_t cb;
    void *cb_param;
    const char *name;
    TaskHandle_t handle;
    bool running;
    SemaphoreHandle_t running_semaphore;
    int32_t min_free_stack_bytes;
    task_state_t state;
    SLIST_ENTRY(task_t) next;
} task_t;

// Message receive callback list head.
SLIST_HEAD(task_list_t, task_t);

// Context.
typedef struct {
    SemaphoreHandle_t lock;
    bool is_shutting_down;
    task_t *min_free_stack_next_task;
    struct task_list_t task_list;
    fgr_task_state_cb_t global_task_state_cb;
    void *global_task_state_cb_param;
} context_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

// Context.
static context_t g_context = {0};

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

// The base task that runs all of the tasks.
static void task_base(void *param)
{
    task_t *task = (task_t *) param;
    task_state_t task_state_stored = task->state;
    task_state_t *task_state = &task->state;

    esp_task_wdt_add(NULL);

    CONTEXT_LOCK(task->running_semaphore, task->name);

    while (task->running) {

        // On ESP-IDF uxTaskGetStackHighWaterMark() returns the
        // minimum free stack available in bytes, not words.
        task->min_free_stack_bytes = uxTaskGetStackHighWaterMark(NULL);

        // Do the thang
        task->cb(task->handle, task->cb_param);

        // If there is a task state callback,
        // pass it an initial "STARTED" indication,
        // or otherwise a "RUNNING" indication
        CONTEXT_LOCK(task_state->lock, "task state 1");
        if (task_state->cb) {
            fgr_task_state_t state = FGR_TASK_STATE_RUNNING;
            if ((task_state->cb != task_state_stored.cb) ||
                (task_state->cb_param != task_state_stored.cb_param)) {
                state = FGR_TASK_STATE_STARTED;
            }
            task_state->cb(state, task->handle, task->name,
                           task_state->cb_param);
        }
        task_state_stored = task->state;
        CONTEXT_UNLOCK(task_state->lock, "task state 1");

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(FGR_UTIL_WATCHDOG_FEED_TIME_MS));
    }

    ESP_LOGI(TAG, "task \"%s\" exiting.", task->name);

    CONTEXT_LOCK(task_state->lock, "task state 2");
    if (task_state->cb) {
        task_state->cb(FGR_TASK_STATE_STOPPED,
                       task->handle, task->name,
                       task_state->cb_param);
    }
    CONTEXT_UNLOCK(task_state->lock, "task state 2");

    CONTEXT_UNLOCK(task->running_semaphore, task->name);

    esp_task_wdt_delete(NULL);
    vTaskDelete(NULL);
}

// Destroy a task and remove it from the linked list.
// IMPORTANT: the context must be locked before this is called.
static void task_destroy(task_t *task)
{
    if (task) {
        // Flag should stop task running
        task->running = false;
        if (task->running_semaphore) {
            // Take the running semaphore to know its stopped
            CONTEXT_LOCK(task->running_semaphore, "task_destroy() 1");
            CONTEXT_UNLOCK(task->running_semaphore, "task_destroy() 1");
            vSemaphoreDelete(task->running_semaphore);
        }
        if (task->state.lock) {
            vSemaphoreDelete(task->state.lock);
        }
        // Remove the task from the list, if present
        task_t *iter;
        task_t *prev = NULL;
        SLIST_FOREACH(iter, &g_context.task_list, next) {
            if (iter->handle == task->handle) {
                if (prev == NULL) {
                    // Removing the first element
                    SLIST_REMOVE_HEAD(&g_context.task_list, next);
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
        free(task);
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: INITIALISATION/DEINITIALISATION
 * -------------------------------------------------------------- */

// Initialise task stuff.
int32_t fgr_task_init()
{
    int32_t err = ESP_OK;

    if (!g_context.lock) {
        // Create mutex
        err = -ESP_ERR_NO_MEM;
        g_context.lock = xSemaphoreCreateMutex();
        SLIST_INIT(&g_context.task_list);
    }

    if (g_context.lock) {
        g_context.is_shutting_down = false;
        err = ESP_OK;
    }

    return err;
}

// Deinitialise task stuff.
void fgr_task_deinit()
{
    if (g_context.lock) {

        // Set this flag so that task_destroy()
        // doesn't get stuck because that task
        // is in the middle of calling one of
        // this API's functions (e.g. fgr_task_min_free_stack()
        // etc.)
        g_context.is_shutting_down = true;

        CONTEXT_LOCK(g_context.lock, "fgr_task_deinit()");

        while (!SLIST_EMPTY(&g_context.task_list)) {
            task_t *p = SLIST_FIRST(&g_context.task_list);
            // Do not destroy the current task (the one calling this function)
            if (p->handle != xTaskGetCurrentTaskHandle()) {
                task_destroy(p);
            } else {
                // ...but remove it from the list
                SLIST_REMOVE_HEAD(&g_context.task_list, next);
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_task_deinit()");
        // The semaphore will be re-used
    }
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: CREATE/DESTROY/STATE
 * -------------------------------------------------------------- */

// Create a task.
int32_t fgr_task_create(fgr_task_cb_t cb, void *cb_param, const char *name,
                        size_t stack_size_bytes, int32_t priority,
                        void *handle)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (cb && handle) {

        err = -ESP_ERR_INVALID_STATE;

        if (g_context.lock && !g_context.is_shutting_down) {

            CONTEXT_LOCK(g_context.lock, "fgr_task_create()");

            err = -ESP_ERR_NO_MEM;
            task_t *task = (task_t *) malloc(sizeof(*task));
            if (task) {
                memset(task, 0, sizeof(*task));
                task_state_t *state = &task->state;
                state->lock = xSemaphoreCreateMutex();
                if (state->lock) {
                    state->cb = g_context.global_task_state_cb;
                    state->cb_param = g_context.global_task_state_cb_param;
                    task->cb = cb;
                    task->cb_param = cb_param;
                    task->name = name;
                    task->running_semaphore = xSemaphoreCreateMutex();
                }
            }
            if (task && task->state.lock && task->running_semaphore) {
                task->running = true;
                if (xTaskCreate(task_base, name, stack_size_bytes,
                                task, priority, &task->handle) == pdPASS) {
                    *((TaskHandle_t *) handle) = task->handle;
                    SLIST_INSERT_HEAD(&g_context.task_list, task, next);
                    err = ESP_OK;
                } else {
                    task->running = false;
                    task->handle = NULL;    // Just in case
                }
            }

            if (err != ESP_OK) {
                task_destroy(task);
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_task_create()");
        }
    }

    return err;
}

// Destroy a task.
void fgr_task_destroy(void *handle)
{
    if (g_context.lock && !g_context.is_shutting_down) {

        CONTEXT_LOCK(g_context.lock, "fgr_task_destroy()");

        task_t *iter;
        SLIST_FOREACH(iter, &g_context.task_list, next) {
            if (iter->handle == handle) {
                ESP_LOGD(TAG, "task \"%s\" high watermark was %d byte(s).",
                         iter->name ? iter->name : "",
                         iter->min_free_stack_bytes);
                task_destroy(iter);
                break;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_task_destroy()");
    }
}

// Determine if the given task is running.
bool fgr_task_is_running(void *handle)
{
    bool task_is_running = false;

    if (g_context.lock && !g_context.is_shutting_down) {

        CONTEXT_LOCK(g_context.lock, "fgr_task_is_running()");

        task_t *iter;
        SLIST_FOREACH(iter, &g_context.task_list, next) {
            if (iter->handle == handle) {
                task_is_running = true;
                break;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_task_is_running()");
    }

    return task_is_running;
}

// Set a callback that will be called every task loop.
int32_t fgr_task_state_cb_set(fgr_task_state_cb_t cb,
                              void *cb_param)
{
    int32_t err = -ESP_ERR_INVALID_STATE;

    if (g_context.lock && !g_context.is_shutting_down) {

        CONTEXT_LOCK(g_context.lock, "fgr_task_state_cb_set()");

        g_context.global_task_state_cb = cb;
        g_context.global_task_state_cb_param = cb_param;

        task_t *iter;
        SLIST_FOREACH(iter, &g_context.task_list, next) {
            task_state_t *task_state = &iter->state;
            CONTEXT_LOCK(task_state->lock, iter->name);
            task_state->cb = g_context.global_task_state_cb;
            task_state->cb_param = g_context.global_task_state_cb_param;
            CONTEXT_UNLOCK(task_state->lock, iter->name);
        }
        err = ESP_OK;

        CONTEXT_UNLOCK(g_context.lock, "fgr_task_state_cb_set()");
    }

    return err;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS: STACK RELATED
 * -------------------------------------------------------------- */

// Get the stack high watermark.
int32_t fgr_task_min_free_stack(void *handle)
{
    int32_t high_watermark = 0;

    if (g_context.lock && !g_context.is_shutting_down) {

        CONTEXT_LOCK(g_context.lock, "fgr_task_min_free_stack()");

        task_t *iter;
        SLIST_FOREACH(iter, &g_context.task_list, next) {
            if (iter->handle == handle) {
                high_watermark = iter->min_free_stack_bytes;
                break;
            }
        }

        CONTEXT_UNLOCK(g_context.lock, "fgr_task_min_free_stack()");
    }

    return high_watermark;
}

// The start the sequence of calls to get the minimum free stack.
int32_t fgr_task_min_free_stack_start(const char **name,
                                      int32_t *min_free)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (name && min_free) {

        err = -ESP_ERR_INVALID_STATE;

        if (g_context.lock && !g_context.is_shutting_down) {

            CONTEXT_LOCK(g_context.lock, "fgr_task_min_free_stack_start()");

            err = -ESP_ERR_NOT_FOUND;

            if (!g_context.min_free_stack_next_task) {
                task_t *iter;
                err = 0;
                SLIST_FOREACH(iter, &g_context.task_list, next) {
                    if (err == 0) {
                        *name = iter->name;
                        *min_free = iter->min_free_stack_bytes;
                    } else if (err == 1) {
                        g_context.min_free_stack_next_task = iter;
                    }
                    err++;
                }

                // Return the number of tasks _remaining_ in the list,
                // will return ESP_FAIL (-1) if there were no tasks at all
                err--;
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_task_min_free_stack_start()");
        }
    }

    return err;
}

// Get the next in the set of minimum free stack values.
int32_t fgr_task_min_free_stack_next(const char **name,
                                     int32_t *min_free)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (name && min_free) {

        err = -ESP_ERR_INVALID_STATE;

        if (g_context.lock && !g_context.is_shutting_down) {

            CONTEXT_LOCK(g_context.lock, "fgr_task_min_free_stack_next()");

            err = -ESP_ERR_NOT_FOUND;

            if (g_context.min_free_stack_next_task) {
                task_t *iter;
                err = 0;
                SLIST_FOREACH(iter, &g_context.task_list, next) {
                    if (err > 0) {
                        if (err == 1) {
                            g_context.min_free_stack_next_task = iter;
                        }
                        // If we are beyond g_context.min_free_stack_next_task,
                        // just increment err
                        err++;
                    }
                    if ((err == 0) && (iter == g_context.min_free_stack_next_task)) {
                        *name = iter->name;
                        *min_free = iter->min_free_stack_bytes;
                        g_context.min_free_stack_next_task = NULL;
                        err++;
                    }
                }

                // Return the number of tasks _remaining_ in the list,
                // will return ESP_FAIL (-1) if there were no tasks at all
                err--;
            }

            CONTEXT_UNLOCK(g_context.lock, "fgr_task_min_free_stack_next()");
        }
    }

    return err;
}

// Stop reading task stack high watermark values.
void fgr_task_min_free_stack_stop()
{
    if (g_context.lock && !g_context.is_shutting_down) {
        CONTEXT_LOCK(g_context.lock, "fgr_task_min_free_stack_stop()");
        g_context.min_free_stack_next_task = NULL;
        CONTEXT_UNLOCK(g_context.lock, "fgr_task_min_free_stack_stop()");
    }
}

// End of file
