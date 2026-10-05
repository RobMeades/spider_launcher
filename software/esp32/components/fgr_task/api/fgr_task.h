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

#ifndef _FGR_TASK_H_
#define _FGR_TASK_H_

/** @file
 * @brief Tasking API for a node of the front garden railway.
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

/** The possible task states.  If you ever add a new state here,
 * don't forget to update the logic over in fgr_monitor.c.
 */
typedef enum {
    FGR_TASK_STATE_STARTED,
    FGR_TASK_STATE_RUNNING,
    FGR_TASK_STATE_STOPPED
} fgr_task_state_t;

/** A task.
 *
 * @param handle the task's handle; this may be needed if the
 *               task is going to be busy for a while and
 *               hence needs to call fgr_monitor_task_wdt_feed().
 * @param param  cb_param as passed to fgr_task_create().
 */
typedef void (*fgr_task_cb_t) (void *handle, void *param);

/** Callback function to provide a tasks's state.
 *
 * @param state  the task state.
 * @param handle the handle of the task.
 * @param name   the task name, may be NULL.
 * @param param  cb_param as passed to fgr_task_state_cb_set().
 */
typedef void (*fgr_task_state_cb_t) (fgr_task_state_t state,
                                     void *handle,
                                     const char *name,
                                     void *param);

/* ----------------------------------------------------------------
 * FUNCTIONS: INITIALISATION/DEINITIALISATION
 * -------------------------------------------------------------- */

/** Initialise tasking.  This function may be called at any time;
 * if it has already been called it will do nothing and return
 * success.
 *
 * Note: this will create a semaphore that is never destroyed.
 *
 * @return ESP_OK on success, else a negative value from esp_err_t.
 */
int32_t fgr_task_init();

/** Deinitialise tasking.  It is always safe to call this function.
 * All tasks that were created will be destroyed except the task
 * calling fgr_task_deinit(): if that is present in the list it
 * will be removed from the list but will _not_ be destroyed; it
 * is up to the calling task to clear itself up.
 */
void fgr_task_deinit();

/** Create a task.  The advantage of doing your task creation this way
 * is that (a) task exit is orchestrated properly and (b) watchdog timers
 * are applied and monitored automagically.  The callback will be called
 * in a loop with FGR_UTIL_WATCHDOG_FEED_TIME_MS loop delay to let the
 * idle task feed its task watchdog.
 *
 * @param cb               the callback that forms the body of the task;
 *                         cannot be NULL.  The callback must return
 *                         in a reasonably time frame, e.g. within
 *                         10 seconds, in order that the rest of the
 *                         task loop runs to feed the watchdogs.
 * @param cb_param         a parameter to pass to the callback; may
 *                         be NULL.
 * @param name             a null-terminated string that forms the
 *                         name of the task, purely for debugging
 *                         purposes, may be truncated if more than
 *                         about 16 characters in length, may be NULL.
 *                         If non-NULL this MUST be a true constant
 *                         that is valid for the duration of the
 *                         task's run.
 * @param stack_size_bytes the amount of stack to allocate for the
 *                         task in bytes.
 * @param priority         the task priority, where higher numbers
 *                         represent a higher priority, normal
 *                         ESP-IDF task priority.
 * @param handle           a pointer to a place to store the task
 *                         handle; cannot be NULL.  The handle will
 *                         be a good 'ole FreeRTOS TaskHandle, nothing
 *                         weird; you can do anything with it that
 *                         you could do with a TaskHandle.  Will not
 *                         be touched on failure.
 * @return                 on success an ID for the task, else a
 *                         negative value from esp_err_t.
 */
int32_t fgr_task_create(fgr_task_cb_t cb, void *cb_param, const char *name,
                        size_t stack_size_bytes, int32_t priority,
                        void *handle);

/** Destroy a task.
 *
 * @param handle  the handle of the task to destroy.
 */
void fgr_task_destroy(void *handle);

/** Determine if the given task is running.
 *
 * @param handle  the handle of the task.
 * @return        true if the task is running, else false.
 */
bool fgr_task_is_running(void *handle);

/** Set a callback that will be called every task loop that
 * gives the task's current state; may be used as a software
 * watchdog.
 *
 * IMPORTANT: don't do much in this callback: it really is
 * called every on every task loop, and do not call into the
 * task API from the callback as that will cause a deadlock.
 *
 * @param cb        the callback; use NULL to cancel a previous
 *                  callback.
 * @param cb_param  parameter that will be passed to cb()
 *                  when it is called; may be NULL.
 * @return          ESP_OK on success, else a negative value
 *                  from esp_err_t.
 */
int32_t fgr_task_state_cb_set(fgr_task_state_cb_t cb,
                              void *cb_param);

/** Get the stack high watermark, i.e. the smallest
 * amount of stack ever left over time.
 *
 * @param handle  the handle of the task.
 * @return        the high watermark in bytes,
 *                zero if handle is not found.
 */
int32_t fgr_task_min_free_stack(void *handle);

/** The start of a sequence of calls to get the minimum
 * free stack values of all of the tasks that fgr_tasks
 * knows about: i.e. fgr_task_min_free_stack() for all.
 *
 * A usage pattern might be:
 *
 *    const char *name = NULL;
 *    int32_t min_free = 0;
 *    if (fgr_task_min_free_stack_start(&name, &min_free) >= 0) {
 *        do {
 *            printf("task %s had min free stack %d.\n", name, min_free);
 *        } while (fgr_task_min_free_stack_next(&name, &min_free) >= 0);
 *        fgr_task_min_free_stack_stop();
 *    }
 *
 * When fgr_task_min_free_stack_start() has returned a
 * non-negative value and you don't loop through all of the
 * entries until fgr_task_min_free_stack_next() returns zero
 * or less, fgr_task_min_free_stack_stop() MUST be
 * called to terminate the sequence (and there is no
 * harm in always calling it at the end of the sequence
 * even if you do loop through the lot).
 *
 * This sequence of functions should only be called from
 * a single thread at any one time; the sequence of
 * calls is single-threaded.
 *
 * If a new task is added with a call to fgr_task_create()
 * it may or may not be included; best not do that.
 *
 * @param name           a place to put the name of the task
 *                       being reported on; cannot be NULL.
 * @param min_free_bytes a place to put the minimum free stack
 *                       value (in bytes); cannot be NULL.
 * @return               on success, the number of calls to
 *                       fgr_task_min_free_stack_next() that
 *                       are required to read all of the
 *                       task stack free min values, else
 *                       negative value from esp_err_t.
 */
int32_t fgr_task_min_free_stack_start(const char **name,
                                      int32_t *min_free);

/** Get the next in the set of minimum free stack values,
 * see fgr_task_min_free_stack_start() for an explanation.
 *
 * @param name           a place to put the name of the task
 *                       being reported on; cannot be NULL.
 * @param min_free_bytes a place to put the minimum free stack
 *                       value (in bytes); cannot be NULL.
 * @return               on success, the number of subsequent
 *                       calls required to read all of the
 *                       task stack free min values, else
 *                       negative value from esp_err_t.
 */
int32_t fgr_task_min_free_stack_next(const char **name,
                                     int32_t *min_free);

/** Stop reading task stack high watermark values, see
 * fgr_task_min_free_stack_start() for an explanation.
 */
void fgr_task_min_free_stack_stop();

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _FGR_TASK_H_

// End of file
