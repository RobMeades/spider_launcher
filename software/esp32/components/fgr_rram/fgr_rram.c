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
 * @brief Retained RAM functions for a node of the front garden railway.
 */

// Ensure we are compiling with maximum debug, can then be trimmed
// at run-time by fgr_log
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "esp_err.h"
#include "esp_rom_crc.h"

#include "fgr_rram.h"

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

// Logging prefix
#define TAG "rram"

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Set a retained RAM variable.
int32_t fgr_rram_set(const void *variable, size_t variable_size,
                     void *rram_variable, size_t rram_variable_size)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (variable && rram_variable) {
        err = -ESP_ERR_INVALID_SIZE;
        size_t variable_size_max = rram_variable_size - sizeof(uint32_t);

        if (variable_size <= variable_size_max) {
            err = ESP_OK;

            size_t whole_words = variable_size / sizeof(uint32_t);
            size_t remaining_bytes = variable_size % sizeof(uint32_t);
            size_t total_container_words = variable_size_max / sizeof(uint32_t);

            // Initialize the running CRC tracking state
            uint32_t crc = 0xFFFFFFFF;

            const uint32_t *src_words = (const uint32_t *) variable;
            uint32_t *dst_words = (uint32_t *) rram_variable;
            // Force a strict, un-widened 32-bit hardware bus copy loop
            // Using volatile here blocks the compiler from converting this into
            // 64-bit/128-bit instructions
            volatile uint32_t *volatile_dst_words = (volatile uint32_t *) dst_words;

            // Copy and CRC all whole 32-bit words cleanly onto the bus
            size_t word_idx = 0;
            while (word_idx < whole_words) {
                uint32_t word = *(src_words + word_idx);
                *(volatile_dst_words + word_idx) = word;
                crc = esp_rom_crc32_le(crc, (const uint8_t *) &word, sizeof(word));
                word_idx++;
            }

            // Handle trailing unaligned bytes safely by packing them into a clean 32-bit word
            if (remaining_bytes > 0) {
                uint32_t final_word = 0;
                const uint8_t *src_bytes = (const uint8_t *) (src_words + word_idx);

                for (size_t byte_idx = 0; byte_idx < remaining_bytes; byte_idx++) {
                    *(((uint8_t *) &final_word) + byte_idx) = *(src_bytes + byte_idx);
                }

                *(volatile_dst_words + word_idx) = final_word;
                crc = esp_rom_crc32_le(crc, (const uint8_t *) &final_word, sizeof(final_word));
                word_idx++;
            }

            // Zero-pad the rest of the available container space on the RTC bus
            // and pass the zeros into the CRC block to make the footprint uniform for GET.
            while (word_idx < total_container_words) {
                uint32_t zero_word = 0;
                *(volatile_dst_words + word_idx) = zero_word;
                crc = esp_rom_crc32_le(crc, (const uint8_t *) &zero_word, sizeof(zero_word));
                word_idx++;
            }

            // Latch the CRC into the absolute final slot of the container layout.
            *(volatile_dst_words + word_idx) = crc;
        }
    }

    return err;
}

// Get a retained RAM variable into a normal variable.
int32_t fgr_rram_get(const void *rram_variable, size_t rram_variable_size,
                     void *variable, size_t variable_size)
{
    int32_t err = -ESP_ERR_INVALID_ARG;

    if (variable && rram_variable) {
        err = -ESP_ERR_INVALID_CRC;

        // Process the data chunks based on the container size to parse the
        // full zero-padded footprint and reach the fixed CRC slot location safely.
        size_t variable_size_max = rram_variable_size - sizeof(uint32_t);
        size_t container_words = variable_size_max / sizeof(uint32_t);

        // Mark the source as volatile to force strict 32-bit reads from the RTC bus
        const volatile uint32_t *volatile_src_words = (const volatile uint32_t *) rram_variable;
        uint32_t *dst_words = (uint32_t *) variable;

        uint32_t calculated_crc = 0xFFFFFFFF;
        size_t word_idx = 0;

        // Loop through the entire capacity of the container's payload area.
        while (word_idx < container_words) {
            uint32_t word = *(volatile_src_words + word_idx);
            calculated_crc = esp_rom_crc32_le(calculated_crc, (const uint8_t *) &word, sizeof(word));

            // Work out how many bytes have been safely written to the destination
            size_t bytes_written_so_far = word_idx * sizeof(word);

            if (bytes_written_so_far + sizeof(word) <= variable_size) {
                // Safe for a full 32-bit word copy instruction
                *(dst_words + word_idx) = word;
            } else if (bytes_written_so_far < variable_size) {
                // Sift only the genuine remaining data bytes into the destination
                size_t remaining_bytes = variable_size - bytes_written_so_far;
                uint8_t *dst_bytes = (uint8_t *) variable;

                for (size_t byte_idx = 0; byte_idx < remaining_bytes; byte_idx++) {
                    *(dst_bytes + bytes_written_so_far + byte_idx) = *(((uint8_t *) &word) + byte_idx);
                }
            }
            // If bytes_written_so_far >= variable_size, we discard the extra padding bytes data
            // but keep cycling the loop to finish validating our CRC against the bus.
            word_idx++;
        }

        // Validate against the fixed CRC tracking slot sitting right at the end of the container
        if (calculated_crc == *(volatile_src_words + word_idx)) {
            err = ESP_OK;
        }
    }

    return err;
}

// Clear a retained RAM variable safely.
void fgr_rram_clear(void *rram_variable, size_t rram_variable_size)
{
    if (rram_variable) {
        size_t payload_size = rram_variable_size - sizeof(uint32_t);
        size_t total_words = payload_size / sizeof(uint32_t);

        // A block of zeroes has a predictable, non-zero, CRC
        uint32_t crc = 0xFFFFFFFF;
        const uint32_t zero_word = 0;

        uint32_t *dst_words = (uint32_t *) rram_variable;

        // The 'volatile' qualifier forces GCC to emit individual S32I (Store 32-bit)
        // assembly instructions, completely blocking the Loop Distribution optimization pass
        volatile uint32_t *volatile_dst_words = (volatile uint32_t *) dst_words;

        for (size_t x = 0; x < total_words; x++) {
            *(volatile_dst_words + x) = 0;

            // Calculate the valid CRC for an all-zero payload inline
            crc = esp_rom_crc32_le(crc, (const uint8_t *) &zero_word, sizeof(uint32_t));
        }

        // Latch the correct "all-zero payload" CRC into the final slot
        *(volatile_dst_words + total_words) = crc;
    }
}

// End of file
