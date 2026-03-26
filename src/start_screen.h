/*
 * FRANK Prince Start Screen
 * Displays system info and errors before game launch
 */

#ifndef START_SCREEN_H
#define START_SCREEN_H

#include <stdbool.h>
#include <stdint.h>

// Error codes for start screen
typedef enum {
    START_OK = 0,
    START_ERROR_NO_SD,
    START_ERROR_NO_DATA_DIR,
    START_ERROR_UNKNOWN
} start_error_t;

/**
 * Show the start screen with system info.
 * Waits for any keypress to continue.
 * @param error Error code to display (START_OK for no error)
 * @param error_msg Optional custom error message (NULL for default)
 */
void start_screen_show(start_error_t error, const char* error_msg);

/**
 * Check if SD card and data directory are available.
 * @return START_OK if all good, error code otherwise
 */
start_error_t start_screen_check_requirements(void);

#endif // START_SCREEN_H
