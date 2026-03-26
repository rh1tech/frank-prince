/*
 * frank-prince - I2S Audio Driver Header
 * Provides a callback-based audio interface for SDL audio emulation
 * 
 * PIO/SM allocation:
 *   - HDMI: pio1 SM0, SM1
 *   - PS/2 Keyboard: pio0 SM0
 *   - I2S Audio: pio0 SM2 (avoiding PS/2 on SM0)
 * 
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef AUDIO_I2S_DRIVER_H
#define AUDIO_I2S_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Audio sample rate (matching SDLPoP's default)
#define AUDIO_SAMPLE_RATE 44100

// Audio buffer configuration
#define AUDIO_BUFFER_SAMPLES 1024
#define AUDIO_BUFFER_COUNT 4

// Audio callback function type (matches SDL_AudioCallback)
typedef void (*audio_callback_fn)(void *userdata, uint8_t *stream, int len);

/**
 * Initialize the I2S audio driver.
 * Uses pio0 SM2, DMA channel 6.
 * @param sample_rate Sample rate in Hz (typically 44100)
 * @param channels Number of channels (1 for mono, 2 for stereo)
 * @param callback Audio callback function
 * @param userdata User data passed to callback
 * @return true if successful
 */
bool audio_i2s_driver_init(uint32_t sample_rate, uint8_t channels,
                           audio_callback_fn callback, void *userdata);

/**
 * Enable or disable audio playback.
 * When enabled, the callback will be called to fill audio buffers.
 * @param enable true to enable, false to pause
 */
void audio_i2s_driver_set_enabled(bool enable);

/**
 * Check if audio is currently enabled.
 * @return true if audio is playing
 */
bool audio_i2s_driver_is_enabled(void);

/**
 * Shutdown the audio driver and release resources.
 */
void audio_i2s_driver_shutdown(void);

/**
 * Get the silence value for the audio format.
 * @return Silence sample value (0 for signed formats)
 */
uint8_t audio_i2s_driver_get_silence(void);

/**
 * Lock audio to prevent callback execution.
 * Must be paired with audio_i2s_driver_unlock().
 */
void audio_i2s_driver_lock(void);

/**
 * Unlock audio after locking.
 */
void audio_i2s_driver_unlock(void);

/**
 * Pump audio buffers from main loop.
 * Call this frequently (every frame) to fill audio buffers.
 */
void audio_i2s_driver_pump(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_I2S_DRIVER_H
