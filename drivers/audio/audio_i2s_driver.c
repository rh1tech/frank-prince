/*
 * frank-prince - I2S Audio Driver Implementation
 * Standalone PIO + DMA ping-pong audio output (no pico-extras dependency)
 *
 * Based on the audio driver from murmheretic by Mikhail Matveev.
 *
 * Architecture:
 *   The SDL audio callback must run on the main loop (not from IRQ) because
 *   SDLPoP's mixer reads game state without locking.  A ring of staging
 *   buffers decouples the main-loop callback from the DMA timeline:
 *
 *     pump() [main loop]  -->  staging ring  -->  DMA IRQ copies to HW
 *
 *   The IRQ handler is fast (memcpy only) and never touches game state.
 *   If the ring runs dry (main loop too slow), the IRQ outputs silence
 *   and re-arms the DMA channel so the ping-pong never stalls.
 *
 * PIO/SM Allocation:
 *   - HDMI: pio1 SM0, SM1 (video output)
 *   - PS/2 Keyboard: pio0 SM0 (input)
 *   - I2S Audio: pio0 SM2, DMA channels 10+11
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "audio_i2s_driver.h"
#include "board_config.h"
#include "audio_i2s.pio.h"

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// Configuration
// ============================================================================

#ifndef AUDIO_I2S_PIO
#define AUDIO_I2S_PIO 0
#endif

// DMA channels for ping-pong
#define AUDIO_DMA_CH_A    10
#define AUDIO_DMA_CH_B    11

// Use DMA_IRQ_0 for audio (HDMI uses DMA_IRQ_1)
#ifndef AUDIO_I2S_DMA_IRQ
#define AUDIO_I2S_DMA_IRQ 0
#endif

#if AUDIO_I2S_DMA_IRQ == 0
#define AUDIO_DMA_IRQ_NUM DMA_IRQ_0
#else
#define AUDIO_DMA_IRQ_NUM DMA_IRQ_1
#endif

// DMA buffer: stereo frames (one uint32_t = L16 + R16)
#define DMA_BUFFER_COUNT      2
#define DMA_BUFFER_MAX_FRAMES AUDIO_BUFFER_SAMPLES

// Staging ring: filled by main-loop pump(), consumed by DMA IRQ.
// 4 slots gives ~92 ms of slack (4 * 1024 frames / 44100 Hz).
#define STAGING_COUNT 4

// Master volume: right-shift applied to every sample when copying to DMA.
// 0 = full volume, 1 = -6 dB, 2 = -12 dB, etc.
#define AUDIO_MASTER_ATTEN 1

// ============================================================================
// State
// ============================================================================

// DMA ping-pong buffers (written by IRQ only)
static uint32_t __attribute__((aligned(4)))
    dma_buffers[DMA_BUFFER_COUNT][DMA_BUFFER_MAX_FRAMES];

// Staging ring (written by main loop, read by IRQ)
static uint32_t __attribute__((aligned(4)))
    staging[STAGING_COUNT][DMA_BUFFER_MAX_FRAMES];
static volatile uint8_t staging_wr = 0;  // next slot main loop writes
static volatile uint8_t staging_rd = 0;  // next slot IRQ reads

static int  dma_channel_a = -1;
static int  dma_channel_b = -1;
static PIO  audio_pio;
static uint audio_sm;
static uint pio_program_offset;
static uint32_t dma_transfer_count;

static volatile bool audio_running = false;

static struct {
    bool initialized;
    volatile bool enabled;
    uint32_t sample_rate;
    uint8_t channels;
    audio_callback_fn callback;
    void *userdata;
    critical_section_t lock;
} audio_state = {0};

static void audio_dma_irq_handler(void);

// ============================================================================
// Staging ring helpers
// ============================================================================

static inline uint8_t staging_avail(void) {
    return (uint8_t)(staging_wr - staging_rd);  // works with wrapping uint8_t
}

static inline bool staging_full(void) {
    return staging_avail() >= STAGING_COUNT;
}

// ============================================================================
// Internal: PIO + DMA setup
// ============================================================================

static bool audio_hw_init(uint32_t sample_rate) {
    audio_pio = (AUDIO_I2S_PIO == 0) ? pio0 : pio1;
    dma_transfer_count = DMA_BUFFER_MAX_FRAMES;

    // Configure GPIO pins for PIO
    uint pio_func = (audio_pio == pio0) ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1;
    gpio_set_function(I2S_DATA_PIN, pio_func);
    gpio_set_function(I2S_CLOCK_PIN_BASE, pio_func);
    gpio_set_function(I2S_CLOCK_PIN_BASE + 1, pio_func);

    gpio_set_drive_strength(I2S_DATA_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2S_CLOCK_PIN_BASE, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2S_CLOCK_PIN_BASE + 1, GPIO_DRIVE_STRENGTH_12MA);

    // Claim state machine
    audio_sm = pio_claim_unused_sm(audio_pio, true);

    // Load PIO program
    pio_program_offset = pio_add_program(audio_pio, &audio_i2s_program);
    audio_i2s_program_init(audio_pio, audio_sm, pio_program_offset,
                           I2S_DATA_PIN, I2S_CLOCK_PIN_BASE);

    pio_sm_clear_fifos(audio_pio, audio_sm);

    // Clock divider: PIO outputs 1 bit per 2 cycles, 32 bits/frame = 64 cycles
    // divider (8.8 fixed) = sys_clk * 4 / sample_rate
    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t divider = sys_clk * 4 / sample_rate;
    DBG_PRINTF("I2S clock: sys=%lu Hz, div=%lu.%lu, target=%lu Hz\n",
               (unsigned long)sys_clk,
               (unsigned long)(divider >> 8), (unsigned long)(divider & 0xff),
               (unsigned long)sample_rate);
    pio_sm_set_clkdiv_int_frac(audio_pio, audio_sm,
                                divider >> 8u, divider & 0xffu);

    // Silence all buffers
    memset(dma_buffers, 0, sizeof(dma_buffers));
    memset(staging, 0, sizeof(staging));
    staging_wr = 0;
    staging_rd = 0;

    // Claim DMA channels
    dma_channel_abort(AUDIO_DMA_CH_A);
    dma_channel_abort(AUDIO_DMA_CH_B);
    while (dma_channel_is_busy(AUDIO_DMA_CH_A) ||
           dma_channel_is_busy(AUDIO_DMA_CH_B)) {
        tight_loop_contents();
    }
    dma_channel_unclaim(AUDIO_DMA_CH_A);
    dma_channel_unclaim(AUDIO_DMA_CH_B);
    dma_channel_claim(AUDIO_DMA_CH_A);
    dma_channel_claim(AUDIO_DMA_CH_B);
    dma_channel_a = AUDIO_DMA_CH_A;
    dma_channel_b = AUDIO_DMA_CH_B;

    // Configure DMA channels in ping-pong chain
    dma_channel_config cfg_a = dma_channel_get_default_config(dma_channel_a);
    channel_config_set_read_increment(&cfg_a, true);
    channel_config_set_write_increment(&cfg_a, false);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
    channel_config_set_dreq(&cfg_a, pio_get_dreq(audio_pio, audio_sm, true));
    channel_config_set_chain_to(&cfg_a, dma_channel_b);

    dma_channel_config cfg_b = dma_channel_get_default_config(dma_channel_b);
    channel_config_set_read_increment(&cfg_b, true);
    channel_config_set_write_increment(&cfg_b, false);
    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
    channel_config_set_dreq(&cfg_b, pio_get_dreq(audio_pio, audio_sm, true));
    channel_config_set_chain_to(&cfg_b, dma_channel_a);

    dma_channel_configure(dma_channel_a, &cfg_a,
                          &audio_pio->txf[audio_sm],
                          dma_buffers[0], dma_transfer_count, false);

    dma_channel_configure(dma_channel_b, &cfg_b,
                          &audio_pio->txf[audio_sm],
                          dma_buffers[1], dma_transfer_count, false);

    // Set up DMA IRQ handler
#if AUDIO_I2S_DMA_IRQ == 0
    dma_hw->ints0 = (1u << dma_channel_a) | (1u << dma_channel_b);
    irq_set_exclusive_handler(AUDIO_DMA_IRQ_NUM, audio_dma_irq_handler);
    irq_set_priority(AUDIO_DMA_IRQ_NUM, 0x80);
    irq_set_enabled(AUDIO_DMA_IRQ_NUM, true);
    dma_channel_set_irq0_enabled(dma_channel_a, true);
    dma_channel_set_irq0_enabled(dma_channel_b, true);
#else
    dma_hw->ints1 = (1u << dma_channel_a) | (1u << dma_channel_b);
    irq_set_exclusive_handler(AUDIO_DMA_IRQ_NUM, audio_dma_irq_handler);
    irq_set_priority(AUDIO_DMA_IRQ_NUM, 0x80);
    irq_set_enabled(AUDIO_DMA_IRQ_NUM, true);
    dma_channel_set_irq1_enabled(dma_channel_a, true);
    dma_channel_set_irq1_enabled(dma_channel_b, true);
#endif

    // Enable PIO state machine
    pio_sm_set_enabled(audio_pio, audio_sm, true);

    audio_running = false;

    return true;
}

static void audio_hw_deinit(void) {
    audio_running = false;

    pio_sm_set_enabled(audio_pio, audio_sm, false);

    irq_set_enabled(AUDIO_DMA_IRQ_NUM, false);

    if (dma_channel_a >= 0) {
#if AUDIO_I2S_DMA_IRQ == 0
        dma_channel_set_irq0_enabled(dma_channel_a, false);
        dma_channel_abort(dma_channel_a);
        dma_hw->ints0 = (1u << dma_channel_a);
#else
        dma_channel_set_irq1_enabled(dma_channel_a, false);
        dma_channel_abort(dma_channel_a);
        dma_hw->ints1 = (1u << dma_channel_a);
#endif
        dma_channel_unclaim(dma_channel_a);
        dma_channel_a = -1;
    }

    if (dma_channel_b >= 0) {
#if AUDIO_I2S_DMA_IRQ == 0
        dma_channel_set_irq0_enabled(dma_channel_b, false);
        dma_channel_abort(dma_channel_b);
        dma_hw->ints0 = (1u << dma_channel_b);
#else
        dma_channel_set_irq1_enabled(dma_channel_b, false);
        dma_channel_abort(dma_channel_b);
        dma_hw->ints1 = (1u << dma_channel_b);
#endif
        dma_channel_unclaim(dma_channel_b);
        dma_channel_b = -1;
    }

    pio_sm_unclaim(audio_pio, audio_sm);
    pio_remove_program(audio_pio, &audio_i2s_program, pio_program_offset);
}

// ============================================================================
// DMA IRQ handler - copies next staging buffer to DMA, re-arms channel
//
// Never calls the SDL callback.  Copies from the staging ring (or silence
// if empty), applies master attenuation, and re-arms for next ping-pong.
// ============================================================================

// Copy staging slot to DMA buffer with master attenuation
static inline void copy_with_atten(uint32_t *dst, const uint32_t *src, uint32_t frames) {
#if AUDIO_MASTER_ATTEN == 0
    memcpy(dst, src, frames * sizeof(uint32_t));
#else
    const int16_t *s = (const int16_t *)src;
    int16_t *d = (int16_t *)dst;
    uint32_t count = frames * 2; // L + R per frame
    for (uint32_t i = 0; i < count; i++) {
        d[i] = s[i] >> AUDIO_MASTER_ATTEN;
    }
#endif
}

static void audio_dma_irq_handler(void) {
#if AUDIO_I2S_DMA_IRQ == 0
    uint32_t ints = dma_hw->ints0;
#else
    uint32_t ints = dma_hw->ints1;
#endif
    uint32_t mask = 0;
    if (dma_channel_a >= 0) mask |= (1u << dma_channel_a);
    if (dma_channel_b >= 0) mask |= (1u << dma_channel_b);
    ints &= mask;
    if (!ints) return;

    if ((dma_channel_a >= 0) && (ints & (1u << dma_channel_a))) {
#if AUDIO_I2S_DMA_IRQ == 0
        dma_hw->ints0 = (1u << dma_channel_a);
#else
        dma_hw->ints1 = (1u << dma_channel_a);
#endif
        if (staging_avail() > 0) {
            copy_with_atten(dma_buffers[0], staging[staging_rd % STAGING_COUNT], dma_transfer_count);
            staging_rd++;
        } else {
            memset(dma_buffers[0], 0, dma_transfer_count * sizeof(uint32_t));
        }
        __dmb();
        dma_channel_set_read_addr(dma_channel_a, dma_buffers[0], false);
        dma_channel_set_trans_count(dma_channel_a, dma_transfer_count, false);
    }

    if ((dma_channel_b >= 0) && (ints & (1u << dma_channel_b))) {
#if AUDIO_I2S_DMA_IRQ == 0
        dma_hw->ints0 = (1u << dma_channel_b);
#else
        dma_hw->ints1 = (1u << dma_channel_b);
#endif
        if (staging_avail() > 0) {
            copy_with_atten(dma_buffers[1], staging[staging_rd % STAGING_COUNT], dma_transfer_count);
            staging_rd++;
        } else {
            memset(dma_buffers[1], 0, dma_transfer_count * sizeof(uint32_t));
        }
        __dmb();
        dma_channel_set_read_addr(dma_channel_b, dma_buffers[1], false);
        dma_channel_set_trans_count(dma_channel_b, dma_transfer_count, false);
    }
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool audio_i2s_driver_init(uint32_t sample_rate, uint8_t channels,
                           audio_callback_fn callback, void *userdata) {
    if (audio_state.initialized) {
        DBG_PRINTF("audio_i2s_driver: already initialized\n");
        return false;
    }

    DBG_PRINTF("audio_i2s_driver: initializing (rate=%lu, ch=%d)\n",
           sample_rate, channels);

    audio_state.sample_rate = sample_rate;
    audio_state.channels = channels;
    audio_state.callback = callback;
    audio_state.userdata = userdata;

    critical_section_init(&audio_state.lock);

    if (!audio_hw_init(sample_rate)) {
        DBG_PRINTF("audio_i2s_driver: hardware init failed\n");
        return false;
    }

    audio_state.initialized = true;
    audio_state.enabled = false; // Paused until SDL_PauseAudio(0)

    DBG_PRINTF("audio_i2s_driver: initialization complete (PIO%d SM%d, DMA %d/%d)\n",
           AUDIO_I2S_PIO, audio_sm, AUDIO_DMA_CH_A, AUDIO_DMA_CH_B);
    return true;
}

void audio_i2s_driver_set_enabled(bool enable) {
    if (!audio_state.initialized) return;

    if (enable && !audio_state.enabled) {
        DBG_PRINTF("audio_i2s_driver: unpausing audio\n");
        audio_state.enabled = true;
    } else if (!enable && audio_state.enabled) {
        DBG_PRINTF("audio_i2s_driver: pausing audio\n");
        audio_state.enabled = false;
    }
}

bool audio_i2s_driver_is_enabled(void) {
    return audio_state.enabled;
}

void audio_i2s_driver_shutdown(void) {
    if (!audio_state.initialized) return;

    DBG_PRINTF("audio_i2s_driver: shutting down\n");
    audio_state.enabled = false;

    audio_hw_deinit();

    audio_state.initialized = false;
}

uint8_t audio_i2s_driver_get_silence(void) {
    return 0; // Signed 16-bit format uses 0 for silence
}

void audio_i2s_driver_lock(void) {
    if (audio_state.initialized) {
        critical_section_enter_blocking(&audio_state.lock);
    }
}

void audio_i2s_driver_unlock(void) {
    if (audio_state.initialized) {
        critical_section_exit(&audio_state.lock);
    }
}

// Pump: fill staging buffers from the SDL callback on the main loop,
// then start DMA on the first call once enough data is queued.
void audio_i2s_driver_pump(void) {
    if (!audio_state.initialized) return;

    int buffer_bytes = (int)(dma_transfer_count * sizeof(uint32_t));

    // Fill as many staging slots as are free
    while (!staging_full()) {
        uint32_t *slot = staging[staging_wr % STAGING_COUNT];

        if (audio_state.enabled && audio_state.callback) {
            audio_state.callback(audio_state.userdata, (uint8_t *)slot, buffer_bytes);
        } else {
            memset(slot, 0, buffer_bytes);
        }
        __dmb();
        staging_wr++;
    }

    // Start DMA once we have enough pre-roll data
    if (!audio_running && staging_avail() >= DMA_BUFFER_COUNT) {
        // Prime both DMA buffers from the staging ring
        memcpy(dma_buffers[0], staging[staging_rd % STAGING_COUNT],
               dma_transfer_count * sizeof(uint32_t));
        staging_rd++;
        memcpy(dma_buffers[1], staging[staging_rd % STAGING_COUNT],
               dma_transfer_count * sizeof(uint32_t));
        staging_rd++;
        __dmb();

        audio_running = true;
        dma_channel_start(dma_channel_a);
    }
}
