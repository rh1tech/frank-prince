/*
 * frank-prince - Prince of Persia (SDLPoP) RP2350 port
 * Entry point: init HDMI + PSRAM + SD, show start screen, then run SDLPoP.
 */

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/structs/qmi.h"
#ifndef USB_HID_ENABLED
#include "pico/stdio_usb.h"
#endif
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "HDMI.h"
#include "psram_init.h"
#include "psram_allocator.h"
#include "pop_fs.h"
#include "ps2kbd/ps2kbd_wrapper.h"
#include "start_screen.h"

// USB HID keyboard support (optional)
#ifdef USB_HID_ENABLED
#include "usbhid/usbhid_sdl_wrapper.h"
#endif

#define FRAME_W 320
#define FRAME_H 240

// Diagnostic: show a known-good 8bpp test pattern and optionally halt.
// This isolates HDMI scanout from SDLPoP rendering.
#ifndef RP2350_BOOT_TEST_PATTERN
#define RP2350_BOOT_TEST_PATTERN 0
#endif

#ifndef RP2350_BOOT_TEST_PATTERN_HALT
#define RP2350_BOOT_TEST_PATTERN_HALT 1
#endif

// 0: 16-color checker (uses basic palette)
// 1: 0..239 horizontal ramp + grayscale palette (exercises most indices)
#ifndef RP2350_BOOT_TEST_PATTERN_MODE
#define RP2350_BOOT_TEST_PATTERN_MODE 0
#endif

// HDMI scanout reads this buffer in a tight per-line ISR.
// Keep it in SRAM for reliable, fast reads (PSRAM can cause visible artifacts).
static uint8_t graphics_buffer_storage[FRAME_W * FRAME_H];

// SDLPoP SDL shim writes frames here.
uint8_t *graphics_buffer = graphics_buffer_storage;

// SDLPoP entrypoint (renamed from main via build defines)
extern int sdlpop_entry(int argc, char *argv[]);

// Flash timing configuration for overclocking
// Must be called BEFORE changing system clock
#define FLASH_MAX_FREQ_MHZ 88

static void __no_inline_not_in_flash_func(set_flash_timings)(int cpu_mhz) {
    const int clock_hz = cpu_mhz * 1000000;
    const int max_flash_freq = FLASH_MAX_FREQ_MHZ * 1000000;

    int divisor = (clock_hz + max_flash_freq - (max_flash_freq >> 4) - 1) / max_flash_freq;
    if (divisor == 1 && clock_hz >= 166000000) {
        divisor = 2;
    }

    int rxdelay = divisor;
    if (clock_hz / divisor > 100000000 && clock_hz >= 166000000) {
        rxdelay += 1;
    }

    qmi_hw->m[0].timing = 0x60007000 |
                          rxdelay << QMI_M0_TIMING_RXDELAY_LSB |
                          divisor << QMI_M0_TIMING_CLKDIV_LSB;
}

static void setup_basic_palette(void) {
    // Avoid 240-243 (HDMI control), set 0..15 and background 255.
    static const uint32_t pal16[16] = {
        0x000000, 0x0000AA, 0x00AA00, 0x00AAAA,
        0xAA0000, 0xAA00AA, 0xAA5500, 0xAAAAAA,
        0x555555, 0x5555FF, 0x55FF55, 0x55FFFF,
        0xFF5555, 0xFF55FF, 0xFFFF55, 0xFFFFFF,
    };

    for (int i = 0; i < 16; i++) {
        graphics_set_palette((uint8_t)i, pal16[i]);
    }

    graphics_set_palette(255, 0x000000);
    graphics_restore_sync_colors();
}

static void rp2350_setup_grayscale_palette_0_239(void) {
    for (int i = 0; i < 240; ++i) {
        const uint32_t v = (uint32_t)i;
        const uint32_t rgb = (v << 16) | (v << 8) | v;
        graphics_set_palette((uint8_t)i, rgb);
    }
    graphics_set_palette(255, 0x000000);
    graphics_restore_sync_colors();
}

static void rp2350_draw_boot_test_pattern(uint8_t *fb, int w, int h) {
    if (RP2350_BOOT_TEST_PATTERN_MODE == 1) {
        // Horizontal ramp across 0..239.
        for (int y = 0; y < h; ++y) {
            uint8_t *row = fb + (size_t)y * (size_t)w;
            for (int x = 0; x < w; ++x) {
                row[x] = (uint8_t)((x * 239) / (w - 1));
            }
        }
        return;
    }

    // Default: 16-color checker (indices 0..15) so it works with the basic palette.
    for (int y = 0; y < h; ++y) {
        uint8_t *row = fb + (size_t)y * (size_t)w;
        for (int x = 0; x < w; ++x) {
            const uint8_t v = (uint8_t)(((x >> 3) ^ (y >> 3)) & 0x0F);
            row[x] = v;
        }
    }
}

int main(void) {
#if CPU_CLOCK_MHZ > 252
    vreg_disable_voltage_limit();
    vreg_set_voltage(CPU_VOLTAGE);
    set_flash_timings(CPU_CLOCK_MHZ);
    sleep_ms(100);
#endif

    if (!set_sys_clock_khz(CPU_CLOCK_MHZ * 1000, false)) {
        set_sys_clock_khz(252 * 1000, true);
    }

    stdio_init_all();

    // USB CDC enumerates when stdio_usb is initialized. Delay *output* for 3 seconds.
    // This keeps the serial device available while avoiding early console chatter.
    sleep_ms(3000);

    DBG_PRINTF("frank-prince - RP2350 SDLPoP bootstrap\n");
    DBG_PRINTF("System Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
#if FRANK_PRINCE_DEBUG
    DBG_PRINTF("Build flags: RP2350_BOOT_TEST_PATTERN=%d RP2350_BOOT_TEST_PATTERN_HALT=%d\n",
        (int)RP2350_BOOT_TEST_PATTERN, (int)RP2350_BOOT_TEST_PATTERN_HALT);
    DBG_PRINTF("Build flags: RP2350_BOOT_TEST_PATTERN_MODE=%d\n", (int)RP2350_BOOT_TEST_PATTERN_MODE);
#endif

    // PSRAM init (CS1)
    uint psram_pin = get_psram_pin();
    psram_init(psram_pin);
    psram_set_sram_mode(0);

    memset(graphics_buffer, 0, FRAME_W * FRAME_H);

    // HDMI init
    graphics_init(g_out_HDMI);
    graphics_set_res(FRAME_W, FRAME_H);
    graphics_set_buffer(graphics_buffer);

    setup_basic_palette();

#if RP2350_BOOT_TEST_PATTERN
    if (RP2350_BOOT_TEST_PATTERN_MODE == 1) {
        DBG_PRINTF("BOOT TEST PATTERN: displaying 0..239 ramp + grayscale palette\n");
        rp2350_setup_grayscale_palette_0_239();
    } else {
        DBG_PRINTF("BOOT TEST PATTERN: displaying 16-color checker\n");
    }
    rp2350_draw_boot_test_pattern(graphics_buffer, FRAME_W, FRAME_H);
    if (RP2350_BOOT_TEST_PATTERN_HALT) {
        DBG_PRINTF("BOOT TEST PATTERN: halting (disable RP2350_BOOT_TEST_PATTERN_HALT to continue)\n");
        while (true) {
            tight_loop_contents();
        }
    }
    sleep_ms(2000);
#endif

    DBG_PRINTF("Initializing PS/2 keyboard...\n");
    ps2kbd_init();

#ifdef USB_HID_ENABLED
    DBG_PRINTF("Initializing USB HID keyboard...\n");
    usbhid_sdl_init();
#endif

    // Main loop: show start screen, run game, repeat on quit
    while (true) {
        // Check SD card and data directory
        DBG_PRINTF("Checking SD card and game data...\n");
        start_error_t err = start_screen_check_requirements();
        
        // Show start screen (waits for keypress if no error)
        DBG_PRINTF("Showing start screen...\n");
        start_screen_show(err, NULL);
        
        // If there was an error, the start screen loops forever
        // (user must fix and reset), so we only get here if OK
        
        // Reset palette for game
        setup_basic_palette();
        
        // Clear screen
        memset(graphics_buffer, 0, FRAME_W * FRAME_H);
        
        DBG_PRINTF("Starting SDLPoP...\n");
        char *argv[] = {"prince", NULL};
        int rc = sdlpop_entry(1, argv);
        
        DBG_PRINTF("SDLPoP exited (rc=%d). Returning to start screen...\n", rc);
        
        // Brief delay before showing start screen again
        sleep_ms(500);
    }
    
    // Never reached
    return 0;
}
