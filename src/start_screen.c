/*
 * FRANK Prince Start Screen
 * Adapted from murmdoom start screen implementation
 * Shows system info and errors before game launch
 */

#include "start_screen.h"
#include "board_config.h"
#include "HDMI.h"
#include "pop_fs.h"
#include "ps2kbd/ps2kbd_wrapper.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <string.h>

#include "ff.h"  // Direct FatFS access

#ifdef USB_HID_ENABLED
#include "usbhid/usbhid_sdl_wrapper.h"
#endif

// Screen dimensions (must match graphics buffer in main.c)
#define SCREEN_W 320
#define SCREEN_H 240

// External graphics buffer from main.c
extern uint8_t *graphics_buffer;

// Local back buffer to avoid flicker (draw here, then copy to graphics_buffer)
static uint8_t back_buffer[SCREEN_W * SCREEN_H];

// Version from build system
#ifndef FRANK_PRINCE_VERSION
#define FRANK_PRINCE_VERSION "?"
#endif

// ============================================================================
// Drawing Primitives (copied from murmdoom doomgeneric_rp2350.c)
// ============================================================================

static void fill_rect(int x, int y, int w, int h, uint8_t color) {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    if (w <= 0 || h <= 0) return;

    for (int yy = y; yy < y + h; ++yy) {
        memset(&back_buffer[yy * SCREEN_W + x], color, (size_t)w);
    }
}

// ============================================================================
// 5x7 Font Glyphs (copied from murmdoom doomgeneric_rp2350.c)
// ============================================================================

static const uint8_t *glyph_5x7(char ch) {
    static const uint8_t glyph_space[7] = {0, 0, 0, 0, 0, 0, 0};
    static const uint8_t glyph_dot[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C};
    static const uint8_t glyph_comma[7] = {0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x08};
    static const uint8_t glyph_colon[7] = {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00};
    static const uint8_t glyph_hyphen[7] = {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
    static const uint8_t glyph_lparen[7] = {0x04, 0x08, 0x08, 0x08, 0x08, 0x08, 0x04};
    static const uint8_t glyph_rparen[7] = {0x04, 0x02, 0x02, 0x02, 0x02, 0x02, 0x04};
    static const uint8_t glyph_slash[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x00, 0x00};

    static const uint8_t glyph_0[7] = {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    static const uint8_t glyph_1[7] = {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
    static const uint8_t glyph_2[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    static const uint8_t glyph_3[7] = {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_4[7] = {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    static const uint8_t glyph_5[7] = {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_6[7] = {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_7[7] = {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    static const uint8_t glyph_8[7] = {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_9[7] = {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E};

    static const uint8_t glyph_a[7] = {0x00, 0x00, 0x0E, 0x01, 0x0F, 0x11, 0x0F};
    static const uint8_t glyph_b[7] = {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x1E};
    static const uint8_t glyph_c[7] = {0x00, 0x00, 0x0E, 0x11, 0x10, 0x11, 0x0E};
    static const uint8_t glyph_d[7] = {0x01, 0x01, 0x0D, 0x13, 0x11, 0x13, 0x0D};
    static const uint8_t glyph_e[7] = {0x00, 0x00, 0x0E, 0x11, 0x1F, 0x10, 0x0F};
    static const uint8_t glyph_f[7] = {0x06, 0x08, 0x1E, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t glyph_g[7] = {0x00, 0x00, 0x0F, 0x11, 0x0F, 0x01, 0x0E};
    static const uint8_t glyph_h[7] = {0x10, 0x10, 0x1E, 0x11, 0x11, 0x11, 0x11};
    static const uint8_t glyph_i[7] = {0x04, 0x00, 0x0C, 0x04, 0x04, 0x04, 0x0E};
    static const uint8_t glyph_j[7] = {0x02, 0x00, 0x06, 0x02, 0x02, 0x12, 0x0C};
    static const uint8_t glyph_k[7] = {0x10, 0x10, 0x11, 0x12, 0x1C, 0x12, 0x11};
    static const uint8_t glyph_l[7] = {0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x06};
    static const uint8_t glyph_m[7] = {0x00, 0x00, 0x1A, 0x15, 0x15, 0x15, 0x15};
    static const uint8_t glyph_n[7] = {0x00, 0x00, 0x1E, 0x11, 0x11, 0x11, 0x11};
    static const uint8_t glyph_o[7] = {0x00, 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_p[7] = {0x00, 0x00, 0x1E, 0x11, 0x1E, 0x10, 0x10};
    static const uint8_t glyph_q[7] = {0x00, 0x00, 0x0D, 0x13, 0x13, 0x0D, 0x01};
    static const uint8_t glyph_r[7] = {0x00, 0x00, 0x16, 0x19, 0x10, 0x10, 0x10};
    static const uint8_t glyph_s[7] = {0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E};
    static const uint8_t glyph_t[7] = {0x04, 0x04, 0x1F, 0x04, 0x04, 0x04, 0x03};
    static const uint8_t glyph_u[7] = {0x00, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D};
    static const uint8_t glyph_v[7] = {0x00, 0x00, 0x11, 0x11, 0x11, 0x0A, 0x04};
    static const uint8_t glyph_w[7] = {0x00, 0x00, 0x11, 0x11, 0x15, 0x15, 0x0A};
    static const uint8_t glyph_x[7] = {0x00, 0x00, 0x11, 0x0A, 0x04, 0x0A, 0x11};
    static const uint8_t glyph_y[7] = {0x00, 0x00, 0x11, 0x11, 0x0F, 0x01, 0x0E};
    static const uint8_t glyph_z[7] = {0x00, 0x00, 0x1F, 0x02, 0x04, 0x08, 0x1F};

    static const uint8_t glyph_A[7] = {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t glyph_B[7] = {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
    static const uint8_t glyph_C[7] = {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    static const uint8_t glyph_D[7] = {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
    static const uint8_t glyph_E[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    static const uint8_t glyph_F[7] = {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t glyph_G[7] = {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_H[7] = {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    static const uint8_t glyph_I[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F};
    static const uint8_t glyph_J[7] = {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C};
    static const uint8_t glyph_K[7] = {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    static const uint8_t glyph_L[7] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    static const uint8_t glyph_M[7] = {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    static const uint8_t glyph_N[7] = {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    static const uint8_t glyph_O[7] = {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_P[7] = {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    static const uint8_t glyph_Q[7] = {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
    static const uint8_t glyph_R[7] = {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    static const uint8_t glyph_S[7] = {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    static const uint8_t glyph_T[7] = {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t glyph_U[7] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    static const uint8_t glyph_V[7] = {0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04};
    static const uint8_t glyph_W[7] = {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A};
    static const uint8_t glyph_X[7] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x0A, 0x11};
    static const uint8_t glyph_Y[7] = {0x11, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04};
    static const uint8_t glyph_Z[7] = {0x1F, 0x02, 0x04, 0x08, 0x10, 0x10, 0x1F};

    int c = (unsigned char)ch;
    switch (c) {
        case ' ': return glyph_space;
        case '.': return glyph_dot;
        case ',': return glyph_comma;
        case ':': return glyph_colon;
        case '-': return glyph_hyphen;
        case '(': return glyph_lparen;
        case ')': return glyph_rparen;
        case '/': return glyph_slash;

        case '0': return glyph_0;
        case '1': return glyph_1;
        case '2': return glyph_2;
        case '3': return glyph_3;
        case '4': return glyph_4;
        case '5': return glyph_5;
        case '6': return glyph_6;
        case '7': return glyph_7;
        case '8': return glyph_8;
        case '9': return glyph_9;

        case 'a': return glyph_a;
        case 'b': return glyph_b;
        case 'c': return glyph_c;
        case 'd': return glyph_d;
        case 'e': return glyph_e;
        case 'f': return glyph_f;
        case 'g': return glyph_g;
        case 'h': return glyph_h;
        case 'i': return glyph_i;
        case 'j': return glyph_j;
        case 'k': return glyph_k;
        case 'l': return glyph_l;
        case 'm': return glyph_m;
        case 'n': return glyph_n;
        case 'o': return glyph_o;
        case 'p': return glyph_p;
        case 'q': return glyph_q;
        case 'r': return glyph_r;
        case 's': return glyph_s;
        case 't': return glyph_t;
        case 'u': return glyph_u;
        case 'v': return glyph_v;
        case 'w': return glyph_w;
        case 'x': return glyph_x;
        case 'y': return glyph_y;
        case 'z': return glyph_z;

        case 'A': return glyph_A;
        case 'B': return glyph_B;
        case 'C': return glyph_C;
        case 'D': return glyph_D;
        case 'E': return glyph_E;
        case 'F': return glyph_F;
        case 'G': return glyph_G;
        case 'H': return glyph_H;
        case 'I': return glyph_I;
        case 'J': return glyph_J;
        case 'K': return glyph_K;
        case 'L': return glyph_L;
        case 'M': return glyph_M;
        case 'N': return glyph_N;
        case 'O': return glyph_O;
        case 'P': return glyph_P;
        case 'Q': return glyph_Q;
        case 'R': return glyph_R;
        case 'S': return glyph_S;
        case 'T': return glyph_T;
        case 'U': return glyph_U;
        case 'V': return glyph_V;
        case 'W': return glyph_W;
        case 'X': return glyph_X;
        case 'Y': return glyph_Y;
        case 'Z': return glyph_Z;

        default: return glyph_space;
    }
}

static void draw_char_5x7(int x, int y, char ch, uint8_t color) {
    const uint8_t *rows = glyph_5x7(ch);
    for (int row = 0; row < 7; ++row) {
        int yy = y + row;
        if (yy < 0 || yy >= SCREEN_H) continue;
        uint8_t bits = rows[row];
        for (int col = 0; col < 5; ++col) {
            int xx = x + col;
            if (xx < 0 || xx >= SCREEN_W) continue;
            if (bits & (1u << (4 - col))) {
                back_buffer[yy * SCREEN_W + xx] = color;
            }
        }
    }
}

static void draw_text_5x7(int x, int y, const char *text, uint8_t color) {
    const int advance = 6;
    for (const char *p = text; *p; ++p) {
        draw_char_5x7(x, y, *p, color);
        x += advance;
    }
}

static int text_width_5x7(const char *text) {
    int n = 0;
    for (const char *p = text; *p; ++p) n++;
    return n * 6;
}

// ============================================================================
// Animated Background (from murmdoom)
// ============================================================================

static void draw_animated_background_border(uint32_t t_ms,
                                           int panel_x,
                                           int panel_y,
                                           int panel_w,
                                           int panel_h) {
    const int t = (int)(t_ms / 80);

    if (panel_x < 0) panel_x = 0;
    if (panel_y < 0) panel_y = 0;
    if (panel_w < 0) panel_w = 0;
    if (panel_h < 0) panel_h = 0;
    if (panel_x + panel_w > SCREEN_W) panel_w = SCREEN_W - panel_x;
    if (panel_y + panel_h > SCREEN_H) panel_h = SCREEN_H - panel_y;

    const int panel_x2 = panel_x + panel_w;
    const int panel_y2 = panel_y + panel_h;

    // Top strip.
    for (int y = 0; y < panel_y; ++y) {
        for (int x = 0; x < SCREEN_W; ++x) {
            const int bx = (x >> 3);
            const int by = (y >> 3);
            uint8_t v = (uint8_t)((bx + by + t) & 0x0F);
            v ^= (uint8_t)(((bx << 1) ^ (by + (t >> 1))) & 0x07);
            back_buffer[y * SCREEN_W + x] = (uint8_t)(2 + (v & 0x0F));
        }
    }

    // Bottom strip.
    for (int y = panel_y2; y < SCREEN_H; ++y) {
        for (int x = 0; x < SCREEN_W; ++x) {
            const int bx = (x >> 3);
            const int by = (y >> 3);
            uint8_t v = (uint8_t)((bx + by + t) & 0x0F);
            v ^= (uint8_t)(((bx << 1) ^ (by + (t >> 1))) & 0x07);
            back_buffer[y * SCREEN_W + x] = (uint8_t)(2 + (v & 0x0F));
        }
    }

    // Left/right strips.
    for (int y = panel_y; y < panel_y2; ++y) {
        for (int x = 0; x < panel_x; ++x) {
            const int bx = (x >> 3);
            const int by = (y >> 3);
            uint8_t v = (uint8_t)((bx + by + t) & 0x0F);
            v ^= (uint8_t)(((bx << 1) ^ (by + (t >> 1))) & 0x07);
            back_buffer[y * SCREEN_W + x] = (uint8_t)(2 + (v & 0x0F));
        }
        for (int x = panel_x2; x < SCREEN_W; ++x) {
            const int bx = (x >> 3);
            const int by = (y >> 3);
            uint8_t v = (uint8_t)((bx + by + t) & 0x0F);
            v ^= (uint8_t)(((bx << 1) ^ (by + (t >> 1))) & 0x07);
            back_buffer[y * SCREEN_W + x] = (uint8_t)(2 + (v & 0x0F));
        }
    }
}

// ============================================================================
// Poll for Any Keypress
// ============================================================================

static bool poll_any_key(void) {
    // Poll PS/2 keyboard
    ps2kbd_tick();
    
    // Check if any key was pressed
    int scancode;
    int pressed;
    int modifier;
    if (ps2kbd_get_key(&pressed, &scancode, &modifier)) {
        if (pressed) {
            return true;
        }
    }
    
#ifdef USB_HID_ENABLED
    // Poll USB HID keyboard
    usbhid_sdl_tick();
    int usb_pressed, usb_scancode, usb_modifier;
    if (usbhid_sdl_get_key(&usb_pressed, &usb_scancode, &usb_modifier)) {
        if (usb_pressed) {
            return true;
        }
    }
#endif
    
    return false;
}

// ============================================================================
// Public Functions
// ============================================================================

start_error_t start_screen_check_requirements(void) {
    // Small delay to let SD card settle after power-up
    sleep_ms(100);
    
    // Try to init filesystem
    DBG_PRINTF("[start_screen] Initializing filesystem...\n");
    if (!pop_fs_init()) {
        DBG_PRINTF("[start_screen] pop_fs_init() FAILED\n");
        return START_ERROR_NO_SD;
    }
    DBG_PRINTF("[start_screen] Filesystem mounted OK\n");
    
    // Debug: list root directory contents
    DBG_PRINTF("[start_screen] Listing root directory:\n");
    DIR dir;
    FILINFO fno;
    FRESULT fr = f_opendir(&dir, "/");
    if (fr != FR_OK) {
        DBG_PRINTF("[start_screen] f_opendir('/') failed: %d\n", (int)fr);
    } else {
        int count = 0;
        for (;;) {
            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK || fno.fname[0] == 0) break;
            DBG_PRINTF("  %c %s\n", (fno.fattrib & AM_DIR) ? 'D' : 'F', fno.fname);
            count++;
        }
        f_closedir(&dir);
        DBG_PRINTF("[start_screen] Found %d entries\n", count);
    }
    
    // Debug: list prince directory contents
    DBG_PRINTF("[start_screen] Listing prince directory:\n");
    fr = f_opendir(&dir, "prince");
    if (fr != FR_OK) {
        DBG_PRINTF("[start_screen] f_opendir('prince') failed: %d\n", (int)fr);
    } else {
        int count = 0;
        for (;;) {
            fr = f_readdir(&dir, &fno);
            if (fr != FR_OK || fno.fname[0] == 0) break;
            DBG_PRINTF("  %c %s\n", (fno.fattrib & AM_DIR) ? 'D' : 'F', fno.fname);
            count++;
        }
        f_closedir(&dir);
        DBG_PRINTF("[start_screen] Found %d entries in prince/\n", count);
    }
    
    // Try to find game data files
    // Check for PRINCE.DAT first, but also check for GUARD.DAT (always present)
    // since some setups use unpacked data (PRINCE directory instead of PRINCE.DAT)
    DBG_PRINTF("[start_screen] Checking for game data files...\n");
    
    // Check in root first
    fr = f_stat("PRINCE.DAT", &fno);
    DBG_PRINTF("[start_screen] f_stat('PRINCE.DAT') = %d\n", (int)fr);
    if (fr == FR_OK) {
        DBG_PRINTF("[start_screen] Found PRINCE.DAT in root, size: %lu\n", (unsigned long)fno.fsize);
        return START_OK;
    }
    
    // Check in prince/ directory
    fr = f_stat("prince/PRINCE.DAT", &fno);
    DBG_PRINTF("[start_screen] f_stat('prince/PRINCE.DAT') = %d\n", (int)fr);
    if (fr == FR_OK) {
        DBG_PRINTF("[start_screen] Found prince/PRINCE.DAT, size: %lu\n", (unsigned long)fno.fsize);
        return START_OK;
    }
    
    // Check for unpacked format (PRINCE directory or other DAT files)
    fr = f_stat("prince/GUARD.DAT", &fno);
    DBG_PRINTF("[start_screen] f_stat('prince/GUARD.DAT') = %d\n", (int)fr);
    if (fr == FR_OK) {
        DBG_PRINTF("[start_screen] Found prince/GUARD.DAT (unpacked format), size: %lu\n", (unsigned long)fno.fsize);
        return START_OK;
    }
    
    // Check for PRINCE directory (unpacked format)
    fr = f_stat("prince/PRINCE", &fno);
    DBG_PRINTF("[start_screen] f_stat('prince/PRINCE') = %d\n", (int)fr);
    if (fr == FR_OK && (fno.fattrib & AM_DIR)) {
        DBG_PRINTF("[start_screen] Found prince/PRINCE directory (unpacked format)\n");
        return START_OK;
    }
    
    DBG_PRINTF("[start_screen] No game data found, returning error\n");
    return START_ERROR_NO_DATA_DIR;
}

void start_screen_show(start_error_t error, const char* error_msg) {
    // Setup palette (same as murmdoom)
    graphics_set_palette(0, 0x000000);  // Black background
    graphics_set_palette(1, 0xFFFFFF);  // White text

    // Background palette (2..17): blue/cyan Prince of Persia style
    static const uint32_t bg_pal[16] = {
        0x000105, 0x000208, 0x00030B, 0x00040E,
        0x010512, 0x020616, 0x02071A, 0x03081E,
        0x030922, 0x040A26, 0x040B2A, 0x050C2E,
        0x050D33, 0x060E38, 0x060F3D, 0x071042,
    };
    for (int i = 0; i < 16; ++i) {
        graphics_set_palette(2 + i, bg_pal[i]);
    }

    // Title highlight: 50% brighter blue/cyan for better readability
    uint32_t title_hl_rgb;
    {
        const uint32_t base = bg_pal[15];
        uint32_t r = (base >> 16) & 0xFF;
        uint32_t g = (base >> 8) & 0xFF;
        uint32_t b = base & 0xFF;
        r = (r * 3) / 2;
        g = (g * 3) / 2;
        b = (b * 3) / 2;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;
        title_hl_rgb = (r << 16) | (g << 8) | b;
        graphics_set_palette(18, title_hl_rgb);
    }
    
    // Error color (red)
    graphics_set_palette(19, 0xFF4444);
    
    // Almost white for FRANK Prince text on blue background
    graphics_set_palette(21, 0xDDDDFF);
    
    graphics_restore_sync_colors();
    
    memset(back_buffer, 0, SCREEN_W * SCREEN_H);

    // Panel dimensions (same as murmdoom)
    const int panel_x = 24;
    const int panel_y = 44;  // Offset for 240 height (was 24 for 200)
    const int panel_w = SCREEN_W - 48;
    const int panel_h = SCREEN_H - 88;  // Keep same panel height

    // Build strings
    const char *title_left = "FRANK Prince";
    char title_right[96];
    snprintf(title_right, sizeof(title_right), " by Mikhail Matveev v%s", FRANK_PRINCE_VERSION);

    const int title_left_w = text_width_5x7(title_left);
    const int title_right_w = text_width_5x7(title_right);
    const int title_w = title_left_w + title_right_w;
    const int title_x = (SCREEN_W - title_w) / 2;
    const int title_y = panel_y + 10;

    // Status strings
#ifndef DBOARD_VARIANT
#if defined(BOARD_M2)
#define DBOARD_VARIANT "M2"
#else
#define DBOARD_VARIANT "M1"
#endif
#endif
#ifndef DPSRAM_SPEED
#define DPSRAM_SPEED PSRAM_MAX_FREQ_MHZ
#endif
#ifndef DCPU_SPEED
#define DCPU_SPEED CPU_CLOCK_MHZ
#endif

    const char *cfg = DBOARD_VARIANT;
    const uint32_t cpu_mhz = (uint32_t)DCPU_SPEED;
    const uint32_t psram_cs = get_psram_pin();
    
    char status1[96];
    char status2[96];
    snprintf(status1, sizeof(status1), "%s, FREQ: %lu MHz, PSRAM: %d MHz, CS: %lu",
             cfg,
             (unsigned long)cpu_mhz,
             (int)DPSRAM_SPEED,
             (unsigned long)psram_cs);
    
    const char *status3 = "https://github.com/rh1tech/frank-prince";
    
    // Error messages
    const char *err_line = NULL;
    if (error != START_OK) {
        if (error_msg) {
            err_line = error_msg;
        } else {
            switch (error) {
                case START_ERROR_NO_SD:
                    err_line = "ERROR: SD card not found!";
                    break;
                case START_ERROR_NO_DATA_DIR:
                    err_line = "ERROR: prince/PRINCE.DAT not found!";
                    break;
                default:
                    err_line = "ERROR: Unknown error!";
                    break;
            }
        }
        snprintf(status2, sizeof(status2), "Insert SD card and reset.");
    } else {
        snprintf(status2, sizeof(status2), "Press any key to start...");
    }

    // Main loop
    bool waiting = true;
    while (waiting) {
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        
        // Draw animated background border
        draw_animated_background_border(now_ms, panel_x, panel_y, panel_w, panel_h);
        
        // Draw black panel
        fill_rect(panel_x, panel_y, panel_w, panel_h, 0);
        
        // Highlight "FRANK Prince" with background and light text
        fill_rect(title_x - 2, title_y - 2, title_left_w + 4, 7 + 4, 18);
        draw_text_5x7(title_x, title_y, title_left, 21);
        draw_text_5x7(title_x + title_left_w, title_y, title_right, 1);

        // Game name
        const char *game_name = "PRINCE OF PERSIA";
        int game_name_w = text_width_5x7(game_name);
        draw_text_5x7((SCREEN_W - game_name_w) / 2, panel_y + 30, game_name, 1);

        // Error or success message
        if (err_line) {
            int err_w = text_width_5x7(err_line);
            draw_text_5x7((SCREEN_W - err_w) / 2, panel_y + 60, err_line, 19);
        } else {
            const char *ok_msg = "SD card OK";
            int ok_w = text_width_5x7(ok_msg);
            draw_text_5x7((SCREEN_W - ok_w) / 2, panel_y + 60, ok_msg, 1);
        }

        // Status lines at bottom (centered)
        const int bottom_y0 = panel_y + panel_h - 32;
        int status1_w = text_width_5x7(status1);
        draw_text_5x7((SCREEN_W - status1_w) / 2, bottom_y0 + 0, status1, 1);
        
        // Blink "Press any key" if no error
        int status2_w = text_width_5x7(status2);
        if (error == START_OK) {
            if ((now_ms / 500) % 2 == 0) {
                draw_text_5x7((SCREEN_W - status2_w) / 2, bottom_y0 + 10, status2, 1);
            }
        } else {
            draw_text_5x7((SCREEN_W - status2_w) / 2, bottom_y0 + 10, status2, 1);
        }
        
        int status3_w = text_width_5x7(status3);
        draw_text_5x7((SCREEN_W - status3_w) / 2, bottom_y0 + 20, status3, 1);

        // Copy back buffer to graphics buffer atomically to prevent flicker
        memcpy(graphics_buffer, back_buffer, SCREEN_W * SCREEN_H);

        sleep_ms(33);  // ~30 FPS
        
        // Check for keypress (allow start even with error to diagnose)
        if (poll_any_key()) {
            waiting = false;
        }
    }
    
    // Clear screen before starting game
    memset(graphics_buffer, 0, SCREEN_W * SCREEN_H);
}
