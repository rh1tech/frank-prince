#include "SDL_port.h"
#include "rp_sdl_config.h"
#include "board_config.h"
#include "HDMI.h"
#include "pico/stdlib.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "psram_allocator.h"
#include "pop_fs.h"
#include "ps2kbd/ps2kbd_wrapper.h"

// USB HID keyboard support (optional)
#ifdef USB_HID_ENABLED
#include "usbhid/usbhid_sdl_wrapper.h"
#endif

#include "third_party/stb/stb_image.h"

// Audio support
#if RP_SDL_FEATURE_AUDIO
#include "audio/audio_i2s_driver.h"
#endif

static const char *g_img_error = "SDL_image not implemented";
static const char *g_sdl_error = "";

// --- Debug A/B test ---
// SDLPoP's main onscreen surface is typically 320x200 @ 8bpp with SDL_FORCE_FULL_PALETTE.
// Pinning that pixel buffer in SRAM helps rule out PSRAM read/corruption issues.
#ifndef RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
#define RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST 1
#endif

#if RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
static uint8_t g_pop_onscreen_pixels_sram[320u * 200u];
static bool g_pop_onscreen_pixels_in_use = false;
static bool g_pop_onscreen_pixels_printed = false;
#endif

static inline bool rp2350_is_pinned_onscreen_surface(const SDL_Surface *s) {
#if RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
    return s && s->pixels == g_pop_onscreen_pixels_sram && s->w == 320 && s->h == 200;
#else
    (void)s;
    return false;
#endif
}

static inline void rp2350_fixup_onscreen_surface_format(SDL_Surface *s, const char *who) {
    if (!rp2350_is_pinned_onscreen_surface(s) || !s->format) return;

    const int bpp_bits = (int)s->format->BitsPerPixel;
    const int bpp_bytes = (int)s->format->BytesPerPixel;
    if (bpp_bits == 8 && bpp_bytes == 1 && s->format->format == SDL_PIXELFORMAT_INDEX8 && s->format->palette) {
        return;
    }

    static bool printed = false;
    if (!printed) {
        printed = true;
        DBG_PRINTF("RP2350_FIXUP: onscreen surface format corrected in %s (BitsPerPixel=%d BytesPerPixel=%d fmt=%lu pal=%p)\n",
               who ? who : "?", bpp_bits, bpp_bytes, (unsigned long)s->format->format, (void*)s->format->palette);
    }

    s->format->BitsPerPixel = 8;
    s->format->BytesPerPixel = 1;
    s->format->format = SDL_PIXELFORMAT_INDEX8;
    // Keep existing palette pointer; it should be valid for the onscreen surface.
}

#define IS_PSRAM(ptr) ((uintptr_t)(ptr) >= 0x11000000 && (uintptr_t)(ptr) < 0x12000000)
#define SDL_PALETTE_FLAG_OWNS_COLORS 0x1

static SDL_Palette *SDL_CreatePaletteInternal(int ncolors) {
    SDL_Palette *pal = (SDL_Palette *)psram_malloc(sizeof(SDL_Palette));
    if (!pal) {
        DBG_PRINTF("SDL_CreatePalette: psram_malloc(SDL_Palette) failed\n");
        return NULL;
    }

    pal->colors = (SDL_Color *)psram_malloc(sizeof(SDL_Color) * ncolors);
    if (!pal->colors) {
        DBG_PRINTF("SDL_CreatePalette: psram_malloc(colors) failed (ncolors=%d)\n", ncolors);
        psram_free(pal);
        return NULL;
    }

    memset(pal->colors, 0, sizeof(SDL_Color) * ncolors);
    pal->ncolors = ncolors;
    pal->refcount = 1;
    pal->flags = SDL_PALETTE_FLAG_OWNS_COLORS;
    return pal;
}

SDL_Palette *SDL_CreatePalette(int ncolors) {
    if (ncolors <= 0) return NULL;
    return SDL_CreatePaletteInternal(ncolors);
}

void SDL_PaletteAddRef(SDL_Palette *palette) {
    if (palette) {
        palette->refcount++;
    }
}

void SDL_PaletteRelease(SDL_Palette *palette) {
    if (!palette) return;
    palette->refcount--;
    if (palette->refcount <= 0) {
        if (palette->colors) {
            if (palette->flags & SDL_PALETTE_FLAG_OWNS_COLORS) {
                if (IS_PSRAM(palette->colors)) psram_free(palette->colors);
                else free(palette->colors);
            }
        }
        if (IS_PSRAM(palette)) psram_free(palette);
        else free(palette);
    }
}

void SDL_SurfaceAdoptPalette(SDL_Surface *surface, SDL_Palette *palette) {
    if (!surface || !surface->format) return;
    if (surface->format->palette == palette) return;
    if (palette) SDL_PaletteAddRef(palette);
    if (surface->format->palette) {
        SDL_PaletteRelease(surface->format->palette);
    }
    surface->format->palette = palette;
}

extern SDL_Surface* onscreen_surface_;

static SDL_Palette* get_screen_palette(void) {
    if (onscreen_surface_ && onscreen_surface_->format) {
        return onscreen_surface_->format->palette;
    }
    return NULL;
}

static Uint8 find_best_palette_index(const SDL_Color *src_color, const SDL_Palette *dst_palette) {
    int best_distance = INT_MAX;
    Uint8 best_index = 0;
    if (!dst_palette || dst_palette->ncolors <= 0) {
        return 0;
    }

    const bool is_screen_palette = (dst_palette == get_screen_palette());

    for (int i = 0; i < dst_palette->ncolors; ++i) {
        // HDMI scanout reserves 240..243 for control/sync patterns.
        if (is_screen_palette && i >= 240 && i <= 243) {
            continue;
        }
        const SDL_Color *dst_color = &dst_palette->colors[i];
        int dr = (int)src_color->r - (int)dst_color->r;
        int dg = (int)src_color->g - (int)dst_color->g;
        int db = (int)src_color->b - (int)dst_color->b;
        int distance = dr * dr + dg * dg + db * db;
        if (distance < best_distance) {
            best_distance = distance;
            best_index = (Uint8)i;
            if (distance == 0) {
                break;
            }
        }
    }
    return best_index;
}

// Globals
static SDL_Surface *screen_surface = NULL;
static uint32_t start_time = 0;

int SDL_Init(Uint32 flags) {
    // HDMI and PS2 are initialized in main.c before calling pop_main
    // But if pop_main calls SDL_Init, we can just return success.
    start_time = time_us_32() / 1000;
    return 0;
}

int SDL_InitSubSystem(Uint32 flags) {
    return 0;
}

void SDL_Quit(void) {
    DBG_PRINTF("SDL_Quit called\n");
    // Blink LED rapidly to indicate quit
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    for(int i=0; i<50; i++) {
        gpio_put(25, 1); sleep_ms(50);
        gpio_put(25, 0); sleep_ms(50);
    }
}

const char *SDL_GetError(void) {
    return (g_sdl_error && g_sdl_error[0]) ? g_sdl_error : "Unknown Error";
}

void SDL_SetHint(const char *name, const char *value) {
}

SDL_Window *SDL_CreateWindow(const char *title, int x, int y, int w, int h, Uint32 flags) {
    static bool printed = false;
    if (!printed) {
        printed = true;
        DBG_PRINTF("SDL_CreateWindow: title=%s size=%dx%d flags=0x%lx\n", title ? title : "(null)", w, h, (unsigned long)flags);
    }
    return (SDL_Window *)1; // Dummy pointer
}

SDL_Renderer *SDL_CreateRenderer(SDL_Window *window, int index, Uint32 flags) {
    static bool printed = false;
    if (!printed) {
        printed = true;
        DBG_PRINTF("SDL_CreateRenderer: index=%d flags=0x%lx\n", index, (unsigned long)flags);
    }
    return (SDL_Renderer *)1; // Dummy pointer
}

int SDL_GetRendererInfo(SDL_Renderer *renderer, SDL_RendererInfo *info) {
    info->flags = SDL_RENDERER_SOFTWARE;
    return 0;
}

SDL_Texture *SDL_CreateTexture(SDL_Renderer *renderer, Uint32 format, int access, int w, int h) {
    return (SDL_Texture *)1; // Dummy pointer
}

void SDL_DestroyTexture(SDL_Texture *texture) {
}

void SDL_DestroyRenderer(SDL_Renderer *renderer) {
}

void SDL_DestroyWindow(SDL_Window *window) {
}

SDL_Surface *SDL_CreateRGBSurface(Uint32 flags, int width, int height, int depth, Uint32 Rmask, Uint32 Gmask, Uint32 Bmask, Uint32 Amask) {
    bool force_full_palette = (flags & SDL_FORCE_FULL_PALETTE) != 0;
    bool no_palette = (flags & SDL_NO_PALETTE) != 0;
    Uint32 stored_flags = flags & ~(SDL_FORCE_FULL_PALETTE | SDL_NO_PALETTE);
    // printf("SDL_CreateRGBSurface: %dx%d %d bpp\n", width, height, depth);
    // Allocate everything in PSRAM to avoid SRAM OOM panics.
    SDL_Surface *s = (SDL_Surface *)psram_malloc(sizeof(SDL_Surface));
    if (!s) {
        DBG_PRINTF("SDL_CreateRGBSurface: psram_malloc(SDL_Surface) failed\n");
        return NULL;
    }

    s->flags = stored_flags;
    s->w = width;
    s->h = height;
    
    s->format = (SDL_PixelFormat *)psram_malloc(sizeof(SDL_PixelFormat));
    if (!s->format) {
        DBG_PRINTF("SDL_CreateRGBSurface: psram_malloc(SDL_PixelFormat) failed\n");
        psram_free(s);
        return NULL;
    }

    memset(s->format, 0, sizeof(SDL_PixelFormat));

    s->format->BitsPerPixel = depth;
    s->format->BytesPerPixel = (depth + 7) / 8;

    s->format->Rmask = Rmask;
    s->format->Gmask = Gmask;
    s->format->Bmask = Bmask;
    s->format->Amask = Amask;

    // Minimal pixel format bookkeeping used by SDLPoP (e.g., SDL_ISPIXELFORMAT_INDEXED,
    // and deciding how to interpret 32bpp pixels).
    if (depth <= 8) {
        s->format->format = SDL_PIXELFORMAT_INDEX8;
    } else if (depth == 24) {
        s->format->format = SDL_PIXELFORMAT_RGB24;
    } else if (depth == 32) {
        // SDLPoP uses masks from types.h:
        // - little-endian: R=0x000000ff, G=0x0000ff00, B=0x00ff0000, A=0xff000000 (RGBA bytes)
        // - big-endian:    R=0x00ff0000, G=0x0000ff00, B=0x000000ff, A=0xff000000
        const bool masks_all_zero = (Rmask | Gmask | Bmask | Amask) == 0;
        const bool is_rgba_masks = (Rmask == 0x000000ffu && Gmask == 0x0000ff00u && Bmask == 0x00ff0000u && Amask == 0xff000000u);
        const bool is_argb_masks = (Rmask == 0x00ff0000u && Gmask == 0x0000ff00u && Bmask == 0x000000ffu && Amask == 0xff000000u);

        // If masks are omitted (common in our IMG_Load_RW), default to RGBA8888 because
        // stb_image produces RGBA bytes in memory.
        if (is_argb_masks) s->format->format = SDL_PIXELFORMAT_ARGB8888;
        else if (is_rgba_masks || masks_all_zero) s->format->format = SDL_PIXELFORMAT_RGBA8888;
        else s->format->format = SDL_PIXELFORMAT_RGBA8888;

        // If masks were all zero, populate them to match the chosen format to keep
        // downstream code consistent.
        if (masks_all_zero) {
            s->format->Rmask = 0x000000ffu;
            s->format->Gmask = 0x0000ff00u;
            s->format->Bmask = 0x00ff0000u;
            s->format->Amask = 0xff000000u;
        }
    } else {
        s->format->format = 0;
    }
    
    // SDLPoP expects 8bpp surfaces to have a full 256-entry palette.
    // Truncating palettes (e.g. to 16 colors) can cause incorrect color mapping and severe artifacts
    // when surfaces are converted/blitted.
    // Also, don't allocate palette for > 8bpp surfaces to save RAM.
    // If SDL_NO_PALETTE is set, skip palette allocation (caller will adopt an existing palette).
    if (depth <= 8 && !no_palette) {
        SDL_Palette *palette = SDL_CreatePalette(256);
        if (!palette) {
            psram_free(s->format);
            psram_free(s);
            return NULL;
        }
        s->format->palette = palette;
    } else {
        s->format->palette = NULL;
    }
    
    s->pitch = width * s->format->BytesPerPixel;

#if RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
    // Special-case SDLPoP onscreen surface: 320x200, 8bpp.
    // Note: some SDLPoP builds may not pass our custom SDL_FORCE_FULL_PALETTE flag,
    // so do not require it for this diagnostic.
    if (depth == 8 && width == 320 && height == 200) {
        if (!g_pop_onscreen_pixels_in_use) {
            g_pop_onscreen_pixels_in_use = true;
            s->pixels = g_pop_onscreen_pixels_sram;
            memset(s->pixels, 0, (size_t)s->pitch * (size_t)height);
            if (!g_pop_onscreen_pixels_printed) {
                g_pop_onscreen_pixels_printed = true;
                DBG_PRINTF("SDL_CreateRGBSurface: onscreen pixels pinned to SRAM (pixels=%p pitch=%d flags=0x%lx force_full_palette=%d)\n",
                       s->pixels, s->pitch, (unsigned long)flags, force_full_palette ? 1 : 0);
            }
        } else {
            DBG_PRINTF("SDL_CreateRGBSurface: onscreen SRAM pixels already in use; falling back to PSRAM\n");
            s->pixels = psram_malloc((size_t)s->pitch * (size_t)height);
        }
    } else {
        s->pixels = psram_malloc((size_t)s->pitch * (size_t)height);
    }
#else
    // Use PSRAM for pixel data if available
    s->pixels = psram_malloc((size_t)s->pitch * (size_t)height);
#endif
    if (!s->pixels) {
        DBG_PRINTF("SDL_CreateRGBSurface: psram_malloc(pixels) failed. Size: %d\n", s->pitch * height);
        if (s->format->palette) SDL_PaletteRelease(s->format->palette);
        psram_free(s->format);
        psram_free(s);
        return NULL;
    }

#if RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
    if (s->pixels != g_pop_onscreen_pixels_sram) {
        memset(s->pixels, 0, (size_t)s->pitch * (size_t)height);
    }
#else
    memset(s->pixels, 0, (size_t)s->pitch * (size_t)height);
#endif

    s->refcount = 1;
    s->colorkey = 0;
    s->use_colorkey = SDL_FALSE;

    // Default blend/alpha behavior (SDL2-like): surfaces with alpha default to BLEND.
    s->blendMode = (depth == 32 || (s->format && s->format->Amask)) ? SDL_BLENDMODE_BLEND : SDL_BLENDMODE_NONE;
    s->alphaMod = 255;
    
    // Initialize clip_rect to full surface
    s->clip_rect.x = 0;
    s->clip_rect.y = 0;
    s->clip_rect.w = width;
    s->clip_rect.h = height;
    
    return s;
}

void SDL_FreeSurface(SDL_Surface *surface) {
    if (surface) {
        if (surface->pixels) {
#if RP2350_POP_ONSCREEN_PIXELS_IN_SRAM_TEST
            if (surface->pixels == g_pop_onscreen_pixels_sram) {
                g_pop_onscreen_pixels_in_use = false;
            } else
#endif
            {
                if (IS_PSRAM(surface->pixels)) psram_free(surface->pixels);
                else free(surface->pixels);
            }
        }
        if (surface->format) {
            if (surface->format->palette) {
                SDL_PaletteRelease(surface->format->palette);
            }
            if (IS_PSRAM(surface->format)) psram_free(surface->format);
            else free(surface->format);
        }
        if (IS_PSRAM(surface)) psram_free(surface);
        else free(surface);
    }
}

// --- RP2350 fast reverse palette map (RGB888 -> index) ---
// Used by SDL_UpdateTexture when the input frame arrives as 24/32bpp.
static uint32_t rp2350_rgb_to_idx_keys[512];
static uint8_t rp2350_rgb_to_idx_vals[512];
static bool rp2350_rgb_to_idx_ready = false;
static bool rp2350_rgb_to_idx_dirty = true;
static uint32_t rp2350_screen_palette_rgb888_cache[256];

static inline uint32_t rp2350_hash_u32(uint32_t x);
static void rp2350_rebuild_rgb_to_index_map(void);
static inline uint8_t rp2350_rgb888_to_index(uint32_t rgb888);

int SDL_SetPaletteColors(SDL_Palette *palette, const SDL_Color *colors, int firstcolor, int ncolors) {
    if (!palette || !colors || ncolors <= 0) return -1;

    SDL_Palette* screen_palette = get_screen_palette();
    bool update_hardware = (palette == screen_palette);
    // printf("SDL_SetPaletteColors: first=%d n=%d\n", firstcolor, ncolors);
    for (int i = 0; i < ncolors; i++) {
        int idx = firstcolor + i;
        if (idx < palette->ncolors) {
            palette->colors[idx] = colors[i];
            if (update_hardware) {
                // HDMI scanout reserves 240-243 for sync/control.
                if (idx >= 240 && idx <= 243) {
                    continue;
                }
                uint32_t color888 = (colors[i].r << 16) | (colors[i].g << 8) | colors[i].b;
                graphics_set_palette(idx, color888);
                if (idx >= 0 && idx < 256) {
                    rp2350_screen_palette_rgb888_cache[idx] = color888;
                    rp2350_rgb_to_idx_dirty = true;
                }
                // if (i < 5) printf("Pal[%d] = %06X\n", idx, color888);
            }
        }
    }
    if (update_hardware) {
        graphics_restore_sync_colors();
    }
    return 0;
}

static inline uint32_t rp2350_hash_u32(uint32_t x) {
    // Simple multiplicative hash; good enough for 256 keys.
    return x * 2654435761u;
}

static void rp2350_rebuild_rgb_to_index_map(void) {
    // Empty sentinel is outside valid RGB888 range.
    for (int i = 0; i < 512; ++i) rp2350_rgb_to_idx_keys[i] = 0xFFFFFFFFu;

    SDL_Palette *pal = get_screen_palette();
    if (!pal || !pal->colors) {
        rp2350_rgb_to_idx_ready = false;
        rp2350_rgb_to_idx_dirty = false;
        return;
    }

    int n = pal->ncolors;
    if (n > 256) n = 256;

    for (int i = 0; i < n; ++i) {
        // Never map RGB colors to HDMI control indices.
        if (i >= 240 && i <= 243) continue;
        SDL_Color c = pal->colors[i];
        const uint32_t rgb = ((uint32_t)c.r << 16) | ((uint32_t)c.g << 8) | (uint32_t)c.b;
        rp2350_screen_palette_rgb888_cache[i] = rgb;

        uint32_t slot = (rp2350_hash_u32(rgb) >> 23) & 511u; // 9 bits
        for (int probe = 0; probe < 512; ++probe) {
            if (rp2350_rgb_to_idx_keys[slot] == 0xFFFFFFFFu || rp2350_rgb_to_idx_keys[slot] == rgb) {
                rp2350_rgb_to_idx_keys[slot] = rgb;
                rp2350_rgb_to_idx_vals[slot] = (uint8_t)i;
                break;
            }
            slot = (slot + 1u) & 511u;
        }
    }

    rp2350_rgb_to_idx_ready = true;
    rp2350_rgb_to_idx_dirty = false;
}

static inline uint8_t rp2350_rgb888_to_index(uint32_t rgb888) {
    if (!rp2350_rgb_to_idx_ready || rp2350_rgb_to_idx_dirty) {
        rp2350_rebuild_rgb_to_index_map();
        if (!rp2350_rgb_to_idx_ready) return 0;
    }

    uint32_t slot = (rp2350_hash_u32(rgb888) >> 23) & 511u;
    for (int probe = 0; probe < 512; ++probe) {
        const uint32_t k = rp2350_rgb_to_idx_keys[slot];
        if (k == rgb888) return rp2350_rgb_to_idx_vals[slot];
        if (k == 0xFFFFFFFFu) break;
        slot = (slot + 1u) & 511u;
    }
    return 0;
}

static bool is_palette_empty(SDL_Palette *pal) {
    if (!pal || !pal->colors) return true;
    // Check if all colors are black.
    for (int i = 0; i < pal->ncolors; i++) {
        if (pal->colors[i].r != 0 || pal->colors[i].g != 0 || pal->colors[i].b != 0) {
            return false;
        }
    }
    return true;
}

int SDL_BlitSurface(SDL_Surface *src, const SDL_Rect *srcrect, SDL_Surface *dst, SDL_Rect *dstrect) {
    if (!src || !dst) return -1;

    rp2350_fixup_onscreen_surface_format(src, "SDL_BlitSurface(src)");
    rp2350_fixup_onscreen_surface_format(dst, "SDL_BlitSurface(dst)");

    SDL_Rect s_rect = (SDL_Rect){0, 0, src->w, src->h};
    if (srcrect) s_rect = *srcrect;

    SDL_Rect d_rect = (SDL_Rect){0, 0, dst->w, dst->h};
    if (dstrect) {
        d_rect.x = dstrect->x;
        d_rect.y = dstrect->y;
        d_rect.w = s_rect.w;
        d_rect.h = s_rect.h;
    }

    // Get clip rect bounds
    int clip_left = dst->clip_rect.x;
    int clip_top = dst->clip_rect.y;
    int clip_right = dst->clip_rect.x + dst->clip_rect.w;
    int clip_bottom = dst->clip_rect.y + dst->clip_rect.h;
    
    // RP2350: HARD CLIP for onscreen surface - game sprites (y < 192) must not draw into health bar
    // Only during gameplay (start_level >= 0), not title/cutscenes
    extern SDL_Surface* onscreen_surface_;
    extern short start_level;
    if (start_level >= 0 && dst == onscreen_surface_ && d_rect.y < 192 && clip_bottom > 192) {
        clip_bottom = 192;  // Force game area clip
    }
    
    // Clipping to destination clip_rect (not just surface bounds)
    if (d_rect.x < clip_left) { int diff = clip_left - d_rect.x; s_rect.x += diff; s_rect.w -= diff; d_rect.w -= diff; d_rect.x = clip_left; }
    if (d_rect.y < clip_top) { int diff = clip_top - d_rect.y; s_rect.y += diff; s_rect.h -= diff; d_rect.h -= diff; d_rect.y = clip_top; }
    if (d_rect.x + d_rect.w > clip_right) { int diff = (d_rect.x + d_rect.w) - clip_right; s_rect.w -= diff; d_rect.w -= diff; }
    if (d_rect.y + d_rect.h > clip_bottom) { int diff = (d_rect.y + d_rect.h) - clip_bottom; s_rect.h -= diff; d_rect.h -= diff; }

    // Debug: trace clipping that actually clipped at bottom
    static int clipped_blit_debug = 0;
    if (clipped_blit_debug < 20 && clip_bottom < 200) {
        int orig_h = srcrect ? srcrect->h : src->h;
        int orig_y = dstrect ? dstrect->y : 0;
        if (orig_y + orig_h > clip_bottom && d_rect.h < orig_h) {
            DBG_PRINTF("[CLIP_APPLIED] orig_y=%d orig_h=%d clip_bot=%d -> clipped_y=%d clipped_h=%d\\n",
                orig_y, orig_h, clip_bottom, d_rect.y, d_rect.h);
            clipped_blit_debug++;
        }
    }

    if (s_rect.w <= 0 || s_rect.h <= 0) return 0;

    const int src_bpp = src->format ? src->format->BytesPerPixel : 1;
    const int dst_bpp = dst->format ? dst->format->BytesPerPixel : 1;
    const bool paletted_copy = (src_bpp == 1 && dst_bpp == 1);
    Uint8 palette_map[256];
    bool use_palette_map = false;

    if (paletted_copy && src->format && dst->format) {
        SDL_Palette *src_palette = src->format->palette;
        SDL_Palette *dst_palette = dst->format->palette;
        
        // Only map if source palette is NOT empty (has colors) AND differs from dest
        if (src_palette && dst_palette && src_palette != dst_palette && !is_palette_empty(src_palette)) {
            bool palettes_differ = true;
            if (src_palette->ncolors == dst_palette->ncolors) {
                 if (memcmp(src_palette->colors, dst_palette->colors, src_palette->ncolors * sizeof(SDL_Color)) == 0) {
                     palettes_differ = false;
                 }
            }

            if (palettes_differ) {
                use_palette_map = true;
                for (int i = 0; i < 256; ++i) {
                    palette_map[i] = (Uint8)i;
                }
                int max_src_colors = src_palette->ncolors;
                if (max_src_colors > 256) max_src_colors = 256;
                for (int i = 0; i < max_src_colors; ++i) {
                    palette_map[i] = find_best_palette_index(&src_palette->colors[i], dst_palette);
                }
            }
        }
    }

    Uint8 *src_pixels = (Uint8 *)src->pixels;
    Uint8 *dst_pixels = (Uint8 *)dst->pixels;

    // Debug print for blit (throttled)
    static int blit_debug_count = 0;
    if (blit_debug_count < 20) {
        DBG_PRINTF("Blit: %dx%d bpp:%d->%d key:%d val:%d map:%d dst_pal=%p\n", 
            s_rect.w, s_rect.h, src_bpp, dst_bpp, src->use_colorkey, src->colorkey, use_palette_map,
            (void*)(dst->format ? dst->format->palette : NULL));
        blit_debug_count++;
    }

    for (int y = 0; y < s_rect.h; y++) {
        Uint8 *s_row = src_pixels + (s_rect.y + y) * src->pitch + s_rect.x * src_bpp;
        Uint8 *d_row = dst_pixels + (d_rect.y + y) * dst->pitch + d_rect.x * dst_bpp;
        
        if (paletted_copy) {
            for (int x = 0; x < s_rect.w; ++x) {
                Uint8 pixel = s_row[x];
                if (src->use_colorkey && pixel == (Uint8)src->colorkey) {
                    continue;
                }
                Uint8 mapped_pixel = use_palette_map ? palette_map[pixel] : pixel;
                d_row[x] = mapped_pixel;
            }
        } else if (src_bpp == 3 && dst_bpp == 1) {
            // 24bpp -> 8bpp conversion (No Alpha)
            // Used for fonts/images loaded as RGB
            SDL_Palette *dst_pal = dst->format->palette;
            if (!dst_pal) dst_pal = get_screen_palette();

            for (int x = 0; x < s_rect.w; ++x) {
                Uint8 r = s_row[x * 3];
                Uint8 g = s_row[x * 3 + 1];
                Uint8 b = s_row[x * 3 + 2];

                // Check colorkey if enabled
                if (src->use_colorkey) {
                    Uint32 pixel = (r << 16) | (g << 8) | b;
                    // Colorkey for 24bpp is usually packed RGB
                    if ((pixel & 0xFFFFFF) == (src->colorkey & 0xFFFFFF)) {
                        continue; // Transparent
                    }
                }

                if (dst_pal) {
                    SDL_Color c = {r, g, b, 255};
                    d_row[x] = find_best_palette_index(&c, dst_pal);
                } else {
                    d_row[x] = 15; // White fallback
                }
            }
        } else if (src_bpp == 4 && dst_bpp == 1) {
            // 32bpp -> 8bpp conversion
            Uint32 *s_row_32 = (Uint32*)s_row;
            SDL_Palette *dst_pal = dst->format->palette;
            if (!dst_pal) dst_pal = get_screen_palette();

            // Debug: print once per blit
            static int rgba_blit_count = 0;
            if (y == 0 && rgba_blit_count < 10) {
                DBG_PRINTF("[RGBA->8bpp] %dx%d dst_pal=%p ncolors=%d blend=%d alphaMod=%d\n",
                       s_rect.w, s_rect.h, (void*)dst_pal, 
                       dst_pal ? dst_pal->ncolors : 0,
                       src->blendMode, src->alphaMod);
                if (s_rect.w > 0) {
                    Uint32 pix = s_row_32[0];
                    DBG_PRINTF("[RGBA->8bpp] first pixel=0x%08lx (r=%d g=%d b=%d a=%d)\n",
                           (unsigned long)pix, (int)(pix&0xFF), (int)((pix>>8)&0xFF),
                           (int)((pix>>16)&0xFF), (int)((pix>>24)&0xFF));
                }
                rgba_blit_count++;
            }

            for (int x = 0; x < s_rect.w; ++x) {
                Uint32 pixel = s_row_32[x];
                // stbi loads RGBA into memory; on little-endian, pixel is 0xAABBGGRR.
                const Uint8 r = (Uint8)(pixel & 0xFF);
                const Uint8 g = (Uint8)((pixel >> 8) & 0xFF);
                const Uint8 b = (Uint8)((pixel >> 16) & 0xFF);
                const Uint8 a = (Uint8)((pixel >> 24) & 0xFF);

                if (src->use_colorkey) {
                    if (((pixel) & 0xFFFFFFu) == ((Uint32)src->colorkey & 0xFFFFFFu)) {
                        continue;
                    }
                }

                const bool do_blend = (src->blendMode == SDL_BLENDMODE_BLEND) || (src->alphaMod != 255);

                if (!do_blend) {
                    if (dst_pal) {
                        SDL_Color c = { r, g, b, 255 };
                        d_row[x] = find_best_palette_index(&c, dst_pal);
                    } else {
                        d_row[x] = 15; // White fallback
                    }
                    continue;
                }

                // Effective alpha = src alpha * surface alphaMod.
                Uint32 a_eff = (Uint32)a;
                if (src->alphaMod != 255) {
                    a_eff = (a_eff * (Uint32)src->alphaMod + 127u) / 255u;
                }

                if (a_eff == 0) {
                    continue; // Preserve destination
                }

                if (a_eff >= 255) {
                    if (dst_pal) {
                        SDL_Color c = { r, g, b, 255 };
                        d_row[x] = find_best_palette_index(&c, dst_pal);
                    } else {
                        d_row[x] = 15;
                    }
                    continue;
                }

                // Blend in RGB, then map back to the destination palette.
                Uint8 dr = 0, dg = 0, db = 0;
                if (dst_pal && dst_pal->colors) {
                    const Uint8 dst_idx = d_row[x];
                    if ((int)dst_idx < dst_pal->ncolors) {
                        const SDL_Color *dc = &dst_pal->colors[dst_idx];
                        dr = dc->r; dg = dc->g; db = dc->b;
                    }
                }

                const Uint32 inv = 255u - a_eff;
                const Uint8 out_r = (Uint8)((r * a_eff + dr * inv + 127u) / 255u);
                const Uint8 out_g = (Uint8)((g * a_eff + dg * inv + 127u) / 255u);
                const Uint8 out_b = (Uint8)((b * a_eff + db * inv + 127u) / 255u);

                if (dst_pal) {
                    SDL_Color out_c = { out_r, out_g, out_b, 255 };
                    d_row[x] = find_best_palette_index(&out_c, dst_pal);
                } else {
                    d_row[x] = 15;
                }
            }
        } else if (src_bpp == 1 && dst_bpp == 4) {
             // 8bpp -> 32bpp conversion (used for read_peel_from_screen)
             Uint32 *d_row_32 = (Uint32*)d_row;
             SDL_Palette *src_pal = src->format->palette;
             if (!src_pal) src_pal = get_screen_palette();
             
             for (int x = 0; x < s_rect.w; ++x) {
                 Uint8 idx = s_row[x];
                if (src->use_colorkey && idx == src->colorkey) {
                    // Transparent source pixel -> Transparent destination pixel
                    // Or should we preserve destination?
                    // Usually blit overwrites unless alpha blending is on.
                    // But for peel reading, we want the exact pixel.
                    // If source is transparent, we write transparent black?
                    // Or do we skip?
                    // If we are reading FROM screen, screen usually doesn't have transparency in the way we think.
                    // But if it does (index 0), we should map it.
                    // Let's just map the color.
                    continue;
                }                 if (src_pal) {
                     SDL_Color c = src_pal->colors[idx];
                     d_row_32[x] = (255 << 24) | (c.r << 16) | (c.g << 8) | c.b;
                 } else {
                     d_row_32[x] = (255 << 24) | (idx << 16) | (idx << 8) | idx; // Grayscale fallback
                 }
             }
        } else if (src_bpp == dst_bpp) {
            if (src->use_colorkey) {
                if (src_bpp == 1) {
                    Uint8 key = (Uint8)src->colorkey;
                    for (int x = 0; x < s_rect.w; ++x) {
                        if (s_row[x] != key) {
                            d_row[x] = s_row[x];
                        }
                    }
                } else {
                    // Fallback for other depths or if implementation is missing
                    memcpy(d_row, s_row, s_rect.w * src_bpp);
                }
            } else {
                memcpy(d_row, s_row, s_rect.w * src_bpp);
            }
        } else {
            int copy_bytes = s_rect.w * (src_bpp < dst_bpp ? src_bpp : dst_bpp);
            memcpy(d_row, s_row, copy_bytes);
        }
    }
    return 0;
}

int SDL_FillRect(SDL_Surface *dst, const SDL_Rect *rect, Uint32 color) {
    if (!dst) return -1;

    rp2350_fixup_onscreen_surface_format(dst, "SDL_FillRect");
    SDL_Rect d_rect = {0, 0, dst->w, dst->h};
    if (rect) d_rect = *rect;

    // Clipping
    if (d_rect.x < 0) { d_rect.w += d_rect.x; d_rect.x = 0; }
    if (d_rect.y < 0) { d_rect.h += d_rect.y; d_rect.y = 0; }
    if (d_rect.x + d_rect.w > dst->w) { d_rect.w = dst->w - d_rect.x; }
    if (d_rect.y + d_rect.h > dst->h) { d_rect.h = dst->h - d_rect.y; }

    if (d_rect.w <= 0 || d_rect.h <= 0) return 0;

    if (!dst->format || !dst->pixels || dst->pitch <= 0) return -1;

    const int bpp = dst->format->BytesPerPixel ? dst->format->BytesPerPixel : 1;
    Uint8 *dst_pixels = (Uint8 *)dst->pixels;
    for (int y = 0; y < d_rect.h; y++) {
        Uint8 *d_row = dst_pixels + (d_rect.y + y) * dst->pitch + d_rect.x * bpp;
        if (bpp == 1) {
            memset(d_row, (Uint8)color, (size_t)d_rect.w);
        } else if (bpp == 4) {
            Uint32 *d32 = (Uint32 *)d_row;
            for (int x = 0; x < d_rect.w; ++x) d32[x] = color;
        } else if (bpp == 3) {
            // Pack as RGB24 using low 24 bits.
            const Uint8 r = (Uint8)((color >> 16) & 0xFF);
            const Uint8 g = (Uint8)((color >> 8) & 0xFF);
            const Uint8 b = (Uint8)(color & 0xFF);
            for (int x = 0; x < d_rect.w; ++x) {
                d_row[x * 3 + 0] = r;
                d_row[x * 3 + 1] = g;
                d_row[x * 3 + 2] = b;
            }
        } else {
            // Unsupported bpp: fill with zeros.
            memset(d_row, 0, (size_t)d_rect.w * (size_t)bpp);
        }
    }
    return 0;
}

int SDL_SetColorKey(SDL_Surface *surface, int flag, Uint32 key) {
    if (!surface) return -1;
    surface->use_colorkey = flag ? SDL_TRUE : SDL_FALSE;
    if (surface->format && surface->format->BytesPerPixel == 1) {
        surface->colorkey = key & 0xFF;
    } else {
        surface->colorkey = key;
    }
    return 0;
}

int SDL_RenderClear(SDL_Renderer *renderer) {
    return 0;
}

int SDL_RenderCopy(SDL_Renderer *renderer, SDL_Texture *texture, const SDL_Rect *srcrect, const SDL_Rect *dstrect) {
    // In SDLPoP, RenderCopy is used to copy the final texture to screen.
    // We intercept UpdateTexture instead, or just assume the texture is the screen.
    return 0;
}

static inline Uint8 *rp2350_framebuffer(void) {
    return (Uint8 *)graphics_get_buffer();
}

#ifndef RP2350_SDL_VISUAL_HEARTBEAT
#define RP2350_SDL_VISUAL_HEARTBEAT 0
#endif

// Diagnostic: bypass SDLPoP pixels and draw a known-good pattern into the framebuffer.
// If this looks correct on HDMI, the scanout path is fine and the bug is in the incoming
// image data/format or how SDLPoP produces it.
#ifndef RP2350_FORCE_TEST_PATTERN
#define RP2350_FORCE_TEST_PATTERN 0
#endif

// Diagnostic: dump a small sample of the first frame's source bytes.
#ifndef RP2350_DUMP_FIRST_FRAME_BYTES
#define RP2350_DUMP_FIRST_FRAME_BYTES 1
#endif

// Diagnostic: overlay a 1-row bar of palette indices (0..319) at the top of the active image.
// Useful to confirm that the framebuffer is being displayed correctly and that palette indices
// 244..255 are usable, while 240..243 are reserved.
#ifndef RP2350_DEBUG_INDEX_BAR
#define RP2350_DEBUG_INDEX_BAR 0
#endif

static void rp2350_draw_test_pattern(uint8_t *dst, int dst_pitch, int w, int h, int y_offset, int output_height) {
    if (!dst || dst_pitch <= 0) return;

    // Clear top/bottom padding.
    if (y_offset > 0) {
        memset(dst, 0, (size_t)dst_pitch * (size_t)y_offset);
    }
    const int bottom_pad = output_height - (y_offset + h);
    if (bottom_pad > 0) {
        memset(dst + (y_offset + h) * dst_pitch, 0, (size_t)dst_pitch * (size_t)bottom_pad);
    }

    // Pattern: 16-color vertical bars + horizontal gradient. Uses indices 0..239 only.
    for (int y = 0; y < h; ++y) {
        uint8_t *row = dst + (y + y_offset) * dst_pitch;
        for (int x = 0; x < w; ++x) {
            const uint8_t bars = (uint8_t)((x >> 3) & 0x0F);          // wide vertical bars (0..15)
            const uint8_t grad = (uint8_t)((x * 239) / (w - 1));      // 0..239 left->right
            row[x] = (y < (h / 2)) ? bars : grad;
        }
    }
}

int SDL_UpdateTexture(SDL_Texture *texture, const SDL_Rect *rect, const void *pixels, int pitch) {
    // Copy pixels to graphics_buffer
    // Assuming 320x200 input and 320x240 output
    // We center it vertically?
    
    const Uint8 *src = (const Uint8 *)pixels;
    Uint8 *dst = rp2350_framebuffer();

    int w = 320;
    int h = 200;
    int dst_pitch = (int)graphics_get_width();
    const int output_height = (int)graphics_get_height();

    if (!dst || dst_pitch <= 0 || output_height <= 0) {
        return 0;
    }
    
    // Center vertically: (240 - 200) / 2 = 20 offset
    int y_offset = (output_height - h) / 2;
    if (y_offset < 0) y_offset = 0;
    int bottom_pad = output_height - (y_offset + h);
    if (bottom_pad < 0) bottom_pad = 0;
    
    // Debug/heartbeat
    static int frame_count = 0;
    static int last_pitch = -1;
    frame_count++;

    // If SDLPoP accidentally corrupts the onscreen surface's format fields (e.g. flips it to 32bpp),
    // some code paths will start doing 32-bit writes into the 8bpp pixel buffer and create the
    // characteristic 00 00 00 03 striping pattern. Keep it pinned to 8bpp.
    rp2350_fixup_onscreen_surface_format(onscreen_surface_, "SDL_UpdateTexture");
    if (frame_count == 1) {
        DBG_PRINTF("SDL_UpdateTexture: first frame src=%p pitch=%d dst=%p fb=%p w=%d h=%d out_h=%d dst_pitch=%d\\n",
               src, pitch, dst, graphics_get_buffer(), w, h, output_height, dst_pitch);
        if (onscreen_surface_ && onscreen_surface_->pixels) {
            DBG_PRINTF("SDL_UpdateTexture: onscreen_surface_ pixels=%p pitch=%d bpp=%u\\n",
                   onscreen_surface_->pixels,
                   onscreen_surface_->pitch,
                   (unsigned)(onscreen_surface_->format ? onscreen_surface_->format->BitsPerPixel : 0));
            if (pixels != onscreen_surface_->pixels || pitch != onscreen_surface_->pitch) {
                DBG_PRINTF("SDL_UpdateTexture: WARNING: update src/pitch do not match onscreen_surface_\\n");
            }
        }
        gpio_init(25);
        gpio_set_dir(25, GPIO_OUT);
        gpio_put(25, 1);
    }
    if (pitch != last_pitch) {
        DBG_PRINTF("SDL_UpdateTexture: pitch change frame=%d pitch=%d (was %d) pixels=%p rect=%p\\n",
               frame_count, pitch, last_pitch, pixels, (const void *)rect);
        last_pitch = pitch;
    }
    if ((frame_count % 60) == 0) {
        gpio_put(25, !gpio_get(25));
    }

#if RP2350_DUMP_FIRST_FRAME_BYTES
    if (frame_count == 1 && pixels && pitch > 0) {
        const uint8_t *b = (const uint8_t *)pixels;
        // Dump first 32 bytes of the first two rows (or fewer if pitch is small).
        const int n = (pitch < 32) ? pitch : 32;
        DBG_PRINTF("SDL_UpdateTexture: src row0 first %d bytes:", n);
        for (int i = 0; i < n; ++i) DBG_PRINTF(" %02X", b[i]);
        DBG_PRINTF("\\n");
        if (pitch >= n) {
            DBG_PRINTF("SDL_UpdateTexture: src row1 first %d bytes:", n);
            for (int i = 0; i < n; ++i) DBG_PRINTF(" %02X", b[pitch + i]);
            DBG_PRINTF("\\n");
        }
    }
#endif

#if RP2350_FORCE_TEST_PATTERN
    rp2350_draw_test_pattern(dst, dst_pitch, w, h, y_offset, output_height);
    return 0;
#endif

    if (y_offset > 0) {
        memset(dst, 0, dst_pitch * y_offset);
    }
    if (bottom_pad > 0) {
        memset(dst + (y_offset + h) * dst_pitch, 0, dst_pitch * bottom_pad);
    }

    // Handle 8bpp (expected), plus 16/24/32bpp frames by mapping RGB back to palette indices.
    // Only infer bpp for the exact tightly-packed cases; otherwise treat as 8bpp indexed.
    const int src_bpp = (pitch == w * 4) ? 4 : ((pitch == w * 3) ? 3 : ((pitch == w * 2) ? 2 : 1));

    if (frame_count <= 3 && src && pitch > 0) {
        DBG_PRINTF("SDL_UpdateTexture: frame=%d inferred src_bpp=%d\\n", frame_count, src_bpp);
        if (src_bpp == 1) {
            // Full-frame min/max and a simple hash to detect periodic corruption.
            uint8_t mn = 255, mx = 0;
            uint32_t hsh = 2166136261u; // FNV-1a basis
            for (int y = 0; y < h; ++y) {
                const uint8_t *row = (const uint8_t *)src + y * pitch;
                for (int x = 0; x < w; ++x) {
                    const uint8_t v = row[x];
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                    hsh ^= (uint32_t)v;
                    hsh *= 16777619u;
                }
            }
            DBG_PRINTF("SDL_UpdateTexture: frame=%d src full min=%u max=%u fnv=0x%08lx\\n",
                   frame_count, (unsigned)mn, (unsigned)mx, (unsigned long)hsh);

            // Dump a compact sample from the middle row to spot stripe patterns.
            const int sample_y = h / 2;
            const uint8_t *mid = (const uint8_t *)src + sample_y * pitch;
            DBG_PRINTF("SDL_UpdateTexture: frame=%d src row%d first 32:", frame_count, sample_y);
            for (int i = 0; i < 32; ++i) DBG_PRINTF(" %02X", mid[i]);
            DBG_PRINTF("\\n");
        }
    }

    for (int y = 0; y < h; y++) {
        Uint8 *drow = dst + (y + y_offset) * dst_pitch;
        const Uint8 *srow = src + y * pitch;

        if (src_bpp == 1) {
            memcpy(drow, srow, w);
            // HDMI scanout reservation: indices 240..243 are control/sync codes.
            for (int x = 0; x < w; ++x) {
                if (drow[x] >= 240 && drow[x] <= 243) drow[x] = 255;
            }
        } else if (src_bpp == 2) {
            // Assume RGB565 little-endian.
            const uint16_t *s16 = (const uint16_t *)srow;
            for (int x = 0; x < w; ++x) {
                const uint16_t p = s16[x];
                const uint8_t r = (uint8_t)((p >> 11) & 0x1F);
                const uint8_t g = (uint8_t)((p >> 5) & 0x3F);
                const uint8_t b = (uint8_t)(p & 0x1F);
                const uint8_t r8 = (uint8_t)((r * 255u) / 31u);
                const uint8_t g8 = (uint8_t)((g * 255u) / 63u);
                const uint8_t b8 = (uint8_t)((b * 255u) / 31u);
                const uint32_t rgb = ((uint32_t)r8 << 16) | ((uint32_t)g8 << 8) | (uint32_t)b8;
                uint8_t idx = rp2350_rgb888_to_index(rgb);
                drow[x] = (idx >= 240 && idx <= 243) ? 255 : idx;
            }
        } else if (src_bpp == 3) {
            for (int x = 0; x < w; ++x) {
                const uint8_t r = srow[x * 3 + 0];
                const uint8_t g = srow[x * 3 + 1];
                const uint8_t b = srow[x * 3 + 2];
                const uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
                uint8_t idx = rp2350_rgb888_to_index(rgb);
                drow[x] = (idx >= 240 && idx <= 243) ? 255 : idx;
            }
        } else {
            const uint32_t *srow32 = (const uint32_t *)srow;
            for (int x = 0; x < w; ++x) {
                // Our 32bpp surfaces are treated as little-endian RGBA in this port.
                const uint32_t p = srow32[x];
                const uint8_t r = (uint8_t)(p & 0xFF);
                const uint8_t g = (uint8_t)((p >> 8) & 0xFF);
                const uint8_t b = (uint8_t)((p >> 16) & 0xFF);
                const uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
                uint8_t idx = rp2350_rgb888_to_index(rgb);
                drow[x] = (idx >= 240 && idx <= 243) ? 255 : idx;
            }
        }
    }

#if RP2350_DEBUG_INDEX_BAR
    // Overlay a single row at the top of the active image: shows indices 0..319.
    // 240..243 will appear as background (clamped to 255) if the reservation is active.
    if (dst && y_offset >= 0 && y_offset < output_height) {
        uint8_t *bar = dst + y_offset * dst_pitch;
        for (int x = 0; x < w; ++x) {
            uint8_t v = (uint8_t)x;
            if (v >= 240 && v <= 243) v = 255;
            bar[x] = v;
        }
    }
#endif

    if (frame_count == 1) {
        // Sanity: show what we actually wrote into the framebuffer.
        const int n = (dst_pitch < 32) ? dst_pitch : 32;
        DBG_PRINTF("SDL_UpdateTexture: dst row%u first %d bytes:", (unsigned)0, n);
        for (int i = 0; i < n; ++i) DBG_PRINTF(" %02X", dst[y_offset * dst_pitch + i]);
        DBG_PRINTF("\\n");
    }

    // Bottom-row pixel heartbeat: not overwritten by the 320x200 copy.
    // This is useful during bring-up but confusing in normal play.
#if RP2350_SDL_VISUAL_HEARTBEAT
    if (dst) dst[(output_height - 1) * dst_pitch + 0] = (frame_count & 1) ? 15 : 0;
#endif
    
    return 0;
}

void SDL_RenderPresent(SDL_Renderer *renderer) {
    // Nothing to do, DMA handles it
}

int SDL_SetRenderDrawColor(SDL_Renderer *renderer, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    return 0;
}

int SDL_RenderFillRect(SDL_Renderer *renderer, const SDL_Rect *rect) {
    return 0;
}

void SDL_Delay(Uint32 ms) {
    sleep_ms(ms);
}

Uint32 SDL_GetTicks(void) {
    return (time_us_32() / 1000) - start_time;
}

static Uint8 keyboard_state[SDL_NUM_SCANCODES];

// Buffered events for returning one at a time
static SDL_Event pending_events[32];
static int pending_event_count = 0;
static int pending_event_index = 0;

// Key timeout tracking to auto-release stuck keys
// Only track movement keys (arrows) and shift - these are most likely to "stick"
#define KEY_TIMEOUT_MS 500  // Auto-release after 500ms with no new press
static Uint32 key_press_time[SDL_NUM_SCANCODES];

// List of scancodes to apply timeout to (movement-critical keys)
static const int timeout_scancodes[] = {
    79,  // SDL_SCANCODE_RIGHT
    80,  // SDL_SCANCODE_LEFT
    81,  // SDL_SCANCODE_DOWN
    82,  // SDL_SCANCODE_UP
    225, // SDL_SCANCODE_LSHIFT
    229, // SDL_SCANCODE_RSHIFT
    -1   // Sentinel
};

static void check_key_timeouts(void) {
    Uint32 now = (time_us_32() / 1000);
    
    for (int i = 0; timeout_scancodes[i] >= 0; i++) {
        int sc = timeout_scancodes[i];
        if (keyboard_state[sc] && key_press_time[sc] != 0) {
            Uint32 elapsed = now - key_press_time[sc];
            // Only timeout if the key has been held for a while AND neither
            // PS/2 nor USB HID driver thinks it's pressed
            int key_still_pressed = ps2kbd_is_key_pressed(sc);
            #ifdef USB_HID_ENABLED
            key_still_pressed = key_still_pressed || usbhid_sdl_is_key_pressed(sc);
            #endif
            if (elapsed > KEY_TIMEOUT_MS && !key_still_pressed) {
                // Generate a synthetic release event
                if (pending_event_count < 32) {
                    keyboard_state[sc] = 0;
                    key_press_time[sc] = 0;
                    
                    SDL_Event* ev = &pending_events[pending_event_count++];
                    memset(ev, 0, sizeof(*ev));
                    ev->type = SDL_KEYUP;
                    ev->key.keysym.scancode = sc;
                    ev->key.keysym.sym = sc;
                    ev->key.keysym.mod = 0;
                    ev->key.state = 0;
                    ev->key.repeat = 0;
                }
            }
        }
    }
}

int SDL_PollEvent(SDL_Event *event) {
    // Pump audio buffers on every event poll
    #if RP_SDL_FEATURE_AUDIO
    audio_i2s_driver_pump();
    #endif
    
    // Poll PS/2 keyboard - process ALL pending PS/2 scancodes
    ps2kbd_tick();
    
    // Poll USB HID keyboard
    #ifdef USB_HID_ENABLED
    usbhid_sdl_tick();
    #endif
    
    // If we have no buffered events, drain all events from PS/2 and USB queues
    if (pending_event_index >= pending_event_count) {
        pending_event_count = 0;
        pending_event_index = 0;
        
        Uint32 now = (time_us_32() / 1000);
        int pressed, scancode, modifier;
        
        // Drain PS/2 events
        while (pending_event_count < 32 && ps2kbd_get_key(&pressed, &scancode, &modifier)) {
            // Update keyboard state immediately
            if (scancode >= 0 && scancode < SDL_NUM_SCANCODES) {
                keyboard_state[scancode] = pressed ? 1 : 0;
                // Track press time for timeout
                key_press_time[scancode] = pressed ? now : 0;
            }
            
            // Buffer the event
            SDL_Event* ev = &pending_events[pending_event_count++];
            memset(ev, 0, sizeof(*ev));
            ev->type = pressed ? SDL_KEYDOWN : SDL_KEYUP;
            ev->key.keysym.scancode = scancode;
            ev->key.keysym.sym = scancode;
            ev->key.keysym.mod = modifier;
            ev->key.state = pressed ? 1 : 0;
            ev->key.repeat = 0;
        }
        
        // Drain USB HID events
        #ifdef USB_HID_ENABLED
        while (pending_event_count < 32 && usbhid_sdl_get_key(&pressed, &scancode, &modifier)) {
            // Update keyboard state immediately
            if (scancode >= 0 && scancode < SDL_NUM_SCANCODES) {
                keyboard_state[scancode] = pressed ? 1 : 0;
                // Track press time for timeout
                key_press_time[scancode] = pressed ? now : 0;
            }
            
            // Buffer the event
            SDL_Event* ev = &pending_events[pending_event_count++];
            memset(ev, 0, sizeof(*ev));
            ev->type = pressed ? SDL_KEYDOWN : SDL_KEYUP;
            ev->key.keysym.scancode = scancode;
            ev->key.keysym.sym = scancode;
            ev->key.keysym.mod = modifier;
            ev->key.state = pressed ? 1 : 0;
            ev->key.repeat = 0;
        }
        #endif
        
        // Check for stuck keys and auto-release them
        check_key_timeouts();
    }
    
    // Return next buffered event
    if (pending_event_index < pending_event_count) {
        if (event) {
            *event = pending_events[pending_event_index];
        }
        pending_event_index++;
        return 1;
    }
    
    return 0;
}

const Uint8 *SDL_GetKeyboardState(int *numkeys) {
    if (numkeys) *numkeys = SDL_NUM_SCANCODES;
    return keyboard_state;
}

// -----------------------------------------------------------------------------
// Windowing-related APIs (single fixed output; no real window system)
// -----------------------------------------------------------------------------
#if RP_SDL_FEATURE_WINDOW
void SDL_ShowCursor(int toggle) { (void)toggle; }
void SDL_SetWindowTitle(SDL_Window *window, const char *title) { (void)window; (void)title; }
void SDL_SetWindowIcon(SDL_Window *window, SDL_Surface *icon) { (void)window; (void)icon; }
#else
void SDL_ShowCursor(int toggle) { (void)toggle; }
void SDL_SetWindowTitle(SDL_Window *window, const char *title) { (void)window; (void)title; }
void SDL_SetWindowIcon(SDL_Window *window, SDL_Surface *icon) { (void)window; (void)icon; }
#endif

// -----------------------------------------------------------------------------
// Audio APIs - I2S output via PIO + DMA
// -----------------------------------------------------------------------------
#if RP_SDL_FEATURE_AUDIO

// Store the current audio spec for callback use
static SDL_AudioSpec g_audio_spec = {0};
static bool g_audio_initialized = false;
static bool g_audio_paused = true;

int SDL_OpenAudio(SDL_AudioSpec *desired, SDL_AudioSpec *obtained) {
    if (!desired || !desired->callback) {
        DBG_PRINTF("SDL_OpenAudio: invalid parameters\\n");
        return -1;
    }

    DBG_PRINTF("SDL_OpenAudio: freq=%d ch=%d samples=%d\\n",
           desired->freq, desired->channels, desired->samples);

    // Store the audio spec
    g_audio_spec = *desired;

    // Initialize the I2S audio driver
    if (!audio_i2s_driver_init(
            (uint32_t)desired->freq,
            desired->channels,
            (audio_callback_fn)desired->callback,
            desired->userdata)) {
        DBG_PRINTF("SDL_OpenAudio: audio_i2s_driver_init failed\\n");
        return -1;
    }

    // Fill in obtained spec if provided
    if (obtained) {
        *obtained = *desired;
        obtained->silence = audio_i2s_driver_get_silence();
        obtained->size = (Uint32)desired->samples * desired->channels * 2;  // 16-bit samples
    }

    g_audio_initialized = true;
    g_audio_paused = true;  // Start paused (SDL convention)

    DBG_PRINTF("SDL_OpenAudio: success\\n");
    return 0;
}

void SDL_PauseAudio(int pause_on) {
    if (!g_audio_initialized) return;

    if (pause_on) {
        if (!g_audio_paused) {
            DBG_PRINTF("SDL_PauseAudio: pausing\\n");
            audio_i2s_driver_set_enabled(false);
            g_audio_paused = true;
        }
    } else {
        if (g_audio_paused) {
            DBG_PRINTF("SDL_PauseAudio: unpausing\\n");
            audio_i2s_driver_set_enabled(true);
            g_audio_paused = false;
        }
    }
}

void SDL_CloseAudio(void) {
    if (!g_audio_initialized) return;
    DBG_PRINTF("SDL_CloseAudio\\n");
    audio_i2s_driver_shutdown();
    g_audio_initialized = false;
    g_audio_paused = true;
}

void SDL_LockAudio(void) {
    if (g_audio_initialized) {
        audio_i2s_driver_lock();
    }
}

void SDL_UnlockAudio(void) {
    if (g_audio_initialized) {
        audio_i2s_driver_unlock();
    }
}

// This function must be called regularly from the main loop to pump audio
void SDL_AudioPump(void) {
    if (g_audio_initialized && !g_audio_paused) {
        extern void audio_i2s_driver_pump(void);
        audio_i2s_driver_pump();
    }
}

#else
// Audio disabled - stub implementations
int SDL_OpenAudio(SDL_AudioSpec *desired, SDL_AudioSpec *obtained) {
    (void)desired;
    (void)obtained;
    // Fail: SDLPoP will mark digi unavailable and disable sound.
    return -1;
}
void SDL_PauseAudio(int pause_on) { (void)pause_on; }
void SDL_CloseAudio(void) {}
void SDL_LockAudio(void) {}
void SDL_UnlockAudio(void) {}
void SDL_AudioPump(void) {}
#endif

// RWops implementation for memory
size_t mem_read(SDL_RWops *context, void *ptr, size_t size, size_t maxnum) {
    size_t total_bytes = size * maxnum;
    size_t bytes_left = context->stop - context->here;
    
    if (total_bytes == 0) return 0;
    
    if (total_bytes > bytes_left) {
        total_bytes = bytes_left;
    }
    
    memcpy(ptr, context->here, total_bytes);
    context->here += total_bytes;
    
    return total_bytes / size;
}

size_t mem_write(SDL_RWops *context, const void *ptr, size_t size, size_t num) {
    return 0; // Read-only
}

Sint32 mem_seek(SDL_RWops *context, Sint32 offset, int whence) {
    const Uint8 *new_pos;
    
    switch (whence) {
        case RW_SEEK_SET:
            new_pos = context->base + offset;
            break;
        case RW_SEEK_CUR:
            new_pos = context->here + offset;
            break;
        case RW_SEEK_END:
            new_pos = context->stop + offset;
            break;
        default:
            return -1;
    }
    
    if (new_pos < context->base) new_pos = context->base;
    if (new_pos > context->stop) new_pos = context->stop;
    
    context->here = new_pos;
    return (Sint32)(context->here - context->base);
}

int mem_close(SDL_RWops *context) {
    if (context) {
        if (context->type == 2 && context->base) {
            // RWops created from SDL_RWFromFile owns its buffer.
            // Buffer may live in PSRAM (preferred) or SRAM.
            if (IS_PSRAM(context->base)) psram_free((void*)context->base);
            else free((void*)context->base);
        }
        free(context);
    }
    return 0;
}

Sint32 mem_tell(SDL_RWops *context) {
    return (Sint32)(context->here - context->base);
}

SDL_RWops *SDL_RWFromConstMem(const void *mem, int size) {
    SDL_RWops *rw = (SDL_RWops *)malloc(sizeof(SDL_RWops));
    if (!rw) return NULL;
    
    rw->base = (const Uint8 *)mem;
    rw->here = rw->base;
    rw->stop = rw->base + size;
    rw->type = 1;
    
    rw->read = mem_read;
    rw->write = mem_write;
    rw->seek = mem_seek;
    rw->close = mem_close;
    
    return rw;
}

SDL_RWops *SDL_RWFromMem(void *mem, int size) {
    return SDL_RWFromConstMem(mem, size);
}

SDL_RWops *SDL_RWFromFile(const char *file, const char *mode) {
    static bool printed = false;
    const bool do_print = (!printed);
    if (do_print) {
        printed = true;
        DBG_PRINTF("SDL_RWFromFile: file=%s mode=%s\\n", file ? file : "(null)", mode ? mode : "(null)");
    }

    FIL* fil = pop_fs_open(file, mode);
    if (!fil) {
        DBG_PRINTF("SDL_RWFromFile: open failed: %s\\n", file);
        return NULL;
    }

    size_t file_size = (size_t)f_size(fil);
    if (do_print) {
        DBG_PRINTF("SDL_RWFromFile: size=%u bytes\\n", (unsigned)file_size);
    }
    if (file_size == 0) {
        pop_fs_close(fil);
        return SDL_RWFromConstMem(NULL, 0);
    }

    // IMPORTANT: Avoid allocating file buffers in SRAM; pico_malloc is configured to panic on OOM.
    // Use the reusable PSRAM file buffer when possible.
    Uint8* buffer = (Uint8*)psram_get_file_buffer(file_size);
    if (!buffer) {
        // Fallback: allocate from PSRAM permanent pool (may leak; acceptable during bring-up).
        buffer = (Uint8*)psram_malloc(file_size);
    }
    if (!buffer) {
        if (do_print) {
            DBG_PRINTF("SDL_RWFromFile: buffer alloc failed (%u bytes)\\n", (unsigned)file_size);
        }
        pop_fs_close(fil);
        return NULL;
    }

    size_t got = pop_fs_read(buffer, 1, file_size, fil);
    pop_fs_close(fil);
    if (do_print) {
        DBG_PRINTF("SDL_RWFromFile: read got=%u\\n", (unsigned)got);
    }
    if (got != file_size) {
        if (IS_PSRAM(buffer)) psram_free(buffer);
        else free(buffer);
        return NULL;
    }

    SDL_RWops* rw = SDL_RWFromConstMem(buffer, (int)file_size);
    if (!rw) {
        free(buffer);
        return NULL;
    }
    // Mark as owning memory so close frees it.
    // Note: if the buffer came from the PSRAM reusable file buffer, psram_free() is a no-op.
    rw->type = 2;
    if (do_print) {
        DBG_PRINTF("SDL_RWFromFile: rw ok type=%d\\n", rw->type);
    }
    return rw;
}

size_t SDL_RWread(SDL_RWops *context, void *ptr, size_t size, size_t maxnum) {
    if (context && context->read) return context->read(context, ptr, size, maxnum);
    return 0;
}

size_t SDL_RWwrite(SDL_RWops *context, const void *ptr, size_t size, size_t num) {
    if (context && context->write) return context->write(context, ptr, size, num);
    return 0;
}

int SDL_RWclose(SDL_RWops *context) {
    if (context && context->close) return context->close(context);
    return 0;
}

Sint32 SDL_RWseek(SDL_RWops *context, Sint32 offset, int whence) {
    if (context && context->seek) return context->seek(context, offset, whence);
    return -1;
}

Sint32 SDL_RWtell(SDL_RWops *context) {
    if (context && (context->type == 1 || context->type == 2)) return mem_tell(context);
    return -1;
}

// SDL_image stubs
int IMG_Init(int flags) { return 0; }
void IMG_Quit(void) {}
SDL_Surface *IMG_Load(const char *file) {
    if (!file) {
        g_img_error = "IMG_Load: NULL filename";
        g_sdl_error = g_img_error;
        return NULL;
    }

    static bool printed = false;
    if (!printed) {
        printed = true;
        DBG_PRINTF("IMG_Load: %s\n", file);
    }

    SDL_RWops *rw = SDL_RWFromFile(file, "rb");
    if (!rw) {
        g_img_error = "IMG_Load: open failed";
        g_sdl_error = g_img_error;
        return NULL;
    }
    return IMG_Load_RW(rw, 1);
}
SDL_Surface *IMG_ReadXPMFromArray(char **xpm) { return NULL; }

// IMG_GetError/IMG_Load_RW are stubbed for now.

int IMG_SavePNG(SDL_Surface *surface, const char *file) {
    return -1;
}

// BlendMode stubs
int SDL_SetSurfaceBlendMode(SDL_Surface *surface, SDL_BlendMode blendMode) {
    if (!surface) return -1;
    surface->blendMode = blendMode;
    return 0;
}

Uint32 SDL_MapRGBA(const SDL_PixelFormat *format, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    if (format && format->palette) {
        SDL_Color c = {r, g, b, a};
        return find_best_palette_index(&c, format->palette);
    }
    // Pack according to the requested 32bpp format.
    if (format && format->format == SDL_PIXELFORMAT_RGBA8888) {
        // 0xAABBGGRR
        return ((Uint32)a << 24) | ((Uint32)b << 16) | ((Uint32)g << 8) | (Uint32)r;
    }
    // Default: ARGB8888 (0xAARRGGBB)
    return ((Uint32)a << 24) | ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
}

Uint32 SDL_MapRGB(const SDL_PixelFormat *format, Uint8 r, Uint8 g, Uint8 b) {
    if (format && format->palette) {
        SDL_Color c = {r, g, b, 255};
        return find_best_palette_index(&c, format->palette);
    }
    if (format && format->format == SDL_PIXELFORMAT_RGBA8888) {
        return (0xFFu << 24) | ((Uint32)b << 16) | ((Uint32)g << 8) | (Uint32)r;
    }
    return (0xFFu << 24) | ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
}

void SDL_RenderGetScale(SDL_Renderer *renderer, float *scale_x, float *scale_y) {
    if (scale_x) *scale_x = 1.0f;
    if (scale_y) *scale_y = 1.0f;
}

void SDL_RenderGetLogicalSize(SDL_Renderer *renderer, int *w, int *h) {
    if (w) *w = 320;
    if (h) *h = 200;
}

void SDL_RenderGetViewport(SDL_Renderer *renderer, SDL_Rect *rect) {
    if (rect) {
        rect->x = 0;
        rect->y = 0;
        rect->w = 320;
        rect->h = 200;
    }
}

Uint32 SDL_GetMouseState(int *x, int *y) {
    if (x) *x = 0;
    if (y) *y = 0;
    return 0;
}

int SDL_SetWindowFullscreen(SDL_Window *window, Uint32 flags) {
    (void)window;
    (void)flags;
    return 0;
}

const char *SDL_GetScancodeName(SDL_Scancode scancode) {
    return "Key";
}

Uint32 SDL_GetWindowFlags(SDL_Window *window) {
    (void)window;
    return 0;
}

Uint64 SDL_GetPerformanceCounter(void) {
    return time_us_64();
}

Uint64 SDL_GetPerformanceFrequency(void) {
    return 1000000;
}

Uint32 SDL_SwapBE32(Uint32 x) {
    return __builtin_bswap32(x);
}

Uint16 SDL_SwapBE16(Uint16 x) {
    return __builtin_bswap16(x);
}

int SDL_ShowSimpleMessageBox(Uint32 flags, const char *title, const char *message, SDL_Window *window) {
    (void)flags;
    (void)window;
    DBG_PRINTF("MessageBox: %s - %s\n", title ? title : "(null)", message ? message : "(null)");
#if RP_SDL_FEATURE_MESSAGEBOX
    // Blink LED rapidly to indicate error
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    for(int i=0; i<20; i++) {
        gpio_put(25, 1); sleep_ms(50);
        gpio_put(25, 0); sleep_ms(50);
    }
#endif
    return 0;
}

SDL_RWops *SDL_RWFromMem(void *mem, int size); // Forward declaration

void SDL_GetVersion(SDL_version *ver) {
    if (ver) {
        ver->major = 2;
        ver->minor = 0;
        ver->patch = 4;
    }
}

SDL_TimerID SDL_AddTimer(Uint32 interval, SDL_TimerCallback callback, void *param) {
    // TODO: Implement timer
    (void)interval;
    (void)callback;
    (void)param;
    return 0;
}

// -----------------------------------------------------------------------------
// Haptics (intentionally unsupported)
// -----------------------------------------------------------------------------
#if RP_SDL_FEATURE_HAPTIC
SDL_Haptic *SDL_HapticOpen(int device_index) { (void)device_index; return NULL; }
int SDL_HapticRumbleInit(SDL_Haptic *haptic) { (void)haptic; return -1; }
int SDL_HapticRumblePlay(SDL_Haptic *haptic, float strength, Uint32 length) {
    (void)haptic;
    (void)strength;
    (void)length;
    return -1;
}
#else
SDL_Haptic *SDL_HapticOpen(int device_index) { (void)device_index; return NULL; }
int SDL_HapticRumbleInit(SDL_Haptic *haptic) { (void)haptic; return -1; }
int SDL_HapticRumblePlay(SDL_Haptic *haptic, float strength, Uint32 length) {
    (void)haptic;
    (void)strength;
    (void)length;
    return -1;
}
#endif

SDL_Surface *SDL_ConvertSurface(SDL_Surface *src, const SDL_PixelFormat *fmt, Uint32 flags) {
    if (!src || !fmt) return NULL;
    
    SDL_Surface *new_surf = SDL_CreateRGBSurface(flags, src->w, src->h, fmt->BitsPerPixel, 0, 0, 0, 0);
    if (!new_surf) return NULL;

    // Copy palette if exists
    if (src->format->palette && new_surf->format->palette) {
        SDL_SetPaletteColors(new_surf->format->palette, src->format->palette->colors, 0, src->format->palette->ncolors);
    } else if (new_surf->format->palette && fmt->palette) {
         SDL_SetPaletteColors(new_surf->format->palette, fmt->palette->colors, 0, fmt->palette->ncolors);
    }

    // Conversion logic
    if (src->format->BitsPerPixel == 32 && fmt->BitsPerPixel == 8) {
        // 32-bit RGBA to 8-bit Indexed
        SDL_Palette *dst_pal = fmt->palette;
        if (!dst_pal && new_surf->format->palette) dst_pal = new_surf->format->palette;
        
        if (dst_pal) {
            Uint32 *src_pixels = (Uint32 *)src->pixels;
            Uint8 *dst_pixels = (Uint8 *)new_surf->pixels;
            for (int i = 0; i < src->w * src->h; i++) {
                Uint32 pixel = src_pixels[i];
                SDL_Color c;
                c.r = pixel & 0xFF;
                c.g = (pixel >> 8) & 0xFF;
                c.b = (pixel >> 16) & 0xFF;
                c.a = (pixel >> 24) & 0xFF;
                
                if (c.a < 128) {
                    dst_pixels[i] = 0; // Transparent? Assuming 0 is transparent
                } else {
                    dst_pixels[i] = find_best_palette_index(&c, dst_pal);
                }
            }
        }
    } else if (src->format->BitsPerPixel == fmt->BitsPerPixel) {
        // Same depth, just copy
        memcpy(new_surf->pixels, src->pixels, src->pitch * src->h);
    } else {
        // Fallback: clear to black
        memset(new_surf->pixels, 0, new_surf->pitch * new_surf->h);
    }

    return new_surf;
}

int SDL_SetSurfacePalette(SDL_Surface *surface, SDL_Palette *palette) {
    if (!surface || !palette) return -1;
    if (surface->format->palette) {
        // We should probably free the old palette if we owned it, but SDL memory management is tricky.
        // For now, let's just copy colors if sizes match, or replace the pointer?
        // SDL_SetSurfacePalette documentation says: "The palette is copied."
        SDL_SetPaletteColors(surface->format->palette, palette->colors, 0, palette->ncolors);
    }
    return 0;
}

int SDL_SetSurfaceAlphaMod(SDL_Surface *surface, Uint8 alpha) {
    if (!surface) return -1;
    surface->alphaMod = alpha;
    return 0;
}

int SDL_LockSurface(SDL_Surface *surface) {
    rp2350_fixup_onscreen_surface_format(surface, "SDL_LockSurface");
    return 0;
}
void SDL_UnlockSurface(SDL_Surface *surface) {}
// SDL_RWFromConstMem is implemented above

const char *IMG_GetError(void) {
    return g_img_error;
}

static void rp_sdl_unpremultiply_rgba_inplace(unsigned char *rgba, size_t pixel_count) {
    if (!rgba || pixel_count == 0) return;

    for (size_t i = 0; i < pixel_count; ++i) {
        unsigned char *p = rgba + i * 4u;
        const unsigned a = (unsigned)p[3];
        if (a == 0u || a == 255u) continue;

        // Unpremultiply with rounding; clamp to [0,255].
        const unsigned r = (unsigned)p[0];
        const unsigned g = (unsigned)p[1];
        const unsigned b = (unsigned)p[2];
        unsigned ur = (r * 255u + (a / 2u)) / a;
        unsigned ug = (g * 255u + (a / 2u)) / a;
        unsigned ub = (b * 255u + (a / 2u)) / a;
        if (ur > 255u) ur = 255u;
        if (ug > 255u) ug = 255u;
        if (ub > 255u) ub = 255u;
        p[0] = (unsigned char)ur;
        p[1] = (unsigned char)ug;
        p[2] = (unsigned char)ub;
    }
}

static bool rp_sdl_should_unpremultiply_rgba(const unsigned char *rgba, size_t pixel_count) {
    if (!rgba || pixel_count == 0) return false;

    // Heuristic: in premultiplied alpha, for semi-transparent pixels we typically
    // see r,g,b <= a. In straight alpha, channels often exceed a.
    const size_t max_samples = 2048u;
    const size_t step = (pixel_count > max_samples) ? (pixel_count / max_samples) : 1u;

    unsigned semi = 0;
    unsigned premul_like = 0;

    for (size_t i = 0; i < pixel_count; i += step) {
        const unsigned char *p = rgba + i * 4u;
        const unsigned a = (unsigned)p[3];
        if (a == 0u || a == 255u) continue;
        ++semi;

        const unsigned r = (unsigned)p[0];
        const unsigned g = (unsigned)p[1];
        const unsigned b = (unsigned)p[2];
        if (r <= a && g <= a && b <= a) ++premul_like;
        if (semi >= 256u) break; // plenty for a decision
    }

    if (semi < 32u) return false;
    // If >95% of sampled semi-transparent pixels look premultiplied, treat as such.
    return (premul_like * 100u) >= (semi * 95u);
}

SDL_Surface *IMG_Load_RW(SDL_RWops *src, int freesrc) {
    if (!src) {
        g_img_error = "IMG_Load_RW: NULL src";
        g_sdl_error = g_img_error;
        return NULL;
    }

    static bool printed = false;
    const bool do_print = (!printed);
    if (do_print) {
        printed = true;
        DBG_PRINTF("IMG_Load_RW: enter type=%d\n", src->type);
    }

    // We primarily use memory-backed RWops on this platform.
    if (!(src->type == 1 || src->type == 2) || !src->base || !src->stop || src->stop < src->base) {
        g_img_error = "IMG_Load_RW: unsupported RWops";
        g_sdl_error = g_img_error;
        if (freesrc) SDL_RWclose(src);
        return NULL;
    }

    const unsigned char *png_bytes = (const unsigned char *)src->base;
    const int png_size = (int)(src->stop - src->base);
    if (png_size <= 0) {
        g_img_error = "IMG_Load_RW: empty input";
        g_sdl_error = g_img_error;
        if (freesrc) SDL_RWclose(src);
        return NULL;
    }

    if (do_print) {
        DBG_PRINTF("IMG_Load_RW: png_size=%d\n", png_size);
    }

    int w = 0, h = 0, comp_in_file = 0;
    if (do_print) {
        DBG_PRINTF("IMG_Load_RW: stbi_load_from_memory...\n");
    }

    // Decode PNG into a temporary PSRAM arena. stb_image allocates a large RGBA
    // output buffer (e.g. 320x200x4 = 256000 bytes) which must not hit the Pico
    // SDK heap (can panic on OOM). We reclaim all temp allocations by restoring
    // the temp offset once we've copied pixels into our surface.
    const size_t temp_mark = psram_get_temp_offset();
    psram_set_temp_mode(1);

    unsigned char *rgba = stbi_load_from_memory(png_bytes, png_size, &w, &h, &comp_in_file, 4);

    psram_set_temp_mode(0);
    if (!rgba) {
        const char *why = stbi_failure_reason();
        g_img_error = why ? why : "stbi_load_from_memory failed";
        g_sdl_error = g_img_error;
        psram_set_temp_offset(temp_mark);
        if (freesrc) SDL_RWclose(src);
        return NULL;
    }

    if (do_print) {
        DBG_PRINTF("IMG_Load_RW: decoded %dx%d comp=%d\n", w, h, comp_in_file);
    }

    // Fix black/dark fringes on assets with premultiplied alpha.
    const size_t pixel_count = (size_t)w * (size_t)h;
#if RP_SDL_STBI_UNPREMULTIPLY_ALPHA_MODE == 2
    rp_sdl_unpremultiply_rgba_inplace(rgba, pixel_count);
#elif RP_SDL_STBI_UNPREMULTIPLY_ALPHA_MODE == 1
    if (rp_sdl_should_unpremultiply_rgba(rgba, pixel_count)) {
        static bool printed_unpremul = false;
        if (do_print || !printed_unpremul) {
            printed_unpremul = true;
            DBG_PRINTF("IMG_Load_RW: unpremultiply alpha enabled for this image\n");
        }
        rp_sdl_unpremultiply_rgba_inplace(rgba, pixel_count);
    }
#endif

    SDL_Surface *surf = SDL_CreateRGBSurface(0, w, h, 32, 0, 0, 0, 0);
    if (!surf || !surf->pixels) {
        stbi_image_free(rgba);
        g_img_error = "IMG_Load_RW: SDL_CreateRGBSurface failed";
        g_sdl_error = g_img_error;
        psram_set_temp_offset(temp_mark);
        if (freesrc) SDL_RWclose(src);
        return NULL;
    }

    if (do_print) {
        DBG_PRINTF("IMG_Load_RW: surface ok pitch=%d\n", surf->pitch);
    }

    // stb_image returns tightly packed RGBA, top-to-bottom, left-to-right.
    memcpy(surf->pixels, rgba, (size_t)w * (size_t)h * 4u);
    stbi_image_free(rgba);
    psram_set_temp_offset(temp_mark);

    if (do_print) {
        DBG_PRINTF("IMG_Load_RW: done\n");
    }

    if (freesrc) SDL_RWclose(src);
    g_img_error = "";
    g_sdl_error = "";
    return surf;
}

// -----------------------------------------------------------------------------
// Joystick / GameController (intentionally unsupported)
// -----------------------------------------------------------------------------
#if RP_SDL_FEATURE_JOYSTICK
int SDL_NumJoysticks(void) { return 0; }
SDL_Joystick *SDL_JoystickOpen(int device_index) { (void)device_index; return NULL; }
#else
int SDL_NumJoysticks(void) { return 0; }
SDL_Joystick *SDL_JoystickOpen(int device_index) { (void)device_index; return NULL; }
#endif

#if RP_SDL_FEATURE_GAMECONTROLLER
int SDL_GameControllerAddMappingsFromFile(const char *file) { (void)file; return 0; }
SDL_bool SDL_IsGameController(int joystick_index) { (void)joystick_index; return SDL_FALSE; }
SDL_GameController *SDL_GameControllerOpen(int joystick_index) { (void)joystick_index; return NULL; }
#else
int SDL_GameControllerAddMappingsFromFile(const char *file) { (void)file; return 0; }
SDL_bool SDL_IsGameController(int joystick_index) { (void)joystick_index; return SDL_FALSE; }
SDL_GameController *SDL_GameControllerOpen(int joystick_index) { (void)joystick_index; return NULL; }
#endif
void SDL_SetTextInputRect(const SDL_Rect *rect) {}
void SDL_StartTextInput(void) {}
void SDL_StopTextInput(void) {}
int SDL_PushEvent(SDL_Event *event) { (void)event; return 0; }
void SDL_RenderSetLogicalSize(SDL_Renderer *renderer, int w, int h) { (void)renderer; (void)w; (void)h; }
int SDL_SetRenderTarget(SDL_Renderer *renderer, SDL_Texture *texture) { (void)renderer; (void)texture; return 0; }
int SDL_BlitScaled(SDL_Surface *src, const SDL_Rect *srcrect, SDL_Surface *dst, SDL_Rect *dstrect) { 
    return SDL_BlitSurface(src, srcrect, dst, dstrect); 
}
SDL_Surface *SDL_ConvertSurfaceFormat(SDL_Surface *src, Uint32 pixel_format, Uint32 flags) { 
    if (!src || !src->format) return NULL;

    // SDLPoP primarily uses this for creating an ARGB8888 copy of glyph/sprite
    // surfaces so it can recolor via direct pixel writes.
    if (pixel_format != SDL_PIXELFORMAT_ARGB8888) {
        // Minimal support: if format requested isn't handled, just make a same-bpp copy.
        SDL_Surface *copy = SDL_CreateRGBSurface(flags, src->w, src->h, src->format->BitsPerPixel, 0, 0, 0, 0);
        if (!copy) return NULL;
        if (src->format->palette && copy->format && copy->format->palette) {
            SDL_SetPaletteColors(copy->format->palette, src->format->palette->colors, 0, src->format->palette->ncolors);
        }
        memcpy(copy->pixels, src->pixels, (size_t)src->pitch * (size_t)src->h);
        copy->colorkey = src->colorkey;
        copy->use_colorkey = src->use_colorkey;
        return copy;
    }

    SDL_Surface *dst = SDL_CreateRGBSurface(flags, src->w, src->h, 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
    if (!dst || !dst->pixels) return NULL;

    const int w = src->w;
    const int h = src->h;
    const int src_bpp = src->format->BytesPerPixel;
    const Uint32 key = src->colorkey;
    const bool use_key = (src->use_colorkey == SDL_TRUE);
    SDL_Palette *src_pal = src->format->palette;

    for (int y = 0; y < h; ++y) {
        const Uint8 *s_row = (const Uint8 *)src->pixels + y * src->pitch;
        Uint32 *d_row = (Uint32 *)((Uint8 *)dst->pixels + y * dst->pitch);

        if (src_bpp == 1) {
            for (int x = 0; x < w; ++x) {
                const Uint8 idx = s_row[x];
                const bool transparent = use_key && (idx == (Uint8)(key & 0xFF));
                Uint8 r = idx, g = idx, b = idx;
                if (src_pal && idx < (Uint8)src_pal->ncolors) {
                    SDL_Color c = src_pal->colors[idx];
                    r = c.r; g = c.g; b = c.b;
                }
                const Uint8 a = transparent ? 0 : 255;
                d_row[x] = ((Uint32)a << 24) | ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
            }
        } else if (src_bpp == 4) {
            const Uint32 *s_row32 = (const Uint32 *)s_row;
            for (int x = 0; x < w; ++x) {
                Uint32 p = s_row32[x];
                const bool transparent = use_key && ((p & 0xFFFFFFu) == (key & 0xFFFFFFu));
                // Our codebase generally treats 32bpp as little-endian RGBA (stb_image).
                Uint8 r = (Uint8)(p & 0xFF);
                Uint8 g = (Uint8)((p >> 8) & 0xFF);
                Uint8 b = (Uint8)((p >> 16) & 0xFF);
                Uint8 a = (Uint8)((p >> 24) & 0xFF);
                if (transparent) a = 0;
                d_row[x] = ((Uint32)a << 24) | ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
            }
        } else if (src_bpp == 3) {
            for (int x = 0; x < w; ++x) {
                Uint8 r = s_row[x * 3 + 0];
                Uint8 g = s_row[x * 3 + 1];
                Uint8 b = s_row[x * 3 + 2];
                const Uint32 packed = ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
                const bool transparent = use_key && ((packed & 0xFFFFFFu) == (key & 0xFFFFFFu));
                const Uint8 a = transparent ? 0 : 255;
                d_row[x] = ((Uint32)a << 24) | ((Uint32)r << 16) | ((Uint32)g << 8) | (Uint32)b;
            }
        } else {
            // Unsupported source depth: clear.
            memset(d_row, 0, (size_t)w * sizeof(Uint32));
        }
    }

    // Preserve colorkey semantics on the new surface too.
    dst->use_colorkey = src->use_colorkey;
    dst->colorkey = src->colorkey;
    return dst;
}
SDL_bool SDL_ISPIXELFORMAT_INDEXED(Uint32 format) { 
    return (format == SDL_PIXELFORMAT_INDEX8) ? SDL_TRUE : SDL_FALSE;
}
SDL_GameController *SDL_GameControllerFromInstanceID(SDL_JoystickID joyid) { return NULL; }
void SDL_GameControllerClose(SDL_GameController *gamecontroller) { (void)gamecontroller; }
void SDL_SetClipRect(SDL_Surface *surface, const SDL_Rect *rect) {
    if (!surface) return;
    if (rect) {
        // Set clip rect, clamping to surface bounds
        surface->clip_rect.x = (rect->x < 0) ? 0 : rect->x;
        surface->clip_rect.y = (rect->y < 0) ? 0 : rect->y;
        int right = rect->x + rect->w;
        int bottom = rect->y + rect->h;
        if (right > surface->w) right = surface->w;
        if (bottom > surface->h) bottom = surface->h;
        surface->clip_rect.w = right - surface->clip_rect.x;
        surface->clip_rect.h = bottom - surface->clip_rect.y;
        if (surface->clip_rect.w < 0) surface->clip_rect.w = 0;
        if (surface->clip_rect.h < 0) surface->clip_rect.h = 0;
    } else {
        // Reset to full surface
        surface->clip_rect.x = 0;
        surface->clip_rect.y = 0;
        surface->clip_rect.w = surface->w;
        surface->clip_rect.h = surface->h;
    }
}

int chdir(const char *path) { (void)path; return 0; }
int mkdir(const char *path, int mode) { (void)path; (void)mode; return 0; }

void debug_blink(int count) {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    for (int i = 0; i < count; i++) {
        gpio_put(25, 1);
        sleep_ms(400); // Slower blink
        gpio_put(25, 0);
        sleep_ms(400);
    }
    sleep_ms(1000); // Longer pause between codes
}
