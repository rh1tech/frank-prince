#include "../src/board_config.h"
#include "HDMI.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdalign.h>
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/structs/bus_ctrl.h"

// Globals expected by the driver
int graphics_buffer_width = 320;
int graphics_buffer_height = 240;
int graphics_buffer_shift_x = 0;
int graphics_buffer_shift_y = 0;
enum graphics_mode_t hdmi_graphics_mode = GRAPHICSMODE_DEFAULT;

static uint8_t *graphics_buffer = NULL;

void graphics_set_buffer(uint8_t *buffer) {
    graphics_buffer = buffer;
}

uint8_t* graphics_get_buffer(void) {
    return graphics_buffer;
}

uint32_t graphics_get_width(void) {
    return graphics_buffer_width;
}

uint32_t graphics_get_height(void) {
    return graphics_buffer_height;
}

void graphics_set_res(int w, int h) {
    graphics_buffer_width = w;
    graphics_buffer_height = h;
}

void graphics_set_shift(int x, int y) {
    graphics_buffer_shift_x = x;
    graphics_buffer_shift_y = y;
}

uint8_t* get_line_buffer(int line) {
    if (!graphics_buffer) return NULL;
    if (line < 0 || line >= graphics_buffer_height) return NULL;
    return graphics_buffer + line * graphics_buffer_width;
}

static struct video_mode_t video_mode[] = {
    { // 640x480 60Hz
        .h_total = 524,
        .h_width = 480,
        .freq = 60,
        .vgaPxClk = 25175000
    }
};

struct video_mode_t graphics_get_video_mode(int mode) {
    return video_mode[0];
}

int get_video_mode() {
    return 0;
}

// Forward declaration - actual implementation is after variable definitions
static void vsync_swap_buffers(void);

void vsync_handler() {
    vsync_swap_buffers();
}

// --- New HDMI Driver Code ---

//PIO параметры
static uint offs_prg0 = 0;
static uint offs_prg1 = 0;

//SM
static int SM_video = -1;
static int SM_conv = -1;

//буфер  палитры 256 цветов в формате R8G8B8
static uint32_t palette[256];

// Fade support: store original palette and apply fade at output
static uint32_t palette_original[256];  // Original colors before fade
static uint8_t g_hdmi_fade_level = 0;   // 0 = full brightness, 64 = full black
static uint16_t g_hdmi_fade_rows = 0;   // Bitmask of rows to fade (0 = all rows)

// Loading mode: skip PSRAM access in IRQ handler, just output sync signals
// This prevents HDMI signal loss during heavy SD card/PSRAM operations
static volatile bool g_hdmi_loading_mode = false;

void graphics_set_loading_mode(bool enable) {
    g_hdmi_loading_mode = enable;
}

bool graphics_get_loading_mode(void) {
    return g_hdmi_loading_mode;
}

// NOTE: HDMI uses indices 240..243 as service/control codes.
// All other indices (including 244..255) are available for visible pixels.


#define SCREEN_WIDTH (320)
#define SCREEN_HEIGHT (240)

// #define HDMI_WIDTH 480 //480 Default
// #define HDMI_HEIGHT 644 //524 Default
// #define HDMI_HZ 52 //60 Default

//DMA каналы
//каналы работы с первичным графическим буфером
static int dma_chan_ctrl;
static int dma_chan;
//каналы работы с конвертацией палитры
static int dma_chan_pal_conv_ctrl;
static int dma_chan_pal_conv;

//DMA буферы
//основные строчные данные
static uint32_t* dma_lines[2] = { NULL,NULL };
static uint32_t* DMA_BUF_ADDR[2];

//ДМА палитра для конвертации - DOUBLE BUFFERED
//Each buffer: 1024 TMDS lookup entries + 200 line buffer entries = 1224
alignas(4096) uint32_t conv_color_a[1224];
alignas(4096) uint32_t conv_color_b[1224];

// Double buffer state: 0 = DMA reads from A, CPU writes to B
//                      1 = DMA reads from B, CPU writes to A
static volatile uint8_t conv_color_active = 0;  // Which buffer DMA is reading from
static volatile bool conv_color_dirty = false;  // Back buffer has been modified

// Pointers to current front/back buffers
static uint32_t* conv_color_front = conv_color_a;  // DMA reads from this
static uint32_t* conv_color_back = conv_color_b;   // CPU writes to this

//индекс, проверяющий зависание
static uint32_t irq_inx = 0;

// Forward declaration for pio_set_x
static void pio_set_x(PIO pio, const int sm, uint32_t v);

// Counter for buffer swaps (diagnostics)
static volatile uint32_t conv_color_swap_count = 0;

uint32_t graphics_get_buffer_swap_count(void) {
    return conv_color_swap_count;
}

// Double buffer swap - called from vsync_handler during vertical blanking
// NOTE: We can't safely update PIO X register while it's running, so instead
// we copy back buffer to front buffer during vblank
static void vsync_swap_buffers(void) {
    // Apply pending palette changes from back buffer to front buffer
    // This is safe because we're copying TO the active buffer, and the copy
    // happens during vblank when there's less contention
    if (conv_color_dirty) {
        // Copy the TMDS lookup table portion (first 1024 entries = 4096 bytes)
        // Line buffers (1024-1223) are written fresh each scanline, no need to copy
        memcpy(conv_color_front, conv_color_back, 1024 * sizeof(uint32_t));
        
        // Mark back buffer as clean
        conv_color_dirty = false;
        conv_color_swap_count++;
    }
}

//функции и константы HDMI

#define BASE_HDMI_CTRL_INX (240)
#define HDMI_CTRL_COUNT (4)
//программа конвертации адреса

uint16_t pio_program_instructions_conv_HDMI[] = {
    //         //     .wrap_target
    0x80a0, //  0: pull   block
    0x40e8, //  1: in     osr, 8
    0x4034, //  2: in     x, 20
    0x8020, //  3: push   block
    //     .wrap
};


const struct pio_program pio_program_conv_addr_HDMI = {
    .instructions = pio_program_instructions_conv_HDMI,
    .length = 4,
    .origin = -1,
};

//программа видеовывода
static const uint16_t instructions_PIO_HDMI[] = {
    0x7006, //  0: out    pins, 6         side 2
    0x7006, //  1: out    pins, 6         side 2
    0x7006, //  2: out    pins, 6         side 2
    0x7006, //  3: out    pins, 6         side 2
    0x7006, //  4: out    pins, 6         side 2
    0x6806, //  5: out    pins, 6         side 1
    0x6806, //  6: out    pins, 6         side 1
    0x6806, //  7: out    pins, 6         side 1
    0x6806, //  8: out    pins, 6         side 1
    0x6806, //  9: out    pins, 6         side 1
};

static const struct pio_program program_PIO_HDMI = {
    .instructions = instructions_PIO_HDMI,
    .length = 10,
    .origin = -1,
};

static uint64_t get_ser_diff_data(const uint16_t dataR, const uint16_t dataG, const uint16_t dataB) {
    uint64_t out64 = 0;
    for (int i = 0; i < 10; i++) {
        out64 <<= 6;
        if (i == 5) out64 <<= 2;
#ifdef PICO_PC
        uint8_t bG = (dataR >> (9 - i)) & 1;
        uint8_t bR = (dataG >> (9 - i)) & 1;
#else
        uint8_t bR = (dataR >> (9 - i)) & 1;
        uint8_t bG = (dataG >> (9 - i)) & 1;
#endif
        uint8_t bB = (dataB >> (9 - i)) & 1;

        bR |= (bR ^ 1) << 1;
        bG |= (bG ^ 1) << 1;
        bB |= (bB ^ 1) << 1;

        if (HDMI_PIN_invert_diffpairs) {
            bR ^= 0b11;
            bG ^= 0b11;
            bB ^= 0b11;
        }
        uint8_t d6;
        if (HDMI_PIN_RGB_notBGR) {
            d6 = (bR << 4) | (bG << 2) | (bB << 0);
        }
        else {
            d6 = (bB << 4) | (bG << 2) | (bR << 0);
        }


        out64 |= d6;
    }
    return out64;
}

//конвертор TMDS
static uint tmds_encoder(const uint8_t d8) {
    int s1 = 0;
    for (int i = 0; i < 8; i++) s1 += (d8 & (1 << i)) ? 1 : 0;
    bool is_xnor = false;
    if ((s1 > 4) || ((s1 == 4) && ((d8 & 1) == 0))) is_xnor = true;
    uint16_t d_out = d8 & 1;
    uint16_t qi = d_out;
    for (int i = 1; i < 8; i++) {
        d_out |= ((qi << 1) ^ (d8 & (1 << i))) ^ (is_xnor << i);
        qi = d_out & (1 << i);
    }

    if (is_xnor) d_out |= 1 << 9;
    else d_out |= 1 << 8;

    return d_out;
}

static void pio_set_x(PIO pio, const int sm, uint32_t v) {
    uint instr_shift = pio_encode_in(pio_x, 4);
    uint instr_mov = pio_encode_mov(pio_x, pio_isr);
    for (int i = 0; i < 8; i++) {
        const uint32_t nibble = (v >> (i * 4)) & 0xf;
        pio_sm_exec(pio, sm, pio_encode_set(pio_x, nibble));
        pio_sm_exec(pio, sm, instr_shift);
    }
    pio_sm_exec(pio, sm, instr_mov);
}

volatile uint32_t hdmi_irq_count = 0;
volatile uint32_t hdmi_fifo_underrun_count = 0;

uint32_t graphics_get_hdmi_irq_count(void) {
    return hdmi_irq_count;
}

uint32_t graphics_get_hdmi_underrun_count(void) {
    return hdmi_fifo_underrun_count;
}

static void dma_handler_HDMI() {
    hdmi_irq_count++;
    
    // Check for PIO TX FIFO underrun (stall bit indicates FIFO was empty when PIO tried to pull)
    // If the FIFO is empty now and the PIO is stalled, we had an underrun
    if (pio_sm_is_tx_fifo_empty(PIO_VIDEO, SM_video)) {
        hdmi_fifo_underrun_count++;
    }
    
    static uint32_t inx_buf_dma;
    static uint line = 0;
    struct video_mode_t mode = graphics_get_video_mode(get_video_mode());
    irq_inx++;

    dma_hw->ints0 = 1u << dma_chan_ctrl;
    dma_channel_set_read_addr(dma_chan_ctrl, &DMA_BUF_ADDR[inx_buf_dma & 1], false);

    if (line >= mode.h_total ) {
        line = 0;
        vsync_handler();
    } else {
        ++line;
    }

    if ((line & 1) == 0) return;
    inx_buf_dma++;

    uint8_t* activ_buf = (uint8_t *)dma_lines[inx_buf_dma & 1];

    if (line < mode.h_width ) {
        uint8_t* output_buffer = activ_buf + 72; //для выравнивания синхры;
        
        // LOADING MODE: Skip PSRAM access entirely, just output black with valid sync.
        // This prevents HDMI signal loss during heavy SD card/PSRAM operations.
        if (g_hdmi_loading_mode) {
            memset(output_buffer, 255, SCREEN_WIDTH);  // 255 = black
        } else {
            int y = line >> 1;
            //область изображения
            uint8_t* input_buffer = get_line_buffer(y);
            if (!input_buffer) {
                // If no buffer, fill with black (255)
                memset(output_buffer, 255, SCREEN_WIDTH);
            } else {
                switch (hdmi_graphics_mode) {
                    case GRAPHICSMODE_DEFAULT:
                        //заполняем пространство сверху и снизу графического буфера
                        if (false || (graphics_buffer_shift_y > y) || (y >= (graphics_buffer_shift_y + graphics_buffer_height))
                            || (graphics_buffer_shift_x >= SCREEN_WIDTH) || (
                                (graphics_buffer_shift_x + graphics_buffer_width) < 0)) {
                            memset(output_buffer, 255, SCREEN_WIDTH);
                            break;
                        }

                        uint8_t* activ_buf_end = output_buffer + SCREEN_WIDTH;
                        //рисуем пространство слева от буфера
                        for (int i = graphics_buffer_shift_x; i-- > 0;) {
                            *output_buffer++ = 255;
                        }

                        //рисуем сам видеобуфер+пространство справа
                        const uint8_t* input_buffer_end = input_buffer + graphics_buffer_width;
                        if (graphics_buffer_shift_x < 0) input_buffer -= graphics_buffer_shift_x;
                        register size_t x = 0;
                        while (activ_buf_end > output_buffer) {
                            if (input_buffer < input_buffer_end) {
                                register uint8_t c = input_buffer[x++];
                                *output_buffer++ = (c >= BASE_HDMI_CTRL_INX && c < (BASE_HDMI_CTRL_INX + HDMI_CTRL_COUNT)) ? 255 : c;
                            }
                            else {
                                *output_buffer++ = 255;
                            }
                        }
                        break;
                    default:
                        for (int i = SCREEN_WIDTH; i--;) {
                            uint8_t i_color = *input_buffer++;
                            if (i_color >= BASE_HDMI_CTRL_INX && i_color < (BASE_HDMI_CTRL_INX + HDMI_CTRL_COUNT)) i_color = 255;
                            *output_buffer++ = i_color;
                        }
                        break;
                }
            }
        }


        // memset(activ_buf,2,320);//test

        //ССИ
        //для выравнивания синхры

        // --|_|---|_|---|_|----
        //---|___________|-----
        memset(activ_buf + 48,BASE_HDMI_CTRL_INX, 24);
        memset(activ_buf,BASE_HDMI_CTRL_INX + 1, 48);
        memset(activ_buf + 392,BASE_HDMI_CTRL_INX, 8);

        //без выравнивания
        // --|_|---|_|---|_|----
        //------|___________|----
        //   memset(activ_buf+320,BASE_HDMI_CTRL_INX,8);
        //   memset(activ_buf+328,BASE_HDMI_CTRL_INX+1,48);
        //   memset(activ_buf+376,BASE_HDMI_CTRL_INX,24);
    }
    else {
        if ((line >= 490) && (line < 492)) {
            //кадровый синхроимпульс
            //для выравнивания синхры
            // --|_|---|_|---|_|----
            //---|___________|-----
            memset(activ_buf + 48,BASE_HDMI_CTRL_INX + 2, 352);
            memset(activ_buf,BASE_HDMI_CTRL_INX + 3, 48);
            //без выравнивания
            // --|_|---|_|---|_|----
            //-------|___________|----

            // memset(activ_buf,BASE_HDMI_CTRL_INX+2,328);
            // memset(activ_buf+328,BASE_HDMI_CTRL_INX+3,48);
            // memset(activ_buf+376,BASE_HDMI_CTRL_INX+2,24);
        }
        else {
            //ССИ без изображения
            //для выравнивания синхры

            memset(activ_buf + 48,BASE_HDMI_CTRL_INX, 352);
            memset(activ_buf,BASE_HDMI_CTRL_INX + 1, 48);

            // memset(activ_buf,BASE_HDMI_CTRL_INX,328);
            // memset(activ_buf+328,BASE_HDMI_CTRL_INX+1,48);
            // memset(activ_buf+376,BASE_HDMI_CTRL_INX,24);
        };
    }


    // y=(y==524)?0:(y+1);
    // inx_buf_dma++;
}


static inline void irq_remove_handler_DMA_core1() {
    irq_set_enabled(VIDEO_DMA_IRQ, false);
    irq_remove_handler(VIDEO_DMA_IRQ, irq_get_exclusive_handler(VIDEO_DMA_IRQ));
}

static inline void irq_set_exclusive_handler_DMA_core1() {
    irq_set_exclusive_handler(VIDEO_DMA_IRQ, dma_handler_HDMI);
    irq_set_priority(VIDEO_DMA_IRQ, 0);
    irq_set_enabled(VIDEO_DMA_IRQ, true);
}

void graphics_set_palette_hdmi(const uint8_t i, const uint32_t color888);

//деинициализация - инициализация ресурсов
static inline bool hdmi_init() {
    //выключение прерывания DMA
    if (VIDEO_DMA_IRQ == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(dma_chan_ctrl, false);
    }
    else {
        dma_channel_set_irq1_enabled(dma_chan_ctrl, false);
    }

    irq_remove_handler_DMA_core1();


    //остановка всех каналов DMA
    dma_hw->abort = (1 << dma_chan_ctrl) | (1 << dma_chan) | (1 << dma_chan_pal_conv) | (
                        1 << dma_chan_pal_conv_ctrl);
    while (dma_hw->abort) tight_loop_contents();

    //выключение SM основной и конвертора

#if ZERO2
    pio_set_gpio_base(PIO_VIDEO, 16);
    pio_set_gpio_base(PIO_VIDEO_ADDR, 16);
#endif

    // pio_sm_restart(PIO_VIDEO, SM_video);
    pio_sm_set_enabled(PIO_VIDEO, SM_video, false);

    //pio_sm_restart(PIO_VIDEO_ADDR, SM_conv);
    pio_sm_set_enabled(PIO_VIDEO_ADDR, SM_conv, false);


    //удаление программ из соответствующих PIO
    pio_remove_program(PIO_VIDEO_ADDR, &pio_program_conv_addr_HDMI, offs_prg1);
    pio_remove_program(PIO_VIDEO, &program_PIO_HDMI, offs_prg0);


    offs_prg1 = pio_add_program(PIO_VIDEO_ADDR, &pio_program_conv_addr_HDMI);
    offs_prg0 = pio_add_program(PIO_VIDEO, &program_PIO_HDMI);
    // PIO X register holds base address for palette lookup (front buffer)
    pio_set_x(PIO_VIDEO_ADDR, SM_conv, ((uint32_t)conv_color_front >> 12));

    //заполнение палитры
    // NOTE: only 240..243 are reserved for HDMI control/sync codes.
    // Program all other indices, including 244..255.
    for (int ci = 0; ci < 256; ci++) {
        if (ci >= BASE_HDMI_CTRL_INX && ci < (BASE_HDMI_CTRL_INX + HDMI_CTRL_COUNT)) continue;
        graphics_set_palette_hdmi((uint8_t)ci, palette[ci]);
    }

    //240-243 служебные данные(синхра) напрямую вносим в массив -конвертер
    // Initialize sync colors in BOTH buffers
    uint64_t* conv_color64_a = (uint64_t *)conv_color_a;
    const uint16_t b0 = 0b1101010100;
    const uint16_t b1 = 0b0010101011;
    const uint16_t b2 = 0b0101010100;
    const uint16_t b3 = 0b1010101011;
    const int base_inx = BASE_HDMI_CTRL_INX;

    // Initialize sync colors in buffer A
    conv_color64_a[2 * base_inx + 0] = get_ser_diff_data(b0, b0, b3);
    conv_color64_a[2 * base_inx + 1] = get_ser_diff_data(b0, b0, b3);
    conv_color64_a[2 * (base_inx + 1) + 0] = get_ser_diff_data(b0, b0, b2);
    conv_color64_a[2 * (base_inx + 1) + 1] = get_ser_diff_data(b0, b0, b2);
    conv_color64_a[2 * (base_inx + 2) + 0] = get_ser_diff_data(b0, b0, b1);
    conv_color64_a[2 * (base_inx + 2) + 1] = get_ser_diff_data(b0, b0, b1);
    conv_color64_a[2 * (base_inx + 3) + 0] = get_ser_diff_data(b0, b0, b0);
    conv_color64_a[2 * (base_inx + 3) + 1] = get_ser_diff_data(b0, b0, b0);
    
    // Initialize sync colors in buffer B
    uint64_t* conv_color64_b = (uint64_t *)conv_color_b;
    conv_color64_b[2 * base_inx + 0] = get_ser_diff_data(b0, b0, b3);
    conv_color64_b[2 * base_inx + 1] = get_ser_diff_data(b0, b0, b3);
    conv_color64_b[2 * (base_inx + 1) + 0] = get_ser_diff_data(b0, b0, b2);
    conv_color64_b[2 * (base_inx + 1) + 1] = get_ser_diff_data(b0, b0, b2);
    conv_color64_b[2 * (base_inx + 2) + 0] = get_ser_diff_data(b0, b0, b1);
    conv_color64_b[2 * (base_inx + 2) + 1] = get_ser_diff_data(b0, b0, b1);
    conv_color64_b[2 * (base_inx + 3) + 0] = get_ser_diff_data(b0, b0, b0);
    conv_color64_b[2 * (base_inx + 3) + 1] = get_ser_diff_data(b0, b0, b0);

    //настройка PIO SM для конвертации

    pio_sm_config c_c = pio_get_default_sm_config();
    sm_config_set_wrap(&c_c, offs_prg1, offs_prg1 + (pio_program_conv_addr_HDMI.length - 1));
    sm_config_set_in_shift(&c_c, true, false, 32);

    pio_sm_init(PIO_VIDEO_ADDR, SM_conv, offs_prg1, &c_c);
    pio_sm_set_enabled(PIO_VIDEO_ADDR, SM_conv, true);

    //настройка PIO SM для вывода данных
    c_c = pio_get_default_sm_config();
    sm_config_set_wrap(&c_c, offs_prg0, offs_prg0 + (program_PIO_HDMI.length - 1));

    //настройка side set
    sm_config_set_sideset_pins(&c_c,beginHDMI_PIN_clk);
    sm_config_set_sideset(&c_c, 2,false,false);
    for (int i = 0; i < 2; i++) {
        pio_gpio_init(PIO_VIDEO, beginHDMI_PIN_clk + i);
        gpio_set_drive_strength(beginHDMI_PIN_clk + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(beginHDMI_PIN_clk + i, GPIO_SLEW_RATE_FAST);
    }

#if ZERO2
    // Настройка направлений пинов для state machines
    pio_sm_set_consecutive_pindirs(PIO_VIDEO, SM_video, HDMI_BASE_PIN, 8, true);
    pio_sm_set_consecutive_pindirs(PIO_VIDEO_ADDR, SM_conv, HDMI_BASE_PIN, 8, true);

    uint64_t mask64 = (uint64_t)(3u << beginHDMI_PIN_clk);
    pio_sm_set_pins_with_mask64(PIO_VIDEO, SM_video, mask64, mask64);
    pio_sm_set_pindirs_with_mask64(PIO_VIDEO, SM_video, mask64, mask64);
    // пины
#else
    pio_sm_set_pins_with_mask(PIO_VIDEO, SM_video, 3u << beginHDMI_PIN_clk, 3u << beginHDMI_PIN_clk);
    pio_sm_set_pindirs_with_mask(PIO_VIDEO, SM_video, 3u << beginHDMI_PIN_clk, 3u << beginHDMI_PIN_clk);
    // пины
#endif

    for (int i = 0; i < 6; i++) {
        gpio_set_slew_rate(beginHDMI_PIN_data + i, GPIO_SLEW_RATE_FAST);
        pio_gpio_init(PIO_VIDEO, beginHDMI_PIN_data + i);
        gpio_set_drive_strength(beginHDMI_PIN_data + i, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(beginHDMI_PIN_data + i, GPIO_SLEW_RATE_FAST);
    }
    pio_sm_set_consecutive_pindirs(PIO_VIDEO, SM_video, beginHDMI_PIN_data, 6, true);
    //конфигурация пинов на выход
    sm_config_set_out_pins(&c_c, beginHDMI_PIN_data, 6);

    //
    sm_config_set_out_shift(&c_c, true, true, 30);
    sm_config_set_fifo_join(&c_c, PIO_FIFO_JOIN_TX);

    int hdmi_hz = graphics_get_video_mode(get_video_mode()).freq;
    const float clk_ratio = (clock_get_hz(clk_sys) / 252000000.0f) * (60.0f / (float)hdmi_hz);
    sm_config_set_clkdiv(&c_c, clk_ratio);
    // Diagnostics: fractional PIO clkdiv can introduce jitter and visible vertical artifacts.
    // pop2350 runs clk_sys=252MHz, producing an integer divider of 1.0.
#if FRANK_PRINCE_DEBUG
    {
        const float frac = clk_ratio - (float)((int)clk_ratio);
        DBG_PRINTF("[HDMI] clk_sys=%lu Hz hdmi_hz=%d clkdiv=%.5f (frac=%.5f)\n",
               (unsigned long)clock_get_hz(clk_sys), hdmi_hz, clk_ratio, frac < 0 ? -frac : frac);
        if (frac > 0.0001f && frac < 0.9999f) {
            DBG_PRINTF("[HDMI] WARNING: fractional PIO clkdiv; try CPU_CLOCK_MHZ=252 or 504 for clean HDMI.\n");
        }
    }
#endif
    pio_sm_init(PIO_VIDEO, SM_video, offs_prg0, &c_c);
    pio_sm_set_enabled(PIO_VIDEO, SM_video, true);

    //настройки DMA
    // Line buffers point into the FRONT buffer (which DMA reads from)
    dma_lines[0] = &conv_color_front[1024];
    dma_lines[1] = &conv_color_front[1124];

    //основной рабочий канал
    dma_channel_config cfg_dma = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_8);
    channel_config_set_chain_to(&cfg_dma, dma_chan_ctrl); // chain to other channel
    channel_config_set_high_priority(&cfg_dma, true);  // High priority for HDMI

    channel_config_set_read_increment(&cfg_dma, true);
    channel_config_set_write_increment(&cfg_dma, false);


    uint dreq = DREQ_PIO1_TX0 + SM_conv;
    if (PIO_VIDEO_ADDR == pio0) dreq = DREQ_PIO0_TX0 + SM_conv;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan,
        &cfg_dma,
        &PIO_VIDEO_ADDR->txf[SM_conv], // Write address
        &dma_lines[0][0], // read address
        400, //
        false // Don't start yet
    );

    //контрольный канал для основного
    cfg_dma = dma_channel_get_default_config(dma_chan_ctrl);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan); // chain to other channel
    channel_config_set_high_priority(&cfg_dma, true);  // High priority for HDMI

    channel_config_set_read_increment(&cfg_dma, false);
    channel_config_set_write_increment(&cfg_dma, false);

    DMA_BUF_ADDR[0] = &dma_lines[0][0];
    DMA_BUF_ADDR[1] = &dma_lines[1][0];

    dma_channel_configure(
        dma_chan_ctrl,
        &cfg_dma,
        &dma_hw->ch[dma_chan].read_addr, // Write address
        &DMA_BUF_ADDR[0], // read address
        1, //
        false // Don't start yet
    );

    //канал - конвертер палитры

    cfg_dma = dma_channel_get_default_config(dma_chan_pal_conv);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan_pal_conv_ctrl); // chain to other channel
    channel_config_set_high_priority(&cfg_dma, true);  // High priority for HDMI

    channel_config_set_read_increment(&cfg_dma, true);
    channel_config_set_write_increment(&cfg_dma, false);

    dreq = DREQ_PIO1_TX0 + SM_video;
    if (PIO_VIDEO == pio0) dreq = DREQ_PIO0_TX0 + SM_video;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan_pal_conv,
        &cfg_dma,
        &PIO_VIDEO->txf[SM_video], // Write address
        &conv_color_front[0], // read address - palette lookup from FRONT buffer
        4, //
        false // Don't start yet
    );

    //канал управления конвертером палитры

    cfg_dma = dma_channel_get_default_config(dma_chan_pal_conv_ctrl);
    channel_config_set_transfer_data_size(&cfg_dma, DMA_SIZE_32);
    channel_config_set_chain_to(&cfg_dma, dma_chan_pal_conv); // chain to other channel

    channel_config_set_read_increment(&cfg_dma, false);
    channel_config_set_write_increment(&cfg_dma, false);

    dreq = DREQ_PIO1_RX0 + SM_conv;
    if (PIO_VIDEO_ADDR == pio0) dreq = DREQ_PIO0_RX0 + SM_conv;

    channel_config_set_dreq(&cfg_dma, dreq);

    dma_channel_configure(
        dma_chan_pal_conv_ctrl,
        &cfg_dma,
        &dma_hw->ch[dma_chan_pal_conv].read_addr, // Write address
        &PIO_VIDEO_ADDR->rxf[SM_conv], // read address
        1, //
        true // start yet
    );

    //стартуем прерывание и канал
    if (VIDEO_DMA_IRQ == DMA_IRQ_0) {
        dma_channel_acknowledge_irq0(dma_chan_ctrl);
        dma_channel_set_irq0_enabled(dma_chan_ctrl, true);
    }
    else {
        dma_channel_acknowledge_irq1(dma_chan_ctrl);
        dma_channel_set_irq1_enabled(dma_chan_ctrl, true);
    }

    // Set high priority for all video DMA channels AFTER configuration
    // (dma_channel_configure overwrites ctrl, so we must set priority here)
    // This prevents bus starvation during heavy PSRAM/SD access
    hw_set_bits(&dma_hw->ch[dma_chan].ctrl_trig, DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);
    hw_set_bits(&dma_hw->ch[dma_chan_ctrl].ctrl_trig, DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);
    hw_set_bits(&dma_hw->ch[dma_chan_pal_conv].ctrl_trig, DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);
    hw_set_bits(&dma_hw->ch[dma_chan_pal_conv_ctrl].ctrl_trig, DMA_CH0_CTRL_TRIG_HIGH_PRIORITY_BITS);

    irq_set_exclusive_handler_DMA_core1();

    dma_start_channel_mask((1u << dma_chan_ctrl));

    return true;
};

// Gamma-corrected fade table: maps (fade_level 0-63) to brightness multiplier (0-255)
// Using gamma 2.2 curve so dark colors fade in proportionally with bright colors
// Formula: 255 * pow((64 - fade_level) / 64.0, 1/2.2)
static const uint8_t fade_gamma_table[64] = {
    255, 253, 251, 249, 247, 245, 243, 241,  // 0-7
    239, 236, 234, 231, 229, 226, 223, 220,  // 8-15
    217, 214, 211, 208, 204, 201, 197, 193,  // 16-23
    189, 185, 181, 176, 172, 167, 162, 156,  // 24-31
    151, 145, 139, 133, 126, 119, 112, 104,  // 32-39
    96,  88,  79,  69,  59,  48,  36,  23,   // 40-47
    0,   0,   0,   0,   0,   0,   0,   0,    // 48-55 (fade to black faster)
    0,   0,   0,   0,   0,   0,   0,   0     // 56-63
};

// Apply fade to a color: reduce brightness based on fade level
// Uses gamma correction so dark and bright colors fade proportionally
static inline uint32_t apply_fade_to_color(uint32_t color888, uint8_t fade_level) {
    if (fade_level == 0) return color888;
    if (fade_level >= 48) return 0;  // Full black at level 48+
    
    uint8_t r = (color888 >> 16) & 0xff;
    uint8_t g = (color888 >> 8) & 0xff;
    uint8_t b = (color888 >> 0) & 0xff;
    
    // Use gamma-corrected lookup table for perceptually uniform fade
    uint16_t scale = fade_gamma_table[fade_level];
    r = (r * scale) >> 8;
    g = (g * scale) >> 8;
    b = (b * scale) >> 8;
    
    return (r << 16) | (g << 8) | b;
}

void graphics_set_palette_hdmi(uint8_t i, uint32_t color888) {
    // Store original color
    palette_original[i] = color888 & 0x00ffffff;
    
    // Apply fade if this row should be faded
    int row = i / 16;
    uint32_t faded_color = color888 & 0x00ffffff;
    if (g_hdmi_fade_level > 0) {
        // When at full black (>= 64), fade ALL rows regardless of mask
        bool should_fade = (g_hdmi_fade_level >= 64) || (g_hdmi_fade_rows == 0) || (g_hdmi_fade_rows & (1 << row));
        if (should_fade) {
            faded_color = apply_fade_to_color(faded_color, g_hdmi_fade_level);
        }
    }
    palette[i] = faded_color;

    // Don't program TMDS for the 4 HDMI control codes (they're set by graphics_restore_sync_colors).
    if (i >= BASE_HDMI_CTRL_INX && i < (BASE_HDMI_CTRL_INX + HDMI_CTRL_COUNT)) return;

    // Write to BACK buffer (double buffering) - DMA reads from front buffer
    uint64_t* conv_color64 = (uint64_t *)conv_color_back;
    const uint8_t R = (faded_color >> 16) & 0xff;
    const uint8_t G = (faded_color >> 8) & 0xff;
    const uint8_t B = (faded_color >> 0) & 0xff;
    conv_color64[i * 2] = get_ser_diff_data(tmds_encoder(R), tmds_encoder(G), tmds_encoder(B));
    conv_color64[i * 2 + 1] = conv_color64[i * 2] ^ 0x0003ffffffffffffl;
    
    // Mark back buffer as dirty - will be swapped at next vsync
    conv_color_dirty = true;
};

// Set fade level and refresh all palette entries
void graphics_set_fade_level(uint8_t fade_level, uint16_t which_rows) {
    g_hdmi_fade_level = fade_level;
    g_hdmi_fade_rows = which_rows;
    
    // Refresh all palette entries with new fade level
    for (int i = 0; i < 256; i++) {
        // Skip HDMI control indices
        if (i >= BASE_HDMI_CTRL_INX && i < (BASE_HDMI_CTRL_INX + HDMI_CTRL_COUNT)) continue;
        
        int row = i / 16;
        uint32_t color = palette_original[i];
        
        if (fade_level > 0) {
            // When at full black (>= 48), fade ALL rows regardless of mask
            // This prevents flashes when transitioning between scenes
            bool should_fade = (fade_level >= 48) || (which_rows == 0) || (which_rows & (1 << row));
            if (should_fade) {
                color = apply_fade_to_color(color, fade_level);
            }
        }
        
        palette[i] = color;
        
        // Write to BACK buffer (double buffering)
        uint64_t* conv_color64 = (uint64_t *)conv_color_back;
        const uint8_t R = (color >> 16) & 0xff;
        const uint8_t G = (color >> 8) & 0xff;
        const uint8_t B = (color >> 0) & 0xff;
        conv_color64[i * 2] = get_ser_diff_data(tmds_encoder(R), tmds_encoder(G), tmds_encoder(B));
        conv_color64[i * 2 + 1] = conv_color64[i * 2] ^ 0x0003ffffffffffffl;
    }
    graphics_restore_sync_colors();
    
    // Mark back buffer as dirty - will be swapped at next vsync
    conv_color_dirty = true;
}

uint8_t graphics_get_fade_level(void) {
    return g_hdmi_fade_level;
}

#define RGB888(r, g, b) ((r<<16) | (g << 8 ) | b )

void graphics_init_hdmi() {
    // PIO и DMA
    SM_video = pio_claim_unused_sm(PIO_VIDEO, true);
    SM_conv = pio_claim_unused_sm(PIO_VIDEO_ADDR, true);
    dma_chan_ctrl = dma_claim_unused_channel(true);
    dma_chan = dma_claim_unused_channel(true);
    dma_chan_pal_conv_ctrl = dma_claim_unused_channel(true);
    dma_chan_pal_conv = dma_claim_unused_channel(true);

    // Configure bus priority: give DMA higher priority than CPU
    // This helps prevent DMA starvation when CPU is doing heavy memory access
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_R_BITS | BUSCTRL_BUS_PRIORITY_DMA_W_BITS;

    hdmi_init();
}

void graphics_set_bgcolor_hdmi(uint32_t color888) //определяем зарезервированный цвет в палитре
{
    graphics_set_palette_hdmi(255, color888);
};

void graphics_restore_sync_colors(void) {
    // Restore HDMI sync control colors after palette updates
    // Write to BOTH buffers since these must always be valid
    const uint16_t b0 = 0b1101010100;
    const uint16_t b1 = 0b0010101011;
    const uint16_t b2 = 0b0101010100;
    const uint16_t b3 = 0b1010101011;
    const int base_inx = BASE_HDMI_CTRL_INX;
    
    // Write to back buffer
    uint64_t* conv_color64_back = (uint64_t *)conv_color_back;
    conv_color64_back[2 * base_inx + 0] = get_ser_diff_data(b0, b0, b3);
    conv_color64_back[2 * base_inx + 1] = get_ser_diff_data(b0, b0, b3);
    conv_color64_back[2 * (base_inx + 1) + 0] = get_ser_diff_data(b0, b0, b2);
    conv_color64_back[2 * (base_inx + 1) + 1] = get_ser_diff_data(b0, b0, b2);
    conv_color64_back[2 * (base_inx + 2) + 0] = get_ser_diff_data(b0, b0, b1);
    conv_color64_back[2 * (base_inx + 2) + 1] = get_ser_diff_data(b0, b0, b1);
    conv_color64_back[2 * (base_inx + 3) + 0] = get_ser_diff_data(b0, b0, b0);
    conv_color64_back[2 * (base_inx + 3) + 1] = get_ser_diff_data(b0, b0, b0);
    
    // Also write to front buffer so sync is always valid
    uint64_t* conv_color64_front = (uint64_t *)conv_color_front;
    conv_color64_front[2 * base_inx + 0] = get_ser_diff_data(b0, b0, b3);
    conv_color64_front[2 * base_inx + 1] = get_ser_diff_data(b0, b0, b3);
    conv_color64_front[2 * (base_inx + 1) + 0] = get_ser_diff_data(b0, b0, b2);
    conv_color64_front[2 * (base_inx + 1) + 1] = get_ser_diff_data(b0, b0, b2);
    conv_color64_front[2 * (base_inx + 2) + 0] = get_ser_diff_data(b0, b0, b1);
    conv_color64_front[2 * (base_inx + 2) + 1] = get_ser_diff_data(b0, b0, b1);
    conv_color64_front[2 * (base_inx + 3) + 0] = get_ser_diff_data(b0, b0, b0);
    conv_color64_front[2 * (base_inx + 3) + 1] = get_ser_diff_data(b0, b0, b0);
}

// Wrappers for existing API
void graphics_init(g_out g_out) {
    graphics_init_hdmi();
}

void graphics_set_palette(uint8_t i, uint32_t color888) {
    graphics_set_palette_hdmi(i, color888);
}

void graphics_set_bgcolor(uint32_t color888) {
    graphics_set_bgcolor_hdmi(color888);
}

void startVIDEO(uint8_t vol) {
    // Stub
}

void set_palette(uint8_t n) {
    // Stub
}
