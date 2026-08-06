#include "ws565f.h"
#include <memory.h>
#include <esp_check.h>
#include <esp_heap_caps.h>
#include <limits.h>

#define WS565F_SLAB_BYTES ((WS565F_DITHER_SLAB_PIXELS + 1) / 2)

// palette order == panel color code: black,white,green,blue,red,yellow,orange
static const uint8_t WS565F_PAL[7][3] = {
    {  0,  0,  0}, {255,255,255}, {  0,255,  0}, {  0,  0,255},
    {255,  0,  0}, {255,255,  0}, {255,128,  0}
};

typedef enum {
    PS_NONE = 0,
    PS_INIT_1,
    PS_INIT_RESET,
    PS_INIT_RESET_WAIT_600,
    PS_INIT_RESET_2,
    PS_INIT_RESET_WAIT_2,
    PS_INIT_RESET_3,
    PS_INIT_RESET_WAIT_200,
    PS_INIT_BUSY_HIGH,
    PS_INIT_2,
    PS_INIT_WAIT_100,
    PS_INIT_3,
    PS_DISPLAY,
    PS_DISPLAY_DATA,
    PS_DISPLAY_DATA_NEXT,
    PS_DISPLAY_END,
    PS_DISPLAY_END_BUSY_HIGH_1,
    PS_DISPLAY_END_2,
    PS_DISPLAY_END_BUSY_HIGH_2,
    PS_DISPLAY_END_3,
    PS_DISPLAY_END_BUSY_LOW,
    PS_DISPLAY_END_WAIT_200,
    PS_SLEEP,
    PS_SLEEP_WAIT_100,
    PS_SLEEP_RESET_LOW,
} ws565f_state_t;

// --- async transaction pool (round-robin) ---------------------------------
static void ws565f_trans_reap_one(ws565f_handle_t* h) {
    spi_transaction_t* done = nullptr;
    ESP_ERROR_CHECK(spi_device_get_trans_result(h->spi, &done, portMAX_DELAY));
    --h->trans_inflight;
}
static spi_transaction_t* ws565f_trans_acquire(ws565f_handle_t* h) {
    if (h->trans_inflight > WS565F_TRANS_POOL_SIZE) ws565f_trans_reap_one(h);
    spi_transaction_t* t = &h->trans_pool[h->trans_head];
    h->trans_head = (h->trans_head + 1) % (WS565F_TRANS_POOL_SIZE+1);
    ++h->trans_inflight;
    memset(t, 0, sizeof(*t));
    return t;
}
static void ws565f_trans_drain(ws565f_handle_t* h) {
    while (h->trans_inflight > 0) ws565f_trans_reap_one(h);
}
static void ws565f_command(ws565f_handle_t* h, uint8_t command,
                          const uint8_t* data , size_t size ) {
    spi_transaction_t trans;
    gpio_set_level(h->cs_pin, 0);                 // select
    memset(&trans, 0, sizeof(trans));
    trans.flags      = SPI_TRANS_USE_TXDATA;
    trans.tx_data[0] = command;
    trans.length     = 8;
    trans.user       = &h->dc_cmd;
    ESP_ERROR_CHECK(spi_device_polling_transmit(h->spi, &trans));
    if (size > 0 && data != nullptr) {
        memset(&trans, 0, sizeof(trans));
        trans.tx_buffer = data;
        trans.length    = size * 8;
        trans.user      = &h->dc_dat;
        ESP_ERROR_CHECK(spi_device_polling_transmit(h->spi, &trans));
    }
    gpio_set_level(h->cs_pin, 1);                 // deselect
}

IRAM_ATTR static void ws565f_spi_pre_cb(spi_transaction_t* trans) {
    const ws565f_dc_ctx_t* ctx = (const ws565f_dc_ctx_t*)trans->user;
    gpio_set_level(ctx->dc_pin, ctx->level);
}
IRAM_ATTR static void ws565f_spi_post_cb(spi_transaction_t* trans) {
    const ws565f_dc_ctx_t* ctx = (const ws565f_dc_ctx_t*)trans->user;
    gpio_set_level(ctx->dc_pin, 1);
}
static void ws565f_init_reset(ws565f_handle_t* h) {
    gpio_set_level(h->rst_pin, 1);
    h->state = PS_INIT_RESET_WAIT_600; h->timestamp = xTaskGetTickCount();
}
static void ws565f_init_reset_2(ws565f_handle_t* h) {
    gpio_set_level(h->rst_pin, 0);
    h->state = PS_INIT_RESET_WAIT_2; h->timestamp = xTaskGetTickCount();
}
static void ws565f_init_reset_3(ws565f_handle_t* h) {
    gpio_set_level(h->rst_pin, 1);
    h->state = PS_INIT_RESET_WAIT_200; h->timestamp = xTaskGetTickCount();
}
static void ws565f_init_busy_high(ws565f_handle_t* h) {
    if (gpio_get_level(h->busy_pin)) h->state = PS_INIT_2;
}
static void ws565f_display_full_end_busy_high(ws565f_handle_t* h) {
    if (gpio_get_level(h->busy_pin))
        h->state = (h->state == PS_DISPLAY_END_BUSY_HIGH_1)
                   ? PS_DISPLAY_END_2 : PS_DISPLAY_END_3;
}
static void ws565f_display_full_end_busy_low(ws565f_handle_t* h) {
    if (!gpio_get_level(h->busy_pin)) { 
        h->timestamp = xTaskGetTickCount();
        h->state = PS_DISPLAY_END_WAIT_200;
    }
}

static void ws565f_init_1(ws565f_handle_t* h) {
    gpio_config_t gpio_cfg;
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.mode = GPIO_MODE_OUTPUT;
    gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_cfg.pin_bit_mask = ((1ULL<<h->cs_pin)|(1ULL<<h->dc_pin)|(1ULL<<h->rst_pin));
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    gpio_cfg.mode = GPIO_MODE_INPUT;
    gpio_cfg.pin_bit_mask = (1ULL<<h->busy_pin);
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
    gpio_set_level(h->cs_pin, 1);   // idle deselected

    spi_device_interface_config_t dev_config;
    memset(&dev_config, 0, sizeof(dev_config));
    dev_config.clock_source   = SPI_CLK_SRC_DEFAULT;
    dev_config.clock_speed_hz = h->clock_speed_hz;
    dev_config.spics_io_num   = -1;             // manual CS (multi-panel)
    dev_config.queue_size     = 10;
    dev_config.pre_cb         = ws565f_spi_pre_cb;
    dev_config.post_cb        = ws565f_spi_post_cb;
    ESP_ERROR_CHECK(spi_bus_add_device(h->host, &dev_config, &h->spi));
    h->state = PS_INIT_RESET;
}
static void ws565f_init_2(ws565f_handle_t* h) {
    ws565f_command(h, 0x00, (uint8_t[]){0xEF,0x08}, 2);
    ws565f_command(h, 0x01, (uint8_t[]){0x37,0x00,0x23,0x23}, 4);
    ws565f_command(h, 0x03, (uint8_t[]){0x00}, 1);
    ws565f_command(h, 0x06, (uint8_t[]){0xC7,0xC7,0x1D}, 3);
    ws565f_command(h, 0x30, (uint8_t[]){0x3C}, 1);
    ws565f_command(h, 0x41, (uint8_t[]){0x00}, 1);
    ws565f_command(h, 0x50, (uint8_t[]){0x37}, 1);
    ws565f_command(h, 0x60, (uint8_t[]){0x22}, 1);
    ws565f_command(h, 0x61, (uint8_t[]){0x02,0x58,0x01,0xC0}, 4);
    ws565f_command(h, 0xE3, (uint8_t[]){0xAA}, 1);
    h->timestamp = xTaskGetTickCount(); h->state = PS_INIT_WAIT_100;
}
static void ws565f_init_3(ws565f_handle_t* h) {
    ws565f_command(h, 0x50, (uint8_t[]){0x37}, 1);
    h->initted = true; h->state = PS_NONE;
}
static void ws565f_display_full_1(ws565f_handle_t* h) {
    ws565f_command(h, 0x61, (uint8_t[]){0x02,0x58,0x01,0xC0}, 4);
    h->state = PS_DISPLAY_DATA;
    h->pixels_remaining = 600 * 448;
}
static void ws565f_display_full_end(ws565f_handle_t* h) {
    ws565f_trans_drain(h);              // wait for all pixel data on the wire
    gpio_set_level(h->cs_pin, 1);      // deselect after the data frame
    ws565f_command(h, 0x04,NULL,0);
    h->state = PS_DISPLAY_END_BUSY_HIGH_1;
}
static void ws565f_display_full_end_2(ws565f_handle_t* h) {
    ws565f_command(h, 0x12,NULL,0); h->state = PS_DISPLAY_END_BUSY_HIGH_2;
}
static void ws565f_display_full_end_3(ws565f_handle_t* h) {
    ws565f_command(h, 0x02,NULL,0); h->state = PS_DISPLAY_END_BUSY_LOW;
}
static void ws565f_sleep_1(ws565f_handle_t* h) {
    ws565f_command(h, 0x07,(uint8_t[]){0xA5},1); h->timestamp = xTaskGetTickCount(); h->state = PS_SLEEP_WAIT_100;
}
static void ws565f_sleep_reset_low(ws565f_handle_t* h) {
    gpio_set_level(h->rst_pin,0);
    h->state = PS_NONE;
}
bool ws565f_initialized(ws565f_handle_t* h) { return h->initted; }

void ws565f_update(ws565f_handle_t* h) {
    switch (h->state) {
        case PS_NONE: break;
        case PS_INIT_1: ws565f_init_1(h); break;
        case PS_INIT_RESET: ws565f_init_reset(h); break;
        case PS_INIT_RESET_WAIT_600:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(600)) h->state = PS_INIT_RESET_2; 
            break;
        case PS_INIT_RESET_2: ws565f_init_reset_2(h); break;
        case PS_INIT_RESET_WAIT_2:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(2)) h->state = PS_INIT_RESET_3; 
            break;
        case PS_INIT_RESET_3: ws565f_init_reset_3(h); break;
        case PS_INIT_RESET_WAIT_200:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(200)) h->state = PS_INIT_BUSY_HIGH; 
            break;
        case PS_INIT_BUSY_HIGH: ws565f_init_busy_high(h); break;
        case PS_INIT_2: ws565f_init_2(h); break;
        case PS_INIT_WAIT_100:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(100)) h->state = PS_INIT_3; 
            break;
        case PS_INIT_3: ws565f_init_3(h); break;
        case PS_DISPLAY: ws565f_display_full_1(h); break;
        case PS_DISPLAY_DATA:
        case PS_DISPLAY_DATA_NEXT:
            if (h->pixels_remaining == 0) h->state = PS_DISPLAY_END; 
            break;
        case PS_DISPLAY_END:
            ws565f_display_full_end(h); h->pixels_remaining = 0; break;
        case PS_DISPLAY_END_BUSY_HIGH_1:
        case PS_DISPLAY_END_BUSY_HIGH_2:
            ws565f_display_full_end_busy_high(h); break;
        case PS_DISPLAY_END_2: ws565f_display_full_end_2(h); break;
        case PS_DISPLAY_END_3: ws565f_display_full_end_3(h); break;
        case PS_DISPLAY_END_BUSY_LOW: ws565f_display_full_end_busy_low(h); break;
        case PS_DISPLAY_END_WAIT_200:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(200)) h->state = PS_NONE; 
            break;
        case PS_SLEEP:
            ws565f_sleep_1(h);
            break;
        case PS_SLEEP_WAIT_100:
            if (xTaskGetTickCount() >= h->timestamp + pdMS_TO_TICKS(100)) h->state = PS_SLEEP_RESET_LOW; 
            break;
        case PS_SLEEP_RESET_LOW:
            ws565f_sleep_reset_low(h);
            break;
    }
}

void ws565f_initialize(const ws565f_config_t* cfg,ws565f_handle_t* out) {
    memset(out, 0, sizeof(*out));
    out->host = cfg->host; out->clock_speed_hz = cfg->clock_speed_hz==0?(10*1000*1000):cfg->clock_speed_hz;
    out->cs_pin = (gpio_num_t)cfg->cs_pin; out->dc_pin = (gpio_num_t)cfg->dc_pin;
    out->rst_pin = (gpio_num_t)cfg->rst_pin; out->busy_pin = (gpio_num_t)cfg->busy_pin;
    out->dc_cmd.dc_pin = (gpio_num_t)cfg->dc_pin; out->dc_cmd.level = 0;
    out->dc_dat.dc_pin = (gpio_num_t)cfg->dc_pin; out->dc_dat.level = 1;
    out->state = PS_INIT_1;   // kicked off on the next ws565f_update(out)
}

bool ws565f_ready(ws565f_handle_t* h) {
    return h->initted && (h->state == PS_NONE || h->state == PS_DISPLAY_DATA);
}
void ws565f_wait(ws565f_handle_t* h) {
    while (!ws565f_ready(h)) { ws565f_update(h); vTaskDelay(5); }
}
void ws565f_display(ws565f_handle_t* h) { h->state = PS_DISPLAY; }

esp_err_t ws565f_write(ws565f_handle_t* h, const uint8_t* pixels, size_t pixel_count) {
    if(h->state==PS_DISPLAY) {
        ws565f_update(h);
    }
    if (h->state != PS_DISPLAY_DATA && h->state != PS_DISPLAY_DATA_NEXT)
        return ESP_ERR_INVALID_STATE;
    if (pixel_count > h->pixels_remaining) pixel_count = h->pixels_remaining;

    if (h->state == PS_DISPLAY_DATA) {
        gpio_set_level(h->cs_pin, 0);           // select for the whole frame
        spi_transaction_t* cmd = ws565f_trans_acquire(h);
        cmd->flags      = SPI_TRANS_USE_TXDATA;
        cmd->tx_data[0] = 0x10;
        cmd->length     = 8;
        cmd->user       = &h->dc_cmd;
        ESP_ERROR_CHECK(spi_device_queue_trans(h->spi, cmd, portMAX_DELAY));
        h->state = PS_DISPLAY_DATA_NEXT;
    }
    if (pixel_count > 0) {
        spi_transaction_t* dat = ws565f_trans_acquire(h);
        dat->tx_buffer = pixels;
        dat->length    = pixel_count * 4;       // 4 bits/pixel
        dat->user      = &h->dc_dat;
        ESP_ERROR_CHECK(spi_device_queue_trans(h->spi, dat, portMAX_DELAY));
        h->pixels_remaining -= pixel_count;
    }
    return ESP_OK;
}
static inline uint8_t ws565f_nearest(int r, int g, int b) {
    int best = 0; long bd = LONG_MAX;
    for (int i = 0; i < 7; ++i) {
        int dr = r - WS565F_PAL[i][0];
        int dg = g - WS565F_PAL[i][1];
        int db = b - WS565F_PAL[i][2];
        long d = (long)dr*dr + (long)dg*dg + (long)db*db;
        if (d < bd) { bd = d; best = i; }
    }
    return (uint8_t)best;
}

// reap the oldest in-flight output slab (FIFO; only our transactions are queued
// during the data frame, so results come back in the order we sent them)
static void ws565f_dither_reap(ws565f_handle_t* h, ws565f_dither_cache_t* c) {
    spi_transaction_t* done = nullptr;
    ESP_ERROR_CHECK(spi_device_get_trans_result(h->spi, &done, portMAX_DELAY));
    --c->pending;
}
// queue the current slab (n bytes) and advance the double buffer
static void ws565f_dither_flush(ws565f_handle_t* h, ws565f_dither_cache_t* c, size_t n) {
    if (n == 0) return;
    spi_transaction_t* t = &c->xfer[c->cur];
    memset(t, 0, sizeof(*t));
    t->tx_buffer = c->slab[c->cur];
    t->length    = n * 8;
    t->user      = &h->dc_dat;                         // DC high (data)
    ESP_ERROR_CHECK(spi_device_queue_trans(h->spi, t, portMAX_DELAY));
    ++c->pending;
    c->cur ^= 1;
    if (c->pending == 2) ws565f_dither_reap(h, c);     // free the slab we swap to
}

typedef enum { WS565F_FMT_RGB565_BE,WS565F_FMT_RGB565_LE, WS565F_FMT_RGB888 } ws565f_fmt_t;

static esp_err_t ws565f_write_dithered(ws565f_handle_t* h, void* cache,
                                       const uint8_t* src, size_t pixel_count,
                                       ws565f_fmt_t fmt) {
    ws565f_dither_cache_t* c = (ws565f_dither_cache_t*)cache;
    if (c == nullptr) return ESP_ERR_INVALID_ARG;

    if (h->state == PS_DISPLAY) ws565f_update(h);       // advance into data phase

    if (h->state == PS_DISPLAY_DATA) {
        // first chunk of the frame: select, send 0x10 start, reset dither state
        gpio_set_level(h->cs_pin, 0);
        spi_transaction_t cmd; memset(&cmd, 0, sizeof(cmd));
        cmd.flags = SPI_TRANS_USE_TXDATA; cmd.tx_data[0] = 0x10;
        cmd.length = 8; cmd.user = &h->dc_cmd;
        ESP_ERROR_CHECK(spi_device_polling_transmit(h->spi, &cmd)); // nothing queued yet

        memset(c->err,   0, sizeof(c->err));
        memset(c->carry, 0, sizeof(c->carry));
        memset(c->dr,    0, sizeof(c->dr));
        c->col = 0; c->phase = 0; c->cur = 0; c->pending = 0;
        h->state = PS_DISPLAY_DATA_NEXT;
    }
    if (h->state != PS_DISPLAY_DATA_NEXT) return ESP_ERR_INVALID_STATE;

    if (pixel_count > h->pixels_remaining) pixel_count = h->pixels_remaining;
    if (pixel_count == 0) return ESP_OK;

    uint8_t* out = c->slab[c->cur];    // always free here (reaped after each swap)
    size_t obytes = 0;

    for (size_t p = 0; p < pixel_count; ++p) {
        int r, g, b;
        switch(fmt) {
            case WS565F_FMT_RGB565_BE: {
                uint16_t v = ((uint16_t)src[0] << 8) | src[1]; src += 2;
                r = ((v >> 11) & 0x1F) * 255 / 31;
                g = ((v >>  5) & 0x3F) * 255 / 63;
                b = ( v        & 0x1F) * 255 / 31;
                break;
            }
            case WS565F_FMT_RGB565_LE: {
                uint16_t v = src[0] | ((uint16_t)src[1]<<8); src += 2;
                r = ((v >> 11) & 0x1F) * 255 / 31;
                g = ((v >>  5) & 0x3F) * 255 / 63;
                b = ( v        & 0x1F) * 255 / 31;
                break;
            }
            default: {
                r = src[0]; g = src[1]; b = src[2]; src += 3;
                break;
            }

        }
        
        const uint32_t x = c->col;
        int16_t* e = &c->err[x * 3];
        int vr = r + e[0] + c->carry[0];
        int vg = g + e[1] + c->carry[1];
        int vb = b + e[2] + c->carry[2];

        uint8_t idx = ws565f_nearest(vr, vg, vb);
        int ch_e[3] = { vr - WS565F_PAL[idx][0],
                        vg - WS565F_PAL[idx][1],
                        vb - WS565F_PAL[idx][2] };

        for (int k = 0; k < 3; ++k) {
            int ev = ch_e[k];
            int e7 = ev*7/16, e5 = ev*5/16, e3 = ev*3/16;
            int e1 = ev - e7 - e5 - e3;                 // conserve total error
            c->carry[k] = (int16_t)e7;                  // -> (x+1, y)
            if (x > 0)                                   // -> (x-1, y+1)
                c->err[(x-1)*3 + k] = (int16_t)(c->err[(x-1)*3 + k] + e3);
            c->err[x*3 + k] = (int16_t)(e5 + c->dr[k]);  // -> (x, y+1) + prior (x,y+1)
            c->dr[k] = (int16_t)e1;                       // -> (x+1, y+1)
        }

        // pack: first pixel of a byte = high nibble
        if (c->phase == 0) { c->partial = (uint8_t)(idx << 4); c->phase = 1; }
        else {
            out[obytes++] = c->partial | idx; c->phase = 0;
            if (obytes == WS565F_SLAB_BYTES) {
                ws565f_dither_flush(h, c, obytes);
                out = c->slab[c->cur]; obytes = 0;
            }
        }

        if (++c->col == WS565F_PANEL_WIDTH) {            // row wrap: drop carries
            c->col = 0;
            c->carry[0] = c->carry[1] = c->carry[2] = 0;
            c->dr[0] = c->dr[1] = c->dr[2] = 0;
        }
    }

    ws565f_dither_flush(h, c, obytes);                   // tail of this chunk
    h->pixels_remaining -= pixel_count;

    if (h->pixels_remaining == 0)                        // last write: let DMA finish
        while (c->pending) ws565f_dither_reap(h, c);     // so PS_DISPLAY_END's 0x04 is safe

    return ESP_OK;
}

esp_err_t ws565f_write_rgb16(ws565f_handle_t* h, void* cache,
                             const uint8_t* rgb565, size_t pixel_count) {
    return ws565f_write_dithered(h, cache, rgb565, pixel_count, WS565F_FMT_RGB565_LE);
}
esp_err_t ws565f_write_rgb16_be(ws565f_handle_t* h, void* cache,
                             const uint8_t* rgb565, size_t pixel_count) {
    return ws565f_write_dithered(h, cache, rgb565, pixel_count, WS565F_FMT_RGB565_BE);
}

esp_err_t ws565f_write_rgb24(ws565f_handle_t* h, void* cache,
                             const uint8_t* rgb888, size_t pixel_count) {
    return ws565f_write_dithered(h, cache, rgb888, pixel_count, WS565F_FMT_RGB888);
}

void* ws565f_create_dither_cache(void) {
    return heap_caps_malloc(WS565F_DITHER_CACHE_SIZE, MALLOC_CAP_DMA);
}

void ws565f_destroy_dither_cache(void* cache) {
    return heap_caps_free(cache);
}

void ws565f_sleep(ws565f_handle_t* h) {
    h->state = PS_SLEEP;
}