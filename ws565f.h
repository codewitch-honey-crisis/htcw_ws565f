// Waveshare 5.65" (F) 7-color e-ink display driver for the ESP-IDF
// colors are (in order) black, white, green, blue, red, yellow, orange
// pixels are 3-bit. packed into 4 bits. Value 7 and above are reserved
#ifndef WS565F_H
#define WS565F_H
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#ifdef __cplusplus
extern "C" {
#endif
#ifndef WS565F_TRANS_POOL_SIZE
#define WS565F_TRANS_POOL_SIZE 1
#endif
#ifndef WS565F_PANEL_WIDTH
#define WS565F_PANEL_WIDTH  600
#endif
#ifndef WS565F_PANEL_HEIGHT
#define WS565F_PANEL_HEIGHT 448
#endif
// Pixels packed per DMA flush. One row by default; the two slabs double-buffer
// so the next block is packed while one is in flight.
#ifndef WS565F_DITHER_SLAB_PIXELS
#define WS565F_DITHER_SLAB_PIXELS WS565F_PANEL_WIDTH
#endif
// Bytes to reserve for a dither cache.
#define WS565F_DITHER_CACHE_SIZE (sizeof(ws565f_dither_cache_t))

// Carried in spi_transaction_t::user so the shared DC callback knows which
// panel's DC pin to drive and to what level. Lives inside the handle.
typedef struct {
    gpio_num_t dc_pin;
    int level;
} ws565f_dc_ctx_t;

typedef struct {
    // configuration
    spi_host_device_t host;
    int clock_speed_hz;
    gpio_num_t cs_pin, dc_pin, rst_pin, busy_pin;

    // spi device + per-level DC callback contexts
    spi_device_handle_t spi;
    ws565f_dc_ctx_t dc_cmd;   // {dc_pin, 0}
    ws565f_dc_ctx_t dc_dat;   // {dc_pin, 1}

    // state machine
    int state;
    TickType_t timestamp;
    bool initted;
    size_t pixels_remaining;

    // async transaction pool (round-robin)
    spi_transaction_t trans_pool[WS565F_TRANS_POOL_SIZE+1];
    size_t trans_head;
    size_t trans_inflight;
} ws565f_handle_t;

/// @brief A display configuration
typedef struct {
    /// @brief The host SPI device to use
    spi_host_device_t host;
    /// @brief The clock speed of the device. 0 = 10MHz
    int clock_speed_hz;
    /// @brief The CS pin
    int cs_pin;
    /// @brief The DC pin
    int dc_pin;
    /// @brief The reset pin
    int rst_pin;
    /// @brief The busy/wait pin
    int busy_pin;
} ws565f_config_t;

// Dithering scratch. MUST live in DMA-capable internal RAM (its output slabs
// are handed straight to SPI DMA). One per concurrent frame source.
typedef struct __attribute__((aligned(4))) {
    int16_t  err[WS565F_PANEL_WIDTH * 3]; // one-row error diffusion line (RGB)
    int16_t  carry[3];                    // 7/16 horizontal carry
    int16_t  dr[3];                       // 1/16 down-right carry
    uint32_t col;                         // current column, 0..WIDTH-1
    uint8_t  partial;                     // half-packed output byte
    uint8_t  phase;                       // 0: next nibble high, 1: low
    uint8_t  cur;                         // slab being filled
    uint8_t  pending;                     // slabs in flight (0..2)
    uint8_t  slab[2][(WS565F_DITHER_SLAB_PIXELS + 1) / 2];
    spi_transaction_t xfer[2];
} ws565f_dither_cache_t;

/// @brief Dither + write RGB565 (little-endian) pixels. Use exactly like
/// ws565f_write(): call after ws565f_display(). The cache resets itself at the
/// first chunk of each frame.
/// @param h A handle to the display
/// @param dither_cache A WS565F_DITHER_CACHE_SIZE buffer in DMA-capable RAM, allocated using ws565_create_dither_cache()
/// @param rgb565 the pixel buffer
/// @param pixel_count the count of pixels
esp_err_t ws565f_write_rgb16(ws565f_handle_t* h, void* dither_cache,
                             const uint8_t* rgb565, size_t pixel_count);
/// @brief Dither + write RGB565 (big-endian) pixels. Use exactly like
/// ws565f_write(): call after ws565f_display(). The cache resets itself at the
/// first chunk of each frame.
/// @param h A handle to the display
/// @param dither_cache A WS565F_DITHER_CACHE_SIZE buffer in DMA-capable RAM, allocated using ws565_create_dither_cache()
/// @param rgb565 the pixel buffer
/// @param pixel_count the count of pixels
esp_err_t ws565f_write_rgb16_be(ws565f_handle_t* h, void* cache,
                            const uint8_t* rgb565, size_t pixel_count);

/// @brief Dither + write RGB888 (R in MSB). See ws565f_write_rgb16.
/// @param h A handle to the display
/// @param dither_cache A WS565F_DITHER_CACHE_SIZE buffer in DMA-capable RAM, allocated using ws565_create_dither_cache()
/// @param rgb888 the pixel buffer
/// @param pixel_count the count of pixels
esp_err_t ws565f_write_rgb24(ws565f_handle_t* h, void* dither_cache,
                             const uint8_t* rgb888, size_t pixel_count);
/// @brief creates a dither cache
/// @return A dither cache, or null if no memory (requires just under 5KB)
void* ws565f_create_dither_cache(void);
/// @brief creates a dither cache
/// @param cache A dither cache created with ws565_create_dither_cache()
void ws565f_destroy_dither_cache(void* cache);

/// @brief Indicates whether the display has been initialized
/// @param h A handle to the display
/// @return True if initialized, otherwise false
bool ws565f_initialized(ws565f_handle_t* h);
/// @brief Called to update the internal display facilities. Should be called in the app loop.
/// @param h A handle to the display
void ws565f_update(ws565f_handle_t* h);
/// @brief Initialize a waveshare 5.65" (F) Display
/// @param cfg The configuration settings
/// @param out The handle to the display
void ws565f_initialize(const ws565f_config_t* cfg,ws565f_handle_t* out);
/// @brief Indicates whether or not the display is ready
/// @param h A handle to the display
/// @return True if ready to take more commands, otherwise false
bool ws565f_ready(ws565f_handle_t* h);
/// @brief Waits for the display to be ready
/// @param h A handle to the displat
void ws565f_wait(ws565f_handle_t* h);
/// @brief Start displaying data
/// @param h A handle to the display
void ws565f_display(ws565f_handle_t* h);
/// @brief Write pixels to the display. Called after ws565f_display()
/// @param h A handle to the display
/// @param pixels A buffer of 4-byte pixels
/// @param pixel_count The count of pixels
/// @return ESP_OK on success, otherwise the error that occurred
esp_err_t ws565f_write(ws565f_handle_t* h, const uint8_t* pixels, size_t pixel_count);
/// @brief Sleep the display
/// @param h A handle to the display
void ws565f_sleep(ws565f_handle_t* h);

#ifdef __cplusplus
}
#endif

#endif // WS565F_H