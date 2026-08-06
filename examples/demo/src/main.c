#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define PIN_NUM_DC   21
#define PIN_NUM_RST  4
#define PIN_NUM_WAIT 15
#define SPI_SPEED_HZ (10*1000*1000)

#include <stdio.h>
#include <memory.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "ws565f.h"
#include "tjpgd.h"
#define STRONG_JPG_IMPLEMENTATION
#include "strong_jpg.h"
#undef STRONG_JPG_IMPLEMENTATION


/* Bytes per pixel of image output */
#define N_BPP (3 - JD_FORMAT)

static ws565f_handle_t panel;
static void* panel_dither_cache;
static TickType_t ts = 0;
/* Session identifier for input/output functions (Name, members and usage are as user defined) */
typedef struct {
    const uint8_t *p;               /* Input stream */
    uint8_t *fbuf;          /* Pointer to the frame buffer */
    unsigned int wfbuf;     /* Width of the frame buffer [pix] */
} IODEV;


static void spi_bus_init() {
    spi_bus_config_t bus_config;
    memset(&bus_config,0,sizeof(bus_config));
    bus_config.data0_io_num = -1;
    bus_config.data1_io_num = -1;
    bus_config.data2_io_num = -1;
    bus_config.data3_io_num = -1;
    bus_config.data4_io_num = -1;
    bus_config.data5_io_num = -1;
    bus_config.data6_io_num = -1;
    bus_config.data7_io_num = -1;
    bus_config.max_transfer_sz = 600/2+8;
    bus_config.mosi_io_num = PIN_NUM_MOSI;
    bus_config.miso_io_num = PIN_NUM_MISO;
    bus_config.sclk_io_num = PIN_NUM_CLK;
    ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)SPI_HOST,&bus_config,SPI_DMA_CH_AUTO));
}

size_t jpg_in_func (    /* Returns number of bytes read (zero on error) */
    JDEC* jd,       /* Decompression object */
    uint8_t* buff,  /* Pointer to the read buffer (null to remove data) */
    size_t nbyte    /* Number of bytes to read/remove */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */


    if (buff) { /* Raad data from imput stream */
        memcpy(buff,dev->p,nbyte);
        dev->p+=nbyte;
        return nbyte;
    } else {    /* Remove data from input stream */
        dev->p+=nbyte;
        return nbyte;
    }
}

int jpg_out_func (      /* Returns 1 to continue, 0 to abort */
    JDEC* jd,       /* Decompression object */
    void* bitmap,   /* Bitmap data to be output */
    JRECT* rect     /* Rectangle region of output image */
)
{
    IODEV *dev = (IODEV*)jd->device;   /* Session identifier (5th argument of jd_prepare function) */
    uint8_t *src, *dst;
    uint16_t y, bws;
    unsigned int bwd;

    /* Copy the output image rectangle to the frame buffer */
    src = (uint8_t*)bitmap;                           /* Output bitmap */
    dst = dev->fbuf + 2 * (rect->left);  /* Left-top of rectangle in the frame buffer */
    bws = N_BPP * (rect->right - rect->left + 1);     /* Width of the rectangle [byte] */
    bwd = N_BPP * dev->wfbuf;                         /* Width of the frame buffer [byte] */
    for (y = rect->top; y <= rect->bottom; y++) {
        memcpy(dst, src, bws);   /* Copy a line */
        src += bws; dst += bwd;  /* Next line */
    }
    if(rect->right+1==WS565F_PANEL_WIDTH) {
        ws565f_write_rgb16(&panel,panel_dither_cache,dev->fbuf,WS565F_PANEL_WIDTH*(rect->bottom-rect->top+1));
    }
    if(xTaskGetTickCount()>=ts+pdMS_TO_TICKS(1000)) {
        ts = xTaskGetTickCount();
        fputs(".",stdout);
        fflush(stdout);
        vTaskDelay(5);
    }
    return 1;    /* Continue to decompress */
}


void app_main() {
    puts("Booted");
    spi_bus_init();
    
    ws565f_config_t cfg = {
        (spi_host_device_t)SPI_HOST, 
        SPI_SPEED_HZ,
        PIN_NUM_CS, PIN_NUM_DC,
        PIN_NUM_RST, PIN_NUM_WAIT
    };
    ws565f_initialize(&cfg,&panel);
    /* Prepare to decompress */
    void* work = (void*)malloc(3500);
    JDEC jdec; 
    IODEV devid;
    devid.p = strong_jpg;
    JRESULT res = jd_prepare(&jdec, jpg_in_func, work, 3500, &devid);
    if (res != JDR_OK) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_SUPPORTED);
    }
    /* Initialize output device (Create a frame buffer) */
    devid.fbuf = (uint8_t*)malloc(2 * jdec.width * 16);
    if(devid.fbuf==NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    panel_dither_cache = ws565f_create_dither_cache();
    if(panel_dither_cache==NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    
    if(jdec.width!=WS565F_PANEL_WIDTH || jdec.height!=WS565F_PANEL_HEIGHT) {
        ESP_ERROR_CHECK(ESP_ERR_INVALID_SIZE);
    }
    devid.wfbuf = jdec.width;
    ws565f_wait(&panel); // wait for initialization to complete
    ws565f_display(&panel); // begin the draw operation
    ts = xTaskGetTickCount();
    size_t free_mem = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    printf("Free heap: %0.2fKB\n",((float)free_mem)/1024.f);
    fputs("Loading jpg",stdout);
    res = jd_decomp(&jdec, jpg_out_func, 0);   /* Start to decompress with 1/1 scaling */
    if (res == JDR_OK) {
        puts("\nJpg loaded");
    } else {
        printf("JPEG decompression failed (rc=%d)\n", res);
        free(devid.fbuf);    /* Discard frame buffer */
        free(work);
        ws565f_destroy_dither_cache(panel_dither_cache);
        return;
    }

    fputs("\nWaiting for display to finish",stdout);
    // could just call ws565f_wait(&panel) here instead of showing progress:
    while(!ws565f_ready(&panel)) {
        ws565f_update(&panel);
        if(xTaskGetTickCount()>=ts+pdMS_TO_TICKS(1000)) {
            ts = xTaskGetTickCount();
            fputs(".",stdout);
            fflush(stdout);
            vTaskDelay(5);
        }
    }
    free(devid.fbuf);    /* Discard frame buffer */
    free(work);
    ws565f_destroy_dither_cache(panel_dither_cache);

    puts("\nDevice updated");
    puts("Sleeping");
    ws565f_sleep(&panel);
    ws565f_wait(&panel);

}