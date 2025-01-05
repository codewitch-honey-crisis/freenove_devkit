#include "fns3devkit.hpp"

#include <ft6336.hpp>
#ifdef NO_LCD_PANEL_API
// this code taken from https://github.com/Bodmer/TFT_eSPI
#include <Arduino.h>
#include <SPI.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>
#include <soc/spi_reg.h>
#else
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
// #include "esp_lcd_gc9a01.h"
#endif
#include <esp_camera.h>
#ifdef ARDUINO
using namespace arduino;
#else
using namespace esp_idf;
#endif
using touch_t = ft6336<240, 320>;
static touch_t lcd_touch;
#ifdef NO_LCD_PANEL_API
#define DC_C GPIO.out_w1tc = (1 << pin::lcd_dc);
#define DC_D GPIO.out_w1ts = (1 << pin::lcd_dc);
static spi_device_handle_t lcd_spi_handle;
static void lcd_command(uint8_t cmd, const uint8_t* args = NULL, size_t len =0) {
    spi_transaction_t tx;
    memset(&tx,0,sizeof(tx));
    tx.length = 8;
    tx.tx_buffer = &cmd;
    tx.user = 0;
    spi_device_polling_transmit(lcd_spi_handle,&tx);
    if(len&&args) {
        tx.user = (void*)1;
        tx.length = 8*len;
        tx.tx_buffer = args;
        tx.user = (void*)1;
        spi_device_polling_transmit(lcd_spi_handle,&tx);
    }
}

static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t args[4];
    args[0]=(x1&0xFF)<<8;
    args[1]=(x1>>8);
    args[2]=(x2&0xFF)<<8;
    args[3]=(x2>>8);
    
    lcd_command(0x2A,args,4);
    args[0]=(y1&0xFF)<<8;
    args[1]=(y1>>8);
    args[2]=(y2&0xFF)<<8;
    args[3]=(y2>>8);
    lcd_command(0x2B,args,4);
    lcd_command(0x2c);
}
static void lcd_write_bitmap(const void* data_in, uint32_t len) {
    if (len) {
        spi_transaction_t tx;
        memset(&tx,0,sizeof(tx));
        tx.length = 8*len;
        tx.tx_buffer = data_in;
        tx.user = (void*)2;
        ESP_ERROR_CHECK(spi_device_queue_trans(lcd_spi_handle,&tx,portMAX_DELAY));
    }    
}
static void lcd_st7789_init() {
    
    lcd_command(0x01);  // reset
    delay(150);  // Wait for reset to complete
    lcd_command(0x11);  // Sleep out
    delay(120);
    lcd_command(0x13);  // Normal display mode on
    static const uint8_t params1 = 0x08;
    lcd_command(0x36,&params1,1);
    static const uint8_t params2[] = {0x0A,0xB2};
    lcd_command(0xB6,params2,2);
    static const uint8_t params3[] = {0x00,0xE0};
    lcd_command(0xB0,params3,2);
    static const uint8_t params4 = 0x55;
    lcd_command(0x3A,&params4,1);
    delay(10);
    static const uint8_t params5[] = {0x0C,0xC,0x00,0x33,0x33};
    lcd_command(0xB2,params5,5);
    static const uint8_t params6 = 0x35;
    lcd_command(0xB7,&params6,1);  // Voltages: VGH / VGL
    static const uint8_t params7 = 0x28;
    lcd_command(0xBB, &params7,1);
    static const uint8_t params8 = 0x0C;
    lcd_command(0xC0,&params8,1);
    static const uint8_t params9[] = {0x01,0xFF};
    lcd_command(0xC2,params9,2);
    static const uint8_t params10 = 0x10;
    lcd_command(0xC3,&params10,1);  // voltage VRHS
    static const uint8_t params11 = 0x20;
    lcd_command(0xC4,&params11,1);
    static const uint8_t params12 = 0x0F;
    lcd_command(0xC6,&params12,1);
    static const uint8_t params13[] = {0xA4,0xA1};
    lcd_command(0xD0,params13,2);
    static const uint8_t params14[] = {
        0xD0,0x00,0x02,0x07,0x0A,0x28,0x32,0x44,0x42,0x06,0x0E,0x12,0x14,0x17
    };
    lcd_command(0xE0,params14,14);
    static const uint8_t params15[] = {
        0xD0,0x00,0x02,0x07,0x0A,0x28,0x31,0x54,0x47,0x0E,0x1C,0x17,0x1B,0x1E
    };
    lcd_command(0xE1,params15,14);
    lcd_command(0x21);
    static const uint8_t params16[] = {
        0x00,0x00,0x00,0xEF
    };
    lcd_command(0x2A,params16,4);  // Column address set
    static const uint8_t params17[] = {
        0x00,0x00,0x01,0x3F
    };
    lcd_command(0x2B,params17,4);  // Row address set
    delay(120);
    lcd_command(0x29);
    delay(120);
    lcd_command(0x20);
}
#else
static esp_lcd_panel_handle_t lcd_handle = nullptr;
static esp_lcd_panel_io_handle_t lcd_io_handle = nullptr;
static bool lcd_flush_ready(esp_lcd_panel_io_handle_t panel_io,
                            esp_lcd_panel_io_event_data_t* edata,
                            void* user_ctx) {
    lcd_on_flush_complete();
    return true;
}
#endif
IRAM_ATTR void lcd_spi_pre_cb(spi_transaction_t *trans) {
    if(!trans->user) {
        DC_C;
    }
}
IRAM_ATTR void lcd_spi_post_cb(spi_transaction_t *trans) {
    if(!trans->user) {
        DC_D;
    } else if((int)trans->user == 2) {
        lcd_on_flush_complete();
    }
}
void lcd_initialize(size_t lcd_transfer_buffer_size) {
#ifdef NO_LCD_PANEL_API
    pinMode(pin::lcd_dc, OUTPUT);
    DC_D;
    spi_bus_config_t bus_cfg;
    memset(&bus_cfg,0,sizeof(bus_cfg));
    bus_cfg.max_transfer_sz = lcd_transfer_buffer_size + 8;
    bus_cfg.miso_io_num = -1;
    bus_cfg.mosi_io_num = pin::spi_mosi;
    bus_cfg.sclk_io_num = pin::spi_clk;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST,&bus_cfg,SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg;
    memset(&dev_cfg,0,sizeof(dev_cfg));
    dev_cfg.dummy_bits = 0;
    dev_cfg.queue_size = 10;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY;
    dev_cfg.spics_io_num = pin::lcd_cs;
    dev_cfg.pre_cb=lcd_spi_pre_cb;
    dev_cfg.post_cb=lcd_spi_post_cb;
    dev_cfg.clock_speed_hz = 80*1000*1000;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST,&dev_cfg,&lcd_spi_handle));
    lcd_st7789_init();
#else
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (((unsigned long long)1) << pin::lcd_dc);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&gpio_conf);
    // configure the SPI bus
    const spi_host_device_t host = SPI3_HOST;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = pin::spi_clk;
    buscfg.mosi_io_num = pin::spi_mosi;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    // declare enough space for the transfer buffers + 8 bytes SPI DMA overhead
    buscfg.max_transfer_sz = lcd_transfer_buffer_size + 8;

    // Initialize the SPI bus on VSPI (SPI3)
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config;
    memset(&io_config, 0, sizeof(io_config));
    io_config.dc_gpio_num = pin::lcd_dc;
    io_config.cs_gpio_num = pin::lcd_cs;
    io_config.pclk_hz = 80 * 1000 * 1000;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;
    io_config.spi_mode = 0;
    io_config.trans_queue_depth = 10;
    io_config.on_color_trans_done = lcd_flush_ready;
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)host,
                                             &io_config, &lcd_io_handle));

    esp_lcd_panel_dev_config_t panel_config;
    memset(&panel_config, 0, sizeof(panel_config));
    panel_config.reset_gpio_num = pin::lcd_rst;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    panel_config.rgb_endian = LCD_RGB_ENDIAN_BGR;
#else
    panel_config.color_space = ESP_LCD_COLOR_SPACE_BGR;
#endif
    panel_config.bits_per_pixel = 16;

    // Initialize the LCD configuration
    ESP_ERROR_CHECK(
        esp_lcd_new_panel_st7789(lcd_io_handle, &panel_config, &lcd_handle));
    // Reset the display
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handle));
    // Initialize LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handle));
    //  Swap x and y axis (Different LCD screens may need different options)
    esp_lcd_panel_swap_xy(lcd_handle, 0);
    esp_lcd_panel_set_gap(lcd_handle, 0, 0);
    esp_lcd_panel_mirror(lcd_handle, 0, 0);
    esp_lcd_panel_invert_color(lcd_handle, 0);
    // Turn on the screen
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_on_off(lcd_handle, true);
#else
    esp_lcd_panel_disp_off(lcd_handle, false);
#endif
#endif
    lcd_touch.initialize();
}
void lcd_flush_bitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                      const void* bitmap) {
#ifdef NO_LCD_PANEL_API
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    lcd_set_window(x1, y1, x2, y2);
    lcd_write_bitmap(bitmap, w * h);
    //lcd_on_flush_complete();
    
    //lcd_on_flush_complete();
#else
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2 + 1, y2 + 1,
                              (void*)bitmap);
#endif
}

void lcd_touch_update() { lcd_touch.update(); }
bool lcd_touch_pressed(uint16_t* out_x, uint16_t* out_y) {
    return lcd_touch.xy(out_x, out_y);
}

void lcd_rotation(uint8_t rotation) {
#ifdef NO_LCD_PANEL_API
    uint8_t param;
    switch (rotation & 3) {
        case 1:
            param=(0x40 | 0x20 | 0x08);
            break;
        case 2:
            param=(0x40 | 0x80 | 0x08);
            break;
        case 3:
            param=(0x20 | 0x80 | 0x08);
            break;
        default:  // case 0:
            param=(0x08);
            break;
    };
    lcd_command(0x36,&param,1);
    
#else
    switch (rotation & 3) {
        case 0:
            esp_lcd_panel_swap_xy(lcd_handle, 0);
            esp_lcd_panel_mirror(lcd_handle, 0, 0);
            break;
        // TODO: test these, they probably don't work
        case 1:
            esp_lcd_panel_swap_xy(lcd_handle, 1);
            esp_lcd_panel_mirror(lcd_handle, 1, 0);
            break;
        case 2:
            esp_lcd_panel_swap_xy(lcd_handle, 0);
            esp_lcd_panel_mirror(lcd_handle, 1, 1);
            break;
        case 3:
            esp_lcd_panel_swap_xy(lcd_handle, 1);
            esp_lcd_panel_mirror(lcd_handle, 0, 1);
            break;
    }
#endif
    lcd_touch.rotation(rotation & 3);
}

static TaskHandle_t camera_task_handle;  // camera thread task handle
static uint8_t* camera_fb = nullptr;
static SemaphoreHandle_t camera_fb_lock = nullptr;
static uint8_t camera_rot = 0;
void camera_copy_rotate(void* bitmap, int rows, int cols) {
    // allocating space for the new rotated image
    const uint16_t* original = (const uint16_t*) bitmap;
    uint16_t* out = (uint16_t*)camera_fb;
    switch(camera_rot) {
        case 1: {
            // rotate 90
            for(int y = 0; y<rows;++y) {
                for(int x = 0;x<cols;++x) {
                    out[(cols-x-1)*rows+y]=*(original++);
                }
            }
            break;
        }
        case 2:  {
            // rotate 180
            for(int y = 0; y<rows;++y) {
                for(int x = 0;x<cols;++x) {
                    out[(rows-y-1)*cols+(cols-x-1)]=*(original++);
                }
            }
            break;
        }
        case 3: {
            // rotate 270
            for(int y = 0; y<rows;++y) {
                for(int x = 0;x<cols;++x) {
                    out[x*rows+(rows-y-1)]=*(original++);
                }
            }
            break;
        }
        default: // case 0:
            memcpy(camera_fb,original,rows*cols*2);
            break;
    }
}
// camera thread
void camera_task(void* pvParameters) {
    camera_fb_t* fb_buf = NULL;
    while (true) {
        fb_buf = esp_camera_fb_get();
        esp_camera_fb_return(fb_buf);
        if (fb_buf != NULL && camera_fb != nullptr) {
            if (pdTRUE == xSemaphoreTake(camera_fb_lock, 50)) {
                camera_copy_rotate(fb_buf->buf, fb_buf->width,fb_buf->height);
                camera_on_frame(camera_fb);
                xSemaphoreGive(camera_fb_lock);
            }
        }
    }
}
const void* camera_lock_frame_buffer(bool wait) {
    if (camera_fb != nullptr &&
        pdTRUE == xSemaphoreTake(camera_fb_lock, wait ? portMAX_DELAY : 0)) {
        return camera_fb;
    }
    return nullptr;
}
void camera_rotation(uint8_t rotation) {
    if(camera_fb==nullptr) {
        camera_rot = rotation&3;
        return;
    }
    xSemaphoreTake(camera_fb_lock, portMAX_DELAY);
    camera_rot = rotation&3;
    xSemaphoreGive(camera_fb_lock);
}
void camera_unlock_frame_buffer() { xSemaphoreGive(camera_fb_lock); }
void camera_initialize(int flags) {
    if (camera_fb_lock != nullptr) {
        return;
    }
    camera_fb_lock = xSemaphoreCreateMutex();
    if (camera_fb_lock == nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    camera_fb = (uint8_t*)heap_caps_malloc(
        2 * 240 * 240, 0 != (flags & CAM_ALLOC_FB_PSRAM) ? MALLOC_CAP_SPIRAM
                                                         : MALLOC_CAP_DEFAULT);
    if (camera_fb == nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = pin::cam_y2;
    config.pin_d1 = pin::cam_y3;
    config.pin_d2 = pin::cam_y4;
    config.pin_d3 = pin::cam_y5;
    config.pin_d4 = pin::cam_y6;
    config.pin_d5 = pin::cam_y7;
    config.pin_d6 = pin::cam_y8;
    config.pin_d7 = pin::cam_y9;
    config.pin_xclk = pin::cam_xclk;
    config.pin_pclk = pin::cam_pclk;
    config.pin_vsync = pin::cam_vsync;
    config.pin_href = pin::cam_href;
    config.pin_sccb_sda = pin::cam_siod;
    config.pin_sccb_scl = pin::cam_sioc;
    config.pin_pwdn = pin::cam_pwdn;
    config.pin_reset = pin::cam_rst;
    config.xclk_freq_hz = 20 * 1000000;
    config.frame_size = 0 != (flags & CAM_FRAME_SIZE_96X96) ? FRAMESIZE_96X96
                                                            : FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = 0 != (flags & CAM_ALLOC_CAM_PSRAM) ? CAMERA_FB_IN_PSRAM
                                                            : CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 10;
    config.fb_count = 2;

    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);  // flip it back
    // s->set_contrast(s,0);
    s->set_hmirror(s, 1);     // horizontal mirror image
    s->set_brightness(s, 0);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
    xTaskCreate(camera_task, "camera_task", 10 * 1024, NULL, 1,
                &camera_task_handle);
}
void camera_levels(cam_level brightness, cam_level contrast, cam_level saturation, cam_level sharpness) {
    if(camera_fb==nullptr) {
        return;
    }
    sensor_t* s = esp_camera_sensor_get();
    if(brightness!=CAM_NO_CHANGE) {
        s->set_brightness(s,(int)brightness);
    }
    if(contrast!=CAM_NO_CHANGE) {
        s->set_contrast(s,(int)contrast);
    }
    if(saturation!=CAM_NO_CHANGE) {
        s->set_saturation(s,(int)saturation);
    }
    if(sharpness!=CAM_NO_CHANGE) {
        s->set_sharpness(s,(int)sharpness);
    }
}
void camera_deinitialize() {
    if (camera_task_handle != nullptr) {
        vTaskDelete(camera_task_handle);
        camera_task_handle = nullptr;
    }
    if (camera_fb != nullptr) {
        free(camera_fb);
        camera_fb = nullptr;
    }
    if (camera_fb_lock != nullptr) {
        vSemaphoreDelete(camera_fb_lock);
    }
    esp_camera_deinit();
}
void camera_on_frame(const void* bitmap) {
    // do nothing
}
