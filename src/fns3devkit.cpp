#include "fns3devkit.hpp"
#include <ft6336.hpp>
#ifdef NO_LCD_PANEL_API
// this code taken from https://github.com/Bodmer/TFT_eSPI
#include <Arduino.h>
#include <SPI.h>
#include <soc/spi_reg.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>
#else
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
//#include "esp_lcd_gc9a01.h"
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
#define SPI_PORT 3
#ifndef REG_SPI_BASE //                      HSPI                 FSPI/VSPI
#define REG_SPI_BASE(i) (((i)>1) ? (DR_REG_SPI3_BASE) : (DR_REG_SPI2_BASE))
#endif

// Fix ESP32S3 IDF bug for name change
#ifndef SPI_MOSI_DLEN_REG
#define SPI_MOSI_DLEN_REG(x) SPI_MS_DLEN_REG(x)
#endif
#define TFT_CASET 0x2A
#define TFT_PASET 0x2B
#define TFT_RAMWR 0x2C
#define _spi_cmd       (volatile uint32_t*)(SPI_CMD_REG(SPI_PORT))
#define _spi_user      (volatile uint32_t*)(SPI_USER_REG(SPI_PORT))
#define _spi_mosi_dlen (volatile uint32_t*)(SPI_MOSI_DLEN_REG(SPI_PORT))
#define _spi_w         (volatile uint32_t*)(SPI_W0_REG(SPI_PORT))
#define SET_BUS_WRITE_MODE *_spi_user = SPI_USR_MOSI
#define SET_BUS_READ_MODE  *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN
#define SPI_BUSY_CHECK while (*_spi_cmd&SPI_USR)
#define CS_L GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32)); GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32))
#define CS_H GPIO.out1_w1ts.val = (1 << (pin::lcd_cs - 32))
#define DC_C GPIO.out_w1tc = (1 << pin::lcd_dc);
#define DC_D GPIO.out_w1ts = (1 << pin::lcd_dc);
#define SPI_UPDATE    (BIT(23))
#define TFT_WRITE_BITS(D, B) *_spi_mosi_dlen = B-1;  \
                               *_spi_w = D;              \
                               *_spi_cmd = SPI_UPDATE;   \
                        while (*_spi_cmd & SPI_UPDATE);  \
                               *_spi_cmd = SPI_USR;      \
                        while (*_spi_cmd & SPI_USR);
#define tft_Write_8(C) TFT_WRITE_BITS(C, 8)
#define tft_Write_32C(C,D)  TFT_WRITE_BITS((uint16_t)((D)<<8 | (D)>>8)<<16 | (uint16_t)((C)<<8 | (C)>>8), 32)

//static TFT_eSPI lcd_panel(240, 320);
SPIClass lcd_spi(HSPI);
static void lcd_begin_write() {
    lcd_spi.beginTransaction(SPISettings(80000000, MSBFIRST, SPI_MODE0));
    CS_L;
    SET_BUS_WRITE_MODE;
}
static void lcd_end_write() {
    SPI_BUSY_CHECK;
    CS_H;
    SET_BUS_READ_MODE;
    lcd_spi.endTransaction();
}
static void lcd_command(uint8_t cmd) {
  DC_C;
  tft_Write_8(cmd);
  DC_D;
}
static void lcd_param(uint8_t arg) {
    DC_D;        // Play safe, but should already be in data mode
    tft_Write_8(arg);
    CS_L;     
}
static void lcd_set_window(uint16_t x1,uint16_t y1, uint16_t x2, uint16_t y2) {
    SPI_BUSY_CHECK;
    DC_C; tft_Write_8(TFT_CASET);
    DC_D; tft_Write_32C(x1, x2);
    DC_C; tft_Write_8(TFT_PASET);
    DC_D; tft_Write_32C(y1, y2);
    DC_C; tft_Write_8(TFT_RAMWR);
    DC_D;
}
static void lcd_write_bitmap(const void* data_in, uint32_t len){
  uint32_t *data = (uint32_t*)data_in;

  if (len > 31)
  {
    WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 511);
    while(len>31)
    {
      while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
      WRITE_PERI_REG(SPI_W0_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W1_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W2_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W3_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W4_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W5_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W6_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W7_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W8_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W9_REG(SPI_PORT),  *data++);
      WRITE_PERI_REG(SPI_W10_REG(SPI_PORT), *data++);
      WRITE_PERI_REG(SPI_W11_REG(SPI_PORT), *data++);
      WRITE_PERI_REG(SPI_W12_REG(SPI_PORT), *data++);
      WRITE_PERI_REG(SPI_W13_REG(SPI_PORT), *data++);
      WRITE_PERI_REG(SPI_W14_REG(SPI_PORT), *data++);
      WRITE_PERI_REG(SPI_W15_REG(SPI_PORT), *data++);
#if CONFIG_IDF_TARGET_ESP32S3
      SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_UPDATE);
      while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_UPDATE);
#endif
      SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
      len -= 32;
    }
  }

  if (len)
  {
    while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
    WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), (len << 4) - 1);
    for (uint32_t i=0; i <= (len<<1); i+=4) WRITE_PERI_REG((SPI_W0_REG(SPI_PORT) + i), *data++);
#if CONFIG_IDF_TARGET_ESP32S3
      SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_UPDATE);
      while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_UPDATE);
#endif
    SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
  }
  while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT))&SPI_USR);
}
static void lcd_st7789_init() {
    pinMode(pin::lcd_cs, OUTPUT);
    digitalWrite(pin::lcd_cs, HIGH); // Chip select high (inactive)
    pinMode(pin::lcd_dc, OUTPUT);
    digitalWrite(pin::lcd_dc, HIGH); // D/C high (data mode)
    lcd_spi.begin(pin::spi_clk,-1,pin::spi_mosi);
    lcd_begin_write();
    lcd_command(0x01); // reset
    lcd_end_write();
    delay(150); // Wait for reset to complete
    lcd_begin_write();
    lcd_command(0x11);   // Sleep out
    delay(120);
    lcd_command(0x13);    // Normal display mode on
    lcd_command(0x36);
    lcd_param(0x08);
    lcd_command(0xB6);
    lcd_param(0x0A);
    lcd_param(0x82);
    lcd_command(0xB0);
    lcd_param(0x00);
    lcd_param(0xE0); // 5 to 6-bit conversion: r0 = r5, b0 = b5
    lcd_command(0x3A);
    lcd_param(0x55);
    delay(10);
    lcd_command(0xB2);
    lcd_param(0x0c);
    lcd_param(0x0c);
    lcd_param(0x00);
    lcd_param(0x33);
    lcd_param(0x33);
    lcd_command(0xB7);      // Voltages: VGH / VGL
    lcd_param(0x35);
    lcd_command(0xBB);
    lcd_param(0x28);		// JLX240 display datasheet
    lcd_command(0xC0);
    lcd_param(0x0C);
    lcd_command(0xC2);
    lcd_param(0x01);
    lcd_param(0xFF);
    lcd_command(0xC3);       // voltage VRHS
    lcd_param(0x10);
    lcd_command(0xC4);
    lcd_param(0x20);
    lcd_command(0xC6);
    lcd_param(0x0F);
    lcd_command(0xD0);
    lcd_param(0xa4);
    lcd_param(0xa1);
    lcd_command(0xE0);
    lcd_param(0xd0);
    lcd_param(0x00);
    lcd_param(0x02);
    lcd_param(0x07);
    lcd_param(0x0a);
    lcd_param(0x28);
    lcd_param(0x32);
    lcd_param(0x44);
    lcd_param(0x42);
    lcd_param(0x06);
    lcd_param(0x0e);
    lcd_param(0x12);
    lcd_param(0x14);
    lcd_param(0x17);
    lcd_command(0xE1);
    lcd_param(0xd0);
    lcd_param(0x00);
    lcd_param(0x02);
    lcd_param(0x07);
    lcd_param(0x0a);
    lcd_param(0x28);
    lcd_param(0x31);
    lcd_param(0x54);
    lcd_param(0x47);
    lcd_param(0x0e);
    lcd_param(0x1c);
    lcd_param(0x17);
    lcd_param(0x1b);
    lcd_param(0x1e);
    lcd_command(0x21);
    lcd_command(0x2A);    // Column address set
    lcd_param(0x00);
    lcd_param(0x00);
    lcd_param(0x00);
    lcd_param(0xEF);    // 239
    lcd_command(0x2B);    // Row address set
    lcd_param(0x00);
    lcd_param(0x00);
    lcd_param(0x01);
    lcd_param(0x3F);    // 319
    lcd_end_write();
    delay(120);
    lcd_begin_write();
    lcd_command(0x29);
    delay(120);
    lcd_command(0x20);
    lcd_end_write();
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
void lcd_initialize(size_t lcd_transfer_buffer_size) {
#ifdef NO_LCD_PANEL_API
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
    lcd_begin_write();
    lcd_set_window(x1,y1,x2,y2);
    lcd_write_bitmap(bitmap, w * h);
    lcd_end_write();
    lcd_on_flush_complete();
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
    //lcd_panel.setRotation(rotation);
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
// camera thread
void camera_task(void* pvParameters) {
    camera_fb_t* fb = NULL;  // data structure of camera frame buffer
    camera_fb_t* fb_buf = NULL;
    while (true) {
        fb = esp_camera_fb_get();
        fb_buf = fb;
        esp_camera_fb_return(fb);
        if (fb_buf != NULL && camera_fb!=nullptr) {
            if(pdTRUE==xSemaphoreTake(camera_fb_lock,50)) {
                /*for (int i = 0; i < fb_buf->len; i += 2) {
                    uint8_t temp = 0;
                    temp = fb_buf->buf[i];
                    camera_fb[i] = fb_buf->buf[i + 1];
                    camera_fb[i + 1] = temp;
                }*/
                memcpy(camera_fb,fb_buf->buf,fb->len);
                camera_on_frame(camera_fb);
                xSemaphoreGive(camera_fb_lock);
                
            }
        }
    }
}
const void* camera_lock_frame_buffer(bool wait) {
    if (camera_fb!=nullptr && pdTRUE == xSemaphoreTake(camera_fb_lock, wait ? portMAX_DELAY : 0)) {
        return camera_fb;
    }
    return nullptr;
}
void camera_unlock_frame_buffer() { xSemaphoreGive(camera_fb_lock); }
void camera_initialize(int flags) {
    camera_fb_lock = xSemaphoreCreateMutex();
    if(camera_fb_lock==nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    camera_fb = (uint8_t*)heap_caps_malloc(2 * 240 * 240, 0!=(flags&CAM_ALLOC_FB_PSRAM)?MALLOC_CAP_SPIRAM:MALLOC_CAP_DEFAULT );
    if (camera_fb == nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    camera_config_t config;
    memset(&config,0,sizeof(config));
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
    config.xclk_freq_hz = 20*1000000;
    config.frame_size = 0!=(flags &CAM_FRAME_SIZE_96X96)?FRAMESIZE_96X96:FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = 0!=(flags&CAM_ALLOC_CAM_PSRAM)?CAMERA_FB_IN_PSRAM:CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 10;
    config.fb_count = 2;
    
    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);      // flip it back
    //s->set_contrast(s,0);
    s->set_hmirror(s, 1);    // horizontal mirror image
    s->set_brightness(s, 0); // up the brightness just a bit
    s->set_saturation(s, 0); // lower the saturation
    xTaskCreate(camera_task, "camera_task", 10*1024, NULL, 1, &camera_task_handle);

}
void camera_deinitialize() {
    if(camera_task_handle!=nullptr) {
        vTaskDelete(camera_task_handle);
        camera_task_handle = nullptr;
    }
    if(camera_fb!=nullptr) {
        free(camera_fb);
        camera_fb = nullptr;
    }
    if(camera_fb_lock!=nullptr) {
        vSemaphoreDelete(camera_fb_lock);
    }
    esp_camera_deinit();
    
}
void camera_on_frame(const void* bitmap) {
    // do nothing
}