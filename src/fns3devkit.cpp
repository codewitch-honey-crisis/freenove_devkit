#include "fns3devkit.hpp"
// #include <ft6336.hpp>
#include <driver/rmt.h>
#ifdef NO_LCD_PANEL_API
// this code adapted from https://github.com/Bodmer/TFT_eSPI
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>
#include <soc/spi_reg.h>
#else
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#endif
#include <driver/i2s.h>
#include <esp_camera.h>

#define NEOPIXEL_TASK_SIZE (1024)
#define NEOPIXEL_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define NEOPIXEL_REFRESH_PERIOD_MS \
    (30U)  // TODO: add as parameter to neopixel_init
#define NEOPIXEL_NUM_RMT_ITEMS_PER_LED \
    (24U)  // Assumes 24 bit color for each led
// RMT Clock source is @ 80 MHz. Dividing it by 8 gives us 10 MHz frequency, or
// 100ns period.
#define NEOPIXEL_RMT_CLK_DIV (8)
/****************************
        WS2812 Timing
 ****************************/
#define NEOPIXEL_RMT_TICKS_BIT_1_HIGH_WS2812 \
    9  // 900ns (900ns +/- 150ns per datasheet)
#define NEOPIXEL_RMT_TICKS_BIT_1_LOW_WS2812 \
    3  // 300ns (350ns +/- 150ns per datasheet)
#define NEOPIXEL_RMT_TICKS_BIT_0_HIGH_WS2812 \
    3  // 300ns (350ns +/- 150ns per datasheet)
#define NEOPIXEL_RMT_TICKS_BIT_0_LOW_WS2812 \
    9  // 900ns (900ns +/- 150ns per datasheet)

// Function pointer for generating waveforms based on different LED drivers
typedef void (*led_fill_rmt_items_fn)(uint32_t* neopixel_buf,
                                      rmt_item32_t* rmt_items,
                                      uint32_t neopixel_length);

static inline void neopixel_fill_item_level(rmt_item32_t* item, int high_ticks,
                                            int low_ticks) {
    item->level0 = 1;
    item->duration0 = high_ticks;
    item->level1 = 0;
    item->duration1 = low_ticks;
}

static inline void neopixel_rmt_bit_1_ws2812(rmt_item32_t* item) {
    neopixel_fill_item_level(item, NEOPIXEL_RMT_TICKS_BIT_1_HIGH_WS2812,
                             NEOPIXEL_RMT_TICKS_BIT_1_LOW_WS2812);
}

static inline void neopixel_rmt_bit_0_ws2812(rmt_item32_t* item) {
    neopixel_fill_item_level(item, NEOPIXEL_RMT_TICKS_BIT_0_HIGH_WS2812,
                             NEOPIXEL_RMT_TICKS_BIT_0_LOW_WS2812);
}

static void neopixel_fill_rmt_items_ws2812(uint32_t* neopixel_buf,
                                           rmt_item32_t* rmt_items) {
    uint32_t rmt_items_index = 0;
    const size_t led_index = 0;
    uint32_t led_color = neopixel_buf[led_index];

    for (uint8_t bit = 8; bit != 0; --bit) {
        uint8_t bit_set = (((led_color & 0x00FF00) >> 8) >> (bit - 1)) & 1;
        if (bit_set) {
            neopixel_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
        } else {
            neopixel_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
        }
        ++rmt_items_index;
    }
    for (uint8_t bit = 8; bit != 0; --bit) {
        uint8_t bit_set = (((led_color & 0xFF0000) >> 16) >> (bit - 1)) & 1;
        if (bit_set) {
            neopixel_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
        } else {
            neopixel_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
        }
        ++rmt_items_index;
    }
    for (uint8_t bit = 8; bit != 0; --bit) {
        uint8_t bit_set = ((led_color & 0xFF) >> (bit - 1)) & 1;
        if (bit_set) {
            neopixel_rmt_bit_1_ws2812(&(rmt_items[rmt_items_index]));
        } else {
            neopixel_rmt_bit_0_ws2812(&(rmt_items[rmt_items_index]));
        }
        ++rmt_items_index;
    }
}
static uint32_t neopixel_value = 0;
static rmt_item32_t* neopixel_rmts = nullptr;
static uint8_t neopixel_rmt_channel = 0;
void neopixel_initialize(uint8_t rmt_channel, uint8_t rmt_interrupt) {
    if (neopixel_rmts != nullptr) {
        neopixel_deinitialize();
    }
    size_t num_items_malloc = NEOPIXEL_NUM_RMT_ITEMS_PER_LED;
    rmt_item32_t* rmts =
        (rmt_item32_t*)malloc(sizeof(rmt_item32_t) * num_items_malloc);
    neopixel_rmt_channel = rmt_channel;
    neopixel_value = 0;
    rmt_config_t rmt_cfg;
    memset(&rmt_cfg, 0, sizeof(rmt_cfg));
    rmt_cfg.rmt_mode = RMT_MODE_TX;
    rmt_cfg.channel = (rmt_channel_t)rmt_channel;
    rmt_cfg.clk_div = NEOPIXEL_RMT_CLK_DIV;
    rmt_cfg.gpio_num = (gpio_num_t)pin::neopixel;
    rmt_cfg.mem_block_num = 1;

    rmt_cfg.tx_config.loop_en = false;
    rmt_cfg.tx_config.carrier_freq_hz =
        100;  // Not used, but has to be set to avoid divide by 0 err
    rmt_cfg.tx_config.carrier_duty_percent = 50;
    rmt_cfg.tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW;
    rmt_cfg.tx_config.carrier_en = false;
    rmt_cfg.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    rmt_cfg.tx_config.idle_output_en = true;

    ESP_ERROR_CHECK(rmt_config(&rmt_cfg));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_cfg.channel, 0, 0));
    neopixel_rmts = rmts;
}
void neopixel_deinitialize() {
    if (neopixel_rmts = nullptr) {
        return;
    }
    rmt_driver_uninstall((rmt_channel_t)neopixel_rmt_channel);
    free(neopixel_rmts);
    neopixel_rmts = nullptr;
}
void neopixel_color(uint32_t color) {
    neopixel_value = color;
    rmt_wait_tx_done((rmt_channel_t)neopixel_rmt_channel, portMAX_DELAY);
    neopixel_fill_rmt_items_ws2812((uint32_t*)&neopixel_value, neopixel_rmts);
    rmt_write_items((rmt_channel_t)neopixel_rmt_channel,
                    (rmt_item32_t*)neopixel_rmts,
                    (NEOPIXEL_NUM_RMT_ITEMS_PER_LED), false);
}
void neopixel_color_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    neopixel_color((w << 24) | (r << 16) | (g << 8) | b);
}
static uint8_t touch_rotation_value;
static bool touch_initialized;
static size_t touch_count;
static uint16_t touch_x_data[2], touch_y_data[2], touch_id_data[2];

static void touch_write_reg(int r, int value) {
    Wire.beginTransmission(0x38);
    Wire.write((uint8_t)r);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}
static void touch_read_all() {
    uint8_t i2cdat[16];
    Wire.beginTransmission(0x38);
    Wire.write((uint8_t)0);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)0x38, (uint8_t)16);
    for (uint8_t i = 0; i < 16; i++) i2cdat[i] = Wire.read();
    touch_count = i2cdat[0x02];
    if (touch_count > 2) {
        touch_count = 0;
    }

    for (uint8_t i = 0; i < 2; i++) {
        touch_x_data[i] = i2cdat[0x03 + i * 6] & 0x0F;
        touch_x_data[i] <<= 8;
        touch_x_data[i] |= i2cdat[0x04 + i * 6];
        touch_y_data[i] = i2cdat[0x05 + i * 6] & 0x0F;
        touch_y_data[i] <<= 8;
        touch_y_data[i] |= i2cdat[0x06 + i * 6];
        touch_id_data[i] = i2cdat[0x05 + i * 6] >> 4;
    }
}
static void touch_translate(uint16_t& x, uint16_t& y) {
    uint16_t tmp;
    switch (touch_rotation_value & 3) {
        case 1:
            tmp = x;
            x = y;
            y = 240 - tmp - 1;
            break;
        case 2:
            x = 240 - x - 1;
            y = 320 - y - 1;
            break;
        case 3:
            tmp = x;
            x = 320 - y - 1;
            y = tmp;
        default:
            break;
    }
}
void touch_initialize(uint8_t threshhold) {
    if (!touch_initialized) {
        Wire.begin(pin::i2c_sda, pin::i2c_scl, 100 * 1000 * 1000);
        touch_write_reg(0x80, threshhold);
        touch_count = 0;
        touch_initialized = true;
    }
}
static bool touch_update() {
    static uint32_t ts = 0;
    if (millis() >= ts + 13) {
        ts = millis();
        touch_read_all();
    }
    return true;
}
static bool touch_read_point(size_t n, uint16_t* out_x, uint16_t* out_y) {
    if (touch_count == 0 || n >= touch_count) {
        if (out_x != nullptr) {
            *out_x = 0;
        }
        if (out_y != nullptr) {
            *out_y = 0;
        }
        return false;
    }
    uint16_t x = touch_x_data[n];
    uint16_t y = touch_y_data[n];
    if (x >= 240) {
        x = 240 - 1;
    }
    if (y >= 320) {
        y = 320 - 1;
    }
    touch_translate(x, y);
    if (out_x != nullptr) {
        *out_x = x;
    }
    if (out_y != nullptr) {
        *out_y = y;
    }
    return true;
}

bool touch_xy(uint16_t* out_x, uint16_t* out_y) {
    touch_update();
    return touch_read_point(0, out_x, out_y);
}
bool touch_xy2(uint16_t* out_x, uint16_t* out_y) {
    touch_update();
    return touch_read_point(1, out_x, out_y);
}
void touch_rotation(uint8_t rotation) { touch_rotation_value = rotation & 3; }
constexpr static const int lcd_speed = 80 * 1000 * 1000;
#ifdef NO_LCD_PANEL_API
#define DC_C GPIO.out_w1tc = (1 << pin::lcd_dc);
#define DC_D GPIO.out_w1ts = (1 << pin::lcd_dc);
#define CS_L                                        \
    GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32)); \
    GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32))
#define CS_H GPIO.out1_w1ts.val = (1 << (pin::lcd_cs - 32))
#ifdef USE_SPI_MASTER
static spi_device_handle_t lcd_spi_handle = nullptr;
#else
static bool lcd_initialized=false;
#define SPI_PORT 3
#ifndef REG_SPI_BASE  //                      HSPI                 FSPI/VSPI
#define REG_SPI_BASE(i) (((i) > 1) ? (DR_REG_SPI3_BASE) : (DR_REG_SPI2_BASE))
#endif

// Fix ESP32S3 IDF bug for name change
#ifndef SPI_MOSI_DLEN_REG
#define SPI_MOSI_DLEN_REG(x) SPI_MS_DLEN_REG(x)
#endif
#define TFT_CASET 0x2A
#define TFT_PASET 0x2B
#define TFT_RAMWR 0x2C
#define _spi_cmd (volatile uint32_t*)(SPI_CMD_REG(SPI_PORT))
#define _spi_user (volatile uint32_t*)(SPI_USER_REG(SPI_PORT))
#define _spi_mosi_dlen (volatile uint32_t*)(SPI_MOSI_DLEN_REG(SPI_PORT))
#define _spi_w (volatile uint32_t*)(SPI_W0_REG(SPI_PORT))
#define SET_BUS_WRITE_MODE *_spi_user = SPI_USR_MOSI
#define SET_BUS_READ_MODE *_spi_user = SPI_USR_MOSI | SPI_USR_MISO | SPI_DOUTDIN
#define SPI_BUSY_CHECK while (*_spi_cmd & SPI_USR)
#define CS_L                                        \
    GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32)); \
    GPIO.out1_w1tc.val = (1 << (pin::lcd_cs - 32))
#define CS_H GPIO.out1_w1ts.val = (1 << (pin::lcd_cs - 32))
#define DC_C GPIO.out_w1tc = (1 << pin::lcd_dc);
#define DC_D GPIO.out_w1ts = (1 << pin::lcd_dc);
#define SPI_UPDATE (BIT(23))
#define TFT_WRITE_BITS(D, B)        \
    *_spi_mosi_dlen = B - 1;        \
    *_spi_w = D;                    \
    *_spi_cmd = SPI_UPDATE;         \
    while (*_spi_cmd & SPI_UPDATE); \
    *_spi_cmd = SPI_USR;            \
    while (*_spi_cmd & SPI_USR);
#define tft_Write_8(C) TFT_WRITE_BITS(C, 8)
#define tft_Write_32C(C, D)                                \
    TFT_WRITE_BITS((uint16_t)((D) << 8 | (D) >> 8) << 16 | \
                       (uint16_t)((C) << 8 | (C) >> 8),    \
                   32)
SPIClass lcd_spi(HSPI);
#endif

static void lcd_begin_write() {
#ifdef USE_SPI_MASTER
    // CS_L;
#else
    lcd_spi.beginTransaction(SPISettings(lcd_speed, MSBFIRST, SPI_MODE0));
    CS_L;
    SET_BUS_WRITE_MODE;
#endif
}
static void lcd_end_write() {
#ifdef USE_SPI_MASTER
    // CS_H;
#else
    SPI_BUSY_CHECK;
    CS_H;
    SET_BUS_READ_MODE;
    lcd_spi.endTransaction();
#endif
}
static void lcd_command(uint8_t cmd, const uint8_t* args = NULL,
                        size_t len = 0) {
#ifdef USE_SPI_MASTER
    // gpio_set_level((gpio_num_t)pin::lcd_dc,0);
    // digitalWrite(pin::lcd_dc,LOW);
    spi_transaction_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.length = 8;
    tx.tx_buffer = &cmd;
    tx.user = 0;
    spi_device_polling_transmit(lcd_spi_handle, &tx);
    // gpio_set_level((gpio_num_t)pin::lcd_dc,1);
    // digitalWrite(pin::lcd_dc,HIGH);
    if (len && args) {
        tx.length = 8 * len;
        tx.tx_buffer = args;
        tx.user = (void*)1;
        spi_device_polling_transmit(lcd_spi_handle, &tx);
    }
#else
    DC_C;
    tft_Write_8(cmd);
    DC_D;
    while (len--) {
        tft_Write_8(*(args++));
    }
#endif
}

static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
#ifdef USE_SPI_MASTER
    uint8_t args[4];
    args[0] = (x1 & 0xFF) << 8;
    args[1] = (x1 >> 8);
    args[2] = (x2 & 0xFF) << 8;
    args[3] = (x2 >> 8);

    lcd_command(0x2A, args, 4);
    args[0] = (y1 & 0xFF) << 8;
    args[1] = (y1 >> 8);
    args[2] = (y2 & 0xFF) << 8;
    args[3] = (y2 >> 8);
    lcd_command(0x2B, args, 4);
    lcd_command(0x2c);
#else
    SPI_BUSY_CHECK;
    DC_C;
    tft_Write_8(TFT_CASET);
    DC_D;
    tft_Write_32C(x1, x2);
    DC_C;
    tft_Write_8(TFT_PASET);
    DC_D;
    tft_Write_32C(y1, y2);
    DC_C;
    tft_Write_8(TFT_RAMWR);
    DC_D;
#endif
}
static void lcd_write_bitmap(const void* data_in, uint32_t len) {
#ifdef USE_SPI_MASTER
    if (len) {
        // gpio_set_level((gpio_num_t)pin::lcd_dc,1);
        // digitalWrite(pin::lcd_dc,HIGH);
        spi_transaction_t tx;
        memset(&tx, 0, sizeof(tx));
        tx.length = 8 * len;
        tx.tx_buffer = data_in;
        tx.user = (void*)2;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, &tx, portMAX_DELAY));
    }
#else
    uint32_t* data = (uint32_t*)data_in;
    if (len > 31) {
        WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), 511);
        while (len > 31) {
            while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT)) & SPI_USR);
            WRITE_PERI_REG(SPI_W0_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W1_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W2_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W3_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W4_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W5_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W6_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W7_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W8_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W9_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W10_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W11_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W12_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W13_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W14_REG(SPI_PORT), *data++);
            WRITE_PERI_REG(SPI_W15_REG(SPI_PORT), *data++);
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_UPDATE);
            while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT)) & SPI_UPDATE);
            SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
            len -= 32;
        }
    }

    if (len) {
        while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT)) & SPI_USR);
        WRITE_PERI_REG(SPI_MOSI_DLEN_REG(SPI_PORT), (len << 4) - 1);
        for (uint32_t i = 0; i <= (len << 1); i += 4)
            WRITE_PERI_REG((SPI_W0_REG(SPI_PORT) + i), *data++);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_UPDATE);
        while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT)) & SPI_UPDATE);
        SET_PERI_REG_MASK(SPI_CMD_REG(SPI_PORT), SPI_USR);
    }
    while (READ_PERI_REG(SPI_CMD_REG(SPI_PORT)) & SPI_USR);
#endif
}
static void lcd_st7789_init() {
    lcd_begin_write();
    lcd_command(0x01);  // reset
    lcd_end_write();
    delay(150);  // Wait for reset to complete
    lcd_begin_write();
    lcd_command(0x11);  // Sleep out
    delay(120);
    lcd_command(0x13);  // Normal display mode on
    static const uint8_t params1 = 0x08;
    lcd_command(0x36, &params1, 1);
    static const uint8_t params2[] = {0x0A, 0xB2};
    lcd_command(0xB6, params2, 2);
    static const uint8_t params3[] = {0x00, 0xE0};
    lcd_command(0xB0, params3, 2);
    static const uint8_t params4 = 0x55;
    lcd_command(0x3A, &params4, 1);
    delay(10);
    static const uint8_t params5[] = {0x0C, 0xC, 0x00, 0x33, 0x33};
    lcd_command(0xB2, params5, 5);
    static const uint8_t params6 = 0x35;
    lcd_command(0xB7, &params6, 1);  // Voltages: VGH / VGL
    static const uint8_t params7 = 0x28;
    lcd_command(0xBB, &params7, 1);
    static const uint8_t params8 = 0x0C;
    lcd_command(0xC0, &params8, 1);
    static const uint8_t params9[] = {0x01, 0xFF};
    lcd_command(0xC2, params9, 2);
    static const uint8_t params10 = 0x10;
    lcd_command(0xC3, &params10, 1);  // voltage VRHS
    static const uint8_t params11 = 0x20;
    lcd_command(0xC4, &params11, 1);
    static const uint8_t params12 = 0x0F;
    lcd_command(0xC6, &params12, 1);
    static const uint8_t params13[] = {0xA4, 0xA1};
    lcd_command(0xD0, params13, 2);
    static const uint8_t params14[] = {0xD0, 0x00, 0x02, 0x07, 0x0A,
                                       0x28, 0x32, 0x44, 0x42, 0x06,
                                       0x0E, 0x12, 0x14, 0x17};
    lcd_command(0xE0, params14, 14);
    static const uint8_t params15[] = {0xD0, 0x00, 0x02, 0x07, 0x0A,
                                       0x28, 0x31, 0x54, 0x47, 0x0E,
                                       0x1C, 0x17, 0x1B, 0x1E};
    lcd_command(0xE1, params15, 14);
    lcd_command(0x21);
    static const uint8_t params16[] = {0x00, 0x00, 0x00, 0xEF};
    lcd_command(0x2A, params16, 4);  // Column address set
    static const uint8_t params17[] = {0x00, 0x00, 0x01, 0x3F};
    lcd_command(0x2B, params17, 4);  // Row address set
    lcd_end_write();
    delay(120);
    lcd_begin_write();
    lcd_command(0x29);
    delay(120);
    lcd_command(0x20);
    lcd_end_write();
}
#ifdef USE_SPI_MASTER
IRAM_ATTR void lcd_spi_pre_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        //for (int i = 0; i < 30; ++i) {
            //gpio_set_level((gpio_num_t)pin::lcd_dc, 0);
            DC_C;
        //}
    } else {
        //gpio_set_level((gpio_num_t)pin::lcd_dc, 1);
        DC_D;
    }
}
IRAM_ATTR void lcd_spi_post_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        //for (int i = 0; i < 30; ++i) {
            //gpio_set_level((gpio_num_t)pin::lcd_dc, 1);
            DC_D;
        //}
    } else {
        //for (int i = 0; i < 30; ++i) {
            //gpio_set_level((gpio_num_t)pin::lcd_dc, 0);
            DC_D;
        //}
        if (((int)trans->user) == 2) {
            lcd_on_flush_complete();
        }
    }
}
#endif
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
void lcd_initialize(size_t lcd_transfer_buffer_size, bool init_touch) {
#ifdef NO_LCD_PANEL_API
#ifdef USE_SPI_MASTER
    spi_bus_config_t bus_cfg;
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.max_transfer_sz = lcd_transfer_buffer_size + 8;
    bus_cfg.miso_io_num = -1;
    bus_cfg.mosi_io_num = pin::spi_mosi;
    bus_cfg.sclk_io_num = pin::spi_clk;
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dummy_bits = 0;
    dev_cfg.queue_size = 10;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX;
    dev_cfg.spics_io_num = pin::lcd_cs;
    dev_cfg.pre_cb = lcd_spi_pre_cb;
    dev_cfg.post_cb = lcd_spi_post_cb;
    dev_cfg.clock_speed_hz = lcd_speed;
    dev_cfg.cs_ena_posttrans = 1;
    dev_cfg.cs_ena_pretrans = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &lcd_spi_handle));
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << pin::lcd_dc);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)pin::lcd_dc, 0);
#else
    if(lcd_initialized) {
        return;
    }
    pinMode(pin::lcd_cs, OUTPUT);
    digitalWrite(pin::lcd_cs, HIGH);  // Chip select high (inactive)
    pinMode(pin::lcd_dc, OUTPUT);
    digitalWrite(pin::lcd_dc, HIGH);  // D/C high (data mode)

    lcd_spi.begin(pin::spi_clk, -1, pin::spi_mosi);
    lcd_initialized = true;
#endif
    lcd_st7789_init();
#else
    if(lcd_handle!=nullptr) {
        return;
    }
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (((unsigned long long)1) << pin::lcd_dc);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&gpio_conf);
    gpio_set_direction((gpio_num_t)pin::lcd_cs, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin::lcd_cs, 0);
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
    io_config.cs_gpio_num = -1;  // pin::lcd_cs;
    io_config.pclk_hz = lcd_speed;
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
    if (init_touch) {
        touch_initialize();
    }
}
void lcd_flush_bitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                      const void* bitmap) {
#ifdef NO_LCD_PANEL_API
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    lcd_begin_write();
    lcd_set_window(x1, y1, x2, y2);
    lcd_write_bitmap(bitmap, w * h);
    lcd_end_write();
    lcd_on_flush_complete();
#else
    esp_lcd_panel_draw_bitmap(lcd_handle, x1, y1, x2 + 1, y2 + 1,
                              (void*)bitmap);
#endif
}
void lcd_deinitialize() {
#ifdef NO_LCD_PANEL_API
#ifdef USE_SPI_MASTER
ESP_ERROR_CHECK(spi_bus_remove_device(lcd_spi_handle));
ESP_ERROR_CHECK(spi_bus_free((spi_host_device_t)SPI3_HOST));
#else
lcd_spi.end();
lcd_initialized = false;
#endif
#else
ESP_ERROR_CHECK(esp_lcd_panel_del(lcd_handle));
lcd_handle = nullptr;
ESP_ERROR_CHECK(esp_lcd_panel_io_del(lcd_io_handle));
lcd_io_handle = nullptr;
ESP_ERROR_CHECK(spi_bus_free((spi_host_device_t)SPI3_HOST));
#endif
}
void lcd_touch_update() { touch_update(); }

void lcd_rotation(uint8_t rotation) {
#ifdef NO_LCD_PANEL_API
    lcd_begin_write();
    uint8_t param;
    switch (rotation & 3) {
        case 1:
            param = (0x40 | 0x20 | 0x08);
            break;
        case 2:
            param = (0x40 | 0x80 | 0x08);
            break;
        case 3:
            param = (0x20 | 0x80 | 0x08);
            break;
        default:  // case 0:
            param = (0x08);
            break;
    };
    lcd_command(0x36, &param, 1);

    lcd_end_write();
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
    touch_rotation(rotation & 3);
}

static TaskHandle_t camera_task_handle;  // camera thread task handle
static uint8_t* camera_fb = nullptr;
static SemaphoreHandle_t camera_fb_lock = nullptr;
static uint8_t camera_rot = 0;
void camera_copy_rotate(void* bitmap, int rows, int cols) {
    // allocating space for the new rotated image
    const uint16_t* original = (const uint16_t*)bitmap;
    uint16_t* out = (uint16_t*)camera_fb;
    switch (camera_rot) {
        case 1: {
            // rotate 90
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(cols - x - 1) * rows + y] = *(original++);
                }
            }
            break;
        }
        case 2: {
            // rotate 180
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(rows - y - 1) * cols + (cols - x - 1)] = *(original++);
                }
            }
            break;
        }
        case 3: {
            // rotate 270
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[x * rows + (rows - y - 1)] = *(original++);
                }
            }
            break;
        }
        default:  // case 0:
            memcpy(camera_fb, original, rows * cols * 2);
            break;
    }
}
// camera thread
void camera_task(void* pvParameters) {
    camera_fb_t* fb_buf = NULL;
    while (true) {
        fb_buf = esp_camera_fb_get();
        if (fb_buf != NULL && camera_fb != nullptr) {
            if (pdTRUE == xSemaphoreTake(camera_fb_lock, 50)) {
                camera_copy_rotate(fb_buf->buf, fb_buf->width, fb_buf->height);
                camera_on_frame(camera_fb);
                xSemaphoreGive(camera_fb_lock);
            }
        }
        esp_camera_fb_return(fb_buf);
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
    if (camera_fb == nullptr) {
        camera_rot = rotation & 3;
        return;
    }
    xSemaphoreTake(camera_fb_lock, portMAX_DELAY);
    camera_rot = rotation & 3;
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
void camera_levels(cam_level brightness, cam_level contrast,
                   cam_level saturation, cam_level sharpness) {
    if (camera_fb == nullptr) {
        return;
    }
    sensor_t* s = esp_camera_sensor_get();
    if (brightness != CAM_NO_CHANGE) {
        s->set_brightness(s, (int)brightness);
    }
    if (contrast != CAM_NO_CHANGE) {
        s->set_contrast(s, (int)contrast);
    }
    if (saturation != CAM_NO_CHANGE) {
        s->set_saturation(s, (int)saturation);
    }
    if (sharpness != CAM_NO_CHANGE) {
        s->set_sharpness(s, (int)sharpness);
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
static uint16_t audio_out_buffer[audio_max_samples];

void audio_initialize(audio_format format) {
    i2s_config_t i2s_config;
    i2s_pin_config_t pins;
    memset(&i2s_config, 0, sizeof(i2s_config_t));
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_config.sample_rate =
        (format == AUDIO_22K_MONO || format == AUDIO_22K_STEREO) ? 22050
                                                                 : 44100;
    i2s_config.bits_per_sample = (i2s_bits_per_sample_t)16;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_MSB;
    i2s_config.dma_buf_count = 14;
    i2s_config.dma_buf_len = audio_max_samples;
    i2s_config.use_apll = true;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2;
    ESP_ERROR_CHECK(i2s_driver_install((i2s_port_t)0, &i2s_config, 0, NULL));
    pins = {.mck_io_num = I2S_PIN_NO_CHANGE,  // Unused
            .bck_io_num = pin::i2s_bclk,
            .ws_io_num = pin::i2s_lrc,
            .data_out_num = pin::i2s_dout,
            .data_in_num = I2S_PIN_NO_CHANGE};
    ESP_ERROR_CHECK(i2s_set_pin((i2s_port_t)0, &pins));

    i2s_set_clk((i2s_port_t)0, 44100, 16,
                (format == AUDIO_22K_STEREO || format == AUDIO_44_1K_STEREO)
                    ? I2S_CHANNEL_STEREO
                    : I2S_CHANNEL_MONO);
    i2s_zero_dma_buffer((i2s_port_t)0);
}
void audio_deinitialize() { i2s_driver_uninstall((i2s_port_t)0); }
size_t audio_write_int16(const int16_t* samples, size_t sample_count) {
    size_t result = 0;
    const int16_t* p = (const int16_t*)samples;
    uint16_t* out = audio_out_buffer;
    while (sample_count) {
        size_t to_write =
            sample_count < audio_max_samples ? sample_count : audio_max_samples;
        for (int i = 0; i < to_write; ++i) {
            *(out++) = uint16_t(*(p++) + 32768);
        }
        size_t written = to_write * 2;
        i2s_write((i2s_port_t)0, audio_out_buffer, to_write * 2, &written,
                  portMAX_DELAY);
        size_t samples_written = written >> 1;
        sample_count -= samples_written;
        result += samples_written;
        if (samples_written != to_write) {
            return samples_written;
        }
    }
    return result;
}

size_t audio_write_float(const float* samples, size_t sample_count, float vel) {
    size_t result = 0;
    const float* p = (const float*)samples;
    uint16_t* out = audio_out_buffer;
    while (sample_count) {
        size_t to_write =
            sample_count < audio_max_samples ? sample_count : audio_max_samples;
        for (int i = 0; i < to_write; ++i) {
            float fval = *(p++) * vel;
            int16_t val = fval > 0 ? fval * 32767 : fval * 32768;
            *(out++) = uint16_t(val + 32768);
        }
        size_t written;
        i2s_write((i2s_port_t)0, audio_out_buffer, to_write * 2, &written,
                  portMAX_DELAY);
        size_t samples_written = written >> 1;
        sample_count -= samples_written;
        result += samples_written;
        if (samples_written != to_write) {
            return samples_written;
        }
    }
    return result;
}

extern void led_enable(bool value) {
    pinMode(pin::led, OUTPUT);
    digitalWrite(pin::led, value ? HIGH : LOW);
}