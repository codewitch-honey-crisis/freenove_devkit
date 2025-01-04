#pragma once
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <stdint.h>
#endif
#include <esp_i2c.hpp>
struct pin {
    constexpr static const int16_t sdmmc_d0=40;
    constexpr static const int16_t sdmmc_clk=39;
    constexpr static const int16_t sdmmc_cmd=38;
    constexpr static const int16_t led=2;
    constexpr static const int16_t neopixel=48;
    constexpr static const int16_t cam_siod=4;
    constexpr static const int16_t cam_sioc=5;
    constexpr static const int16_t cam_vsync=6;
    constexpr static const int16_t cam_href=7;
    constexpr static const int16_t cam_xclk=15;
    constexpr static const int16_t cam_y9=16;
    constexpr static const int16_t cam_y8=17;
    constexpr static const int16_t cam_y7=18;
    constexpr static const int16_t cam_y6=10;
    constexpr static const int16_t cam_y5=10;
    constexpr static const int16_t cam_y4=8;
    constexpr static const int16_t cam_y3=9;
    constexpr static const int16_t cam_y2=11;
    constexpr static const int16_t cam_pclk=13;
    constexpr static const int16_t cam_pwdn=-1;
    constexpr static const int16_t cam_rst=-1;
    constexpr static const int16_t i2s_blck=42;
    constexpr static const int16_t i2s_dout=41;
    constexpr static const int16_t i2s_lrc=14;
    constexpr static const int16_t batt_in=19;
    constexpr static const int16_t i2c_sda=2;
    constexpr static const int16_t i2c_scl=1;
    constexpr static const int16_t touch_int=-1;
    constexpr static const int16_t touch_rst=-1;
    constexpr static const int16_t spi_mosi=20;
    constexpr static const int16_t spi_clk=21;
    constexpr static const int16_t lcd_cs=47;
    constexpr static const int16_t lcd_dc=0;
    constexpr static const int16_t lcd_rst=-1;
    constexpr static const int16_t lcd_bkl=-1;
};
#ifdef ARDUINO
using i2c_t = arduino::esp_i2c<0,pin::i2c_sda,pin::i2c_scl>;
#else
using i2c_t = esp_idf::esp_i2c<0,pin::i2c_sda,pin::i2c_scl>;
#endif
enum {
    CAM_ALLOC_FB_PSRAM=(1<<0),
    CAM_ALLOC_CAM_PSRAM=(1<<1),
    CAM_FRAME_SIZE_96X96=(1<<2)
};
extern void lcd_initialize(size_t max_transfer_size = 32768);
extern void lcd_on_flush_complete(); // implemented by user
extern void lcd_flush_bitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const void* bitmap);
extern void lcd_touch_update();
extern bool lcd_touch_pressed(uint16_t* out_x,uint16_t* out_y);
extern void lcd_rotation(uint8_t rotation);

extern void camera_initialize(int flags=CAM_ALLOC_CAM_PSRAM|CAM_ALLOC_FB_PSRAM);
extern void camera_deinitialize();
extern void camera_on_frame(const void* bitmap)  __attribute__((weak));; // optionall implemented by user
extern const void* camera_lock_frame_buffer(bool lock=true);
extern void camera_unlock_frame_buffer();