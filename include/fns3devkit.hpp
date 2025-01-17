#pragma once
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <stdint.h>
#endif

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
    constexpr static const int16_t i2s_bclk=42;
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

enum {
    CAM_ALLOC_FB_PSRAM=(1<<0),
    CAM_ALLOC_CAM_PSRAM=(1<<1),
    CAM_FRAME_SIZE_96X96=(1<<2)
};
enum cam_level {
    CAM_NO_CHANGE = -3,
    CAM_LOWEST,
    CAM_LOW,
    CAM_MEDIUM,
    CAM_HIGH,
    CAM_HIGHEST
};
enum audio_format {
    AUDIO_44_1K_MONO=0,
    AUDIO_44_1K_STEREO,
    AUDIO_22K_MONO,
    AUDIO_22K_STEREO,
    AUDIO_11K_MONO,
    AUDIO_11K_STEREO,
};
extern void neopixel_initialize(uint8_t rmt_channel = 0, uint8_t rmt_interrupt=23);
extern void neopixel_color(uint32_t rgbw);
extern void neopixel_color_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w=0);
extern void neopixel_deinitialize();

extern void lcd_initialize(size_t max_transfer_size = 32768, bool initialize_touch = true);
extern void lcd_deinitialize();
#ifdef IRAM_ATTR
IRAM_ATTR
#endif
void lcd_on_flush_complete(); // implemented by user
extern void lcd_flush_bitmap(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const void* bitmap);
extern void lcd_rotation(uint8_t rotation);

extern void touch_initialize(uint8_t threshhold = 32);
extern void touch_rotation(uint8_t rotation);
extern bool touch_xy(uint16_t* out_x, uint16_t* out_y);
extern bool touch_xy2(uint16_t* out_x, uint16_t* out_y);

extern void camera_initialize(int flags=CAM_ALLOC_CAM_PSRAM|CAM_ALLOC_FB_PSRAM);
extern void camera_levels(cam_level brightness, cam_level contrast=CAM_NO_CHANGE, cam_level saturation=CAM_NO_CHANGE, cam_level sharpness=CAM_NO_CHANGE);
extern void camera_rotation(uint8_t rotation);
extern void camera_deinitialize();
extern void camera_on_frame(const void* bitmap)  __attribute__((weak));; // optionally implemented by user
extern const void* camera_lock_frame_buffer(bool lock=true);
extern void camera_unlock_frame_buffer();

constexpr const size_t audio_max_samples = 1024;
extern void audio_initialize(audio_format format = AUDIO_44_1K_STEREO);
extern void audio_deinitialize();
extern size_t audio_write_int16(const int16_t* samples, size_t sample_count);
extern size_t audio_write_float(const float* samples, size_t sample_count, float vel = 1.0f);

extern void led_enable(bool value);

