#include "freenove_s3_devkit.h"
#include <memory.h>
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
// TODO: Update the following when the camera component is updated
#include "driver/i2c.h"

#define LCD_DC 0
#define LCD_CS 47
#define LCD_MOSI 20
#define LCD_CLK 21
#define LCD_SPEED (80 * 1000 * 1000)

#define LED 2

#define CAM_SIOD 4
#define CAM_SIOC 5
#define CAM_VSYNC 6
#define CAM_HREF 7
#define CAM_XCLK 15
#define CAM_Y9 16
#define CAM_Y8 17
#define CAM_Y7 18
#define CAM_Y6 10
#define CAM_Y5 10
#define CAM_Y4 8
#define CAM_Y3 9
#define CAM_Y2 11
#define CAM_PCLK 13
#define CAM_PWDN -1
#define CAM_RST -1
#define CAM_SPEED (20 * 1000 * 1000)

#define AUD_BCLK 42
#define AUD_DOUT 41
#define AUD_LRC 14

#define NEOPIXEL 48

#define I2C_SCL 1
#define I2C_SDA 2
#define I2C_SPEED (200*1000)

#define SDMMC_D0 40
#define SDMMC_CLK 39
#define SDMMC_CMD 38

#define DC_C GPIO.out_w1tc = (1 << LCD_DC);
#define DC_D GPIO.out_w1ts = (1 << LCD_DC);

#define LED_OFF GPIO.out_w1tc = (1 << LED);
#define LED_ON GPIO.out_w1ts = (1 << LED);

static spi_device_handle_t lcd_spi_handle = NULL;
static spi_transaction_t lcd_trans[14];
static size_t lcd_trans_index = 0;
static int lcd_rot = 0;
static volatile int lcd_flushing = 0;

static void lcd_command(uint8_t cmd, const uint8_t* args, size_t len) {
    spi_transaction_t* tx = &lcd_trans[lcd_trans_index++];
    if (lcd_trans_index > 13) lcd_trans_index = 0;
    tx->length = 8;
    tx->tx_data[0] = cmd;
    tx->user = (void*)0;
    tx->flags = SPI_TRANS_USE_TXDATA;
    spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY);
    if (len && args) {
        tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->length = 8 * len;
        if (len <= 4) {
            memcpy(tx->tx_data, args, len);
            tx->flags = SPI_TRANS_USE_TXDATA;
        } else {
            tx->tx_buffer = args;
            tx->flags = 0;
        }
        tx->user = (void*)1;
        spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY);
    }
}
IRAM_ATTR static void lcd_spi_pre_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        DC_C;
    }
}
IRAM_ATTR static void lcd_spi_post_cb(spi_transaction_t* trans) {
    lcd_flushing = 0;
    if (((int)trans->user) == 0) {
        DC_D;
    } else {
        if (((int)trans->user) == 2) {
            lcd_on_flush_complete();
        }
    }
}
static void lcd_st7789_init() {
    lcd_command(0x01, NULL, 0);      // reset
    vTaskDelay(pdMS_TO_TICKS(120));  // Wait for reset to complete
    lcd_command(0x11, NULL, 0);      // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x13, NULL, 0);  // Normal display mode on
    static const uint8_t params1 = 0x08;
    lcd_command(0x36, &params1, 1);
    static const uint8_t params2[] = {0x0A, 0xB2};
    lcd_command(0xB6, params2, 2);
    static const uint8_t params3[] = {0x00, 0xE0};
    lcd_command(0xB0, params3, 2);
    static const uint8_t params4 = 0x55;
    lcd_command(0x3A, &params4, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
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
    lcd_command(0x21, NULL, 0);
    static const uint8_t params16[] = {0x00, 0x00, 0x00, 0xEF};
    lcd_command(0x2A, params16, 4);  // Column address set
    static const uint8_t params17[] = {0x00, 0x00, 0x01, 0x3F};
    lcd_command(0x2B, params17, 4);  // Row address set
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x20, NULL, 0);
}
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t args[4];
    args[0] = (x1 >> 8);
    args[1] = (x1 & 0xFF);
    args[2] = (x2 >> 8);
    args[3] = (x2 & 0xFF);
    lcd_command(0x2A, args, 4);
    args[0] = (y1 >> 8);
    args[1] = (y1 & 0xFF);
    args[2] = (y2 >> 8);
    args[3] = (y2 & 0xFF);
    lcd_command(0x2B, args, 4);
}

static void lcd_write_bitmap(const void* data_in, uint32_t len) {
    if (len) {
        spi_transaction_t* tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->user = (void*)0;
        tx->flags = SPI_TRANS_USE_TXDATA;
        tx->tx_data[0] = 0x2C;  // RAMWR
        tx->length = 8;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));

        tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->flags = 0;
        tx->length = 8 * (len * 2);
        tx->tx_buffer = data_in;
        tx->user = (void*)2;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));
    } else {
        lcd_flushing = 0;
        lcd_on_flush_complete();
    }
}

int lcd_wait_flush(uint32_t timeout) {
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t total_ms = 0;
    while ((timeout == 0 || total_ms <= timeout) && lcd_flushing > 0) {
        taskYIELD();
        uint32_t new_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        total_ms += (new_ms - ms);
        ms = new_ms;
    }
    return timeout == 0 || total_ms < timeout;
}
void lcd_on_flush_complete() {}
void lcd_rotation(int rotation) {
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
    lcd_rot = rotation;
}

void lcd_initialize(size_t max_transfer_size) {
    if (lcd_spi_handle != NULL) {
        return;
    }
    memset(lcd_trans, 0, sizeof(spi_transaction_t) * 14);
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (((unsigned long long)1) << LCD_DC);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_conf);
    gpio_set_direction((gpio_num_t)LCD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LCD_CS, 0);

    // configure the SPI bus
    const spi_host_device_t host = SPI3_HOST;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = LCD_CLK;
    buscfg.mosi_io_num = LCD_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    // declare enough space for the transfer buffers + 8 bytes SPI DMA overhead
    buscfg.max_transfer_sz = max_transfer_size + 8;
    // Initialize the SPI bus on HSPI (SPI3)
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dummy_bits = 0;
    dev_cfg.queue_size = 14;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX;
    dev_cfg.spics_io_num = LCD_CS;
    dev_cfg.pre_cb = lcd_spi_pre_cb;
    dev_cfg.post_cb = lcd_spi_post_cb;
    dev_cfg.clock_speed_hz = LCD_SPEED;
    dev_cfg.cs_ena_posttrans = 1;
    dev_cfg.cs_ena_pretrans = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &lcd_spi_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(lcd_spi_handle, portMAX_DELAY));
    // if we don't configure GPIO 0 after we init the SPI it stays high for some
    // reason
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << LCD_DC);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)LCD_DC, 0);

    lcd_st7789_init();
}
void lcd_deinitialize() {
    if (lcd_spi_handle == NULL) {
        return;
    }
    spi_device_release_bus(lcd_spi_handle);
    ESP_ERROR_CHECK(spi_bus_remove_device(lcd_spi_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI3_HOST));
    lcd_spi_handle = NULL;
}
void lcd_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
               const void* bitmap) {
    lcd_flushing=1;
    lcd_set_window(x1, y1, x2, y2);
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    lcd_write_bitmap(bitmap, w * h);
}
static int led_initialized = 0;
void led_enable(int value) {
    if (!led_initialized) {
        return;
    }
    if (value == 0) {
        LED_OFF;
    } else {
        LED_ON;
    }
}
static int i2c_initialized = 0;
void led_initialize() {
    if (led_initialized || i2c_initialized) {
        return;
    }
    gpio_set_direction((gpio_num_t)LED, GPIO_MODE_OUTPUT);
    LED_OFF;
    led_initialized = true;
}
void led_deinitialize() {
    LED_OFF;
    led_initialized = false;
}

static void* camera_fb = NULL;
static int camera_rot = 0;
static int camera_initialized = 0;
static int camera_flags = 0;
static camera_fb_t* camera_current_fb = NULL;
void camera_rotation(int rotation) { camera_rot = rotation & 3; }
static void camera_copy_rotate(const void* bitmap, int rows, int cols) {
    // allocating space for the new rotated image
    const uint16_t* original = (const uint16_t*)bitmap;
    uint16_t* out = (uint16_t*)camera_fb;
    size_t count;
    switch (camera_rot) {
        case 1:
            // rotate 90
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(cols - x - 1) * rows + y] = *(original++);
                }
            }
            break;
        case 2:
            // rotate 180
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(rows - y - 1) * cols + (cols - x - 1)] = *(original++);
                }
            }
            break;
        case 3:
            // rotate 270
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[x * rows + (rows - y - 1)] = *(original++);
                }
            }
            break;
        default:  // case 0:
            count = rows * cols;
            while (count--) {
                *(out++) = *(original++);
            }
            break;
    }
}
const void* camera_frame_buffer() {
    if (!camera_initialized) {
        return NULL;
    }
    camera_current_fb = esp_camera_fb_get();
    if (camera_current_fb != NULL) {
        camera_copy_rotate(camera_current_fb->buf,
                           (camera_flags & CAM_FRAME_SIZE_96X96) ? 96 : 240,
                           (camera_flags & CAM_FRAME_SIZE_96X96) ? 96 : 240);
        esp_camera_fb_return(camera_current_fb);
        return camera_fb;
    }
    esp_camera_fb_return(camera_current_fb);
    return NULL;
}

void camera_initialize(int flags) {
    if (camera_initialized) {
        return;
    }
    camera_flags = flags;
    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_Y2;
    config.pin_d1 = CAM_Y3;
    config.pin_d2 = CAM_Y4;
    config.pin_d3 = CAM_Y5;
    config.pin_d4 = CAM_Y6;
    config.pin_d5 = CAM_Y7;
    config.pin_d6 = CAM_Y8;
    config.pin_d7 = CAM_Y9;
    config.pin_xclk = CAM_XCLK;
    config.pin_pclk = CAM_PCLK;
    config.pin_vsync = CAM_VSYNC;
    config.pin_href = CAM_HREF;
    config.pin_sccb_sda = CAM_SIOD;
    config.pin_sccb_scl = CAM_SIOC;
    config.pin_pwdn = CAM_PWDN;
    config.pin_reset = CAM_RST;
    config.xclk_freq_hz = CAM_SPEED;
    config.frame_size = 0 != (flags & CAM_FRAME_SIZE_96X96) ? FRAMESIZE_96X96
                                                            : FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = 0 != (flags & CAM_ALLOC_CAM_PSRAM) ? CAMERA_FB_IN_PSRAM
                                                            : CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 10;
    config.fb_count = CAMERA_FB_IN_PSRAM ? 6 : 2;
    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);       // flip it back
    s->set_hmirror(s, 1);     // horizontal mirror image
    s->set_brightness(s, 0);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
    camera_initialized = 1;
    const size_t camera_size =
        (flags & CAM_FRAME_SIZE_96X96) ? 96 * 96 * 2 : 240 * 240 * 2;
    camera_fb = heap_caps_malloc(camera_size, (flags & CAM_ALLOC_FB_PSRAM)
                                                  ? MALLOC_CAP_SPIRAM
                                                  : MALLOC_CAP_DEFAULT);
    if (camera_fb == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
void camera_levels(int brightness, int contrast, int saturation,
                   int sharpness) {
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
    if (!camera_initialized) {
        return;
    }
    camera_current_fb = NULL;
    camera_initialized = 0;
    esp_camera_deinit();
    if (camera_fb != NULL) {
        free(camera_fb);
        camera_fb = NULL;
    }
}

static i2s_chan_handle_t neopixel_handle = NULL;
static uint8_t neopixel_out_buffer[12] = {0};
static uint8_t neopixel_zero_buffer[48] = {0};

static const uint16_t neopixel_bit_patterns[4] = {0x88, 0x8e, 0xe8, 0xee};

void neopixel_color(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t red = r, green = g, blue = b;

    neopixel_out_buffer[0] = neopixel_bit_patterns[green >> 6 & 0x03];
    neopixel_out_buffer[1] = neopixel_bit_patterns[green >> 4 & 0x03];
    neopixel_out_buffer[2] = neopixel_bit_patterns[green >> 2 & 0x03];
    neopixel_out_buffer[3] = neopixel_bit_patterns[green & 0x03];

    neopixel_out_buffer[4] = neopixel_bit_patterns[red >> 6 & 0x03];
    neopixel_out_buffer[5] = neopixel_bit_patterns[red >> 4 & 0x03];
    neopixel_out_buffer[6] = neopixel_bit_patterns[red >> 2 & 0x03];
    neopixel_out_buffer[7] = neopixel_bit_patterns[red & 0x03];

    neopixel_out_buffer[8] = neopixel_bit_patterns[blue >> 6 & 0x03];
    neopixel_out_buffer[9] = neopixel_bit_patterns[blue >> 4 & 0x03];
    neopixel_out_buffer[10] = neopixel_bit_patterns[blue >> 2 & 0x03];
    neopixel_out_buffer[11] = neopixel_bit_patterns[blue & 0x03];
    i2s_channel_enable(neopixel_handle);
    i2s_channel_write(neopixel_handle, neopixel_out_buffer,
                      sizeof(neopixel_out_buffer), NULL, portMAX_DELAY);
    i2s_channel_write(neopixel_handle, neopixel_zero_buffer,
                      sizeof(neopixel_zero_buffer), NULL, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2s_channel_disable(neopixel_handle);
}

void neopixel_initialize() {
    if (neopixel_handle != NULL) {
        return;
    }

    i2s_chan_config_t chan_cfg;
    memset(&chan_cfg, 0, sizeof(chan_cfg));
    chan_cfg.id = I2S_NUM_1;
    chan_cfg.role = I2S_ROLE_MASTER;
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = sizeof(neopixel_out_buffer);
    chan_cfg.auto_clear = 0;
    chan_cfg.intr_priority = 0;

    i2s_new_channel(&chan_cfg, &neopixel_handle, NULL);

    i2s_std_config_t std_cfg;
    memset(&std_cfg, 0, sizeof(std_cfg));
    std_cfg.clk_cfg.sample_rate_hz = 93650;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M;
    std_cfg.clk_cfg.mclk_multiple = 128;
    std_cfg.gpio_cfg.bclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.ws = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.dout = NEOPIXEL;
    std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_bit_width = 0;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    std_cfg.slot_cfg.ws_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.ws_pol = 0;
    std_cfg.slot_cfg.bit_shift = 0;
    std_cfg.slot_cfg.left_align = 1;
    std_cfg.slot_cfg.big_endian = 0;
    std_cfg.slot_cfg.bit_order_lsb = 0;
    i2s_channel_init_std_mode(neopixel_handle, &std_cfg);
}

void neopixel_deinitialize() {
    if (neopixel_handle == NULL) {
        return;
    }
    i2s_channel_disable(neopixel_handle);
    i2s_del_channel(neopixel_handle);
    neopixel_handle = NULL;
};

static void i2c_initialize() {
    if (led_initialized || i2c_initialized) {
        return;
    }
    i2c_config_t config;
    memset(&config, 0, sizeof(config));
    config.master.clk_speed = I2C_SPEED;
    config.mode = I2C_MODE_MASTER;
    config.scl_io_num = I2C_SCL;
    config.sda_io_num = I2C_SDA;
    config.scl_pullup_en = 1;
    config.sda_pullup_en = 1;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    i2c_initialized = 1;
}
// static void i2c_deinitialize() {
//     if(!i2c_initialized) {
//         return;
//     }
//     ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
//     i2c_initialized = 0;
// }

static int touch_rot = 0;
static int touch_initialized = 0;
static uint32_t touch_timestamp = 0;
static size_t touch_count = 0;
static uint16_t touch_x_data[2], touch_y_data[2], touch_id_data[2];
static int touch_write_reg(int r, int value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (ESP_OK != i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, 0x38 << 1 | I2C_MASTER_WRITE,
                                        I2C_MASTER_ACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    uint8_t data[2];
    data[0] = r;
    data[1] = value;
    if (ESP_OK != i2c_master_write(cmd, data, sizeof(data), I2C_MASTER_ACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_cmd_begin(I2C_NUM_0, cmd, portMAX_DELAY)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);
    return 1;
}
static int touch_read_all() {
    static const uint8_t ACK_CHECK_EN = 0x1;
    uint8_t i2cdat[16];
    // Read data
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (ESP_OK != i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, (0x38 << 1), ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, 0, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000))) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    if (ESP_OK != i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_write_byte(cmd, (0x38 << 1) | 1, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK !=
        i2c_master_read(cmd, i2cdat, sizeof(i2cdat), I2C_MASTER_LAST_NACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if (ESP_OK != i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000))) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);

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
    return 1;
}
void touch_initialize(int threshhold) {
    if (touch_initialized) {
        return;
    }
    i2c_initialize();
    if (0 == touch_write_reg(0x80, threshhold)) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }

    touch_count = 0;
    touch_initialized = 1;
}
void touch_deinitialize() {
    if (!touch_initialized) {
        return;
    }
    // nothing for now
    touch_initialized = 0;
}
void touch_rotation(int rotation) { touch_rot = rotation & 3; }
static int touch_update() {
    if (!touch_initialized) {
        return 0;
    }
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    if (ms > touch_timestamp + 13) {
        if (!touch_read_all()) {
            return 0;
        }
        touch_timestamp = ms;
    }
    return 1;
}
static void touch_translate(uint16_t* x, uint16_t* y) {
    uint16_t tmp;
    switch (touch_rot) {
        case 1:
            tmp = *x;
            *x = *y;
            *y = 240 - tmp - 1;
            break;
        case 2:
            *x = 240 - *x - 1;
            *y = 320 - *y - 1;
            break;
        case 3:
            tmp = *x;
            *x = 320 - *y - 1;
            *y = tmp;
        default:
            break;
    }
}
static int touch_read_point(size_t n, uint16_t* out_x, uint16_t* out_y) {
    if (touch_count == 0 || n >= touch_count) {
        if (out_x != NULL) {
            *out_x = 0;
        }
        if (out_y != NULL) {
            *out_y = 0;
        }
        return 0;
    }
    uint16_t x = touch_x_data[n];
    uint16_t y = touch_y_data[n];
    if (x >= 240) {
        x = 240 - 1;
    }
    if (y >= 320) {
        y = 320 - 1;
    }
    touch_translate(&x, &y);
    if (out_x != NULL) {
        *out_x = x;
    }
    if (out_y != NULL) {
        *out_y = y;
    }
    return 1;
}
int touch_xy(uint16_t* out_x, uint16_t* out_y) {
    if (!touch_update()) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    return touch_read_point(0, out_x, out_y);
}
int touch_xy2(uint16_t* out_x, uint16_t* out_y) {
    if (!touch_update()) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    return touch_read_point(1, out_x, out_y);
}

static uint16_t* audio_out_buffer = NULL;
static i2s_chan_handle_t audio_handle = NULL;

void audio_initialize(audio_format_t format) {
    if (audio_out_buffer != NULL) {
        return;
    }
    const int freq = (format==AUDIO_11K_MONO||format==AUDIO_11K_STEREO)?11025:((format==AUDIO_22K_MONO||format==AUDIO_22K_STEREO)?22050:44100);
    const int stereo = (format == AUDIO_11K_STEREO || format==AUDIO_22K_STEREO || format == AUDIO_44_1K_STEREO);
    
    audio_out_buffer = (uint16_t*)heap_caps_malloc(
        AUDIO_MAX_SAMPLES * sizeof(uint16_t), MALLOC_CAP_DEFAULT);
    if (audio_out_buffer == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    /* This helper macro is defined in `i2s_common.h` and shared by all the I2S
     * communication modes. It can help to specify the I2S role and port ID */
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    /* Allocate a new TX channel and get the handle of this channel */
    chan_cfg.dma_desc_num = 14;
    chan_cfg.dma_frame_num = AUDIO_MAX_SAMPLES/(1+stereo);
    chan_cfg.intr_priority = 5;
    i2s_new_channel(&chan_cfg, &audio_handle, NULL);
    /* Setting the configurations, the slot configuration and clock
     * configuration can be generated by the macros These two helper macros are
     * defined in `i2s_std.h` which can only be used in STD mode. They can help
     * to specify the slot and clock configurations for initialization or
     * updating */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(freq),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    stereo?I2S_SLOT_MODE_STEREO:I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = AUD_BCLK,
                .ws = AUD_LRC,
                .dout = AUD_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    
    
    /* Initialize the channel */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_handle, &std_cfg));
    
    /* Before writing data, start the TX channel first */
    ESP_ERROR_CHECK(i2s_channel_enable(audio_handle));
}
void audio_deinitialize() {
    if (audio_out_buffer == NULL) {
        return;
    }
    i2s_channel_disable(audio_handle);
    i2s_del_channel(audio_handle);
    audio_handle = NULL;
    free(audio_out_buffer);
    audio_out_buffer = NULL;
}
size_t audio_write_int16(const int16_t* samples, size_t sample_count) {
    size_t result = 0;
    const int16_t* p = (const int16_t*)samples;
    uint16_t* out = audio_out_buffer;
    while (sample_count) {
        size_t to_write =
            sample_count < AUDIO_MAX_SAMPLES ? sample_count : AUDIO_MAX_SAMPLES;
        for (int i = 0; i < to_write; ++i) {
            *(out++) = (uint16_t)(*(p++) + 32768);
        }
        size_t written = to_write * 2;
        i2s_channel_write(audio_handle, audio_out_buffer, to_write * 2,
                          &written, portMAX_DELAY);
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
            sample_count < AUDIO_MAX_SAMPLES ? sample_count : AUDIO_MAX_SAMPLES;
        for (int i = 0; i < to_write; ++i) {
            float fval = *(p++) * vel;
            if (fval < -1.f)
                fval = -1.f;
            else if (fval > 1.f)
                fval = 1.f;
            int16_t val = fval > 0 ? fval * 32767 : fval * 32768;
            *(out++) = (uint16_t)(val + 32768);
        }
        size_t written;
        i2s_channel_write(audio_handle, audio_out_buffer, to_write * 2,
                          &written, portMAX_DELAY);
        size_t samples_written = written >> 1;
        sample_count -= samples_written;
        result += samples_written;
        if (samples_written != to_write) {
            return samples_written;
        }
    }
    return result;
}
static int prox_sensor_initialized = 0;
typedef struct {
    uint32_t red[4];
    uint32_t IR[4];
    uint32_t green[4];
    uint8_t head;
    uint8_t tail;
} prox_sensor_info_t;  // This is our circular buffer of readings from the
                       // sensor

static prox_sensor_info_t prox_sensor_data;
static uint8_t prox_sensor_active_leds;
//
static esp_err_t prox_sensor_read(uint8_t* data_rd, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x57 << 1) | I2C_MASTER_READ, I2C_MASTER_NACK);
    i2c_master_read(cmd, data_rd, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t prox_sensor_write(uint8_t* data_wr, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x57 << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
    while(size--) {
        i2c_master_write_byte(cmd,*(data_wr++), I2C_MASTER_ACK);
    }
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}
static void prox_sensor_read_reg(uint8_t reg_addr, uint8_t* data_reg,
                                 size_t bytes_to_read) {
    ESP_ERROR_CHECK(prox_sensor_write(&reg_addr, 1));
    ESP_ERROR_CHECK(prox_sensor_read(data_reg, bytes_to_read));
}

static void prox_sensor_write_reg(uint8_t command, uint8_t reg) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x57 << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
    i2c_master_write_byte(cmd, command, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    ESP_ERROR_CHECK(ret);
}
// Given a register, read it, mask it, and then set the thing
static void prox_sensor_mask_reg(uint8_t reg, uint8_t mask, uint8_t thing) {
    // Grab current register context
    uint8_t originalContents;
    prox_sensor_read_reg(reg, &originalContents, 1);
    // Zero-out the portions of the register we're interested in
    originalContents = originalContents & mask;

    // Change contents
    prox_sensor_write_reg(reg, originalContents | thing);
}

// Read the FIFO Write Pointer
static uint8_t prox_sensor_write_pointer(void) {
    uint8_t result;
    prox_sensor_read_reg(0x04, &result, 1);
    return result;
}

// Read the FIFO Read Pointer
static uint8_t prox_sensor_read_pointer(void) {
    uint8_t result;
    prox_sensor_read_reg(0x06, &result, 1);
    return result;
}
// static void prox_sensor_soft_reset(void) {
//     prox_sensor_mask_reg(0x09, 0xBF, 0x40);
//     // Poll for bit to clear, reset is then complete
//     // Timeout after 100ms
//     uint32_t startTime = pdTICKS_TO_MS(xTaskGetTickCount());
//     while (pdTICKS_TO_MS(xTaskGetTickCount()) - startTime < 100) {
//         uint8_t response;
//         prox_sensor_read_reg(0x09, &response, 1);
//         if ((response & 0x40) == 0) break;  // We're done!
//         vTaskDelay(pdMS_TO_TICKS(1));       // Let's not over burden the I2C bus
//     }
// }

static void prox_sensor_led_mode(uint8_t mode) {
    // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
    // See datasheet, page 19
    prox_sensor_mask_reg(0x09, 0xF8, mode);
}

static void prox_sensor_adc_range(uint8_t adcRange) {
    // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
    prox_sensor_mask_reg(0x0A, 0x9F, adcRange);
}

static void prox_sensor_sample_rate(uint8_t sampleRate) {
    // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000,
    // _1600, _3200
    prox_sensor_mask_reg(0x0A, 0xE3, sampleRate);
}

static void prox_sensor_pulse_width(uint8_t pulseWidth) {
    // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
    prox_sensor_mask_reg(0x0A, 0xFC, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
static void prox_sensor_pulse_amp_red(uint8_t amplitude) {
    prox_sensor_write_reg(0x0C, amplitude);
}

static void prox_sensor_pulse_amp_ir(uint8_t amplitude) {
    prox_sensor_write_reg(0x0D, amplitude);
}

static void prox_sensor_pulse_amp_green(uint8_t amplitude) {
    prox_sensor_write_reg(0x0C, amplitude);
}

static void prox_sensor_pulse_amp_prox(uint8_t amplitude) {
    prox_sensor_write_reg(0x10, amplitude);
}

// Set sample average (Table 3, Page 18)
static void prox_sensor_fifo_average(uint8_t numberOfSamples) {
    prox_sensor_mask_reg(0x08, (uint8_t)~0b11100000, numberOfSamples);
}

// Resets all points to start in a known state
// Page 15 recommends clearing FIFO before beginning a read
void prox_sensor_clear_fifo(void) {
    prox_sensor_write_reg(0x04, 0);
    prox_sensor_write_reg(0x05, 0);
    prox_sensor_write_reg(0x06, 0);
}
// Enable roll over if FIFO over flows
static void prox_sensor_fifo_rollover_enable(void) {
    prox_sensor_mask_reg(0x08, 0xEF, 0x10);
}

// Disable roll over if FIFO over flows
// static void prox_sensor_fifo_rollover_disable(void) {
//     prox_sensor_mask_reg(0x08, 0xEF, 0x00);
// }

static void prox_sensor_enable_slot(uint8_t slotNumber, uint8_t device) {
    static const uint8_t MAX30105_MULTILEDCONFIG1 = 0x11;
    static const uint8_t MAX30105_MULTILEDCONFIG2 = 0x12;
    static const uint8_t MAX30105_SLOT1_MASK = 0xF8;
    static const uint8_t MAX30105_SLOT2_MASK = 0x8F;
    static const uint8_t MAX30105_SLOT3_MASK = 0xF8;
    static const uint8_t MAX30105_SLOT4_MASK = 0x8F;
    //uint8_t originalContents;

    switch (slotNumber) {
        case (1):
            prox_sensor_mask_reg(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK,
                                 device);
            break;
        case (2):
            prox_sensor_mask_reg(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK,
                                 device << 4);
            break;
        case (3):
            prox_sensor_mask_reg(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK,
                                 device);
            break;
        case (4):
            prox_sensor_mask_reg(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK,
                                 device << 4);
            break;
        default:
            // Shouldn't be here!
            break;
    }
}
// Polls the sensor for new data
// Call regularly
// If new data is available, it updates the head and tail in the main struct
// Returns number of new samples obtained
static uint16_t prox_sensor_update_impl(void) {
    // Read register FIDO_DATA in (3-uint8_t * number of active LED) chunks
    // Until FIFO_RD_PTR = FIFO_WR_PTR
    uint8_t readPointer = prox_sensor_read_pointer();
    uint8_t writePointer = prox_sensor_write_pointer();
    //printf("read: %d, write: %d\n",readPointer,writePointer);
    int numberOfSamples = 0;

    // Do we have new data?
    if (readPointer != writePointer) {
        //puts("new data");
        // Calculate the number of readings we need to get from sensor
        numberOfSamples = writePointer - readPointer;
        if (numberOfSamples < 0) numberOfSamples += 32;  // Wrap condition

        // We now have the number of readings, now calc bytes to read
        // For this example we are just doing Red and IR (3 bytes each)

        int bytesLeftToRead = numberOfSamples * prox_sensor_active_leds * 3;

        // Get ready to read a burst of data from the FIFO register
        uint8_t tmp = 0x07;
        //prox_sensor_write(&tmp, 1);
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (0x57 << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
        i2c_master_write_byte(cmd, tmp, I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000)));
        i2c_cmd_link_delete(cmd);

        static uint8_t block[64];
        // We may need to read as many as 288 bytes so we read in blocks no
        // larger than I2C_BUFFER_LENGTH I2C_BUFFER_LENGTH changes based on the
        // platform. 64 bytes for SAMD21, 32 bytes for Uno. Wire.requestFrom()
        // is limited to BUFFER_LENGTH which is 32 on the Uno
        //printf("bytes to fetch: %d\n",bytesLeftToRead);
        while (bytesLeftToRead > 0) {
            int toGet = bytesLeftToRead;
            if (toGet > 64) {
                // If toGet is 32 this is bad because we read 6 bytes (Red+IR *
                // 3 = 6) at a time 32 % 6 = 2 left over. We don't want to
                // request 32 bytes, we want to request 30. 32 % 9
                // (Red+IR+GREEN) = 5 left over. We want to request 27.

                toGet = 64 - (64 % (prox_sensor_active_leds *
                                    3));  // Trim toGet to be a multiple of
                                          // the samples we need to read
            }

            bytesLeftToRead -= toGet;

            // Request toGet number of bytes from sensor
            prox_sensor_read(block, toGet);
            while (toGet > 0) {
                prox_sensor_data
                    .head++;  // Advance the head of the storage struct
                prox_sensor_data.head %= 4;  // Wrap condition

                uint8_t temp[sizeof(uint32_t)];  // Array of 4 bytes that we
                                                 // will convert into long
                uint32_t tempLong;
                uint8_t burst[3];
                // Burst read three bytes - RED
                prox_sensor_read(burst, 3);
                temp[3] = 0;
                temp[2] = burst[0];
                temp[1] = burst[1];
                temp[0] = burst[2];
                // Convert array to long
                memcpy(&tempLong, temp, sizeof(tempLong));

                tempLong &= 0x3FFFF;  // Zero out all but 18 bits

                prox_sensor_data.red[prox_sensor_data.head] =
                    tempLong;  // Store this reading into the sense array

                if (prox_sensor_active_leds>1) {
                    // Burst read three more bytes - IR
                    prox_sensor_read(burst, 3);
                    temp[3] = 0;
                    temp[2] = burst[0];
                    temp[1] = burst[1];
                    temp[0] = burst[2];
                    // Convert array to long
                    memcpy(&tempLong, temp, sizeof(tempLong));

                    tempLong &= 0x3FFFF;  // Zero out all but 18 bits

                    prox_sensor_data.IR[prox_sensor_data.head] = tempLong;
                }

                if (prox_sensor_active_leds>2) {
                    // Burst read three more bytes - Green
                    prox_sensor_read(burst, 3);
                    temp[3] = 0;
                    temp[2] = burst[0];
                    temp[1] = burst[1];
                    temp[0] = burst[2];
                    // Convert array to long
                    memcpy(&tempLong, temp, sizeof(tempLong));

                    tempLong &= 0x3FFFF;  // Zero out all but 18 bits

                    prox_sensor_data.green[prox_sensor_data.head] = tempLong;
                }

                toGet -= prox_sensor_active_leds * 3;
            }

        }  // End while (bytesLeftToRead > 0)

    }  // End readPtr != writePtr
    return (numberOfSamples);
}
static int prox_sensor_update_timeout(uint32_t maxTimeToCheck) {
    uint32_t markTime = pdTICKS_TO_MS(xTaskGetTickCount());

    while (1) {
        if (maxTimeToCheck > 0 &&
            (pdTICKS_TO_MS(xTaskGetTickCount()) - markTime > maxTimeToCheck))
            return (0);

        if (prox_sensor_update_impl() == 1)  // We found new data!
            return (1);

        vTaskDelay(1);
    }
}

void prox_sensor_configure(prox_sens_amp_t powerLevel, prox_sens_sampleavg_t sampleAverage, prox_sens_mode_t mode ,
           prox_sens_samplerate_t sampleRate, prox_sens_pulsewidth_t pulseWidth , prox_sens_adcrange_t adcRange) {
    
    //prox_sensor_soft_reset();
    prox_sensor_active_leds = 0;
    // FIFO Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // The chip will average multiple samples of same type together if you wish
    prox_sensor_fifo_average((uint8_t)sampleAverage);  // No averaging per FIFO record

    // setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full'
    // interrupt
    prox_sensor_fifo_rollover_enable();  // Allow FIFO to wrap/roll over
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // Mode Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    prox_sensor_led_mode((uint8_t)mode);  // Watch all three LED channels
    //prox_sensor_mode = mode;
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // Particle Sensing Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    prox_sensor_adc_range((uint8_t)adcRange);  // 7.81pA per LSB

    prox_sensor_sample_rate((uint8_t)sampleRate);  // Take 50 samples per second

    // The longer the pulse width the longer range of detection you'll have
    // At 69us and 0.4mA it's about 2 inches
    // At 411us and 0.4mA it's about 6 inches
    prox_sensor_pulse_width((uint8_t)pulseWidth);  // Page 26, Gets us 15 bit resolution
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // LED Pulse Amplitude Configuration
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    // Default is 0x1F which gets us 6.4mA
    // powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
    // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
    // powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
    // powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

    prox_sensor_pulse_amp_red((uint8_t)powerLevel);
    prox_sensor_pulse_amp_ir((uint8_t)powerLevel);
    prox_sensor_pulse_amp_green((uint8_t)powerLevel);
    prox_sensor_pulse_amp_prox((uint8_t)powerLevel);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // Multi-LED Mode Configuration, Enable the reading of the three LEDs
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    prox_sensor_enable_slot(1, (uint8_t)PROX_SENS_SLOT_RED_LED);
    ++prox_sensor_active_leds;
    if (mode == PROX_SENS_MODE_REDIRONLY || mode == PROX_SENS_MODE_MULTILED) {
        prox_sensor_enable_slot(2, (uint8_t)PROX_SENS_SLOT_IR_LED);
        ++prox_sensor_active_leds;
    }
    if (mode == PROX_SENS_MODE_MULTILED) {
        prox_sensor_enable_slot(3, (uint8_t)PROX_SENS_SLOT_GREEN_LED);
        ++prox_sensor_active_leds;
    }
    // prox_sensor_enable_slot(1, SLOT_RED_PILOT);
    // prox_sensor_enable_slot(2, SLOT_IR_PILOT);
    // prox_sensor_enable_slot(3, SLOT_GREEN_PILOT);
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    prox_sensor_clear_fifo();  // Reset the FIFO before we begin checking the
                               // sensor
}

int prox_sensor_read_raw(uint32_t* out_red, uint32_t* out_ir,
                         uint32_t* out_green, uint32_t timeout) {
    if (!prox_sensor_initialized) {
        return 0;
    }
    if (!prox_sensor_update_timeout(timeout)) return 0;
    if (out_red) {
        *out_red = prox_sensor_data.red[prox_sensor_data.head];
    }
    if (out_ir) {
        *out_ir = prox_sensor_data.IR[prox_sensor_data.head];
    }
    if (out_green) {
        *out_green = prox_sensor_data.green[prox_sensor_data.head];
    }
    return 1;
}

static uint8_t prox_sensor_part_id(void) {
    uint8_t result;
    prox_sensor_read_reg(0xFF, &result,1);
    return result;
}

// static uint8_t prox_sensor_revision_id(void) {
//     uint8_t result;
//     prox_sensor_read_reg(0xFE, &result,1);
//     return result;
// }

void prox_sensor_initialize(void) {
    if (prox_sensor_initialized) {
        return;
    }
    i2c_initialize();
    if(prox_sensor_part_id()!=0x15) {
        ESP_ERROR_CHECK (ESP_ERR_INVALID_RESPONSE);
    }
    prox_sensor_initialized=true;
}
void prox_sensor_deinitialize(void) {
    if (!prox_sensor_initialized) {
        return;
    }
}

void prox_sensor_pulse_amp_threshold(int16_t red, int16_t ir, int16_t green, int16_t prox, int16_t thresh) {
    if(!prox_sensor_initialized) {
        return;
    }
    if(red>=0&&red<=0xFF) {
        prox_sensor_write_reg(0x0C,red);
    }
    if(ir>=0&&ir<=0xFF) {
        prox_sensor_write_reg(0x0D,ir);
    }
    if(green>=0&&green<=0xFF) {
        prox_sensor_write_reg(0x0E,green);
    }
    if(prox>=0&&prox<=0xFF) {
        prox_sensor_write_reg(0x10,prox);
    }
    if(thresh>=0&&thresh<=0xFF) {
        prox_sensor_write_reg(0x30,thresh);
    }
}
static int sd_initialized = 0;
static sdmmc_card_t *sd_card_handle = NULL;
static const char* sd_mount_point = NULL;
int sd_initialize(const char* mount_point, size_t max_files, size_t alloc_unit_size, uint32_t freq, int flags) {
    if(sd_initialized) {
        return 1;
    }
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = (flags&SD_FLAGS_FORMAT_ON_FAIL),
        .max_files = (int)max_files,
        .allocation_unit_size = alloc_unit_size
    };
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT; //use 1-line SD mode
    host.max_freq_khz =freq;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = SDMMC_CLK;
    slot_config.cmd = SDMMC_CMD;
    slot_config.d0 = SDMMC_D0;
    slot_config.width = 1;
    // assuming the board is built correctly, we don't need this:
    // slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &sd_card_handle);
    if(ret!=ESP_OK) {
        return 0;
    }
    sd_mount_point = mount_point;
    sd_initialized=1;
    return 1;
}
void sd_deinitialize() {
    if(!sd_initialized) {
        return;
    }
    ESP_ERROR_CHECK(esp_vfs_fat_sdcard_unmount(sd_mount_point, sd_card_handle));
    sd_card_handle = NULL;
    sd_mount_point = NULL;
    sd_initialized=false;
}
sdmmc_card_t* sd_card() {
    if(!sd_initialized) {
        return NULL;
    }
    return sd_card_handle;
}