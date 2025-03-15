#define SILENCE
#include "Arduino.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "freenove_s3_devkit.h"
#include "sdmmc_cmd.h"
#include "gfx.h"
#include "uix.h"
#define TSF_IMPLEMENTATION
#include "tsf.h"
#define TML_IMPLEMENTATION
#include "tml.h"
#define VGA_8X8_IMPLEMENTATION
#include "assets/vga_8x8.h"

using namespace gfx;
using namespace uix;

static constexpr const int big_cam = 1;

static SemaphoreHandle_t audio_sync = NULL;
static float audio_amplitude = 0;
static uint32_t prox_average;  // Average IR at power up

struct tsf_allocator tsf_alloc;
struct tsf* tsf_handle;
struct tml_allocator tml_alloc;
struct tml_message* tml_messages;
struct tml_message* tml_message_cursor;

static constexpr const size_t lcd_transfer_size = 96 * 96 * 2;
static void* lcd_transfer_buffer1 = NULL;
static void* lcd_transfer_buffer2 = NULL;

static uix::display lcd_display;
using screen_t = uix::screen<rgb_pixel<16>>;
using label_t = label<typename screen_t::control_surface_type>;
using color_t = color<typename screen_t::pixel_type>;
bitmap<typename screen_t::pixel_type> cam_bmp;

class camera_view : public control<screen_t::control_surface_type> {
    using base_type = control<screen_t::control_surface_type>;
    
public:
    camera_view() : base_type() {
     
    }
    virtual ~camera_view() {
        
    }
    void update() {
        this->invalidate();
    }
protected:

    void on_paint(control_surface_type& destination, const srect16& clip) {
        if(cam_bmp.begin()!=nullptr) {
            // draw the bitmap to the control surface, clipping so we only
            // draw what we need to
            draw::bitmap(destination,(rect16)clip,cam_bmp,((rect16)clip).crop(cam_bmp.bounds()));
            //draw::bitmap(destination,destination.bounds(),m_bmp,m_bmp.bounds());
        }
    }
};

static float* audio_output_buffer;
static void audio_task(void* arg) {
    uint64_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint64_t wdt_ms = start_ms;
    while (true) {
        uint64_t ms = pdTICKS_TO_MS(xTaskGetTickCount());

        while (tml_message_cursor &&
               tml_message_cursor->time <= (ms - start_ms)) {
            if (ms > wdt_ms + 150) {
                wdt_ms = ms;
                vTaskDelay(pdMS_TO_TICKS(1));
                ++ms;
            }
            switch (tml_message_cursor->type) {
                case TML_PROGRAM_CHANGE:  // channel program (preset) change
                                          // (special handling for 10th MIDI
                                          // channel with drums)
                    tsf_channel_set_presetnumber(
                        tsf_handle, tml_message_cursor->channel,
                        tml_message_cursor->program,
                        (tml_message_cursor->channel == 9));
                    break;
                case TML_NOTE_ON:  // play a note
                    tsf_channel_note_on(tsf_handle, tml_message_cursor->channel,
                                        tml_message_cursor->key,
                                        tml_message_cursor->velocity / 127.0f);
                    break;
                case TML_NOTE_OFF:  // stop a note
                    tsf_channel_note_off(tsf_handle,
                                         tml_message_cursor->channel,
                                         tml_message_cursor->key);
                    break;
                case TML_PITCH_BEND:  // pitch wheel modification
                    tsf_channel_set_pitchwheel(tsf_handle,
                                               tml_message_cursor->channel,
                                               tml_message_cursor->pitch_bend);
                    break;
                case TML_CONTROL_CHANGE:  // MIDI controller messages
                    tsf_channel_midi_control(tsf_handle,
                                             tml_message_cursor->channel,
                                             tml_message_cursor->control,
                                             tml_message_cursor->control_value);
                    break;
            }
            tml_message_cursor = tml_message_cursor->next;
        }
        if (tml_message_cursor == NULL) {
            start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            tml_message_cursor = tml_messages;
        }
#ifndef SILENCE
        float amp;
        xSemaphoreTake(audio_sync, portMAX_DELAY);
        amp = audio_amplitude;
        xSemaphoreGive(audio_sync);

        tsf_render_float(tsf_handle, (float*)audio_output_buffer,
                         AUDIO_MAX_SAMPLES >> 1, 0);
        audio_write_float(audio_output_buffer, AUDIO_MAX_SAMPLES, amp);
#endif
    }
}
static void uix_on_flush(const rect16& bounds,
                             const void *bitmap, void* state) {
    lcd_flush(bounds.x1, bounds.y1, bounds.x2, bounds.y2, bitmap);
}
void lcd_on_flush_complete() {
    lcd_display.flush_complete();
}
static const_buffer_stream fps_font_stream(vga_8x8,sizeof(vga_8x8));
static win_font fps_font(fps_font_stream);
static screen_t main_screen;
static camera_view cam_view;
static label_t fps_label;

void setup() {
    Serial.begin(115200);

    if (sd_initialize("/sdcard", 2, 16384, SD_FREQ_DEFAULT, SD_FLAGS_DEFAULT)) {
        sdmmc_card_print_info(stdout, sd_card());
        tsf_alloc.alloc = ps_malloc;
        tsf_alloc.realloc = ps_realloc;
        tsf_alloc.free = free;
        tsf_handle = tsf_load_filename("/sdcard/1mgm.sf2", &tsf_alloc);
        if (tsf_handle == NULL) {
            puts("Unable to load soundfont");
            ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
        }
        tml_alloc.alloc = ps_malloc;
        tml_alloc.realloc = ps_realloc;
        tml_alloc.free = free;
        tml_messages = tml_load_filename("/sdcard/furelise.mid", &tml_alloc);
        if (tml_messages == NULL) {
            puts("Unable to load midi");
            ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
        }
        tml_message_cursor = tml_messages;
        // Initialize preset on special 10th MIDI channel to use percussion
        // sound bank (128) if available
        tsf_channel_set_bank_preset(tsf_handle, 9, 128, 0);
        // Set the SoundFont rendering output mode
        tsf_set_output(tsf_handle, TSF_STEREO_INTERLEAVED, 44100, 0.0f);
        tsf_set_max_voices(tsf_handle, 4);
        sd_deinitialize();
    } else {
        puts("This demo requires a prepared SD card");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
    }
    audio_sync = xSemaphoreCreateMutex();
    if (audio_sync == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    audio_output_buffer = (float*)malloc(AUDIO_MAX_SAMPLES * sizeof(float));
    if (audio_output_buffer == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    memset(audio_output_buffer, 0, AUDIO_MAX_SAMPLES * sizeof(float));
    audio_initialize(AUDIO_44_1K_STEREO);
    TaskHandle_t audio_handle;
    xTaskCreatePinnedToCore(audio_task, "audio_task", 8192, NULL, 10,
                            &audio_handle,
                            1 - xTaskGetAffinity(xTaskGetCurrentTaskHandle()));
    if (audio_handle == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        while (1);
    }
    lcd_transfer_buffer1 = heap_caps_malloc(lcd_transfer_size, MALLOC_CAP_DMA);
    lcd_transfer_buffer2 = heap_caps_malloc(lcd_transfer_size, MALLOC_CAP_DMA);
    if (lcd_transfer_buffer1 == NULL || lcd_transfer_buffer2 == NULL) {
        puts("Could not allocate transfer buffers");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    lcd_initialize(lcd_transfer_size);
    touch_initialize(TOUCH_THRESH_DEFAULT);
    uint32_t total_ms = 0;
    int frames = 0;
    uint32_t ts_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    const int wmo = big_cam?239:95;
    srect16 sr(0,0,wmo,wmo);
    cam_bmp = create_bitmap<typename screen_t::pixel_type>((size16)sr.dimensions(),ps_malloc);
    if(cam_bmp.begin()==nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    
    camera_initialize(CAM_ALLOC_CAM_PSRAM | CAM_ALLOC_FB_PSRAM);
    camera_rotation(0);
    lcd_rotation(0);
    touch_rotation(0);
    neopixel_initialize();
    // Setup to sense up to 18 inches, max LED brightness
    prox_sensor_initialize();
    lcd_display.buffer_size(lcd_transfer_size);
    lcd_display.buffer1((uint8_t*)lcd_transfer_buffer1);
    lcd_display.buffer2((uint8_t*)lcd_transfer_buffer2);
    lcd_display.on_flush_callback(uix_on_flush);
    main_screen.dimensions({240,320});
    main_screen.background_color(color_t::black);
    sr.center_inplace(main_screen.bounds());
    cam_view.bounds(sr);
    main_screen.register_control(cam_view);
    fps_font.initialize();
    sr.y1=cam_view.bounds().y1-10;
    sr.y2 = sr.y1+8;
    sr.x1=0;
    sr.x2 = main_screen.bounds().x2;
    fps_label.bounds(sr);
    fps_label.font(fps_font);
    fps_label.padding({0,0});
    fps_label.color(color<rgba_pixel<32>>::red);
    fps_label.text("");
    fps_label.text_justify(uix_justify::top_left);
    main_screen.register_control(fps_label);
    lcd_display.active_screen(main_screen);

    prox_sensor_configure(PROX_SENS_AMP_50MA, PROX_SENS_SAMPLEAVG_4,
                          PROX_SENS_MODE_REDIRONLY, PROX_SENS_SAMPLERATE_400,
                          PROX_SENS_PULSEWIDTH_411, PROX_SENS_ADCRANGE_2048);
    // Take an average of IR readings at power up
    prox_average = 0;
    int avg_div = 0;
    for (int j = 0; j < 10; ++j) {
        for (uint8_t x = 0; x < 32; ++x) {
            uint32_t ir;
            if (0 != prox_sensor_read_raw(NULL, &ir, NULL, 250)) {
                prox_average += ir;
                ++avg_div;
            }
            vTaskDelay(1);
        }
        if (avg_div >= 32) {
            break;
        }
    }
    if (avg_div == 0) {
        puts("Retry count for prox sensor exceeded");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_RESPONSE);
    }
    prox_average /= avg_div;
    // led_initialize(); // conflicts with touch/i2c
    printf("Free SRAM: %0.2fKB, free PSRAM: %0.2fMB\n",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024.f,
           heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024.f / 1024.f);
}
static int frames = 0;
static uint32_t total_ms = 0;
static uint32_t ts_ms = 0;
static int col = 0;
static char fps_buf[64];
void loop() {
    uint32_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    const_bitmap<rgb_pixel<16>> cbmp((size16)cam_view.dimensions(),camera_frame_buffer());
    draw::bitmap(cam_bmp,cam_bmp.bounds(),cbmp,cbmp.bounds());
    cam_view.update();
    lcd_display.update();
    uint16_t x, y;
    if (touch_xy(&x, &y)) {
        printf("touch: (%d, %d)\n", x, y);
    }
    uint32_t ir;
    prox_sensor_read_raw(NULL, &ir, NULL, 250);
    long currentDelta = ir - prox_average;
    if (currentDelta > 500) {
        currentDelta = 500;
    } else if (currentDelta < 0) {
        currentDelta = 0;
    }
    float amp = currentDelta / 1000.f;
    if (ir < prox_average) {
        amp = .025;
    }
    xSemaphoreTake(audio_sync, portMAX_DELAY);
    audio_amplitude = amp;
    xSemaphoreGive(audio_sync);
    ++frames;
    uint32_t end_ms = pdTICKS_TO_MS(xTaskGetTickCount());

    total_ms += (end_ms - start_ms);
    if (end_ms > ts_ms + 1000) {
        ts_ms = end_ms;
        if (++col == 4) {
            col = 0;
        }
        neopixel_color(255 * (col == 1), 255 * (col == 2), 255 * (col == 3));
        if (frames > 0) {
            sprintf(fps_buf, "FPS: %d, avg ms: %0.2f", frames,
                   (float)total_ms / (float)frames);
            fps_label.text(fps_buf);
            puts(fps_buf);
        }
        total_ms = 0;
        frames = 0;
    }
}
