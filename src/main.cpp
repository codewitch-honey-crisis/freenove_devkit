#define CAM_VIEW_LOCK_RENDER
#include <Arduino.h>
#include <SPIFFS.h>
#include "fns3devkit.hpp"
#include <gfx.hpp>
#include <uix.hpp>
#include <sfx.hpp>
#define TSF_IMPLEMENTATION
#include "tsf.h"
#include "midi_sampler.hpp"
#define VGA_8X8_IMPLEMENTATION
#include "assets/vga_8x8.h"


using namespace gfx;
using namespace uix;
using namespace sfx;
static uix::display lcd_display;

using screen_t = uix::screen<rgb_pixel<16>>;
using label_t = label<typename screen_t::control_surface_type>;
using color_t = color<typename screen_t::pixel_type>;

class camera_view : public control<screen_t::control_surface_type> {
    using base_type = control<screen_t::control_surface_type>;
#ifdef CAM_VIEW_LOCK_RENDER
    const void* m_bmp;
#endif
public:
    camera_view() : base_type() {
     
    }
    virtual ~camera_view() {
     
    }
    void update() {
        this->invalidate();
    }
protected:
#ifdef CAM_VIEW_LOCK_RENDER
    void on_before_paint() override {
        m_bmp = camera_lock_frame_buffer();
    }
#endif
    void on_paint(control_surface_type& destination, const srect16& clip) {
        // get the camera frame buffer
        const void* bitmap_data =
#ifdef CAM_VIEW_LOCK_RENDER
           m_bmp;
#else        
           camera_lock_frame_buffer();
#endif
        // wrap it with a const bitmap (lightweight)
        const_bitmap<rgb_pixel<16>> cbmp(destination.dimensions(),bitmap_data);
        // draw the bitmap to the control surface, clipping so we only
        // draw what we need to
        draw::bitmap(destination,clip,cbmp,((rect16)clip).crop(cbmp.bounds()));
#ifndef CAM_VIEW_LOCK_RENDER
        // free up the frame buffer
        camera_unlock_frame_buffer();
#endif
    }
#ifdef CAM_VIEW_LOCK_RENDER
    void on_after_paint() override {
        // free up the frame buffer
        camera_unlock_frame_buffer();
    }
#endif
};

static tsf_allocator tsf_alloc;
static tsf* tsf_handle;

class midi_sender : public sfx::midi_output {
    virtual sfx_result send(const midi_message& message) override {
        switch(message.type()) {
            case midi_message_type::note_on:
                if(message.lsb()>0) {
                    tsf_channel_note_on(tsf_handle,message.channel(),message.msb(),message.lsb()/127.f);
                } else {
                    tsf_channel_note_off(tsf_handle,message.channel(),message.msb());
                }
                break;
            case midi_message_type::note_off:
                tsf_channel_note_off(tsf_handle,message.channel(),message.msb());
                break;
            case midi_message_type::program_change:
                tsf_channel_set_presetnumber(tsf_handle, message.channel(), message.msb(), message.channel() == 9);
                break;
        }
        return sfx_result::success;
    }
};


static void uix_on_flush(const rect16& bounds,
                             const void *bitmap, void* state) {
    lcd_flush_bitmap(bounds.x1, bounds.y1, bounds.x2, bounds.y2, bitmap);
}
static void uix_on_touch(point16* out_locations,size_t* in_out_locations_size,void* state) {
    if(*in_out_locations_size>0) {
        uint16_t x,y;
        bool pressed = touch_xy(&x,&y);
        if(pressed) {
            out_locations->x=x;
            out_locations->y=y;
            *in_out_locations_size = 1;
        } else {
            *in_out_locations_size = 0;
        }
    }
}
constexpr static const size_t lcd_transfer_size = ((320*240)*2)/10;
static uint8_t* lcd_transfer_buffer = nullptr;

static void lcd_initialize_buffers() {
    lcd_transfer_buffer=(uint8_t*)malloc(lcd_transfer_size);
    if(lcd_transfer_buffer==nullptr) {
        puts("Unable to initialize transfer buffers.");
        ESP_ERROR_CHECK(ERR_MEM);
    }
}

void lcd_on_flush_complete() {
    lcd_display.flush_complete();
}
const_buffer_stream fps_font_stream(vga_8x8,sizeof(vga_8x8));
win_font fps_font(fps_font_stream);
static screen_t main_screen;
static camera_view cam_view;
static label_t fps_label;
static const_buffer_stream midi_stm(nullptr,0);
static midi_file_source midi_src;
static midi_clock main_clock;
static midi_sampler sampler;
static midi_sender midi_out;
static const constexpr bool big_cam = false;

void setup() {
    Serial.begin(115200);
    lcd_initialize(lcd_transfer_size);
    lcd_initialize_buffers();
    lcd_rotation(0);
    camera_initialize((big_cam?0:CAM_FRAME_SIZE_96X96));
    camera_levels(CAM_LOWEST,CAM_MEDIUM,CAM_MEDIUM,CAM_HIGH);
    camera_rotation(0);
    
    lcd_display.buffer_size(lcd_transfer_size);
    lcd_display.buffer1(lcd_transfer_buffer);
    lcd_display.on_flush_callback(uix_on_flush);
    main_screen.dimensions({240,320});
    main_screen.background_color(color_t::black);
    const int wmo = big_cam?239:95;
    srect16 sr(0,0,wmo,wmo);
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

    // SPIFFS.begin();
    // audio_initialize(AUDIO_11K_MONO);
    // tsf_alloc.alloc = malloc;
    // tsf_alloc.realloc = realloc;
    // tsf_alloc.free = free;
    // File file = SPIFFS.open("/1mgm.sf2","rb");
    // const size_t sf2_size = file.size();
    // void* sf2_data = ps_malloc(sf2_size);
    // file.read((uint8_t*)sf2_data,sf2_size);
    // tsf_handle = tsf_load_memory(sf2_data,sf2_size,&tsf_alloc);
    // free(sf2_data);
    // file.close();
    // file = SPIFFS.open("/fallout4.mid","rb");
    // const size_t midi_size = file.size();
    // void* midi_data = malloc(midi_size);
    // file.read((uint8_t*)midi_data,midi_size);
    // file.close();
    // SPIFFS.end();
    //midi_stm.set((const uint8_t*)midi_data,midi_size);
    //midi_file mid_file;
    //midi_file::read(midi_stm,&mid_file);
    //midi_stm.seek(0);
    //midi_sampler::read(midi_stm,&sampler);
    //sampler.output(&midi_out);
    //sampler.tempo_multiplier(1.0);
    // tsf_set_output(tsf_handle, TSF_MONO, 11025, 0);
    // tsf_set_max_voices(tsf_handle,8);
    // for(int i = 0;i<16;++i) {
    //     tsf_channel_init(tsf_handle,i);
    // }
    // tsf_channel_set_presetnumber(tsf_handle,9,0,1);
    // TaskHandle_t audio_task;
    // xTaskCreatePinnedToCore([](void* parm){
    //     uint32_t time_ts=0;
    //     while(1) {
    //         sampler.update();
    //         static float tsf_render_buffer[audio_max_samples];
    //         tsf_render_float(tsf_handle,tsf_render_buffer,audio_max_samples,0);
    //         audio_write_float(tsf_render_buffer,audio_max_samples,0.025);
    //         if(millis()>time_ts+150) {
    //             time_ts = millis();
    //             vTaskDelay(5);
    //         }
    //     }
    // },"audio_task",4096,nullptr,20,&audio_task,xTaskGetAffinity(xTaskGetCurrentTaskHandle()));

    Serial.printf("Free SRAM: %0.2fKB\n",(float)ESP.getFreeHeap()/1024.f);
}

void loop() {
    static char fps_text[64];
    static int frames = 0;
    static uint32_t ts = 0;
    static uint64_t total_ms=0;
    uint32_t start_ms = millis();
    cam_view.update();
    //main_screen.invalidate();//{0,0,0,0});
    lcd_display.update();
    uint32_t end_ms = millis();
    total_ms+=(end_ms-start_ms);
    ++frames;
    if(millis()>ts+1000) {
        ts=millis();
        sprintf(fps_text,"FPS: %d, Avg: %0.2fms",frames,total_ms/((float)frames==0?.001f:frames));
        puts(fps_text);
        fps_label.text(fps_text);
        frames = 0;
        total_ms = 0;
        
    } 
    
}
