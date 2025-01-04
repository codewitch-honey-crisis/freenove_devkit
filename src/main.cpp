#include <Arduino.h>
#include "fns3devkit.hpp"
#include <gfx.hpp>
#include <uix.hpp>
#define VGA_8X8_IMPLEMENTATION
#include "assets/vga_8x8.h"
using namespace gfx;
using namespace uix;

static uix::display lcd_display;

using screen_t = uix::screen<rgb_pixel<16>>;
using label_t = label<typename screen_t::control_surface_type>;
using color_t = color<typename screen_t::pixel_type>;
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
    void on_before_paint() override {
        
    }
    void on_paint(control_surface_type& destination, const srect16& clip) {
        const void* bitmap_data = camera_lock_frame_buffer();
        const_bitmap<rgb_pixel<16>> cbmp(destination.dimensions(),bitmap_data);
        draw::bitmap(destination,clip,cbmp,((rect16)clip).crop(cbmp.bounds()));
        camera_unlock_frame_buffer();
    }
    void on_after_paint() override {
        
    }
};
static void uix_on_flush(const rect16& bounds,
                             const void *bitmap, void* state) {
    lcd_flush_bitmap(bounds.x1, bounds.y1, bounds.x2, bounds.y2, bitmap);
}
static void uix_on_touch(point16* out_locations,size_t* in_out_locations_size,void* state) {
    if(*in_out_locations_size>0) {
        uint16_t x,y;
        lcd_touch_update();
        
        bool pressed = lcd_touch_pressed(&x,&y);
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
        puts("Unable to initialize transfer buffer.");
        ESP_ERROR_CHECK(ERR_MEM);
    }
}

void lcd_on_flush_complete() {
    lcd_display.flush_complete();
}
const_buffer_stream fps_font_stream(vga_8x8,sizeof(vga_8x8));
win_font fps_font(fps_font_stream,0);
static screen_t main_screen;
static camera_view cam_view;
static label_t fps_label;
void setup() {
    Serial.begin(115200);
    lcd_initialize(lcd_transfer_size);
    lcd_rotation(0);
    lcd_initialize_buffers();
    camera_initialize(CAM_FRAME_SIZE_96X96);
    lcd_display.buffer_size(lcd_transfer_size);
    lcd_display.buffer1(lcd_transfer_buffer);
    lcd_display.on_flush_callback(uix_on_flush);
    main_screen.dimensions({240,320});
    main_screen.background_color(color_t::white);
    srect16 sr(0,0,95,95);
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
    fps_label.color(color<rgba_pixel<32>>::black);
    fps_label.text("");
    fps_label.text_justify(uix_justify::top_left);
    main_screen.register_control(fps_label);
    lcd_display.active_screen(main_screen);
}

void loop() {
    static char fps_text[64];
    static int frames = 0;
    static uint32_t ts = 0;
    static uint64_t total_ms=0;
    uint32_t start_ms = millis();
    cam_view.update();
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
