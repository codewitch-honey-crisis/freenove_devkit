#include <Arduino.h>
#include <fns3devkit.hpp>
static uint16_t bmp = 0; // one black pixel
static volatile int flushing =0;
void setup() {
    puts("TEST");
       
    lcd_initialize();
}
void lcd_on_flush_complete() {
    flushing = 0;
}
void loop() {
    delay(10);
    while(flushing) {
        taskYIELD();
    }
    flushing=1;
    lcd_flush_bitmap(0,0,0,0,&bmp);
}