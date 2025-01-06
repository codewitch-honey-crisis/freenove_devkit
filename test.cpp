#include <Arduino.h>
#include <fns3devkit.hpp>
static uint16_t bmp = 0; // one black pixel
void setup() {
    lcd_initialize();
}
void lcd_on_flush_complete() {

}
void loop() {
    delay(1000);
    lcd_flush_bitmap(0,0,0,0,&bmp);
}