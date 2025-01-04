#define USER_SETUP_INFO "Freenove"

#define ST7789_DRIVER      // Full configuration option, define additional parameters below for this display

  #define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red

 #define TFT_INVERSION_OFF

#define TFT_MOSI 20 // In some display driver board, it might be written as "SDA" and so on.
#define TFT_SCLK 21
#define TFT_CS   47  // Chip select control pin
#define TFT_DC   0  // Data Command control pin

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters

 #define SPI_FREQUENCY  80000000

#define USE_HSPI_PORT
