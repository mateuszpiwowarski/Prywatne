// Lokalny setup dla TFT_eSPI dla wyswietlacza ST7789 1.92" 240x280.
#define USER_SETUP_LOADED

#define ST7789_DRIVER
#define TFT_WIDTH 240
#define TFT_HEIGHT 280
#define CGRAM_OFFSET

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 17

#define SPI_FREQUENCY 27000000
#define SPI_READ_FREQUENCY 20000000
#define TFT_SPI_MODE SPI_MODE3

#define LOAD_GFXFF
#define SMOOTH_FONT
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
