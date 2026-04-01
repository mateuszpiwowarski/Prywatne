// tft_setup.h — lokalny setup dla TFT_eSPI
#define USER_SETUP_LOADED



// Sterownik i rozmiar
#define ST7789_DRIVER
#define TFT_WIDTH  172
#define TFT_HEIGHT 320
#define TFT_X_OFFSET 34
#define TFT_Y_OFFSET 0

// Piny SPI (dopasuj do swojej płytki!)
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS    5
#define TFT_DC    16
#define TFT_RST   17
// #define TFT_BL   15    // jeśli masz pin podświetlenia (opcjonalnie)

// Częstotliwości SPI (bezpiecznie zacząć od 27–40 MHz)
#define SPI_FREQUENCY       27000000
#define SPI_READ_FREQUENCY  20000000

// ST7789 zwykle wymaga trybu SPI_MODE3 (biblioteka ustawi to sama, ale można jawnie)
#define TFT_SPI_MODE SPI_MODE3
#define LOAD_GFXFF    // włącz FreeFonts (Adafruit_GFX)
#define SMOOTH_FONT   // (opcjonalnie) ładniejsze wygładzanie

// Lekkie fonty (opcjonalnie)
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8