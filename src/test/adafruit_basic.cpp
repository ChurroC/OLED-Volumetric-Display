// Adafruit Basic Test
// adafruit_basic.cpp

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <Arduino.h>
#include <SPI.h>

// Pin definitions for ESP32-S3
#define SCLK_PIN 12 // SPI Clock - sends timing pulses to sync data
#define MOSI_PIN 11 // Master Out Slave In - data from ESP32 to display
#define CS_PIN 10   // Chip Select - tells display "I'm talking to you now"
#define DC_PIN                                                                 \
    13 // Data/Command - tells display if you're sending a command or pixel data
#define RST_PIN 14 // Reset - hardware reset pin to restart the display
/*
SCLK: The "metronome" keeping everyone in sync
MOSI: The actual data being sent
CS: "Hey display, listen to me now!"
DC: "This is a command" vs "This is pixel data"
*/

/*
Pin Colors
GND - Black
VCC - Red
10 - Orange
11 - Blue
12 - Yellow
13 - Green
14 - White
*/

#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

// Display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

// Create display object
Adafruit_SSD1351 display =
    Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN,
                     RST_PIN); // We use & to pass the SPI instance or address

void setup() {
    Serial.begin(115200); // Bit speed
    Serial.println("Starting OLED test...");

    // Initialize SPI with custom pins
    SPI.begin(SCLK_PIN, -1, MOSI_PIN, CS_PIN); // -1 means "no MISO pin needed"

    // Initialize display
    display.begin();
    Serial.println("Display working!");

    // Fill screen with colors (these are RGB565 format - 16-bit color)
    // 0xF800 = Red (11111 000000 00000 in binary)
    // 0x is this number is in hexadecimal (base 16)
    // F    8    0    0
    // ↓    ↓    ↓    ↓
    // 1111 1000 0000 0000  (in binary)
    // So red has 5 bits so 11111 000000 00000 or max red with value of 31 (2^5-1)
    display.fillScreen(RED);
    delay(200);
    display.fillScreen(GREEN);
    delay(200);
    display.fillScreen(BLUE);
    delay(200);
    display.fillScreen(WHITE);
    delay(200);
    display.fillScreen(BLACK);

    display.drawLine(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1,
                     YELLOW); // Diagonal line

    Serial.println("Test complete!");
}

void loop() {
    display.fillScreen(RED);
    delay(1000);

    display.fillScreen(GREEN);
    delay(1000);
}