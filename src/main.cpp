#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>

// Pin definitions for ESP32-S3
#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN   13
#define RST_PIN  14

// Display size
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

// Create display object
Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting OLED test...");
  
  // Initialize SPI with custom pins
  SPI.begin(SCLK_PIN, -1, MOSI_PIN, CS_PIN);
  
  // Initialize display
  display.begin();
  
  Serial.println("Display working!");
  
  // Fill screen with colors
  display.fillScreen(0xF800);  // Red
  delay(1000);
  
  display.fillScreen(0x07E0);  // Green
  delay(1000);
  
  display.fillScreen(0x001F);  // Blue
  delay(1000);
  
  display.fillScreen(0xFFFF);  // White
  
  Serial.println("Test complete!");
}

void loop() {
  display.fillScreen(0xF800);  // Red
  delay(1000);
  
  display.fillScreen(0x07E0);  // Green
  delay(1000);
}