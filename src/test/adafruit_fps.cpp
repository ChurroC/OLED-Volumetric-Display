// Adafruit FPS Test

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SPI.h>

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN   13
#define RST_PIN  14

#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

Adafruit_SSD1351 display = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN); // We use & to pass the SPI instance or address

void setup() {
  Serial.begin();
  Serial.println("Starting OLED test...");
  
  // Initialize SPI with custom pins
  SPI.begin(SCLK_PIN, -1, MOSI_PIN, CS_PIN);

  // Initialize display
  display.begin();

  display.fillScreen(BLACK);
}

void loop() {
  unsigned long startTime = millis();
  
  display.fillScreen(RED);
  display.fillScreen(BLUE);
  
  unsigned long endTime = millis();
  float frameTime = endTime - startTime;
  float fps = 2000.0 / frameTime; // 2 full screen updates
  
  Serial.print("Screen FPS: ");
  Serial.println(fps);
  
  delay(1000);
}

// 28.57 FPS approximately for two full screen updates (RED and BLUE)