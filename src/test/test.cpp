// Adafruit FPS Test

#include <Arduino.h>
#include <SPI.h>

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN   13
#define RST_PIN  14

#define BLUE     0x001F
#define RED      0xF800

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

void setup() {
  Serial.begin();
  Serial.println("Starting OLED test...");
  SPI.begin(SCLK_PIN, -1, MOSI_PIN, CS_PIN);
}

void loop() {
  unsigned long startTime = millis();
  

  
  unsigned long endTime = millis();
  float frameTime = endTime - startTime;
  float fps = 2000.0 / frameTime;
  
  Serial.print("Screen FPS: ");
  Serial.println(fps);
  
  delay(1000);
}

// 28.57 FPS approximately for two full screen updates (RED and BLUE)