// Custom Logic for sending data FPS

#include <Arduino.h>
#include <SPI.h>

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN   13
#define RST_PIN  14

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

#define RED     0xF800
#define BLUE    0x001F
#define BLACK   0x0000

#define DC_COMMAND() digitalWrite(DC_PIN, LOW)
#define DC_DATA()    digitalWrite(DC_PIN, HIGH)
#define CS_ENABLE()  digitalWrite(CS_PIN, LOW)
#define CS_DISABLE() digitalWrite(CS_PIN, HIGH)

SPIClass spi = SPI;

void sendCommand(uint8_t cmd) {
  CS_ENABLE();
  DC_COMMAND();
  spi.transfer(cmd);
  CS_DISABLE();
}

void sendData(uint8_t* data, size_t length) {
  CS_ENABLE();
  DC_DATA();
  spi.writeBytes(data, length);
  CS_DISABLE();
}

void writePixelData(uint16_t color) {
  uint8_t data[2];
  data[0] = color >> 8;    // High byte
  data[1] = color & 0xFF;  // Low byte
  sendData(data, 2);
}

// Initally I created an array to send to sendData but don't want to create so many array or disable and enable CS pin so often
// This will write the same color the numPixels times
void writeColor(uint16_t color, uint32_t numPixels) {
  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;

    CS_ENABLE();
    DC_DATA();

  for (uint32_t i = 0; i < numPixels; i++) {
    sendData(&hi, 1);
    sendData(&lo, 1);
  }
}
void setup() {
}

void loop() {
}
