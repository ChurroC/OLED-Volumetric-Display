#include <Arduino.h>
#include <SPI.h>

// === Pin definitions ===
#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN   10
#define DC_PIN   13
#define RST_PIN  14

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

// === Basic color constants (RGB565) ===
#define RED     0xF800
#define BLUE    0x001F
#define BLACK   0x0000

// === SPI frequency ===
#define SPI_FREQ 40000000UL  // 40 MHz

// === Helper macros ===
#define DC_COMMAND() digitalWrite(DC_PIN, LOW)
#define DC_DATA()    digitalWrite(DC_PIN, HIGH)
#define CS_ENABLE()  digitalWrite(CS_PIN, LOW)
#define CS_DISABLE() digitalWrite(CS_PIN, HIGH)

SPIClass spi = SPI;

void sendCommand(uint8_t cmd) {
  // Use a transaction so the SPI settings are guaranteed for this transfer
  spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  DC_COMMAND();
  CS_ENABLE();
  spi.write(cmd);          // send single command byte
  CS_DISABLE();
  spi.endTransaction();
}

void sendData(const uint8_t *data, uint32_t len) {
  if (len == 0) return;
  spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  DC_DATA();
  CS_ENABLE();
  // writeBytes is non-blocking in name but blocks until FIFO accepts bytes; it's fine here
  spi.writeBytes(data, len);
  CS_DISABLE();
  spi.endTransaction();
}

void writeColor(uint16_t color, int nPixels) {
  // For large buffers you may want to chunk this to avoid big blocking sections
  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;

  spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
  DC_DATA();
  CS_ENABLE();
  for (int i = 0; i < nPixels; i++) {
    spi.write(hi);
    spi.write(lo);
  }
  CS_DISABLE();
  spi.endTransaction();
}

// === SSD1351 initialization sequence (expanded, safe minimal) ===
void ssd1351_init() {
  pinMode(CS_PIN, OUTPUT);
  pinMode(DC_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(DC_PIN, HIGH);
  digitalWrite(RST_PIN, HIGH);

  // Hardware reset
  digitalWrite(RST_PIN, LOW);
  delay(20);
  digitalWrite(RST_PIN, HIGH);
  delay(20);

  // Unlock, basic functional setup. This sequence is commonly used and known to work.
  sendCommand(0xFD); uint8_t unlock1[] = {0x12}; sendData(unlock1, 1);
  sendCommand(0xFD); uint8_t unlock2[] = {0xB1}; sendData(unlock2, 1);
  sendCommand(0xAE); // Display Off
  sendCommand(0xB3); uint8_t clk[] = {0xF1}; sendData(clk, 1); // Clock div / oscillator (common safe value)
  sendCommand(0xCA); uint8_t mux[] = {0x7F}; sendData(mux, 1); // Multiplex ratio (127 = 128MUX)
  sendCommand(0xA0); uint8_t remap[] = {0x74}; sendData(remap, 1); // RGB remap (common for 128x128)
  sendCommand(0xA1); uint8_t startline[] = {0x00}; sendData(startline, 1);
  sendCommand(0xA2); uint8_t offset[] = {0x00}; sendData(offset, 1);
  sendCommand(0xAB); uint8_t func[] = {0x01}; sendData(func, 1); // Enable internal VDD regulator (if required)
  sendCommand(0xB4); uint8_t prechg[] = {0xA0, 0xB5, 0x55}; sendData(prechg, 3); // Pre-charge
  sendCommand(0xC1); uint8_t contrast[] = {0xC8, 0x80, 0xC8}; sendData(contrast, 3);
  sendCommand(0xC7); uint8_t vcom[] = {0x0F}; sendData(vcom, 1);
  sendCommand(0xB1); uint8_t phase[] = {0x32}; sendData(phase, 1);
  sendCommand(0xB5); uint8_t gpio[] = {0x00}; sendData(gpio, 1);
  sendCommand(0xB6); uint8_t prechg2[] = {0x01}; sendData(prechg2, 1);
  sendCommand(0xAF); // Display ON
}

void setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  sendCommand(0x15); uint8_t col[] = {x0, x1}; sendData(col, 2); // Set column
  sendCommand(0x75); uint8_t row[] = {y0, y1}; sendData(row, 2); // Set row
  sendCommand(0x5C); // Write RAM
}

void fillScreen(uint16_t color) {
  setWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
  writeColor(color, SCREEN_WIDTH * SCREEN_HEIGHT);
}

void setup() {
  Serial.begin(115200);
  Serial.println("SSD1351 Direct SPI test (fixed)");

  // Start SPI but don't hand CS to the SPI driver (we manage CS manually)
  // Use -1 for the "SS" param so SPI driver won't control CS.
  spi.begin(SCLK_PIN, -1, MOSI_PIN, -1);
  // Note: we don't call beginTransaction globally here. it's used per-transfer.

  // ssd1351_init();
  // delay(50);
  // fillScreen(BLACK); // clear
  // delay(50);
}

void loop() {
  unsigned long start = millis();

  // fillScreen(RED);
  // delay(10);
  // fillScreen(BLUE);

  unsigned long elapsed = millis() - start;
  float fps = 2000.0f / (float)elapsed;
  Serial.printf("FPS: %.2f\n", fps);

  delay(1000);
}
