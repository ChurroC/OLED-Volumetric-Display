// Custom Logic for sending data FPS

#include <Arduino.h>
#include <SPI.h>

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define DC_PIN 13
#define RST_PIN 14

#define SPI_FREQ 40000000UL

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

#define RED 0xF800
#define BLUE 0x001F
#define BLACK 0x0000

#define DC_COMMAND() digitalWrite(DC_PIN, LOW)
#define DC_DATA() digitalWrite(DC_PIN, HIGH)
#define CS_ENABLE() digitalWrite(CS_PIN, LOW)
#define CS_DISABLE() digitalWrite(CS_PIN, HIGH)

SPIClass spi = SPI;

void sendCommand(uint8_t cmd) {
    CS_ENABLE();
    DC_COMMAND();
    spi.transfer(cmd);
    CS_DISABLE();
}

void sendData(uint8_t *data, size_t length) {
    CS_ENABLE();
    DC_DATA();
    spi.writeBytes(data, length);
    CS_DISABLE();
}

void writePixelData(uint16_t color) {
    uint8_t data[2];
    data[0] = color >> 8;   // High byte
    data[1] = color & 0xFF; // Low byte
    sendData(data, 2);
}

// Initally I created an array to send to sendData but don't want to create so
// many array or disable and enable CS pin so often This will write the same
// color the numPixels times
void writeColor(uint16_t color, uint32_t numPixels) {
    // Since SPI can only transfer bytes, we have to split the 16-bit color into two 8-bit values
    // Lets say we have full red and blue
    // 1111 1000 0001 1111
    // RRRRR GGG GGG BBBBB
    //   31   0   0   31
    // >> 8 shifts right by 8 bits
    // from 1111 1000 0001 1111 we get 0000 0000 1111 1000
    // and casting gets us 111 1000 = 0xF8
    uint8_t hi = color >> 8;
    // & 0xFF masks everything but the lower 8 bits
    // from 1111 1000 0001 1111 we get 0000 0000 0001 1111
    // and casting gets us 000 1111 = 0x1F
    uint8_t lo = color & 0xFF;

    CS_ENABLE();
    DC_DATA();

    // For each pixel we need to send the high byte then the low byte
    for (uint32_t i = 0; i < numPixels; i++) {
        sendData(&hi, 1);
        sendData(&lo, 1);
    }

    CS_DISABLE();
}

// Const since we won't modify it and the 16-bit is not a color but pointer to an array of colors
void writeColorArray(const uint16_t *colors, uint32_t numPixels) {
    CS_ENABLE();
    DC_DATA();

    for (int i = 0; i < numPixels; i++) {
        uint16_t color = colors[i];
        spi.write(color >> 8);
        spi.write(color & 0xFF);
    }

    CS_DISABLE();
}

void ssd1351_init() {
    // --- Setup GPIO pins ---
    pinMode(CS_PIN, OUTPUT);
    pinMode(DC_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);  // Start with display deselected
    digitalWrite(DC_PIN, HIGH);  // Default to data mode
    digitalWrite(RST_PIN, HIGH); // Not in reset

    // --- Hardware Reset Sequence ---
    // This physically resets the display controller
    digitalWrite(RST_PIN, LOW);  // Pull reset LOW to reset
    delay(20);                   // Hold for 20ms
    digitalWrite(RST_PIN, HIGH); // Release reset
    delay(20);                   // Wait for display to stabilize

    // --- Configuration Commands ---
    // Each command (0xXX) configures a specific display feature

    // Unlock command interface (security feature of SSD1351)
    sendCommand(0xFD);
    uint8_t unlock1[] = {0x12};
    sendData(unlock1, 1);

    sendCommand(0xFD);
    uint8_t unlock2[] = {0xB1};
    sendData(unlock2, 1);

    // Turn display OFF during configuration
    sendCommand(0xAE);

    // Clock divider and oscillator frequency
    sendCommand(0xB3);
    uint8_t clk[] = {0xF1}; // 0xF1 = fast refresh rate
    sendData(clk, 1);

    // Multiplex ratio (number of rows)
    sendCommand(0xCA);
    uint8_t mux[] = {0x7F}; // 0x7F = 128 rows (127 + 1)
    sendData(mux, 1);

    // Remap and color depth settings
    sendCommand(0xA0);
    uint8_t remap[] = {0x74}; // 0x74 configures:
                              // - RGB color order
                              // - Column address mapping
                              // - Scan direction
    sendData(remap, 1);

    // Display start line (usually 0)
    sendCommand(0xA1);
    uint8_t startline[] = {0x00};
    sendData(startline, 1);

    // Display offset (vertical shift, usually 0)
    sendCommand(0xA2);
    uint8_t offset[] = {0x00};
    sendData(offset, 1);

    // Function selection (enable internal voltage regulator)
    sendCommand(0xAB);
    uint8_t func[] = {0x01};
    sendData(func, 1);

    // Pre-charge voltage levels (affects refresh quality)
    sendCommand(0xB4);
    uint8_t prechg[] = {0xA0, 0xB5, 0x55};
    sendData(prechg, 3);

    // Contrast for R, G, B channels
    sendCommand(0xC1);
    uint8_t contrast[] = {0xC8, 0x80, 0xC8}; // Red, Green, Blue
    sendData(contrast, 3);

    // Master contrast (overall brightness)
    sendCommand(0xC7);
    uint8_t vcom[] = {0x0F};
    sendData(vcom, 1);

    // Pre-charge period
    sendCommand(0xB1);
    uint8_t phase[] = {0x32};
    sendData(phase, 1);

    // GPIO configuration
    sendCommand(0xB5);
    uint8_t gpio[] = {0x00};
    sendData(gpio, 1);

    // Second pre-charge period
    sendCommand(0xB6);
    uint8_t prechg2[] = {0x01};
    sendData(prechg2, 1);

    // Turn display ON
    sendCommand(0xAF);
}

void setup() {
    Serial.begin(115200);
    Serial.println("SSD1351 Direct SPI test (fixed)");

    // Initialize SPI hardware
    // Parameters: (SCLK pin, MISO pin (unused=-1), MOSI pin, SS pin (unused=-1))
    spi.begin(SCLK_PIN, -1, MOSI_PIN, -1);

    // Set SPI settings ONCE at startup (since we only have one SPI device)
    spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));

    // Initialize the display
    ssd1351_init();
    delay(50);

    // Clear screen to black
    writeColor(RED, SCREEN_WIDTH * SCREEN_HEIGHT);
    delay(50);
}

void loop() { writeColor(RED, SCREEN_WIDTH * SCREEN_HEIGHT); }
