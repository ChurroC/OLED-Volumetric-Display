// ball_test.cpp
// Custom Logic for sending data FPS

#include <Arduino.h>
#include <SPI.h>

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define DC_PIN 13
#define RST_PIN 14

#define SPI_FREQ 30000000UL

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 128

#define RED 0xF800
#define BLUE 0x001F
#define BLACK 0x0000
#define WHITE 0xFFFF

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
    data[0] = color >> 8;
    data[1] = color & 0xFF;
    sendData(data, 2);
}

void writeColorBuffer(uint16_t color, uint32_t numPixels) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    const size_t BUFFER_SIZE = 16;
    uint8_t buffer[BUFFER_SIZE];

    for (size_t i = 0; i < BUFFER_SIZE; i += 2) {
        buffer[i] = hi;
        buffer[i + 1] = lo;
    }

    uint32_t bytesToSend = numPixels * 2;
    while (bytesToSend >= BUFFER_SIZE) {
        spi.writeBytes(buffer, BUFFER_SIZE);
        bytesToSend -= BUFFER_SIZE;
    }
    if (bytesToSend > 0) {
        spi.writeBytes(buffer, bytesToSend);
    }
}

// void writeColorArrayBuffer(const uint16_t *colors, uint32_t numPixels) {
//     const size_t BUFFER_SIZE = 16;
//     uint8_t buffer[BUFFER_SIZE];

//     CS_ENABLE();
//     DC_DATA();

//     uint8_t bufferSends = for (int i = 0; i < numPixels; i++) {}

//     CS_DISABLE();
// }

void ssd1351_init() {
    spi.begin(SCLK_PIN, -1, MOSI_PIN, -1);
    spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));

    pinMode(CS_PIN, OUTPUT);
    pinMode(DC_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    digitalWrite(DC_PIN, HIGH);
    digitalWrite(RST_PIN, HIGH);
    delay(20);

    digitalWrite(RST_PIN, LOW);
    delay(20);
    digitalWrite(RST_PIN, HIGH);
    delay(20);

    sendCommand(0xFD);
    uint8_t unlock1[] = {0x12};
    sendData(unlock1, 1);

    sendCommand(0xFD);
    uint8_t unlock2[] = {0xB1};
    sendData(unlock2, 1);

    sendCommand(0xAE);

    sendCommand(0xB3);
    uint8_t clk[] = {0xF0};
    sendData(clk, 1);

    sendCommand(0xCA);
    uint8_t mux[] = {0x7F};
    sendData(mux, 1);

    sendCommand(0xA0);
    uint8_t remap[] = {0x74};
    sendData(remap, 1);

    sendCommand(0xA1);
    uint8_t startline[] = {0x00};
    sendData(startline, 1);

    sendCommand(0xA2);
    uint8_t offset[] = {0x00};
    sendData(offset, 1);

    sendCommand(0xB5);
    uint8_t gpio[] = {0x00};
    sendData(gpio, 1);

    sendCommand(0xAB);
    uint8_t func[] = {0x01};
    sendData(func, 1);

    sendCommand(0xA6);

    sendCommand(0xC1);
    uint8_t contrast[] = {0xFF, 0xFF, 0xFF};
    sendData(contrast, 3);

    sendCommand(0xC7);
    uint8_t vcom[] = {0x0F};
    sendData(vcom, 1);

    sendCommand(0xB2);
    uint8_t framerate[] = {0xA4, 0x00, 0x00};
    sendData(framerate, 3);

    sendCommand(0xAF);
}

void setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    sendCommand(0x15);
    uint8_t col[] = {x0, x1};
    sendData(col, 2);

    sendCommand(0x75);
    uint8_t row[] = {y0, y1};
    sendData(row, 2);

    sendCommand(0x5C);
}

void setup() {
    Serial.begin(115200);
    Serial.println("SSD1351 Direct SPI test (fixed)");

    ssd1351_init();
    delay(50);

    // setWindow(0, 0, 127, 127);
    // writeColorBuffer(BLACK, 128 * 128);
    // delay(2000);

    // // Top quarter: RED
    // setWindow(0, 0, 127, 31);
    // writeColorBuffer(RED, 128 * 32);
    // delay(200);

    // // Second quarter: BLUE
    // setWindow(0, 32, 127, 63);
    // writeColorBuffer(BLUE, 128 * 32);
    // delay(500);

    // setWindow(0, 0, 127, 127);
    // writeColorBuffer(BLACK, 128 * 128);
}

const uint8_t screenSize = 20;
const uint8_t ballSize = 16;
uint8_t row = 20;
uint8_t col = 20;

void loop() {
    unsigned long startTime = micros();

    setWindow(row, col, row + screenSize - 1, col + screenSize - 1);

    CS_ENABLE();
    DC_DATA();
    // First in this screen the top 2 and bottom 2 rows are
    writeColorBuffer(RED, screenSize);
    writeColorBuffer(RED, 1);
    writeColorBuffer(BLACK, screenSize - 2);
    writeColorBuffer(RED, 1);

    int radius = ballSize / 2;
    int center = (screenSize - 2) / 2;

    for (int y = 0; y < ballSize; y++) {
        writeColorBuffer(RED, 1);

        // Calculate distance from center
        int dy = y - radius;
        int dy_squared = dy * dy;

        // Find horizontal extent of circle at this row
        int dx = 0;
        while (dx * dx + dy_squared <= radius * radius) {
            dx++;
        }
        dx--; // Step back to last valid position

        // Draw the row
        int blackBefore = center - dx;
        int whiteWidth = dx * 2 + 1;
        int blackAfter = screenSize - 2 - blackBefore - whiteWidth;

        writeColorBuffer(BLACK, blackBefore);
        writeColorBuffer(WHITE, whiteWidth);
        writeColorBuffer(BLACK, blackAfter);

        writeColorBuffer(RED, 1);
    }

    writeColorBuffer(RED, 1);
    writeColorBuffer(BLACK, screenSize - 2);
    writeColorBuffer(RED, 1);
    writeColorBuffer(RED, screenSize);

    unsigned long endTime = micros();
    float frameTime = endTime - startTime;
    float fps = 2000000.0 / frameTime;

    Serial.print("Screen FPS: ");
    Serial.println(fps);

    delay(3);

    writeColorBuffer(BLACK, screenSize * screenSize);
    CS_DISABLE();

    row += screenSize / 15;
    col += screenSize / 15;
}

// Removed CS since it causes FPS to drop from around 5381 to 4300 - 20%ish
// Need to change CS due to change from command to data mode for window to bytes to draw which gets fps to 4807
// But if I move to the final buffer turns out that writing spi_write bytes requires me to do cs diable and enable every time to run
// Need to figure out when to use CS_ENABLE and DISABLE correctly to minimize the use
// Also need to see if I can do a custom implemtaton of SPI with my custom DMA to further speed this up