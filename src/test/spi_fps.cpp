// SPI with no Adafruit Library
// spi_fps.cpp

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
        spi.write(hi);
        spi.write(lo);
    }

    CS_DISABLE();
}

void writeColorBuffer(uint16_t color, uint32_t numPixels) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    // Buffer for 1024 pixels (2 bytes each) - Don't really matter since less is more call and more doesn't really help
    const size_t BUFFER_SIZE = 16;
    uint8_t buffer[BUFFER_SIZE];

    CS_ENABLE();
    DC_DATA();

    // We don't need to go through all pixel since they all the same just one buffer is good
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
    // Initialize SPI hardware
    // Parameters: (SCLK pin, MISO pin (unused=-1), MOSI pin, SS pin (unused=-1))
    spi.begin(SCLK_PIN, -1, MOSI_PIN, -1);
    // Set SPI settings ONCE at startup (since we only have one SPI device)
    spi.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));

    // --- Setup GPIO pins ---
    pinMode(CS_PIN, OUTPUT);
    pinMode(DC_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);  // Start with display deselected
    digitalWrite(DC_PIN, HIGH);  // Default to data mode
    digitalWrite(RST_PIN, HIGH); // Not in reset
    delay(20);

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
    uint8_t clk[] = {0xF0};
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
    // 0x74 =  0b01110100
    //         ||||||||
    //         |||||||└─ Bit 0: Column address remap (0=normal, 1=reversed)
    //         ||||||└── Bit 1: Color sequence (0=ABC, 1=CBA)
    //         |||||└─── Bit 2: Column scan direction
    //         ||||└──── Bit 3: (reserved)
    //         |||└───── Bit 4: COM scan direction (up/down)
    //         ||└────── Bit 5: Odd/even split
    //         |└─────── Bit 6: 65k/262k color mode
    //         └──────── Bit 7: (reserved)
    sendData(remap, 1);

    // Display start line (usually 0)
    sendCommand(0xA1);
    uint8_t startline[] = {0x00};
    sendData(startline, 1);

    // Display offset (vertical shift, usually 0)
    sendCommand(0xA2);
    uint8_t offset[] = {0x00};
    sendData(offset, 1);

    // // Pre-charge voltage levels (affects refresh quality)
    // sendCommand(0xB4);
    // uint8_t prechg[] = {0xA0, 0xB5, 0x55};
    // sendData(prechg, 3);

    // GPIO configuration
    sendCommand(0xB5);
    uint8_t gpio[] = {0x00};
    sendData(gpio, 1);

    // Function selection (enable internal voltage regulator)
    sendCommand(0xAB);
    uint8_t func[] = {0x01};
    sendData(func, 1);

    sendCommand(0xA6);

    // Contrast for R, G, B channels
    sendCommand(0xC1);
    // uint8_t contrast[] = {0xC8, 0x80, 0xC8}; // Red, Green, Blue
    uint8_t contrast[] = {0xFF, 0xFF, 0xFF}; // Red, Green, Blue
    sendData(contrast, 3);

    // Master contrast (overall brightness)
    sendCommand(0xC7);
    uint8_t vcom[] = {0x0F};
    sendData(vcom, 1);

    // // Pre-charge period
    // sendCommand(0xB1);
    // uint8_t phase[] = {0x32};
    // sendData(phase, 1);

    // // Second pre-charge period
    // sendCommand(0xB6);
    // uint8_t prechg2[] = {0x01};
    // sendData(prechg2, 1);

    sendCommand(0xB2);
    uint8_t framerate[] = {0xA4, 0x00, 0x00};
    sendData(framerate, 3);

    // Turn display ON
    sendCommand(0xAF);
}

void setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    // Set column (X) address range
    sendCommand(0x15);        // Column address command
    uint8_t col[] = {x0, x1}; // Start column, End column
    sendData(col, 2);

    // Set row (Y) address range
    sendCommand(0x75);        // Row address command
    uint8_t row[] = {y0, y1}; // Start row, End row
    sendData(row, 2);

    // Prepare to write pixel data
    sendCommand(0x5C); // Write to RAM command
}

void setup() {
    Serial.begin(115200);
    Serial.println("SSD1351 Direct SPI test (fixed)");

    // Initialize the display
    ssd1351_init();
    delay(50);

    // Fill entire screen with BLACK first
    setWindow(0, 0, 127, 127);
    writeColor(BLACK, 128 * 128);
    delay(200);

    // Top quarter: RED
    setWindow(0, 0, 127, 31);
    writeColor(RED, 128 * 32);
    delay(200);

    // Second quarter: BLUE
    setWindow(0, 32, 127, 63);
    writeColor(BLUE, 128 * 32);
    delay(500);

    setWindow(0, 0, 127, 127);
    writeColor(BLACK, 128 * 128);

    setWindow(39, 39, 88, 88);
}

void loop() {
    unsigned long startTime = millis();

    writeColorBuffer(RED, 50 * 50);
    writeColorBuffer(BLUE, 50 * 50);

    unsigned long endTime = millis();
    float frameTime = endTime - startTime;
    float fps = 2000.0 / frameTime;

    Serial.print("Screen FPS: ");
    Serial.println(fps);

    delay(1000);
}

// 90.91 though with a buffer
/*
I get 500 if I reduce from 128 by 128 to 50 by 50
GOT A GOOD SOL
so when I render or get slices of the 3d images I then compares them to previous slice to see what simialrities
and then I can do a python script to analyze what is the average amoutn of pixels being sent
speically how many times do we change the windows since I don't yet know if window is a bottle neck
but I do know that sending less than a a byte in each buffer starts to slow things down
so I got to do the math to see how much
Since 16 is the minimum buffer size I'll try to batch the same pixels together at a minimum of hopefully 16 pixels at a time

but then it's also prov better to do the 3 speicfic pixels then try to batch with a bunch og unnecsary pixels
*/