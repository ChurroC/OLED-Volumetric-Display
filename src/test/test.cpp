// Custom SPI FPS Test
// custom_spi_fps.cpp

#include "driver/spi_master.h"
#include <Arduino.h>

spi_device_handle_t spi_device;

#define SCLK_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define DC_PIN 13
#define RST_PIN 14

#define SPI_FREQ 30000000UL

#define RED 0xF800
#define BLACK 0x0000
#define WHITE 0xFFFF

#define DC_COMMAND() digitalWrite(DC_PIN, LOW)
#define DC_DATA() digitalWrite(DC_PIN, HIGH)
#define CS_ENABLE() digitalWrite(CS_PIN, LOW)
#define CS_DISABLE() digitalWrite(CS_PIN, HIGH)

void setupSPIWithDMA() {
    // 1. Configure the SPI BUS (the hardware pins/wires)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = MOSI_PIN, // Data out pin (Master Out Slave In)
        .miso_io_num =
            -1, // Data in pin (-1 = not used, display is output only)
        .sclk_io_num = SCLK_PIN,      // Clock pin
        .quadwp_io_num = -1,          // Quad SPI pins (not used)
        .quadhd_io_num = -1,          // Quad SPI pins (not used)
        .max_transfer_sz = 64 * 1024, // Maximum bytes per DMA transfer (64KB)
    };

    // 2. Configure the DEVICE settings (how to talk to your display)
    spi_device_interface_config_t dev_cfg = {
        .mode = 3,                    // SPI Mode 3 (CPOL=1, CPHA=1)
                                      // Clock idle HIGH, sample on rising edge
        .clock_speed_hz = SPI_FREQ,   // 40MHz clock (can go higher!)
        .spics_io_num = CS_PIN,       // Chip Select pin
        .flags = SPI_DEVICE_NO_DUMMY, // No dummy bits between transfers
        .queue_size = 7,              // How many transactions can be queued
                                      // (allows queuing multiple transfers)
    };

    // 3. Initialize the SPI bus with DMA enabled
    // SPI_DMA_CH_AUTO = let ESP32 pick a DMA channel automatically
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    // 4. Attach your display device to the bus
    spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_device);
}

void fastDMAWrite(uint8_t *data, size_t len) {
    // Create a transaction descriptor
    spi_transaction_t trans = {
        .length = len * 8, // Length in BITS (not bytes!)
                           // So if len=1024 bytes, this is 8192 bits
        .tx_buffer = data, // Pointer to your data
        // Other fields we're not using:
        // .rx_buffer = NULL,   // No receive (display doesn't send data back)
        // .user = NULL,        // Custom user data
    };

    // Queue the transaction WITHOUT WAITING!
    // DMA sends in background while CPU continues
    spi_device_queue_trans(spi_device, &trans, portMAX_DELAY);

    /*
    Does NOT block! Returns immediately
    Queues the transaction in the background
    DMA sends while CPU continues executing
    Best for large transfers when you can do other work
    Use for: Large pixel buffers

    When to use: When you want to prepare the next frame while the current one is sending.
    */
}

void waitForDMAComplete() {
    spi_transaction_t *result;
    // Wait for queued transaction to finish
    spi_device_get_trans_result(spi_device, &result, portMAX_DELAY);
    /*
    - Waits for a **queued** transaction to finish
    - Must be called after `spi_device_queue_trans()`
    - Blocks until the transaction completes
    - Use for: Syncing after non-blocking sends

    **When to use:** After `queue_trans()` when you need to know it's done (e.g., before reusing the buffer).
    */
}

void sendCommand(uint8_t cmd) {
    DC_COMMAND();

    spi_transaction_t trans = {
        .flags =
            SPI_TRANS_USE_TXDATA, // Use tx_data instead of tx_buffer for small transfers
        .length = 8,
        .tx_data = {cmd}, // Inline data buffer for small commands
    };

    spi_device_polling_transmit(spi_device,
                                &trans); // Blocking, but fast for single byte
    /*
    Blocks until transfer completes
    Uses polling (busy-wait loop checking hardware status)
    No interrupts, no task switching
    Fastest for tiny transfers (< 32 bytes)
    Use for: Commands, single bytes, small data

    When to use: Single commands where you need it done NOW and the data is tiny.
    */
}

void sendData(uint8_t *data, size_t length) {
    DC_DATA();

    if (length <= 4) {
        // For small amounts of data, use inline buffer (faster)
        spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = length * 8,
        };
        for (size_t i = 0; i < length; i++) {
            trans.tx_data[i] = data[i];
        }
        spi_device_polling_transmit(spi_device, &trans);
    } else if (length < 1024) {
        // Medium data: blocking DMA
        spi_transaction_t trans = {
            .length = length * 8,
            .tx_buffer = data,
        };
        spi_device_transmit(spi_device, &trans); // ← BLOCKING DMA
    } else {
        // Large data: non-blocking DMA
        spi_transaction_t trans = {
            .length = length * 8,
            .tx_buffer = data,
        };
        spi_device_queue_trans(spi_device, &trans,
                               portMAX_DELAY); // ← NON-BLOCKING
    }
    /*
    Blocks until transfer completes
    Uses interrupts (CPU can do other things while waiting)
    CPU sleeps while DMA works, then interrupt wakes it
    Good for medium transfers (32 bytes - 4KB)
    Use for: Multi-byte data where you want to wait

    When to use: When you need the transfer done before continuing, but it's large enough that DMA saves CPU cycles.
    */
}

/*
## Visual Comparison:

POLLING_TRANSMIT:
CPU: [Send][Busy-Wait████████████][Done] ← CPU stuck waiting
DMA: [████████████████████████████]

TRANSMIT:
CPU: [Send][Sleep...][Wake][Done] ← CPU can context-switch
DMA: [████████████████████████]
          ↑ interrupt

QUEUE_TRANS:
CPU: [Queue][Continue doing other work...][Call get_result][Done]
DMA:        [████████████████████████]

QUEUE + GET_RESULT:
CPU: [Queue][Do work][Do work][Get Result][Wait if needed][Done]
DMA:        [████████████][Done]
                              ↑ CPU checks here

1-4 bytes: polling_transmit
Overhead of DMA/interrupts costs more than just sending

5-1K: transmit (blocking DMA) DMA helps, interrupts allow multitasking

1KB+: queue_trans + get_resultNon-blocking lets CPU do other work
*/

void ssd1351_init() {
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

void writeColorBuffer(uint16_t color, uint32_t numPixels) {
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    const size_t BUFFER_SIZE = 4096; // 2KB buffer
    static uint8_t buffer[BUFFER_SIZE];

    // Fill buffer with color
    for (size_t i = 0; i < BUFFER_SIZE; i += 2) {
        buffer[i] = hi;
        buffer[i + 1] = lo;
    }

    uint32_t bytesToSend = numPixels * 2;
    while (bytesToSend >= BUFFER_SIZE) {
        fastDMAWrite(buffer, BUFFER_SIZE);
        waitForDMAComplete();
        bytesToSend -= BUFFER_SIZE;
    }
    if (bytesToSend > 0) {
        fastDMAWrite(buffer, bytesToSend);
        waitForDMAComplete();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("SSD1351 Direct SPI test (fixed)");

    setupSPIWithDMA();

    ssd1351_init();
    delay(50);

    setWindow(0, 0, 127, 127);

    CS_ENABLE();
    DC_DATA();
    writeColorBuffer(BLACK, 128 * 128);
    CS_DISABLE();
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

        // Calculate horizontal extent directly
        int dx = (int)sqrt(radius * radius - dy_squared);

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

// Check SPI frequency now