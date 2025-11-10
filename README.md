Items:
Waveshare 1.5in RGB OLED - It uses SPI - It uses the driver chip SSD1351 which is fastest for a color OLED but for industry really fast I could get a SSD1322 in grayscale
Hosyond ESP32-S3 - It has 16mb flash and 8mb of PSRAM and ESP-S3 has a 80 Mhz for SPI
# Could get FPGA which could easily do 100+ Mhz but hella hard to work with

Code:
Python:
Need a script to break down 3d files into prerendered images to send or images to raw dara
Also need script to cut blender into 360 images

C++:
Could use Adafruit plug in but it is slow
We could isntead do a custom directly to SPI
We could also do DMA (direct memory acces) which transfer the data without CPU so no waiting
We could doubel buffering which I've seen before don't know if it applies
We could use the direct hardware of SPI isntead of the software - since I think the ESP32 has dedicted SPI hardware which we can use instead of togglin the GPIO hella - but the adapfruit could use the hardware too but still overhead



Right now adafruit is what this does for example
application - fillScreen(RED) - what we're going to draw
driver / protocol - Adafruit_SSD1351 - how to talk to display controller
Hardware - SPI - how to send bytes

so for that driver / protocol we could do all the protocols like what the dc pins should be for command or data



TODO:
set up env so only test has the adafruit libraries

FPS Results:
Pure Adafruit - 28.57

Sites to use:
https://learn.adafruit.com/adafruit-gfx-graphics-library/graphics-primitives - Adafruit GFX
https://www.circuitbasics.com/basics-of-the-spi-communication-protocol/ - Basics of SPI
https://youtu.be/0nVNwozXsIc?si=CIpX_w9Aohazvl-p - Also even better idea of SPI