# Actros Light & Sound
This is a fork of the module developed by TheDIYGuy999 modified for own needs.

This version was stripped down and works only with the combination of specific components used for my truck:
- Tamiya MB Actros with 3-speed-transmission
- Quicrun 1080 ESC
- Graupner mz-12 PRO transmitter
- gr-12l receiver working in SUMD mode on CH6 connector (all other modes like SBUS and PWM completely removed from code)
- 12 channels
- WS2812 RGB LED for MB star
- 80x160 SPI RGB display for the dashboard
- shaker motor support completely removed

## Dashboard
If you only interested in using the dashboard - read on.
For the dashboard I used a cheap 0.96 80x160 pixel color LCD with SPI control.
The graphics are drawn using the Arduino TFT_eSPI library - you have to install it using the Library Manager.
To integrate the dashboard in your project copy the files `dashboard.h` and `dashboard.cpp` to your project folder and check the documentation in the header file.
TFT_eSPI lib requires a display specific configuration to work, this process is described in the documentation.

To summarize it:
- create the folder `TFT_eSPI_Setups` inside you Arduino library directory
- move/copy the file `Setup43_ST7735_ESP32_80x160.h` to `TFT_eSPI_Setups` directory
- replace the line `#include <User_Setup.h>` in the file `User_Setup_Select.h` inside TFT_eSPI library folder with `#include <../TFT_eSPI_Setups/Setup43_ST7735_ESP32_80x160.h>`
- modify the `Setup43_ST7735_ESP32_80x160.h` file if you use other pins for the display connection

In my setup I use following pins for the LCD board:
- TFT_MOSI (SDA pin on LCD board) - IO23 (I don't have the shaker motor)
- TFT_SCLK (SCL pin on LCD board) - IO18 (I plan to connect side lights (if any) to low beam lights)
- TFT_CS (connect CS pin of LCD board to GND) - -1 (not used)  
- TFT_DC (DC pin on LCD board) - IO19 (my truck has no blue lights)
- TFT_RST (RES pin on LCD board) - IO21 (my truck has no blue lights)

Since the ESP32 is quite flexible in the choice of the pins for the SPI communication, try other unused pins of your board.
