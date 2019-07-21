/*************************************************** 
  This is a library for the Adafruit 1.8" SPI display.
  This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
  as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ST7789_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

 
#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

void  ST7789_t3::setRotation(uint8_t m) 
{
  beginSPITransaction();
  writecommand(ST7735_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB);

     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     writedata(ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _xstart = _rowstart;
     _ystart = _colstart;
     break;
  case 2:
     writedata(ST77XX_MADCTL_RGB);
 
     _xstart = _colstart;
     _ystart = 0; //_rowstart;
     break;

   case 3:
     writedata(ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB);

     _xstart = 0; //_rowstart;
     _ystart = _colstart;
     break;
  }

  _rot = m;  
  endSPITransaction();
//  Serial.printf("Set rotation %d start(%d %d) row: %d, col: %d\n", m, _xstart, _ystart, _rowstart, _colstart);
}

#define ST7789_240x240_XSTART 0
#define ST7789_240x240_YSTART 80

// Probably should use generic names like Adafruit..
#define DELAY 0x80
static const uint8_t PROGMEM
  cmd_240x240[] = {                  // Initialization commands for 7735B screens
    10,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      150,                     //    150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x55,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, ST7789_240x240_XSTART,             //     XSTART = 0
    (240+ST7789_240x240_XSTART) >> 8, (240+ST7789_240x240_XSTART) & 0xFF,             //      XEND = 240
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, ST7789_240x240_YSTART,             //     YSTART = 0
      (240+ST7789_240x240_YSTART) >> 8, (240+ST7789_240x240_YSTART) & 0xFF,             //      YEND = 240
    ST7735_INVON ,   DELAY,  // hack
      10,
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
    255 };                  //     255 = 500 ms delay


void  ST7789_t3::init(uint16_t width, uint16_t height, uint8_t mode)
{
  Serial.printf("ST7789_t3::init mode: %x\n", mode);
	commonInit(NULL, mode);

	_colstart = ST7789_240x240_XSTART;
	_rowstart = ST7789_240x240_YSTART;
	_height = 240;
	_width = 240;
	commandList(cmd_240x240);
   setRotation(0);
}

