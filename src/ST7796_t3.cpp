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

#include "ST7796_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

 
#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00
#define ST77XX_MADCTL_BGR 0x08
#define ST7796S_NOP            0x00
#define ST7796S_SWRESET        0x01

#define ST7796S_RDDID          0x04
#define ST7796S_RDDST          0x09
#define ST7796S_RDMODE         0x0A
#define ST7796S_RDMADCTL       0x0B
#define ST7796S_RDPIXFMT       0x0C
#define ST7796S_RDIMGFMT       0x0D
#define ST7796S_RDSELFDIAG     0x0F

#define ST7796S_SLPIN          0x10
#define ST7796S_SLPOUT         0x11
#define ST7796S_PTLON          0x12
#define ST7796S_NORON          0x13

#define ST7796S_INVOFF         0x20
#define ST7796S_INVON          0x21
//#define ST7796S_GAMMASET       0x26
#define ST7796S_DISPOFF        0x28
#define ST7796S_DISPON         0x29

#define ST7796S_CASET          0x2A
#define ST7796S_PASET          0x2B
#define ST7796S_RAMWR          0x2C
#define ST7796S_RAMRD          0x2E

#define ST7796S_PTLAR          0x30
#define ST7796S_VSCRDEF        0x33
#define ST7796S_MADCTL         0x36
#define ST7796S_VSCRSADD       0x37     /* Vertical Scrolling Start Address */
#define ST7796S_PIXFMT         0x3A     /* COLMOD: Pixel Format Set */

#define ST7796S_RGB_INTERFACE  0xB0     /* RGB Interface Signal Control */
#define ST7796S_FRMCTR1        0xB1
#define ST7796S_FRMCTR2        0xB2
#define ST7796S_FRMCTR3        0xB3
#define ST7796S_INVCTR         0xB4
#define ST7796S_DFUNCTR        0xB6     /* Display Function Control */

#define ST7796S_PWCTR1         0xC0
#define ST7796S_PWCTR2         0xC1
#define ST7796S_PWCTR3         0xC2
#define ST7796S_PWCTR4         0xC3
#define ST7796S_PWCTR5         0xC4
#define ST7796S_VMCTR1         0xC5

#define ST7796S_RDID1          0xDA
#define ST7796S_RDID2          0xDB
#define ST7796S_RDID3          0xDC
#define ST7796S_RDID4          0xDD

#define ST7796S_GMCTRP1        0xE0
#define ST7796S_GMCTRN1        0xE1
#define ST7796S_DGCTR1         0xE2
#define ST7796S_DGCTR2         0xE3

#define ST7796S_CSCON           0xFF

//-----------------------------------------------------------------------------
#define ST7796S_MAD_RGB        0x08
#define ST7796S_MAD_BGR        0x00

#define ST7796S_MAD_VERTICAL   0x20
#define ST7796S_MAD_X_LEFT     0x00
#define ST7796S_MAD_X_RIGHT    0x40
#define ST7796S_MAD_Y_UP       0x80
#define ST7796S_MAD_Y_DOWN     0x00



ST7796_t3::ST7796_t3(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST) : ST7735_t3(CS, RS, SID, SCLK, RST) 
{
  // Assume the majority of ones.
  tabcolor = INIT_ST7789_TABCOLOR;  // not used so set same as 89
  _screenHeight = 480;
  _screenWidth = 320;   
  
	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
  textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	setClipRect();
	setOrigin();
}

ST7796_t3::ST7796_t3(uint8_t CS, uint8_t RS, uint8_t RST) : 
      ST7735_t3(CS, RS, RST) 
{
  tabcolor = INIT_ST7789_TABCOLOR;
  _screenHeight = 480;
  _screenWidth = 320; 

	cursor_y  = cursor_x    = 0;
	textsize_x  = 1;
  textsize_y  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	setClipRect();
	setOrigin();
}


void  ST7796_t3::setRotation(uint8_t m) 
{
  beginSPITransaction();
  writecommand(ST7735_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata_last(ST77XX_MADCTL_MX | ST77XX_MADCTL_BGR);

     _xstart = _colstart;
     _ystart = _rowstart;
     _width = _screenWidth;
     _height = _screenHeight;
     break;
   case 1:
     writedata_last(ST77XX_MADCTL_MV | ST77XX_MADCTL_BGR);

     _xstart = _rowstart;
     _ystart = _colstart2;
     _height = _screenWidth;
     _width = _screenHeight;
     break;
  case 2:
     writedata_last(ST77XX_MADCTL_MY | ST77XX_MADCTL_BGR); 
     _xstart = _colstart2;
     _ystart = _rowstart2;
     _width = _screenWidth;
     _height = _screenHeight;
     break;

   case 3:
     writedata_last(ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_BGR);
     _xstart = _rowstart2;
     _ystart = _colstart;
     _height = _screenWidth;
     _width = _screenHeight;
     break;
  }

  _rot = m;  
  endSPITransaction();
//  Serial.printf("Set rotation %d start(%d %d) row: %d, col: %d\n", m, _xstart, _ystart, _rowstart, _colstart);
  setClipRect();
  setOrigin();
	
	cursor_x = 0;
	cursor_y = 0;
}

#define ST796SS_DFC 96
// Probably should use generic names like Adafruit..
#define DELAY 0x80
static const uint8_t PROGMEM
  cmd_ST7796[] = {                  // Initialization commands for ST7796 screens
    17,                             // 9 commands in list:

    ST7735_SWRESET,   DELAY,        //  1: Software reset, no args, w/delay
      150,                     //    150 ms delay

    ST7796S_SLPOUT,   DELAY,        // sleep exit
      120,  

    0xF0, 1, 0xC3,                  //Command Set control Enable extension command 2 partI                                
    0xF0, 1, 0x96,                  //Command Set control Enable extension command 2 partII
    ST7796S_MADCTL, 1, 0x48,        //Memory Data Access Control MX, MY, RGB mode                                    
                                    //X-Mirror, Top-Left to right-Buttom, RGB  
  
    ST7796S_PIXFMT, 1, 0x55,           //Interface Pixel Format 16 bits
  
    ST7796S_INVCTR, 1, 0x01,        // 1 dont inversion.
    ST7796S_DFUNCTR,3, 0x80, 0x02, 0x3B, //

    0xE8, 8,               //Display Output Ctrl Adjust
      0x40, 0x8A, 0x00, 0x00,
      0x29,    //Source eqaulizing period time= 22.5 us
      0x19,    //Timing for "Gate start"=25 (Tclk)
      0xA5,    //Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
      0x33,

   ST7796S_PWCTR2, 1,//Power control2                          
      0x06,         //VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+( vcom+vcom offset)
    ST7796S_PWCTR3, 1, // power control 3
      0xA7,    //Source driving current level=low, Gamma driving current level=High
   
    ST7796S_VMCTR1, DELAY | 1, //VCOM Control
      0x18,    //VCOM=0.9
      120,      // delay

  //ST7796 Gamma Sequence
    ST7796S_GMCTRP1,  14, //Gamma"+"                                             
      0xF0, 0x09, 0x0b, 0x06,
      0x04, 0x15, 0x2F, 0x54,
      0x42, 0x3C, 0x17, 0x14,
      0x18, 0x1B,
   
    ST7796S_GMCTRN1, DELAY | 14,  //Gamma"-"                                             
      0xE0, 0x09, 0x0B, 0x06,
      0x04, 0x03, 0x2B, 0x43,
      0x42, 0x3B, 0x16, 0x14,
      0x17, 0x1B,
      120,                      // delay
  
    0xF0, 1, 0x3C,            // disable command set part 1

    0xF0, DELAY | 1, 0x69,            // disable command set part 2
      120,                  // delay

    ST7796S_DISPON, DELAY,
      120
};

void  ST7796_t3::init(uint16_t width, uint16_t height, uint8_t mode)
{
  Serial.printf("ST7796_t3::init mode: %x\n", mode);
	commonInit(NULL, mode);
  if ((width == 320) && (height == 480)) {
    _colstart = 0;   
    _colstart2 = 0; 
    _rowstart = 0;   
    _rowstart2 = 0;
  } else {  // lets compute it.
    // added support for other sizes
    _rowstart = _rowstart2 = (int)((480 - height) / 2);
    _colstart = _colstart2 = (int)((320 - width) / 2);
  }
  
  _height = height;
  _width = width;
  _screenHeight = height;
  _screenWidth = width;   

  commandList(cmd_ST7796);
  setRotation(0);
  cursor_y  = cursor_x    = 0;
  textsize_x = textsize_y = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap      = true;
  font      = NULL;
  setClipRect();
  setOrigin();
  
}

