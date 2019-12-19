This library supports ST7735 and ST7789 with and without a CS pin, such as https://www.amazon.com/gp/product/B07P9X3L7M/?pldnSite=1 which is a ST7789 240x240 display.

To initialize an instance of the display the user has a choice of constructors:
```c++
	// This Teensy3 and 4 native optimized and extended version
	// requires specific pins. 
	// If you use the short version of the constructor and the DC
	// pin is hardware CS pin, then it will be slower.

	#define TFT_MISO  12
	#define TFT_MOSI  11  //a12
	#define TFT_SCK   13  //a13
	#define TFT_DC   9 
	#define TFT_CS   10  
	#define TFT_RST  8

	// Note the above pins are for the SPI object.  For those Teensy boards which have
	// more than one SPI object, such as T3.5, T3.6, T4 which have at SPI1 and SPI2
	// LC with SPI1, look at the cards that come with the teensy or the web page
	// https://www.pjrc.com/teensy/pinout.html to select the appropriate IO pins.

	#include <ST7735_t3.h> // Hardware-specific library
	#include <ST7789_t3.h> // Hardware-specific library
	#include <SPI.h>

	// Option 1: use any pins but a little slower
	// Note: code will detect if specified pins are the hardware SPI pins
	//       and will use hardware SPI if appropriate
	// For 1.44" and 1.8" TFT with ST7735 use
	ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

	// For 1.54" or other TFT with ST7789, This has worked with some ST7789
	// displays without CS pins, for those you can pass in -1 or 0xff for CS
	// More notes by the tft.init call
	//ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

	// Option 2: must use the hardware SPI pins
	// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
	// an output. This is much faster - also required if you want
	// to use the microSD card (see the image drawing example)
	// For 1.44" and 1.8" TFT with ST7735 use
	//ST7735_t3 tft = ST7735_t3(cs, dc, rst);

	// For 1.54" TFT with ST7789
	//ST7789_t3 tft = ST7789_t3(TFT_CS,  TFT_DC, TFT_RST);
```

NOTE: If the Teensy has more than one SPI buss. And the IO pins are all on a different SPI buss then that buss will be used. (i.e. you can use SPI1 or SPI2).  With this, on a board such as a T4 or T3.5 or T3.6 you can potentially have three displays all on different SPI busses and using the Async updates you can have all three of them updating their display at the same time. 

For the case of a ST7789 the CS pin would be identied with a -1.  This constructor is used for those cases with the display on SPI and no CS pin:

```c++
	//ST7789_t3 tft = ST7789_t3(TFT_CS,  TFT_DC, TFT_RST);
```

The library supports most if not all display resolutions:

```c++ 
  // Use this initializer if you're using a 1.8" TFT 128x160 displays
  //tft.initR(INITR_BLACKTAB);

  // Or use this initializer (uncomment) if you're using a 1.44" TFT (128x128)
  //tft.initR(INITR_144GREENTAB);

  // Or use this initializer (uncomment) if you're using a .96" TFT(160x80)
  //tft.initR(INITR_MINI160x80);

  // Or use this initializer (uncomment) for Some 1.44" displays use different memory offsets
  // Try it if yours is not working properly
  // May need to tweek the offsets
  //tft.setRowColStart(32,0);

  // Or use this initializer (uncomment) if you're using a 1.54" 240x240 TFT
  //tft.init(240, 240);   // initialize a ST7789 chip, 240x240 pixels

  // OR use this initializer (uncomment) if using a 2.0" 320x240 TFT:
  tft.init(240, 320);           // Init ST7789 320x240

  // OR use this initializer (uncomment) if using a 240x240 clone 
  // that does not have a CS pin2.0" 320x240 TFT:
  //tft.init(240, 240, SPI_MODE2);           // Init ST7789 240x240 no CS
```


Frame Buffer
------------

The teensy 4.x, 3.6 and 3.5 have a lot more memory than previous Teensy processors, so on these boards, we borrowed some ideas from the ILI9341_t3DMA library and added code to be able to use a logical Frame Buffer.  

Since the smaller ST7735 and maybe ST7789 displays have fewer pixels, you can on some of them enable a frame buffer on a T3.2 as well. I believe in this case I did add support for Async updates as well. 

To enable this we added a couple of API's 

```c++
    uint8_t useFrameBuffer(boolean b) - if b non-zero it will allocate memory and start using
    void	freeFrameBuffer(void) - Will free up the memory that was used.
    void	updateScreen(void); - Will update the screen with all of your updates...
	void	setFrameBuffer(uint16_t *frame_buffer); - Now have the ability allocate the frame buffer and pass it in, to avoid use of malloc
```

Asynchronous Update support (Frame buffer)
------------------------

The code now has support to use DMA for Asynchronous updates of the screen.  You can choose to do the updates once or in continuous mode.  Note: I mainly use the oneshot as we prefer more control on when the screen updates which helps to minimize things like flashing and tearing. 
Some of the New methods for this include: 

```c++
	bool	updateScreenAsync(bool update_cont = false); - Starts an update either one shot or continuous
	void	waitUpdateAsyncComplete(void);  - Wait for any active update to complete
	void	endUpdateAsync();			 - Turn of the continuous mode.
	boolean	asyncUpdateActive(void)      - Lets you know if an async operation is still active
```

Additional APIs
---------------
In addition, this library now has some of the API's and functionality that has been requested in a pull request.  In particular it now supports, the ability to set a clipping rectangle as well as setting an origin that is used with the drawing primitives.   These new API's include:

```c++
	void setOrigin(int16_t x = 0, int16_t y = 0); 
	void getOrigin(int16_t* x, int16_t* y);
	void setClipRect(int16_t x1, int16_t y1, int16_t w, int16_t h); 
	void setClipRect();
```

It also incorporates functionality from the TFT_ILI9341_ESP, https://github.com/Bodmer/TFT_ILI9341_ESP, for additional functions:

```c++
    int16_t  drawNumber(long long_num,int poX, int poY);
    int16_t  drawFloat(float floatNumber,int decimal,int poX, int poY);   
    int16_t drawString(const String& string, int poX, int poY);
    int16_t drawString1(char string[], int16_t len, int poX, int poY);
    void setTextDatum(uint8_t datum);
```

In addition, scrolling text has been added using appropriate function from, https://github.com/vitormhenrique/ILI9341_t3:

```c++
    void enableScroll(void);
    void resetScrollBackgroundColor(uint16_t color);
    void setScrollTextArea(int16_t x, int16_t y, int16_t w, int16_t h);
    void setScrollBackgroundColor(uint16_t color);
    void scrollTextArea(uint8_t scrollSize);
    void resetScrollBackgroundColor(uint16_t color);
```
	
Font Support
------------
This library tries to support three different font types.  This includes the original font support that is in the ILI9341_t3 library, which is 
built in system font, as well as the new font format. 

In addition, we added support to use the Adafruit GFX fonts as well. This includes the ability to output the text in Opaque mode. 

Discussion regarding this optimized version:
==========================

https://forum.pjrc.com/threads/57015-ST7789_t3-(part-of-ST7735-library)-support-for-displays-without-CS-pin?highlight=st7735


Adafruit library info
=======================

But as this code is based of of their work, their original information is included below:

------------------------------------------

This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618
 
Check out the links above for our tutorials and wiring diagrams.
These displays use SPI to communicate, 4 or 5 pins are required
to interface (RST is optional).
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
MIT license, all text above must be included in any redistribution

To download. click the DOWNLOADS button in the top right corner, rename the uncompressed folder Adafruit_ST7735. Check that the Adafruit_ST7735 folder contains Adafruit_ST7735.cpp and Adafruit_ST7735.

Place the Adafruit_ST7735 library folder your <arduinosketchfolder>/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE

Also requires the Adafruit_GFX library for Arduino.