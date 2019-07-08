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

#ifndef __ST7735_t3_H_
#define __ST7735_t3_H_

#include "Arduino.h"
#include <SPI.h>
#include <Adafruit_GFX.h>


#define ST7735_SPICLOCK 24000000

// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB 0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1
#define INITR_144GREENTAB_OFFSET   0x4

#define ST7735_TFTWIDTH  128
// for 1.44" display
#define ST7735_TFTHEIGHT_144 128
// for 1.8" display
#define ST7735_TFTHEIGHT_18  160

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

// Also define them in a non specific ST77XX specific name
#define ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define ST77XX_RED        0xF800
#define ST77XX_GREEN      0x07E0
#define ST77XX_BLUE       0x001F
#define ST77XX_CYAN       0x07FF
#define ST77XX_MAGENTA    0xF81F
#define ST77XX_YELLOW     0xFFE0
#define ST77XX_ORANGE     0xFC00



class ST7735_t3 : public Adafruit_GFX {

 public:

  ST7735_t3(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST = -1);
  ST7735_t3(uint8_t CS, uint8_t RS, uint8_t RST = -1);

  void     initB(void),                             // for ST7735B displays
           initR(uint8_t options = INITR_GREENTAB), // for ST7735R
           setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  virtual void setRotation(uint8_t r);
  void     invertDisplay(boolean i);
  void     setRowColStart(uint8_t x, uint8_t y);
  uint8_t  rowStart() {return _rowstart;}
  uint8_t  colStart() {return _colstart;}

  void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
    __attribute__((always_inline)) {
        writecommand(ST7735_CASET); // Column addr set
        writedata16(x0+_xstart);   // XSTART 
        writedata16(x1+_xstart);   // XEND
        writecommand(ST7735_RASET); // Row addr set
        writedata16(y0+_ystart);   // YSTART
        writedata16(y1+_ystart);   // YEND
  }

  void sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes);



  // Pass 8-bit (each) R,G,B, get back 16-bit packed color
  inline uint16_t Color565(uint8_t r, uint8_t g, uint8_t b) {
           return ((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3);
  }
  void setBitrate(uint32_t n);

  /* These are not for current use, 8-bit protocol only!
  uint8_t  readdata(void),
           readcommand8(uint8_t);
  uint16_t readcommand16(uint8_t);
  uint32_t readcommand32(uint8_t);
  void     dummyclock(void);
  */
  // Useful methods added from ili9341_t3 
  void writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors);

 protected:
  uint8_t  tabcolor;

  void     spiwrite(uint8_t),
           writecommand(uint8_t c),
           writecommand_last(uint8_t c),
           writedata(uint8_t d),
           writedata16(uint16_t d),
           writedata16_last(uint16_t d),
           commandList(const uint8_t *addr),
           commonInit(const uint8_t *cmdList, uint8_t mode=SPI_MODE0);
//uint8_t  spiread(void);

  boolean  hwSPI;


  uint8_t _colstart, _rowstart, _xstart, _ystart, _rot, _screenHeight;
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
  uint8_t  _cs, _rs, _rst, _sid, _sclk;
  uint8_t pcs_data, pcs_command;
  uint32_t ctar;
  volatile uint8_t *datapin, *clkpin, *cspin, *rspin;

  SPIClass *_pspi = nullptr;
  KINETISK_SPI_t *_pkinetisk_spi;
  void waitTransmitComplete(void);
  void waitTransmitComplete(uint32_t mcr);
  uint32_t _fifo_full_test;

  inline void beginSPITransaction() {
    if (_pspi) _pspi->beginTransaction(SPISettings(ST7735_SPICLOCK, MSBFIRST, SPI_MODE0));
    if (cspin) *cspin = 0;
  }

  inline void endSPITransaction()
  {
    if (cspin) *cspin = 1;
    if (_pspi) _pspi->endTransaction();  
  }


#endif
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
  SPIClass *_pspi = nullptr;
  SPISettings _spiSettings;
  IMXRT_LPSPI_t *_pimxrt_spi = nullptr;
  uint8_t _pending_rx_count = 0;
  uint32_t _spi_tcr_current;


  void DIRECT_WRITE_LOW(volatile uint32_t * base, uint32_t mask)  __attribute__((always_inline)) {
    *(base+34) = mask;
  }
  void DIRECT_WRITE_HIGH(volatile uint32_t * base, uint32_t mask)  __attribute__((always_inline)) {
    *(base+33) = mask;
  }
  
  #define TCR_MASK  (LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK )

  void maybeUpdateTCR(uint32_t requested_tcr_state) {
  if ((_spi_tcr_current & TCR_MASK) != requested_tcr_state) {
      bool dc_state_change = (_spi_tcr_current & LPSPI_TCR_PCS(3)) != (requested_tcr_state & LPSPI_TCR_PCS(3));
      _spi_tcr_current = (_spi_tcr_current & ~TCR_MASK) | requested_tcr_state ;
      // only output when Transfer queue is empty.
      if (!dc_state_change || !_dcpinmask) {
        while ((_pimxrt_spi->FSR & 0x1f) )  ;
        _pimxrt_spi->TCR = _spi_tcr_current;  // update the TCR

      } else {
        waitTransmitComplete();
        if (requested_tcr_state & LPSPI_TCR_PCS(3)) DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
        else DIRECT_WRITE_LOW(_dcport, _dcpinmask);
        _pimxrt_spi->TCR = _spi_tcr_current & ~(LPSPI_TCR_PCS(3) | LPSPI_TCR_CONT); // go ahead and update TCR anyway?  

      }
    }
  }

  inline void beginSPITransaction() {
    if (hwSPI) _pspi->beginTransaction(_spiSettings);
    if (_cs != 0xff)DIRECT_WRITE_LOW(_csport, _cspinmask);
  }

  inline void endSPITransaction() {
    DIRECT_WRITE_HIGH(_csport, _cspinmask);
    if (hwSPI) _pspi->endTransaction();  
  }

 
  void waitFifoNotFull(void) {
    uint32_t tmp __attribute__((unused));
    do {
        if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)  {
            tmp = _pimxrt_spi->RDR;  // Read any pending RX bytes in
            if (_pending_rx_count) _pending_rx_count--; //decrement count of bytes still levt
        }
    } while ((_pimxrt_spi->SR & LPSPI_SR_TDF) == 0) ;
 }
 void waitTransmitComplete(void)  {
    uint32_t tmp __attribute__((unused));
//    digitalWriteFast(2, HIGH);

    while (_pending_rx_count) {
        if ((_pimxrt_spi->RSR & LPSPI_RSR_RXEMPTY) == 0)  {
            tmp = _pimxrt_spi->RDR;  // Read any pending RX bytes in
            _pending_rx_count--; //decrement count of bytes still levt
        }
    }
    _pimxrt_spi->CR = LPSPI_CR_MEN | LPSPI_CR_RRF;       // Clear RX FIFO
//    digitalWriteFast(2, LOW);
}


  uint8_t  _cs, _rs, _rst, _sid, _sclk;

  uint32_t _cspinmask;
  volatile uint32_t *_csport;
  uint32_t _dcpinmask;
  volatile uint32_t *_dcport;
  uint32_t _mosipinmask;
  volatile uint32_t *_mosiport;
  uint32_t _sckpinmask;
  volatile uint32_t *_sckport;
  

  uint32_t ctar;
#endif
#if defined(__MKL26Z64__)
volatile uint8_t *dataport, *clkport, *csport, *rsport;
  uint8_t  _cs, _rs, _rst, _sid, _sclk,
           datapinmask, clkpinmask, cspinmask, rspinmask;
  boolean  hwSPI1;
  inline void beginSPITransaction() {
    if (hwSPI) SPI.beginTransaction(SPISettings(ST7735_SPICLOCK, MSBFIRST, SPI_MODE0));
    else if (hwSPI1) SPI1.beginTransaction(SPISettings(ST7735_SPICLOCK, MSBFIRST, SPI_MODE0));
    *csport &= ~cspinmask;
  }

  inline void ST7735_t3::endSPITransaction() {
    *csport |= cspinmask;
    if (hwSPI) SPI.endTransaction(); 
    else if (hwSPI1)  SPI1.endTransaction();  
  }
#endif 

};

#endif
