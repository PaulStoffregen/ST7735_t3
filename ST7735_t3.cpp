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

#include "ST7735_t3.h"
#include "ST7789_t3.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>


// Constructor when using software SPI.  All output pins are configurable.
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t sid, uint8_t sclk, uint8_t rst) :
 Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18)
{
	_cs   = cs;
	_rs   = rs;
	_sid  = sid;
	_sclk = sclk;
	_rst  = rst;
	hwSPI = false;
	_screenHeight = ST7735_TFTHEIGHT_18;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ST7735_t3::ST7735_t3(uint8_t cs, uint8_t rs, uint8_t rst) :
  Adafruit_GFX(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18) {
	_cs   = cs;
	_rs   = rs;
	_rst  = rst;
	hwSPI = true;
	_sid  = _sclk = (uint8_t)-1;
	_screenHeight = ST7735_TFTHEIGHT_18;
}



/***************************************************************/
/*     Teensy 3.0, 3.1, 3.2, 3.5, 3.6                          */
/***************************************************************/
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

inline void ST7735_t3::waitTransmitComplete(void)  {
    uint32_t tmp __attribute__((unused));
    while (!(_pkinetisk_spi->SR & SPI_SR_TCF)) ; // wait until final output done
    tmp = _pkinetisk_spi->POPR;                  // drain the final RX FIFO word
}

inline void ST7735_t3::waitTransmitComplete(uint32_t mcr) {
    uint32_t tmp __attribute__((unused));
    while (1) {
        uint32_t sr = _pkinetisk_spi->SR;
        if (sr & SPI_SR_EOQF) break;  // wait for last transmit
        if (sr &  0xF0) tmp = _pkinetisk_spi->POPR;
    }
    _pkinetisk_spi->SR = SPI_SR_EOQF;
    _pkinetisk_spi->MCR = mcr;
    while (_pkinetisk_spi->SR & 0xF0) {
        tmp = _pkinetisk_spi->POPR;
    }
}

inline void ST7735_t3::spiwrite(uint8_t c)
{
	for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*datapin = ((c & bit) ? 1 : 0);
		*clkpin = 1;
		*clkpin = 0;
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 0;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void ST7735_t3::writecommand_last(uint8_t c) {
	uint32_t mcr = _pkinetisk_spi->MCR;
	_pkinetisk_spi->PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
	waitTransmitComplete(mcr);
}

void ST7735_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(c);
		*cspin = 1;
	}
}

void ST7735_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1);
		while (((_pkinetisk_spi->SR) & (15 << 12)) > _fifo_full_test) ; // wait if FIFO full
	} else {
		*rspin = 1;
		*cspin = 0;
		spiwrite(d >> 8);
		spiwrite(d);
		*cspin = 1;
	}
}



void ST7735_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		uint32_t mcr = _pkinetisk_spi->MCR;
		_pkinetisk_spi->PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	} else {
		*rspin = 1;
		spiwrite(d >> 8);
		spiwrite(d);
	}
}


#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 24000000) {
		ctar = CTAR_24MHz;
	} else if (n >= 16000000) {
		ctar = CTAR_16MHz;
	} else if (n >= 12000000) {
		ctar = CTAR_12MHz;
	} else if (n >= 8000000) {
		ctar = CTAR_8MHz;
	} else if (n >= 6000000) {
		ctar = CTAR_6MHz;
	} else {
		ctar = CTAR_4MHz;
	}
	SIM_SCGC6 |= SIM_SCGC6_SPI0;
	_pkinetisk_spi->MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
	_pkinetisk_spi->CTAR0 = ctar | SPI_CTAR_FMSZ(7);
	_pkinetisk_spi->CTAR1 = ctar | SPI_CTAR_FMSZ(15);
	_pkinetisk_spi->MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
}


/***************************************************************/
/*     Teensy 4.                                               */
/***************************************************************/
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
inline void ST7735_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (_pspi) {
		_pspi->transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) DIRECT_WRITE_HIGH(_mosiport, _mosipinmask);
			else        DIRECT_WRITE_LOW(_mosiport, _mosipinmask);
			DIRECT_WRITE_HIGH(_sckport, _sckpinmask);
			asm("nop; nop; nop; nop; nop; nop; nop; nop; nop; nop; nop;");
			DIRECT_WRITE_LOW(_sckport, _sckpinmask);
		}
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}
}

void ST7735_t3::writecommand_last(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(0) | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		spiwrite(c);
	}

}

void ST7735_t3::writedata(uint8_t c)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));
		_pimxrt_spi->TDR = c;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(c);
	}
} 


void ST7735_t3::writedata16(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
		_pimxrt_spi->TDR = d;
		_pending_rx_count++;	//
		waitFifoNotFull();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

void ST7735_t3::writedata16_last(uint16_t d)
{
	if (hwSPI) {
		maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(15));
		_pimxrt_spi->TDR = d;
//		_pimxrt_spi->SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	} else {
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		spiwrite(d >> 8);
		spiwrite(d);
	}
} 

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}


/***************************************************************/
/*     Teensy LC                                               */
/***************************************************************/
#elif defined(__MKL26Z64__)


inline void ST7735_t3::spiwrite(uint8_t c)
{
//Serial.println(c, HEX);
	if (hwSPI) {
		SPI.transfer(c);
	} else if (hwSPI1) {
		SPI1.transfer(c);
	} else {
		// Fast SPI bitbang swiped from LPD8806 library
		for(uint8_t bit = 0x80; bit; bit >>= 1) {
			if(c & bit) *dataport |=  datapinmask;
			else        *dataport &= ~datapinmask;
			*clkport |=  clkpinmask;
			*clkport &= ~clkpinmask;
		}
	}
}

void ST7735_t3::writecommand(uint8_t c)
{
	*rsport &= ~rspinmask;
	spiwrite(c);
}
void ST7735_t3::writecommand_last(uint8_t c)
{
	*rsport &= ~rspinmask;
	spiwrite(c);
}

void ST7735_t3::writedata(uint8_t c)
{
	*rsport |=  rspinmask;
	spiwrite(c);
} 

void ST7735_t3::writedata16(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

void ST7735_t3::writedata16_last(uint16_t d)
{
	*rsport |=  rspinmask;
	spiwrite(d >> 8);
	spiwrite(d);
} 

void ST7735_t3::setBitrate(uint32_t n)
{
	if (n >= 8000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV2);
	} else if (n >= 4000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV4);
	} else if (n >= 2000000) {
		SPI.setClockDivider(SPI_CLOCK_DIV8);
	} else {
		SPI.setClockDivider(SPI_CLOCK_DIV16);
	}
}
#endif //#if defined(__SAM3X8E__)


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {         // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd2green144_offset[] = {         // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x03,             //     XSTART = 0
      0x00, 0x7F+0x03 },           //     XEND = 127

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ST7735_t3::commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	beginSPITransaction();
	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	//Serial.printf("CommandList: numCmds:%d\n", numCommands); Serial.flush();
	while(numCommands--) {				// For each command...
		writecommand_last(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs--) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
		}

		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			//Serial.printf("delay %d\n", ms); Serial.flush();
			endSPITransaction();
			delay(ms);
			beginSPITransaction();
		}
	}
	endSPITransaction();
}


// Initialization code common to both 'B' and 'R' type displays
void ST7735_t3::commonInit(const uint8_t *cmdList, uint8_t mode)
{
	_colstart  = _rowstart = 0; // May be overridden in init func
  	_ystart = _xstart = 0;

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;

	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk) && SPI.pinIsChipSelect(_rs)) {
		_pspi = &SPI;
		_pkinetisk_spi = &KINETISK_SPI0;  // Could hack our way to grab this from SPI object, but...
		_fifo_full_test = (3 << 12);
		//Serial.println("ST7735_t3::commonInit SPI");

    #if  defined(__MK64FX512__) || defined(__MK66FX1M0__)
	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk) && SPI1.pinIsChipSelect(_rs)) {
		_pspi = &SPI1;
		_pkinetisk_spi = &KINETISK_SPI1;
		_fifo_full_test = (0 << 12);
		//Serial.println("ST7735_t3::commonInit SPI1");
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk) && SPI2.pinIsChipSelect(_rs)) {
		_pspi = &SPI2;
		_pkinetisk_spi = &KINETISK_SPI2;
		_fifo_full_test = (0 << 12);
		//Serial.println("ST7735_t3::commonInit SPI2");
    #endif		
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->setMOSI(_sid);
		_pspi->setSCK(_sclk);
		_pspi->begin();
		//Serial.println("After SPI begin");
		// See if both CS and DC are valid CS pins.
		if (_pspi->pinIsChipSelect(_rs, _cs)) {
			pcs_data = _pspi->setCS(_cs);
			pcs_command = pcs_data | _pspi->setCS(_rs);
			cspin = 0; // Let know that we are not setting manual
			//Serial.println("Both CS and DC are SPI pins");
		} else {
			// We already verified that _rs was valid CS pin above.
			pcs_data = 0;
			pcs_command = pcs_data | _pspi->setCS(_rs);
			pinMode(_cs, OUTPUT);
			cspin = portOutputRegister(digitalPinToPort(_cs));
			*cspin = 1;
		}
	} else {
		hwSPI = false;
		cspin = portOutputRegister(digitalPinToPort(_cs));
		rspin = portOutputRegister(digitalPinToPort(_rs));
		clkpin = portOutputRegister(digitalPinToPort(_sclk));
		datapin = portOutputRegister(digitalPinToPort(_sid));
		*cspin = 1;
		*rspin = 0;
		*clkpin = 0;
		*datapin = 0;
		pinMode(_cs, OUTPUT);
		pinMode(_rs, OUTPUT);
		pinMode(_sclk, OUTPUT);
		pinMode(_sid, OUTPUT);
	}
	// Teensy 4
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	if (SPI.pinIsMOSI(_sid) && SPI.pinIsSCK(_sclk)) {
		_pspi = &SPI;
		_pimxrt_spi = &IMXRT_LPSPI4_S;  // Could hack our way to grab this from SPI object, but...

	} else if (SPI1.pinIsMOSI(_sid) && SPI1.pinIsSCK(_sclk)) {
		_pspi = &SPI1;
		_pimxrt_spi = &IMXRT_LPSPI3_S;
	} else if (SPI2.pinIsMOSI(_sid) && SPI2.pinIsSCK(_sclk)) {
		_pspi = &SPI2;
		_pimxrt_spi = &IMXRT_LPSPI1_S;
	} else _pspi = nullptr;

	if (_pspi) {
		hwSPI = true;
		_pspi->begin();
		_pending_rx_count = 0;
		_spiSettings = SPISettings(ST7735_SPICLOCK, MSBFIRST, mode);
		_pspi->beginTransaction(_spiSettings); // 4 MHz (half speed)
		_pspi->endTransaction();
		_spi_tcr_current = _pimxrt_spi->TCR; // get the current TCR value 
			// TODO:  Need to setup DC to actually work.
	
	} else {
		hwSPI = false;
		_sckport = portOutputRegister(_sclk);
		_sckpinmask = digitalPinToBitMask(_sclk);
		pinMode(_sclk, OUTPUT);	
		DIRECT_WRITE_LOW(_sckport, _sckpinmask);

		_mosiport = portOutputRegister(_sid);
		_mosipinmask = digitalPinToBitMask(_sid);
		pinMode(_sid, OUTPUT);	
		DIRECT_WRITE_LOW(_mosiport, _mosipinmask);

	}
	_csport = portOutputRegister(_cs);
	_cspinmask = digitalPinToBitMask(_cs);
	pinMode(_cs, OUTPUT);	
	DIRECT_WRITE_HIGH(_csport, _cspinmask);
	if (_pspi && _pspi->pinIsChipSelect(_rs)) {
	 	_pspi->setCS(_rs);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	} else {
		//Serial.println("ILI9341_t3n: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_rs);
		_dcpinmask = digitalPinToBitMask(_rs);
		pinMode(_rs, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
	}
	maybeUpdateTCR(LPSPI_TCR_PCS(1) | LPSPI_TCR_FRAMESZ(7));

    // Teensy LC
#elif defined(__MKL26Z64__)
	hwSPI1 = false;
	if (_sid == (uint8_t)-1) _sid = 11;
	if (_sclk == (uint8_t)-1) _sclk = 13;
	
	// See if pins are on standard SPI0
	if ((_sid == 7 || _sid == 11) && (_sclk == 13 || _sclk == 14)) {
		hwSPI = true;
	} else {
		hwSPI = false;
		if ((_sid == 0 || _sid == 21) && (_sclk == 20 )) {
			hwSPI1 = true;
		}
	}
 
	pinMode(_rs, OUTPUT);
	pinMode(_cs, OUTPUT);
	csport    = portOutputRegister(digitalPinToPort(_cs));
	rsport    = portOutputRegister(digitalPinToPort(_rs));
	cspinmask = digitalPinToBitMask(_cs);
	rspinmask = digitalPinToBitMask(_rs);

	if(hwSPI) { // Using hardware SPI
		if (_sclk == 14) SPI.setSCK(14);
		if (_sid == 7) SPI.setMOSI(7);
		SPI.begin();
		SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI.setBitOrder(MSBFIRST);
		SPI.setDataMode(SPI_MODE0);
	} else if(hwSPI1) { // Using hardware SPI
		SPI1.setSCK(_sclk);
		SPI1.setMOSI(_sid);
		SPI1.begin();
		SPI1.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz (half speed)
		SPI1.setBitOrder(MSBFIRST);
		SPI1.setDataMode(SPI_MODE0);
	} else {
		pinMode(_sclk, OUTPUT);
		pinMode(_sid , OUTPUT);
		clkport     = portOutputRegister(digitalPinToPort(_sclk));
		dataport    = portOutputRegister(digitalPinToPort(_sid));
		clkpinmask  = digitalPinToBitMask(_sclk);
		datapinmask = digitalPinToBitMask(_sid);
		*clkport   &= ~clkpinmask;
		*dataport  &= ~datapinmask;
	}
	// toggle RST low to reset; CS low so it'll listen to us
	*csport &= ~cspinmask;

#endif
	// BUGBUG
//	digitalWrite(_cs, LOW);
	if (_rst) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(500);
		digitalWrite(_rst, LOW);
		delay(500);
		digitalWrite(_rst, HIGH);
		delay(500);
	}

	if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void ST7735_t3::initB(void)
{
	commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void ST7735_t3::initR(uint8_t options)
{
	commonInit(Rcmd1);
	if (options == INITR_GREENTAB) {
		commandList(Rcmd2green);
		_colstart = 2;
		_rowstart = 1;
	} else if(options == INITR_144GREENTAB) {
		_screenHeight = ST7735_TFTHEIGHT_144;
		commandList(Rcmd2green144);
		_colstart = 0;
		_rowstart = 32;
	} else if(options == INITR_144GREENTAB_OFFSET) {
		_screenHeight = ST7735_TFTHEIGHT_144;
		//commandList(Rcmd2green144_offset);
		commandList(Rcmd2green144);
		_colstart = 2;
		_rowstart = 3;
	} else {
		// _colstart, _rowstart left at default '0' values
		commandList(Rcmd2red);
	}
	commandList(Rcmd3);

	// if black, change MADCTL color filter
	if (options == INITR_BLACKTAB) {
		writecommand(ST7735_MADCTL);
		writedata(0xC0);
	}

	tabcolor = options;
	setRotation(0);
}


void ST7735_t3::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	beginSPITransaction();
	setAddr(x0, y0, x1, y1);
	writecommand(ST7735_RAMWR); // write to RAM
	endSPITransaction();
}


void ST7735_t3::pushColor(uint16_t color)
{
	beginSPITransaction();
	writedata16_last(color);
	endSPITransaction();
}

void ST7735_t3::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	beginSPITransaction();
	setAddr(x,y,x+1,y+1);
	writecommand(ST7735_RAMWR);
	writedata16_last(color);
	endSPITransaction();
}


void ST7735_t3::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height) h = _height-y;
	beginSPITransaction();
	setAddr(x, y, x, y+h-1);
	writecommand(ST7735_RAMWR);
	while (h-- > 1) {
		writedata16(color);
	}
	writedata16_last(color);
	endSPITransaction();
}


void ST7735_t3::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)  w = _width-x;
	beginSPITransaction();
	setAddr(x, y, x+w-1, y);
	writecommand(ST7735_RAMWR);
	while (w-- > 1) {
		writedata16(color);
	}
	writedata16_last(color);
	endSPITransaction();
}



void ST7735_t3::fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void ST7735_t3::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height)) return;
	if ((x + w - 1) >= _width)  w = _width  - x;
	if ((y + h - 1) >= _height) h = _height - y;
	beginSPITransaction();
	setAddr(x, y, x+w-1, y+h-1);
	writecommand(ST7735_RAMWR);
	for (y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16(color);
		}
		writedata16_last(color);
	}
	endSPITransaction();
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ST7735_t3::setRotation(uint8_t m)
{
	beginSPITransaction();
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = _screenHeight;
	    _xstart = _colstart;
	    _ystart = _rowstart;
		break;
	case 1:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		}
		_height = ST7735_TFTWIDTH;
		_width = _screenHeight;
     	_ystart = _colstart;
     	_xstart = _rowstart;
		break;
	case 2:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_RGB);
		} else {
			writedata(MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = _screenHeight;
     	_xstart = _colstart;
     	// hack to make work on a couple different displays
     	_ystart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	case 3:
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
		} else {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		_width = _screenHeight;
		_height = ST7735_TFTWIDTH;
     	_ystart = _colstart;
     	// hack to make work on a couple different displays
     	_xstart = (_rowstart==0 || _rowstart==32)? 0 : 1;//_rowstart;
		break;
	}
	_rot = rotation;	// remember the rotation... 
	//Serial.printf("SetRotation(%d) _xstart=%d _ystart=%d _width=%d, _height=%d\n", _rot, _xstart, _ystart, _width, _height);
	endSPITransaction();
}

void ST7735_t3::setRowColStart(uint8_t x, uint8_t y) {
	_rowstart = x;
	_colstart = y;
	if (_rot != 0xff) setRotation(_rot);
}


void ST7735_t3::invertDisplay(boolean i)
{
	beginSPITransaction();
	writecommand_last(i ? ST7735_INVON : ST7735_INVOFF);
	endSPITransaction();

}

/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and const data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
void ST7735_t3::sendCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    beginSPITransaction();

    writecommand_last(commandByte); // Send the command byte
  
    for (int i=0; i<numDataBytes; i++) {
	  writedata(*dataBytes); // Send the data bytes
    }
  
    endSPITransaction();
}