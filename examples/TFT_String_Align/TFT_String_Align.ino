/*
Tests string alignment

Normally strings are printed relative to the top left corner but this can be
changed with the setTextDatum() function. The library has #defines for:

TL_DATUM = Top left
TC_DATUM = Top centre
TR_DATUM = Top right
ML_DATUM = Middle left
MC_DATUM = Middle centre
MR_DATUM = Middle right
BL_DATUM = Bottom left
BC_DATUM = Bottom centre
BR_DATUM = Bottom right
*/
#include <SPI.h>
#include <ST7735_t3.h> // Hardware-specific library
#include <ST7789_t3.h> // Hardware-specific library
#include <st7735_t3_font_Arial.h>


#define TFT_MISO  12
#define TFT_MOSI  11  //a12
#define TFT_SCK   13  //a13
#define TFT_DC   9 
#define TFT_CS   10  
#define TFT_RST  8

ST7735_t3 tft = ST7735_t3(TFT_CS, TFT_DC, TFT_RST);

unsigned long drawTime = 0;

void setup(void) {
  Serial.begin(115200);
  Serial.print("hello!");
  
  tft.initR(INITR_144GREENTAB);
  tft.setRotation(3);
  tft.setFont(Arial_10);
  //tft.setTextSize(4);
}

void loop() {

  tft.fillScreen(ST7735_BLACK);
  
  for(byte datum = 0; datum < 9; datum++) {
    tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
    
    tft.setTextDatum(datum);
    
    tft.drawNumber(88,64,50);
    tft.fillCircle(160,120,5,ST7735_RED);
    
    tft.setTextDatum(datum);
    
    tft.setTextColor(ST7735_YELLOW);
    tft.drawString("TEENSY 4",3,64,70);
    delay(1000);
    tft.fillScreen(ST7735_BLACK);
  }

  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ST7735_BLACK);
  tft.drawString("X",64,70);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);
  
  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ST7735_BLACK);
  tft.drawString("X",64,70);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);

  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);

  tft.setTextDatum(MC_DATUM);

  //Test floating point drawing function
  float test = 67.125;
  tft.drawFloat(test, 4,64,70);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);
  test = -0.555555;
  tft.drawFloat(test, 3, 64,70);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);
  test = 0.1;
  tft.drawFloat(test, 4, 64,70);
  delay(1000);
  tft.fillScreen(ST7735_BLACK);
  test = 9999999;
  tft.drawFloat(test, 1, 64,70);
  delay(1000);
  
  tft.fillCircle(160,180,5,ST7735_YELLOW);
  
  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ST7735_BLACK);
  tft.drawString("X",160,180);

  delay(4000);
}