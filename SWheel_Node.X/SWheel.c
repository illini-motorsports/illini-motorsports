/**
 * SWheel
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "SWheel.h"

// Count number of milliseconds since start of code execution
volatile uint32_t millis = 0;

/**
 * Main function
 */

uint8_t readdData() {
  /*
  while (SPI1STATbits.SPIBUSY);// Wait for idle SPI module
  SPI1CONbits.ON = 0;
  SPI1CONbits.MODE16 = 1;// 32/16-Bit Communication Select bits (16-bit)
  SPI1CONbits.ON = 1;
   */
  while (SPI1STATbits.SPIBUSY);// Wait for idle SPI module
  LCD_CS_LAT = 0;
  SPI1BUF = RA8875_DATAREAD;
  while (SPI1STATbits.SPIBUSY);// Wait for idle SPI module
  uint8_t readVal = SPI1BUF;
  //SPI1BUF = 0b11000000;
  //while (SPI1STATbits.SPIBUSY);// Wait for idle SPI module
  SPI1BUF = 0x00;
  while (SPI1STATbits.SPIBUSY /*&!SPI1STATbits.SPIRBF*/);// Wait for idle SPI module
  readVal = SPI1BUF;
  LCD_CS_LAT = 1;
  //LATBbits.LATB7 = SPI1STATbits.SPIRBF;
  //while (SPI1STATbits.SPIBUSY);// Wait for idle SPI module
  /*
  if (SPI1STATbits.SPIRBF) {
    LATBbits.LATB7 = 0;
    readVal = SPI1BUF;
    //readVal = 0x01;
  }
  if (SPI1STATbits.SPIRBF) {
    LATBbits.LATB7 = 1;
  }
  SPI1CONbits.ON = 0;
  SPI1CONbits.MODE16 = 0;// 32/16-Bit Communication Select bits (16-bit)
  SPI1CONbits.ON = 1;
   */

  return readVal;
}

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_spi();// Initialize SPI interface
  STI();// Enable interrupts

  LCD_CS_TRIS = OUTPUT;
  LCD_CS_LAT = 1;
  LCD_RST_TRIS = OUTPUT;
  TRISBbits.TRISB6 = OUTPUT;
  TRISBbits.TRISB7 = OUTPUT;
  LATBbits.LATB6 = 1;
  LATBbits.LATB7 = 1;

  reset();
  initialize();

  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX
  PWM1config(1, RA8875_PWM_CLK_DIV1024);// PWM output for backlight
  PWM1out(255);

  fillScreen(RA8875_RED);
  //fillCircle(360, 90, 50, RA8875_BLUE);
  //fillCircle(360, 183, 50, RA8875_BLUE);
  //fillRect(50, 99, 300, 75, RA8875_BLUE);
  //fillTriangle(20, 20, 300, 20, 20, 300, RA8875_GREEN);
  //fillEllipse(150, 150, 150, 75, RA8875_CYAN);
  drawChevron(100, 10, 200, 150, 3);


  while (1) {
    writeCommand(RA8875_VSTR0);
    uint8_t readVal = readdData();
    if (readVal) {
      LATBbits.LATB6 = 0;
    }
  }
}

void drawChevron(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t thick) {
  int mid = (w / 2) + x;
  int i = y;
  for (;i < y + h;i += 15) {
    fillTriangle(x, i, mid, 100 + i, mid, 100 + i + thick, RA8875_CYAN);
    fillTriangle(x, i, x, i + thick, mid, 100 + i + thick, RA8875_CYAN);
    fillTriangle(x + w, i, x + w, i + thick, mid, 100 + i, RA8875_CYAN);
    fillTriangle(mid, 100 + i, mid, 100 + i + thick, x + w, i + thick, RA8875_CYAN);
    /*
    drawLine(50, i, 150, 100 + i, RA8875_CYAN);
    drawLine(50, i + 1, 150, 100 + i + 1, RA8875_CYAN);
    drawLine(50, i + 2, 150, 100 + i + 2, RA8875_CYAN);
    drawLine(150, 100 + i, 250, i, RA8875_CYAN);
    drawLine(150, 100 + i + 1, 250, i + 1, RA8875_CYAN);
    drawLine(150, 100 + i + 2, 250, i + 2, RA8875_CYAN);
     */
  }
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {

  millis++;// Increment millis count
  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

void delay(uint32_t num) {
  uint32_t start = millis;
  while (millis - start < num);
}