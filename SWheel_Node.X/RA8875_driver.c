/**
 * RA8875 Driver
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "RA8875_driver.h"
#include "SWheel.h"

void initialize(void) {
  PLLinit();
  writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  /* Timing values */
  uint8_t pixclk;
  uint8_t hsync_start;
  uint8_t hsync_pw;
  uint8_t hsync_finetune;
  uint8_t hsync_nondisp;
  uint8_t vsync_pw;
  uint16_t vsync_nondisp;
  uint16_t vsync_start;

  pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
  hsync_nondisp = 10;
  hsync_start = 8;
  hsync_pw = 48;
  hsync_finetune = 0;
  vsync_nondisp = 3;
  vsync_start = 8;
  vsync_pw = 10;

  writeReg(RA8875_PCSR, pixclk);
  delay(1);

  /* Horizontal settings registers */
  writeReg(RA8875_HDWR, (WIDTH / 8) - 1); // H width: (HDWR + 1) * 8 = 480
  writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) / 8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  writeReg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
  writeReg(RA8875_HPWR, RA8875_HPWR_LOW + (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

  /* Vertical settings registers */
  writeReg(RA8875_VDHR0, (uint16_t) (HEIGHT - 1) & 0xFF);
  writeReg(RA8875_VDHR1, (uint16_t) (HEIGHT - 1) >> 8);
  writeReg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
  writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
  writeReg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
  writeReg(RA8875_VSTR1, vsync_start >> 8);
  writeReg(RA8875_VPWR, RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

  /* Set active window X */
  writeReg(RA8875_HSAW0, 0); // horizontal start point
  writeReg(RA8875_HSAW1, 0);
  writeReg(RA8875_HEAW0, (uint16_t) (WIDTH - 1) & 0xFF); // horizontal end point
  writeReg(RA8875_HEAW1, (uint16_t) (WIDTH - 1) >> 8);

  /* Set active window Y */
  writeReg(RA8875_VSAW0, 0); // vertical start point
  writeReg(RA8875_VSAW1, 0);
  writeReg(RA8875_VEAW0, (uint16_t) (HEIGHT - 1) & 0xFF); // horizontal end point
  writeReg(RA8875_VEAW1, (uint16_t) (HEIGHT - 1) >> 8);

  /* Clear the entire window */
  writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  delay(500);
}

void PLLinit(void) {
  writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
  delay(1);
  writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
  delay(1);
}

void writeReg(uint8_t reg, uint8_t val) {
  writeCommand(reg);
  writeData(val);
}

void SPI_double_send(uint8_t one, uint8_t two) {
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LCD_CS_LAT = 0;
  SPI1BUF = one;
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  SPI1BUF = two;
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LCD_CS_LAT = 1;
}

void writeData(uint8_t d) {
  SPI_double_send(RA8875_DATAWRITE, d);
}

void writeCommand(uint8_t d) {
  SPI_double_send(RA8875_CMDWRITE, d);
}

void displayOn(uint8_t on) {
  if(on) {
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  } else {
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
  }
}

void GPIOX(uint8_t on) {
  if(on) {
    writeReg(RA8875_GPIOX, 1);
  } else {
    writeReg(RA8875_GPIOX, 0);
  }
}

void rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled) {
  /* Set X */
  writeCommand(0x91);
  writeData(x);
  writeCommand(0x92);
  writeData(x >> 8);

  /* Set Y */
  writeCommand(0x93);
  writeData(y);
  writeCommand(0x94);
  writeData(y >> 8);

  /* Set X1 */
  writeCommand(0x95);
  writeData(w);
  writeCommand(0x96);
  writeData((w) >> 8);

  /* Set Y1 */
  writeCommand(0x97);
  writeData(h);
  writeCommand(0x98);
  writeData((h) >> 8);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if(filled) {
    writeData(0xB0);
  } else {
    writeData(0x90);
  }

  /* Wait for the command to finish */
  //waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
  delay(100);
}

void PWM1config(uint8_t on, uint8_t clock) {
  if(on) {
    writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

void PWM1out(uint8_t p) {
  writeReg(RA8875_P1DCR, p);
}

void graphicsMode(void) {
  writeCommand(RA8875_MWCR0);
  //uint8_t temp = readData();
  //temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  //writeData(temp);
  writeData(0x00);
}

void fillScreen(uint16_t color) {
  rectHelper(0, 0, 480 - 1, 272 - 1, color, 1);
}

void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  circleHelper(x0, y0, r, color, 1);
}

void circleHelper(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint8_t filled) {
  /* Set X */
  writeCommand(0x99);
  writeData(x0);
  writeCommand(0x9a);
  writeData(x0 >> 8);

  /* Set Y */
  writeCommand(0x9b);
  writeData(y0);
  writeCommand(0x9c);
  writeData(y0 >> 8);

  /* Set Radius */
  writeCommand(0x9d);
  writeData(r);

  /* Set Color */
  writeCommand(0x63);
  writeData((color & 0xf800) >> 11);
  writeCommand(0x64);
  writeData((color & 0x07e0) >> 5);
  writeCommand(0x65);
  writeData((color & 0x001f));

  /* Draw! */
  writeCommand(RA8875_DCR);
  if(filled) {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  //waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
  delay(100);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  rectHelper(x, y, x + w, y + h, color, 1);
}