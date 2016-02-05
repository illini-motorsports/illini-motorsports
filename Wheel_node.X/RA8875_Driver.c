/* 
 * File: RA8875_Driver.c
 * Author: Jake Leonard
 * Comments: Ported from the Adafruit arduino library
 */

#include "RA8875_Driver.h"

// Initialize Screen
void lcd_init(){
  // init PLL
  writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 10);
  delay(1);
  writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
  delay(1);
  writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  // Set env variables
  uint8_t pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_4CLK;
  uint8_t hsync_start = 8;
  uint8_t hsync_pw = 48;
  uint8_t hsync_finetune = 0;
  uint8_t hsync_nondisp = 10;
  uint8_t vsync_pw = 10;
  uint16_t vsync_nondisp = 3;
  uint16_t vsync_start = 8;
  uint16_t _width = 480;
  uint16_t _height = 272;
  writeReg(RA8875_PCSR, pixclk);
  delay(1);
  
  /* Horizontal settings registers */
  writeReg(RA8875_HDWR, (_width / 8) - 1);                          // H width: (HDWR + 1) * 8 = 480
  writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
  writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2)/8);    // H non-display: HNDR * 8 + HNDFTR + 2 = 10
  writeReg(RA8875_HSTR, hsync_start/8 - 1);                         // Hsync start: (HSTR + 1)*8 
  writeReg(RA8875_HPWR, RA8875_HPWR_LOW + (hsync_pw/8 - 1));        // HSync pulse width = (HPWR+1) * 8
  
  /* Vertical settings registers */
  writeReg(RA8875_VDHR0, (uint16_t)(_height - 1) & 0xFF);
  writeReg(RA8875_VDHR1, (uint16_t)(_height - 1) >> 8);
  writeReg(RA8875_VNDR0, vsync_nondisp-1);                          // V non-display period = VNDR + 1
  writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
  writeReg(RA8875_VSTR0, vsync_start-1);                            // Vsync start position = VSTR + 1
  writeReg(RA8875_VSTR1, vsync_start >> 8);
  writeReg(RA8875_VPWR, RA8875_VPWR_LOW + vsync_pw - 1);            // Vsync pulse width = VPWR + 1
  
  /* Set active window X */
  writeReg(RA8875_HSAW0, 0);                                        // horizontal start point
  writeReg(RA8875_HSAW1, 0);
  writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF);            // horizontal end point
  writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);
  
  /* Set active window Y */
  writeReg(RA8875_VSAW0, 0);                                        // vertical start point
  writeReg(RA8875_VSAW1, 0);  
  writeReg(RA8875_VEAW0, (uint16_t)(_height - 1) & 0xFF);           // horizontal end point
  writeReg(RA8875_VEAW1, (uint16_t)(_height - 1) >> 8);
  
  /* Clear the entire window */
  writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
  delay(500); 
}

 
void displayOn(uint8_t on) 
{
 if (on) 
   writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
 else
   writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

void  writeReg(uint8_t reg, uint8_t val) 
{
  writeCommand(reg);
  writeData(val);
}

void writeCommand(uint8_t d) {
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATBbits.LATB8 = 0; // CS selected
  SPI1BUF = RA8875_CMDWRITE;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  SPI1BUF = d;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATBbits.LATB8 = 1; // CS deselected
}

void writeData(uint8_t d) {
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATBbits.LATB8 = 0; // CS selected
  SPI1BUF = RA8875_DATAWRITE;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  SPI1BUF = d;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATBbits.LATB8 = 1; // CS deselected
}

void delay(int d){
	int i = 0;
	for(;i<d*10;i++);
}

/*
void Adafruit_RA8875::sleep(boolean sleep) 
{
 if (sleep) 
   writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
 else
   writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

void  Adafruit_RA8875::writeReg(uint8_t reg, uint8_t val) 
{
  writeCommand(reg);
  writeData(val);
}

uint8_t  Adafruit_RA8875::readReg(uint8_t reg) 
{
  writeCommand(reg);
  return readData();
}

void  Adafruit_RA8875::writeData(uint8_t d) 
{
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_DATAWRITE);
  SPI.transfer(d);
  digitalWrite(_cs, HIGH);
}

uint8_t  Adafruit_RA8875::readData(void) 
{
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_DATAREAD);
  uint8_t x = SPI.transfer(0x0);
  digitalWrite(_cs, HIGH);
  return x;
}

void  Adafruit_RA8875::writeCommand(uint8_t d) 
{
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_CMDWRITE);
  SPI.transfer(d);
  digitalWrite(_cs, HIGH);
}


uint8_t  Adafruit_RA8875::readStatus(void) 
{
  digitalWrite(_cs, LOW);
  SPI.transfer(RA8875_CMDREAD);
  uint8_t x = SPI.transfer(0x0);
  digitalWrite(_cs, HIGH);
  return x;
}
*/