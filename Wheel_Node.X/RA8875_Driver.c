/*
 * File: RA8875_Driver.c
 * Author: Jake Leonard
 * Comments: Functions for interfacing with the RA8875 Display Driver
 */
#include "RA8875_Driver.h"

// Turns Display on and off based on isOn Argument
void Display_On(uint8_t isOn) {
  if (isOn) {
    writeReg(LCD_POWER, LCD_DISP_ON);
  } else {
    writeReg(LCD_POWER, LCD_DISP_OFF);
  }
}

// Hard reset for LCD
void LCD_Reset(void) {
  LCD_RST_LAT = 1;
  delay(10);
  LCD_RST_LAT = 0;
  delay(220);
  LCD_RST_LAT = 1;
  delay(300);
}

// First init function for the LCD
void PLL_Init(uint8_t pll1, uint8_t pll2) {
  writeReg(0x88, pll1);// PLL Control Reg 1
  //writeCommand(0x88);
  //writeData(0x0a);
  delay(1);
  writeReg(0x89, pll2);// PLL Control Reg 2
  //writeCommand(0x89);
  //writeData(0x02);
  delay(1);
  writeReg(0x04, 0x82);// Set Pixel Clock
  delay(1);
}

// Main LCD init function
void LCD_Init(void) {
  PLL_Init(0x07, 0x03);// Initial PLL Set?

  //Set color space to 8 bit, 65k Color depth
  writeReg(0x10, 0x0C);//SYSR

  delay(1);

  //Horizontal set
  writeReg(LCD_HRZ_WIDTH, 0x3B);//HDWR: Horizontal Display Width Setting Bit[6:0] Horizontal display width(pixels) = (HDWR + 1)*8
  writeReg(LCD_HNDFTR, 0x00);//Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)
  writeReg(LCD_HNDR, 0x01);//HNDR: Horizontal Non-Display Period Bit[4:0]
  writeReg(LCD_HSTR, 0x01);//HSTR: HSYNC Start Position[4:0]
  writeReg(LCD_HPWR, 0x05);//HPWR: HSYNC Polarity ,The period width of HSYNC.
  delay(1);

  //Vertical set
  writeReg(LCD_VDHR0, 0x0f);//VDHR0: Vertical Display Height Bit [7:0]
  writeReg(LCD_VDHR1, 0x01);//VDHR1: Vertical Display Height Bit [8]
  writeReg(LCD_VNDR0, 0x02);//VNDR0: Vertical Non-Display Period Bit [7:0]
  writeReg(LCD_VNDR1, 0x00);//VNDR1: Vertical Non-Display Period Bit [8]
  writeReg(LCD_VSTR0, 0x07);//VSTR0: VSYNC Start Position[7:0]
  writeReg(LCD_VSTR1, 0x00);//VSTR1: VSYNC Start Position[8]
  writeReg(LCD_VPWR, 0x09);//VPWR: VSYNC Polarity ,VSYNC Pulse Width[6:0]
  delay(1);

  /*
   * NOT CONVERTED YET
   * *******************************************************************
   */
  //Active window set
  //setting active window X
  writeCommand(LCD_HSAW0);//Horizontal Start Point 0 of Active Window (HSAW0)
  writeData(0x00);//Horizontal Start Point of Active Window [7:0]
  writeCommand(LCD_HSAW1);//Horizontal Start Point 1 of Active Window (HSAW1)
  writeData(0x00);//Horizontal Start Point of Active Window [9:8]
  writeCommand(LCD_HEAW0);//Horizontal End Point 0 of Active Window (HEAW0)
  writeData(0xDF);//Horizontal End Point of Active Window [7:0]
  writeCommand(LCD_HEAW1);//Horizontal End Point 1 of Active Window (HEAW1)
  writeData(0x01);//Horizontal End Point of Active Window [9:8]
  //setting active window Y
  writeCommand(LCD_VSAW0);//Vertical Start Point 0 of Active Window (VSAW0)
  writeData(0x00);//Vertical Start Point of Active Window [7:0]
  writeCommand(LCD_VSAW1);//Vertical Start Point 1 of Active Window (VSAW1)
  writeData(0x00);//Vertical Start Point of Active Window [8]
  writeCommand(LCD_VEAW0);//Vertical End Point of Active Window 0 (VEAW0)
  writeData(0x0F);//Vertical End Point of Active Window [7:0]
  writeCommand(LCD_VEAW1);//Vertical End Point of Active Window 1 (VEAW1)
  writeData(0x01);//Vertical End Point of Active Window [8]

  // **********************************************************************
  delay(10);
  PLL_Init(0x0B, 0x01);
  delay(10);
}

// Writes a value to a specific register
void writeReg(uint8_t reg, uint8_t data) {
  writeCommand(reg);
  writeData(data);
}

// Sends a command
void writeCommand(uint8_t command) {
  SPI_Double_Send(LCD_CMDWRITE, command);
}

// Sends data
void writeData(uint8_t data) {
  SPI_Double_Send(LCD_DATAWRITE, data);
}

// Sends two 8 bit messages without putting CS High
void SPI_Double_Send(uint8_t data1, uint8_t data2) {
  while (SPI_BUSY);// Wait until its ready
  LCD_CS_LAT = 0;// Set CS to low
  SPI_BUFFER = data1;// Send the first message
  while (SPI_BUSY);// Wait again
  SPI_BUFFER = data2;// Send the next message
  while (SPI_BUSY);// Wait a third time
  LCD_CS_LAT = 1;// We gucci
}