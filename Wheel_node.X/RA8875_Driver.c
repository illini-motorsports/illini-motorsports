/*
 * File: RA8875_Driver.c
 * Author: Jake Leonard
 * Comments: Functions for interfacing with the RA8875 Display Driver
 */
#include "RA8875_Driver.h"

// Turns Display on and off based on isOn Argument

void Display_On(uint8_t isOn) {
    writeCommand(LCD_POWER);
    if (isOn) {
        writeData(LCD_DISP_ON);
    } else {
        writeData(LCD_DISP_OFF);
    }
}

// Hard reset for LCD

void LCD_Reset(void) {
    LCD_RST_LAT = 0;
    delay(1);
    LCD_RST_LAT = 1;
    delay(10);
}

// First init function for the LCD

void PLL_Init(void) {
    writeCommand(0x88);
    writeData(0x0a);
    delay(1);
    writeCommand(0x89);
    writeData(0x02);
    delay(1);
}

// Main LCD init function

void LCD_Init(void) {
    PLL_Init();
    //Set color space to 8 bit, 65k Color depth
    writeCommand(0x10); //SYSR
    writeData(0x0c);

    writeCommand(0x04); //set PCLK invers
    writeData(0x82);
    delay(1);

    //Horizontal set
    writeCommand(LCD_HRZ_WIDTH); //HDWR//Horizontal Display Width Setting Bit[6:0]
    writeData(0x3B); //Horizontal display width(pixels) = (HDWR + 1)*8
    writeCommand(LCD_HNDFTR); //Horizontal Non-Display Period Fine Tuning Option Register (HNDFTR)
    writeData(0x00); //Horizontal Non-Display Period Fine Tuning(HNDFT) [3:0]
    writeCommand(LCD_HNDR); //HNDR//Horizontal Non-Display Period Bit[4:0]
    writeData(0x01); //Horizontal Non-Display Period (pixels) = (HNDR + 1)*8
    writeCommand(LCD_HSTR); //HSTR//HSYNC Start Position[4:0]
    writeData(0x00); //HSYNC Start Position(PCLK) = (HSTR + 1)*8
    writeCommand(LCD_HPWR); //HPWR//HSYNC Polarity ,The period width of HSYNC.
    writeData(0x05); //HSYNC Width [4:0] HSYNC Pulse width(PCLK) = (HPWR + 1)*8

    //Vertical set
    writeCommand(LCD_VDHR0); //VDHR0 //Vertical Display Height Bit [7:0]
    writeData(0x0f); //Vertical pixels = VDHR + 1
    writeCommand(LCD_VDHR1); //VDHR1 //Vertical Display Height Bit [8]
    writeData(0x01); //Vertical pixels = VDHR + 1
    writeCommand(LCD_VNDR0); //VNDR0 //Vertical Non-Display Period Bit [7:0]
    writeData(0x02); //VSYNC Start Position(PCLK) = (VSTR + 1)
    writeCommand(LCD_VNDR1); //VNDR1 //Vertical Non-Display Period Bit [8]
    writeData(0x00); //Vertical Non-Display area = (VNDR + 1)
    writeCommand(LCD_VSTR0); //VSTR0 //VSYNC Start Position[7:0]
    writeData(0x07); //VSYNC Start Position(PCLK) = (VSTR + 1)
    writeCommand(LCD_VSTR1); //VSTR1 //VSYNC Start Position[8]
    writeData(0x00); //VSYNC Start Position(PCLK) = (VSTR + 1)
    writeCommand(LCD_VPWR); //VPWR //VSYNC Polarity ,VSYNC Pulse Width[6:0]
    writeData(0x09); //VSYNC Pulse Width(PCLK) = (VPWR + 1)

    //Active window set
    //setting active window X
    writeCommand(LCD_HSAW0); //Horizontal Start Point 0 of Active Window (HSAW0)
    writeData(0x00); //Horizontal Start Point of Active Window [7:0]
    writeCommand(LCD_HSAW1); //Horizontal Start Point 1 of Active Window (HSAW1)
    writeData(0x00); //Horizontal Start Point of Active Window [9:8]
    writeCommand(LCD_HEAW0); //Horizontal End Point 0 of Active Window (HEAW0)
    writeData(0xDF); //Horizontal End Point of Active Window [7:0]
    writeCommand(LCD_HEAW1); //Horizontal End Point 1 of Active Window (HEAW1)
    writeData(0x01); //Horizontal End Point of Active Window [9:8]
    //setting active window Y
    writeCommand(LCD_VSAW0); //Vertical Start Point 0 of Active Window (VSAW0)
    writeData(0x00); //Vertical Start Point of Active Window [7:0]
    writeCommand(LCD_VSAW1); //Vertical Start Point 1 of Active Window (VSAW1)
    writeData(0x00); //Vertical Start Point of Active Window [8]
    writeCommand(LCD_VEAW0); //Vertical End Point of Active Window 0 (VEAW0)
    writeData(0x0F); //Vertical End Point of Active Window [7:0]
    writeCommand(LCD_VEAW1); //Vertical End Point of Active Window 1 (VEAW1)
    writeData(0x01); //Vertical End Point of Active Window [8]
}

void writeCommand(uint8_t command) {
    SPI_Send(LCD_CMDWRITE);
    SPI_Send(command);
}

void writeData(uint8_t data) {
    SPI_Send(LCD_DATAWRITE);
    SPI_Send(data);
}

void SPI_Send(uint8_t data) {
    while (SPI_BUSY); // Wait for idle SPI module
    LCD_CS_LAT = 0; // CS selected
    SPI_BUFFER = data;
    while (SPI_BUSY); // Wait for idle SPI module
    LCD_CS_LAT = 1; // CS deselected
}

void SPI_Double_Send(uint8_t data1, uint8_t data2) {
    while (SPI_BUSY);
    LCD_CS_LAT = 0;
    SPI_BUFFER = data1;
    while (SPI_BUSY);
    SPI_BUFFER = data2;
    while (SPI_BUSY);
    LCD_CS_LAT = 1;
}