/**
 * RA8875 Driver
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#include "RA8875_driver.h"
#include "Wheel.h"

SPIConn ra8875Connection;

/*
 ***********************Formula Functions***************************************
 */
void drawChevron(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fg, uint16_t bg) {
  int mid = (w / 2) + x;
  int spacing = (h-(w/2))/11;
  int bgOffset = spacing/2;
  int i;
  for(i = 0;i<11;i++){
    fillTriangle(x,y+h-(w/2)-(i*spacing),x+w,y+h-(w/2)-(i*spacing),
        x+(w/2),y+h-(i*spacing),fg);
    fillTriangle(x,y+h-(w/2)-(i*spacing)-bgOffset,x+w,y+h-(w/2)-(i*spacing)-bgOffset,
        x+(w/2),y+h-(i*spacing)-bgOffset,bg);
  }
  fillRect(x,y,w/10,h,bg);
  fillRect(x+w-(w/10),y,w/10,h,bg);
  fillTriangle(x,y,x+(w/2),y,x,y+(w/2),bg);
  fillTriangle(x+w,y,x+(w/2),y,x+w,y+(w/2),bg);
}

// Displays a decimal number using the seven segment helper functions
void sevenSegmentDecimal(uint16_t x, uint16_t y, uint16_t numWidth, uint16_t numNums, 
    uint16_t decDigits, uint16_t color, double number){
  if(decDigits == 0){
    sevenSegmentMultDigit(x, y, numWidth, numNums, color, (uint16_t) number);
    return;
  }
  uint16_t intNum = number;
  uint16_t decNum = (number - intNum) * pow(10, decDigits);
  sevenSegmentMultDigit(x, y, numWidth, numNums - decDigits, color, intNum);
  fillCircle(x + (numWidth*1.2*(numNums - decDigits)), y + (numWidth*1.7), numWidth/10, color);
  sevenSegmentMultDigit(x + (numWidth*1.2*(numNums - decDigits)) + numWidth/5, 
      y, numWidth, decDigits, color, decNum);
}

// Displays multiple digits using the seven segment helper functions
void sevenSegmentMultDigit(uint16_t x, uint16_t y, uint16_t numWidth, uint16_t numNums, 
    uint16_t color, uint16_t number){
  uint16_t offset = numWidth*1.2;
  int i = 0;
  for(;i<numNums;i++){
    sevenSegmentDigit(x + (offset*(numNums - i - 1)), y, numWidth, color, number%10);
    number /=10;
  }
}

// Determines what bars of the seven segments to display
void sevenSegmentDigit(uint16_t x, uint16_t y, uint16_t w, uint16_t color, uint8_t digit){
  switch(digit){
    case 0:
      sevenSegment(x, y, w, color, SEVEN_SEG_0);
      break;
    case 1:
      sevenSegment(x, y, w, color, SEVEN_SEG_1);
      break;
    case 2:
      sevenSegment(x, y, w, color, SEVEN_SEG_2);
      break;
    case 3:
      sevenSegment(x, y, w, color, SEVEN_SEG_3);
      break;
    case 4:
      sevenSegment(x, y, w, color, SEVEN_SEG_4);
      break;
    case 5:
      sevenSegment(x, y, w, color, SEVEN_SEG_5);
      break;
    case 6:
      sevenSegment(x, y, w, color, SEVEN_SEG_6);
      break;
    case 7:
      sevenSegment(x, y, w, color, SEVEN_SEG_7);
      break;
    case 8:
      sevenSegment(x, y, w, color, SEVEN_SEG_8);
      break;
    case 9:
      sevenSegment(x, y, w, color, SEVEN_SEG_9);
      break;
  }
}

// Individually draws each bar and triangles in the seven segment
void sevenSegment(uint16_t x, uint16_t y, uint16_t w, uint16_t color, uint8_t bMask){
  uint16_t h = w*1.75;
  uint8_t boxD = w/5;
  uint8_t hBoxD = w/10;
  uint8_t barHeight = (h - (3*boxD))/2;
  uint8_t fhOffset = boxD + barHeight;
  uint8_t shOffset = (2*boxD) + (2*barHeight);

  if(bMask & 0x40){ // Top Horizontal Bar
    fillRect(x+boxD,y,w-(2*boxD),boxD,color);
    fillTriangle(x+boxD,y,x+hBoxD,y+hBoxD,x+boxD,y+boxD,color);
    fillTriangle(x+w-boxD,y,x+w-hBoxD,y+hBoxD,x+w-boxD,y+boxD,color);
  }
  if(bMask & 0x01){ // Middle Horizontal Bar
    fillRect(x+boxD,y+boxD+barHeight,w-(2*boxD),boxD,color);
    fillTriangle(x+boxD,y+fhOffset,x+hBoxD,y+hBoxD+fhOffset,x+boxD,
        y+boxD+fhOffset,color);
    fillTriangle(x+w-boxD,y+fhOffset,x+w-hBoxD,y+hBoxD+fhOffset,x+w-boxD,
        y+boxD+fhOffset,color);
  }
  if(bMask & 0x08){ // Bottom Horizontal Bar
    fillRect(x+boxD,y+(2*boxD)+(2*barHeight),w-(2*boxD),boxD,color);
    fillTriangle(x+boxD,y+shOffset,x+hBoxD,y+hBoxD+shOffset,x+boxD,
        y+boxD+shOffset,color);
    fillTriangle(x+w-boxD,y+shOffset,x+w-hBoxD,y+hBoxD+shOffset,x+w-boxD,
        y+boxD+shOffset,color);
  }
  if(bMask & 0x02){ // Top Left Vertical Bar
    fillRect(x,y+boxD,boxD,barHeight,color);
    fillTriangle(x,y+boxD,x+boxD,y+boxD,x+hBoxD,y+hBoxD,color);
    fillTriangle(x,y+boxD+barHeight,x+boxD,y+boxD+barHeight,x+hBoxD,
        y+hBoxD+boxD+barHeight,color);
  }

  if(bMask & 0x20){ // Bottom Left Vertical Bar
    fillRect(x+w-boxD,y+boxD,boxD,barHeight,color);
    fillTriangle(x+w-boxD,y+boxD,x+w,y+boxD,x+w-hBoxD,y+hBoxD,color);
    fillTriangle(x+w-boxD,y+boxD+barHeight,x+w,y+boxD+barHeight,x+w-hBoxD,
        y+hBoxD+boxD+barHeight,color);
  }

  if(bMask & 0x04){ // Top Right Vertical Bar
    fillRect(x,y+boxD+boxD+barHeight,boxD,barHeight,color);
    fillTriangle(x,y+boxD+fhOffset,x+boxD,y+boxD+fhOffset,x+hBoxD,
        y+hBoxD+fhOffset,color);
    fillTriangle(x,y+boxD+barHeight+fhOffset,x+boxD,y+boxD+barHeight+fhOffset,
        x+hBoxD,y+hBoxD+boxD+barHeight+fhOffset,color);
  }

  if(bMask & 0x10){ // Bottom Right Vertical Bar
    fillRect(x+w-boxD,y+boxD+boxD+barHeight,boxD,barHeight,color);
    fillTriangle(x+w-boxD,y+boxD+fhOffset,x+w,y+boxD+fhOffset,x+w-hBoxD,
        y+hBoxD+fhOffset,color);
    fillTriangle(x+w-boxD,y+boxD+barHeight+fhOffset,x+w,y+boxD+barHeight+fhOffset,
        x+w-hBoxD,y+hBoxD+boxD+barHeight+fhOffset,color);
  }
}

/*
 * Init Functions
 */
void reset() {
  LCD_RST_LAT = 0;
  delay(10);// Changed from 100
  LCD_RST_LAT = 1;
  delay(100);
}

void initialize(void) {
  TRISFbits.TRISF12 = INPUT;

  /* Initialize PLL */
  PLLinit();
  writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

  /* Timing values */
  writeReg(RA8875_PCSR, RA8875_PCSR_PDATL | RA8875_PCSR_4CLK);// Pixclk
  delay(1);

  /* Horizontal settings registers */
  writeReg(RA8875_HDWR, (WIDTH / 8) - 1);// H width
  writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH);
  writeReg(RA8875_HNDR, 1);// H non-display
  writeReg(RA8875_HSTR, 0);// Hsync start
  writeReg(RA8875_HPWR, RA8875_HPWR_LOW + 5);// HSync pulse width

  /* Vertical settings registers */
  writeReg(RA8875_VDHR0, (uint16_t) (HEIGHT - 1) & 0xFF);
  writeReg(RA8875_VDHR1, (uint16_t) (HEIGHT - 1) >> 8);
  writeReg(RA8875_VNDR0, 9);// V non-display period
  writeReg(RA8875_VNDR1, 0);
  writeReg(RA8875_VSTR0, 7);// Vsync start position
  writeReg(RA8875_VSTR1, 0);
  writeReg(RA8875_VPWR, RA8875_VPWR_LOW + 9);// Vsync pulse width

  /* Set active window X */
  writeReg(RA8875_HSAW0, 0);// horizontal start point
  writeReg(RA8875_HSAW1, 0);
  writeReg(RA8875_HEAW0, (uint16_t) (WIDTH - 1) & 0xFF);// horizontal end point
  writeReg(RA8875_HEAW1, (uint16_t) (WIDTH - 1) >> 8);

  /* Set active window Y */
  writeReg(RA8875_VSAW0, 0);// vertical start point
  writeReg(RA8875_VSAW1, 0);
  writeReg(RA8875_VEAW0, (uint16_t) (HEIGHT - 1) & 0xFF);// horizontal end point
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

void displayOn(uint8_t on) {
  if (on) {
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
  } else {
    writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
  }
}

void GPIOX(uint8_t on) {
  if (on) {
    writeReg(RA8875_GPIOX, 1);
  } else {
    writeReg(RA8875_GPIOX, 0);
  }
}

void PWM1config(uint8_t on, uint8_t clock) {
  if (on) {
    writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
  } else {
    writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
  }
}

void PWM1out(uint8_t p) {
  writeReg(RA8875_P1DCR, p);
}

/*
 ****************** Graphics Stuff *********************************************
 */

void graphicsMode(void) {
  writeCommand(RA8875_MWCR0);
  //uint8_t temp = readData();
  //temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
  //writeData(temp);
  writeData(0x00);
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  rectHelper(x, y, x + w, y + h, color, 0);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  rectHelper(x, y, x + w, y + h, color, 1);
}

void fillScreen(uint16_t color) {
  rectHelper(0, 0, _width - 1, _height - 1, color, 1);
}

void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  circleHelper(x0, y0, r, color, 0);
}

void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
  circleHelper(x0, y0, r, color, 1);
}

void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, 0);
}

void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  triangleHelper(x0, y0, x1, y1, x2, y2, color, 1);
}

void drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, 0);
}

void fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color) {
  ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, 1);
}

void drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, 0);
}

void fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color) {
  curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, 1);
}

void fillCircleSquare(int16_t x, int16_t y, int16_t w, int16_t h, int16_t corner, uint16_t color){
  circleSquareHelper(x,y,x+w,y+h,corner,color,1);
}

/*
 ****************** Graphics Helpers *******************************************
 */

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
  // Begin Coordinates
  writeCoordinates(RA8875_RECT_X0_0, x0, y0);

  // End Coordinates Coordinates
  writeCoordinates(RA8875_RECT_X1_0, x1, y1);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeReg(RA8875_DCR, 0x80);

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

void rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled) {
  // Begin Coordinates
  writeCoordinates(RA8875_RECT_X0_0, x, y);

  // End Coordinates
  writeCoordinates(RA8875_RECT_X1_0, w, h);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xB0);
  } else {
    writeData(0x90);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

void circleSquareHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t corner, uint16_t color, uint8_t filled){
  // Begin Coordinates
  writeCoordinates(RA8875_CIRC_SQUARE_X0, x, y);
  // End Coordinates
  writeCoordinates(RA8875_CIRC_SQUARE_X1, w, h);
  // Circle Corners
  writeCoordinates(RA8875_CIRC_SQUARE_CORN, corner, corner);
  // Set Color
  setColor(color, 1);
  // Draw
  if(filled){
    writeReg(RA8875_CIRC_SQUARE_DCR, RA8875_CIRC_SQUARE_STRT | RA8875_CIRC_SQUARE_FILL);
  }
  else{
    writeReg(RA8875_CIRC_SQUARE_DCR, RA8875_CIRC_SQUARE_STRT);
  }
  // Wait till it's done
  waitPoll(RA8875_CIRC_SQUARE_DCR, RA8875_CIRC_SQUARE_STAT);
}

void circleHelper(int16_t x, int16_t y, int16_t r, uint16_t color, uint8_t filled) {
  writeCoordinates(RA8875_CIRC_X_0, x, y);

  /* Set Radius */
  writeReg(RA8875_CIRC_RAD, r);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
  } else {
    writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

void triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, 
    int16_t y2, uint16_t color, uint8_t filled) {

  /* Set Point 0 */
  writeCoordinates(0x91, x0, y0);

  /* Set Point 1 */
  writeCoordinates(0x95, x1, y1);

  /* Set Point 2 */
  writeCoordinates(0xA9, x2, y2);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeCommand(RA8875_DCR);
  if (filled) {
    writeData(0xA1);
  } else {
    writeData(0x81);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

void ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, 
    uint16_t color, uint8_t filled) {

  /* Set Center Point */
  writeCoordinates(0xA5, xCenter, yCenter);

  /* Set Long and Short Axis */
  writeCoordinates(0xA1, longAxis, shortAxis);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xC0);
  } else {
    writeData(0x80);
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

void curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, 
    uint8_t curvePart, uint16_t color, uint8_t filled) {
  /* Set Center Point */
  writeCoordinates(0xA5, xCenter, yCenter);

  /* Set Long and Short Axis */
  writeCoordinates(0xA1, longAxis, shortAxis);

  /* Set Color */
  setColor(color, 1);

  /* Draw! */
  writeCommand(0xA0);
  if (filled) {
    writeData(0xD0 | (curvePart & 0x03));
  } else {
    writeData(0x90 | (curvePart & 0x03));
  }

  /* Wait for the command to finish */
  waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/*
 *************** Text Functions ************************************************
 */

void textMode(void) {
  /* Set text mode */
  writeCommand(RA8875_MWCR0);
  uint8_t temp = readData();
  temp |= RA8875_MWCR0_TXTMODE;// Set bit 7
  writeData(temp);

  /* Select the internal (ROM) font */
  writeCommand(0x21);
  temp = readData();
  temp &= ~((1 << 7) | (1 << 5));// Clear bits 7 and 5
  writeData(temp);
}

void textSetCursor(uint16_t x, uint16_t y) {
  /* Set cursor location */
  writeCoordinates(0x2A, x, y);
}

void textColor(uint16_t foreColor, uint16_t bgColor) {
  /* Set Fore Color */
  setColor(foreColor, 1);

  /* Set Background Color */
  setColor(bgColor, 0);

  /* Clear transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(1 << 6);// Clear bit 6
  writeData(temp);
}

void textTransparent(uint16_t foreColor) {
  /* Set Fore Color */
  setColor(foreColor, 1);

  /* Set transparency flag */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp |= (1 << 6);// Set bit 6
  writeData(temp);
}

void textEnlarge(uint8_t scale) {
  if (scale > 3) scale = 3;

  /* Set font size flags */
  writeCommand(0x22);
  uint8_t temp = readData();
  temp &= ~(0xF); // Clears bits 0..3
  temp |= scale << 2;
  temp |= scale;
  writeData(temp);

  _textScale = scale;
}

/*
 * ************ SPI and Direct Read/Write Functions ****************************
 */

void textWrite(const char* buffer){
  textWriteHelper(buffer, 0);
}

void textWriteHelper(const char* buffer, uint16_t len) {
  if (len == 0) len = strlen(buffer);
  writeCommand(RA8875_MRWC);
  uint16_t i = 0;
  for (;i < len;i++) {
    writeData(buffer[i]);
    if(_textScale >= 1){
      delay(1);
    }
  }
}

void drawPixel(int16_t x, int16_t y, uint16_t color){
  writeCoordinates(RA8875_CURH0, x, y);
  writeCommand(RA8875_MRWC);
  LCD_CS_LAT = 0;
  send_spi(RA8875_DATAWRITE, &ra8875Connection);
  send_spi(color >> 8, &ra8875Connection);
  send_spi(color, &ra8875Connection);
  LCD_CS_LAT = 1;
}

uint8_t waitPoll(uint8_t reg, uint8_t flag){
  while(1){
    uint8_t readVal = readReg(reg);
    if(!(flag & readVal)){
      return 1;
    }
  }
  return 0;
}

void setColor(uint16_t color, uint8_t isForeground) {
  if (isForeground) {
    writeReg(RA8875_FGCR_RED, (color & RA8875_RED) >> 11);
    writeReg(RA8875_FGCR_GREEN, (color & RA8875_GREEN) >> 5);
    writeReg(RA8875_FGCR_BLUE, color & RA8875_BLUE);
  } else {
    writeReg(RA8875_BGCR_RED, (color & RA8875_RED) >> 11);
    writeReg(RA8875_BGCR_GREEN, (color & RA8875_GREEN) >> 5);
    writeReg(RA8875_BGCR_BLUE, color & RA8875_BLUE);
  }
}

void writeCoordinates(uint8_t s_reg, uint16_t x, uint16_t y) {
  /* Set X */
  writeReg(s_reg, x);
  writeReg(s_reg + 1, x >> 8);

  /* Set Y */
  writeReg(s_reg + 2, y);
  writeReg(s_reg + 3, y >> 8);
}

uint8_t readReg(uint8_t reg){
  writeCommand(reg);
  return readData();
}

void writeReg(uint8_t reg, uint8_t val) {
  writeCommand(reg);
  writeData(val);
}

uint8_t readData() {
  return ra8875_send_spi(RA8875_DATAREAD, 0, &ra8875Connection);
}

void writeData(uint8_t d) {
  ra8875_send_spi(RA8875_DATAWRITE, d, &ra8875Connection);
}

void writeCommand(uint8_t d) {
  ra8875_send_spi(RA8875_CMDWRITE, d, &ra8875Connection);
}

SPIConn* init_ra8875(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  init_spi(bus, 2, 8);
  ra8875Connection.send_fp = get_send_spi(bus);
  ra8875Connection.cs_lat = cs_lat;
  ra8875Connection.cs_num = cs_num;
  reset();
  initialize();
}

uint8_t ra8875_send_spi(uint8_t val1, uint8_t val2, SPIConn *conn){
  *(conn->cs_lat) &= ~(1 << (conn->cs_num)); // Set CS Low
  conn->send_fp(val1);
  uint8_t buff = conn->send_fp(val2); // only care about the response from last 8 bits
  *(conn->cs_lat) |= 1 << conn->cs_num; // Set CS High
  return buff;
}
