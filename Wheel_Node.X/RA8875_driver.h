/**
 * RA8875 Driver Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#ifndef RA8875_DRIVER_H
#define RA8875_DRIVER_H

#include <xc.h>
#include <sys/types.h>
#include <math.h>
#include "../FSAE.X/FSAE_spi.h"

#define WIDTH   480
#define HEIGHT    272
#define LCD_WAIT  PORTFbits.RF12

// Screen State Functions
void reset(); //Toggles reset pin
void displayOn(uint8_t on); // Sends an spi command to wake up screen
void PLLinit(void);   // Initialize Clock
void initialize(void);  // Sends commands to configure the driver
SPIConn* init_ra8875(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num); // initializes spi and calls other init functions
void reset_init(); // Intended to restart ra8875 after it has entered error state
uint8_t ra8875_send_spi(uint8_t val1, uint8_t val2, SPIConn *conn); // Specialized send_spi function for RA8875

/* Backlight */
void GPIOX(uint8_t on);
void PWM1config(uint8_t on, uint8_t clock);
void PWM2config(uint8_t on, uint8_t clock);
void PWM1out(uint8_t p);
void PWM2out(uint8_t p);

/* Text functions */
void textMode(void);  // Switches drawing mode to text
void textSetCursor(uint16_t x, uint16_t y); // Sets the coordinates of the cursor
void textColor(uint16_t foreColor, uint16_t bgColor);
void textTransparent(uint16_t foreColor); // Only writes foreground pixels
void textEnlarge(uint8_t scale); // Scales text size, not super precise
void textWrite(const char* buffer); // Writes content of null terminated buffer
void textWriteHelper(const char* buffer, uint16_t len);

// Seven Segment Functions
// Helper seven segment drawer, uses a bitmask to determine segments to draw
void sevenSegment(uint16_t x, uint16_t y, uint16_t w, uint16_t color, uint8_t bMask);
// Draws a single seven segment digit
void sevenSegmentDigit(uint16_t x, uint16_t y, uint16_t w, uint16_t color, uint8_t digit);
// Draws Multiple Digits
void sevenSegmentMultDigit(uint16_t x, uint16_t y, uint16_t w, uint16_t numNums, uint16_t color, uint16_t number);
// Draws a decimal number
void sevenSegmentDecimal(uint16_t x, uint16_t y, uint16_t numWidth, uint16_t numNums, uint16_t decDigits, uint16_t color, double number);
// Draw the FSAE Chevron
void drawChevron(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t fg, uint16_t bg);

/* Graphics functions */
void graphicsMode(void); // Switch to graphics mode
void fillScreen(uint16_t color);
void drawPixel(int16_t x, int16_t y, uint16_t color);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void drawEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color);
void fillEllipse(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color);
void drawCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color);
void fillCurve(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color);
void fillCircleSquare(int16_t x, int16_t y, int16_t w, int16_t h, int16_t corner, uint16_t color);
void setColor(uint16_t color, uint8_t isForeground);
void writeCoordinates(uint8_t s_reg, uint16_t x, uint16_t y); // Coordinate helper function

/* Low level access */
void writeReg(uint8_t reg, uint8_t val);
uint8_t readReg(uint8_t reg);
void writeData(uint8_t d);
uint8_t readData(void);
void writeCommand(uint8_t d);
uint8_t readData(void);
uint8_t readReg(uint8_t reg);
// Spams spi, can we do it better?
uint8_t waitPoll(uint8_t reg, uint8_t flag);
uint8_t isDisplayOn(); // used to determine if display is in an unresponsive error state

/* GFX Helper Functions */
void circleHelper(int16_t x, int16_t y, int16_t r, uint16_t color, uint8_t filled);
void rectHelper(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint8_t filled);
void triangleHelper(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint8_t filled);
void ellipseHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint16_t color, uint8_t filled);
void curveHelper(int16_t xCenter, int16_t yCenter, int16_t longAxis, int16_t shortAxis, uint8_t curvePart, uint16_t color, uint8_t filled);
void circleSquareHelper(int16_t x, int16_t y, int16_t w, int16_t h, int16_t corner, uint16_t color, uint8_t filled);

uint8_t _cs, _rst;
uint16_t _width, _height;
uint8_t _textScale;

/* Jake Defined Values */
#define SEVEN_SEG_0     0x7E
#define SEVEN_SEG_1     0x30
#define SEVEN_SEG_2     0x6D
#define SEVEN_SEG_3     0x79
#define SEVEN_SEG_4     0x33
#define SEVEN_SEG_5     0x5B
#define SEVEN_SEG_6     0x5F
#define SEVEN_SEG_7     0x70
#define SEVEN_SEG_8     0x7F
#define SEVEN_SEG_9     0x7B
#define SEVEN_SEG_N     0x76
#define SEVEN_SEG_E     0x4F

// Rectangle Coordinates
#define RA8875_RECT_X0_0  0x91
#define RA8875_RECT_X0_1  0x92
#define RA8875_RECT_Y0_0  0x93
#define RA8875_RECT_Y0_1  0x94

#define RA8875_RECT_X1_0  0x95
#define RA8875_RECT_X1_1  0x96
#define RA8875_RECT_Y1_0  0x97
#define RA8875_RECT_Y1_1  0x98

// Circle Coordinates
#define RA8875_CIRC_X_0   0x99
#define RA8875_CIRC_X_1   0x9a
#define RA8875_CIRC_Y_0   0x9b
#define RA8875_CIRC_Y_1   0x9c
#define RA8875_CIRC_RAD   0x9d

//Circle Square Coordinates
#define RA8875_CIRC_SQUARE_X0   0x91
#define RA8875_CIRC_SQUARE_X1   0x95
#define RA8875_CIRC_SQUARE_CORN 0xA1
#define RA8875_CIRC_SQUARE_DCR  0xA0
#define RA8875_CIRC_SQUARE_STRT 0xA0
#define RA8875_CIRC_SQUARE_FILL 0x40
#define RA8875_CIRC_SQUARE_STAT 0x80

// Background Color Registers
#define RA8875_BGCR_RED         0x60
#define RA8875_BGCR_GREEN       0x61
#define RA8875_BGCR_BLUE        0x62

// Foreground Color Registers
#define RA8875_FGCR_RED         0x63
#define RA8875_FGCR_GREEN       0x64
#define RA8875_FGCR_BLUE        0x65


// Colors (RGB565)
#define RA8875_BLACK            0x0000
#define RA8875_BLUE             0x001F
#define RA8875_RED              0xF800
#define RA8875_GREEN            0x07E0
#define RA8875_CYAN             0x07FF
#define RA8875_MAGENTA          0xF81F
#define RA8875_YELLOW           0xFFE0
#define RA8875_WHITE            0xFFFF
#define RA8875_GREY             0xBAF7

// Command/Data pins for SPI
#define RA8875_DATAWRITE        0x00
#define RA8875_DATAREAD         0x40
#define RA8875_CMDWRITE         0x80
#define RA8875_CMDREAD          0xC0

// Registers & bits
#define RA8875_PWRR             0x01
#define RA8875_PWRR_DISPON      0x80
#define RA8875_PWRR_DISPOFF     0x00
#define RA8875_PWRR_SLEEP       0x02
#define RA8875_PWRR_NORMAL      0x00
#define RA8875_PWRR_SOFTRESET   0x01

#define RA8875_MRWC             0x02

#define RA8875_GPIOX            0xC7

#define RA8875_PLLC1            0x88
#define RA8875_PLLC1_PLLDIV2    0x80
#define RA8875_PLLC1_PLLDIV1    0x00

#define RA8875_PLLC2            0x89
#define RA8875_PLLC2_DIV1       0x00
#define RA8875_PLLC2_DIV2       0x01
#define RA8875_PLLC2_DIV4       0x02
#define RA8875_PLLC2_DIV8       0x03
#define RA8875_PLLC2_DIV16      0x04
#define RA8875_PLLC2_DIV32      0x05
#define RA8875_PLLC2_DIV64      0x06
#define RA8875_PLLC2_DIV128     0x07

#define RA8875_SYSR             0x10
#define RA8875_SYSR_8BPP        0x00
#define RA8875_SYSR_16BPP       0x0C
#define RA8875_SYSR_MCU8        0x00
#define RA8875_SYSR_MCU16       0x03

#define RA8875_PCSR             0x04
#define RA8875_PCSR_PDATR       0x00
#define RA8875_PCSR_PDATL       0x80
#define RA8875_PCSR_CLK         0x00
#define RA8875_PCSR_2CLK        0x01
#define RA8875_PCSR_4CLK        0x02
#define RA8875_PCSR_8CLK        0x03

#define RA8875_HDWR             0x14

#define RA8875_HNDFTR           0x15
#define RA8875_HNDFTR_DE_HIGH   0x00
#define RA8875_HNDFTR_DE_LOW    0x80

#define RA8875_HNDR             0x16
#define RA8875_HSTR             0x17
#define RA8875_HPWR             0x18
#define RA8875_HPWR_LOW         0x00
#define RA8875_HPWR_HIGH        0x80

#define RA8875_VDHR0            0x19
#define RA8875_VDHR1            0x1A
#define RA8875_VNDR0            0x1B
#define RA8875_VNDR1            0x1C
#define RA8875_VSTR0            0x1D
#define RA8875_VSTR1            0x1E
#define RA8875_VPWR             0x1F
#define RA8875_VPWR_LOW         0x00
#define RA8875_VPWR_HIGH        0x80

#define RA8875_HSAW0            0x30
#define RA8875_HSAW1            0x31
#define RA8875_VSAW0            0x32
#define RA8875_VSAW1            0x33

#define RA8875_HEAW0            0x34
#define RA8875_HEAW1            0x35
#define RA8875_VEAW0            0x36
#define RA8875_VEAW1            0x37

#define RA8875_MCLR             0x8E
#define RA8875_MCLR_START       0x80
#define RA8875_MCLR_STOP        0x00
#define RA8875_MCLR_READSTATUS  0x80
#define RA8875_MCLR_FULL        0x00
#define RA8875_MCLR_ACTIVE      0x40

#define RA8875_DCR                    0x90
#define RA8875_DCR_LINESQUTRI_START   0x80
#define RA8875_DCR_LINESQUTRI_STOP    0x00
#define RA8875_DCR_LINESQUTRI_STATUS  0x80
#define RA8875_DCR_CIRCLE_START       0x40
#define RA8875_DCR_CIRCLE_STATUS      0x40
#define RA8875_DCR_CIRCLE_STOP        0x00
#define RA8875_DCR_FILL               0x20
#define RA8875_DCR_NOFILL             0x00
#define RA8875_DCR_DRAWLINE           0x00
#define RA8875_DCR_DRAWTRIANGLE       0x01
#define RA8875_DCR_DRAWSQUARE         0x10


#define RA8875_ELLIPSE          0xA0
#define RA8875_ELLIPSE_STATUS   0x80

#define RA8875_MWCR0            0x40
#define RA8875_MWCR0_GFXMODE    0x00
#define RA8875_MWCR0_TXTMODE    0x80

#define RA8875_CURH0            0x46
#define RA8875_CURH1            0x47
#define RA8875_CURV0            0x48
#define RA8875_CURV1            0x49

#define RA8875_P1CR             0x8A
#define RA8875_P1CR_ENABLE      0x80
#define RA8875_P1CR_DISABLE     0x00
#define RA8875_P1CR_CLKOUT      0x10
#define RA8875_P1CR_PWMOUT      0x00

#define RA8875_P1DCR            0x8B

#define RA8875_P2CR             0x8C
#define RA8875_P2CR_ENABLE      0x80
#define RA8875_P2CR_DISABLE     0x00
#define RA8875_P2CR_CLKOUT      0x10
#define RA8875_P2CR_PWMOUT      0x00

#define RA8875_P2DCR            0x8D

#define RA8875_PWM_CLK_DIV1     0x00
#define RA8875_PWM_CLK_DIV2     0x01
#define RA8875_PWM_CLK_DIV4     0x02
#define RA8875_PWM_CLK_DIV8     0x03
#define RA8875_PWM_CLK_DIV16    0x04
#define RA8875_PWM_CLK_DIV32    0x05
#define RA8875_PWM_CLK_DIV64    0x06
#define RA8875_PWM_CLK_DIV128   0x07
#define RA8875_PWM_CLK_DIV256   0x08
#define RA8875_PWM_CLK_DIV512   0x09
#define RA8875_PWM_CLK_DIV1024  0x0A
#define RA8875_PWM_CLK_DIV2048  0x0B
#define RA8875_PWM_CLK_DIV4096  0x0C
#define RA8875_PWM_CLK_DIV8192  0x0D
#define RA8875_PWM_CLK_DIV16384 0x0E
#define RA8875_PWM_CLK_DIV32768 0x0F

#define RA8875_TPCR0                  0x70
#define RA8875_TPCR0_ENABLE           0x80
#define RA8875_TPCR0_DISABLE          0x00
#define RA8875_TPCR0_WAIT_512CLK      0x00
#define RA8875_TPCR0_WAIT_1024CLK     0x10
#define RA8875_TPCR0_WAIT_2048CLK     0x20
#define RA8875_TPCR0_WAIT_4096CLK     0x30
#define RA8875_TPCR0_WAIT_8192CLK     0x40
#define RA8875_TPCR0_WAIT_16384CLK    0x50
#define RA8875_TPCR0_WAIT_32768CLK    0x60
#define RA8875_TPCR0_WAIT_65536CLK    0x70
#define RA8875_TPCR0_WAKEENABLE       0x08
#define RA8875_TPCR0_WAKEDISABLE      0x00
#define RA8875_TPCR0_ADCCLK_DIV1      0x00
#define RA8875_TPCR0_ADCCLK_DIV2      0x01
#define RA8875_TPCR0_ADCCLK_DIV4      0x02
#define RA8875_TPCR0_ADCCLK_DIV8      0x03
#define RA8875_TPCR0_ADCCLK_DIV16     0x04
#define RA8875_TPCR0_ADCCLK_DIV32     0x05
#define RA8875_TPCR0_ADCCLK_DIV64     0x06
#define RA8875_TPCR0_ADCCLK_DIV128    0x07

#define RA8875_TPCR1            0x71
#define RA8875_TPCR1_AUTO       0x00
#define RA8875_TPCR1_MANUAL     0x40
#define RA8875_TPCR1_VREFINT    0x00
#define RA8875_TPCR1_VREFEXT    0x20
#define RA8875_TPCR1_DEBOUNCE   0x04
#define RA8875_TPCR1_NODEBOUNCE 0x00
#define RA8875_TPCR1_IDLE       0x00
#define RA8875_TPCR1_WAIT       0x01
#define RA8875_TPCR1_LATCHX     0x02
#define RA8875_TPCR1_LATCHY     0x03

#define RA8875_TPXH             0x72
#define RA8875_TPYH             0x73
#define RA8875_TPXYL            0x74

#define RA8875_INTC1            0xF0
#define RA8875_INTC1_KEY        0x10
#define RA8875_INTC1_DMA        0x08
#define RA8875_INTC1_TP         0x04
#define RA8875_INTC1_BTE        0x02

#define RA8875_INTC2            0xF1
#define RA8875_INTC2_KEY        0x10
#define RA8875_INTC2_DMA        0x08
#define RA8875_INTC2_TP         0x04
#define RA8875_INTC2_BTE        0x02

#endif /* RA8875_DRIVER_H */
