/* 
 * File: Wheel.c
 * Author: Jake Leonard
 * Comments: Main code that controls the wheel module
 */
#ifndef WHEEL_H
#define	WHEEL_H

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <sys/types.h>
#include "FSAE_config_32.h"
#include "../FSAE_32/FSAE_CAN_32.h"
#include "../FSAE_32/FSAE_adc_32.h"
//#include "RA8875_Driver.h"

#define TERMINATING 1

// Indicator LED's
#define PIC_FN_TRIS 	TRISBbits.TRISB6
#define PIC_MODE_TRIS 	TRISBbits.TRISB7
#define PIC_FN_LAT 	LATBbits.LATB6
#define PIC_MODE_LAT 	LATBbits.LATB7

#define LCD_CS_TRIS 	TRISBbits.TRISB8
#define LCD_CS_LAT	LATBbits.LATB8
#define SPI_BUSY 	SPI1STATbits.SPIBUSY
#define SPI_BUFFER	SPI1BUF
#define LCD_BACKLITE_TRIS	TRISEbits.TRISE8
#define LCD_BACKLITE_LAT	LATEbits.LATE8

// Momentary Switches
#define SW_MOMENT1_TRIS	TRISDbits.TRISD5
#define SW_MOMENT2_TRIS	TRISDbits.TRISD4
#define SW_MOMENT3_TRIS	TRISDbits.TRISD13
#define SW_MOMENT4_TRIS	TRISDbits.TRISD12
#define SW_MOMENT1_LAT	LATDbits.LATD5
#define SW_MOMENT2_LAT	LATDbits.LATD4
#define SW_MOMENT3_LAT	LATDbits.LATD13
#define SW_MOMENT4_LAT	LATDbits.LATD12

// Toggle Switches
#define SW_TOG1_TRIS	TRISGbits.TRISG0
#define SW_TOG2_TRIS	TRISGbits.TRISG1
#define SW_TOG3_TRIS	TRISFbits.TRISF1
#define SW_TOG4_TRIS	TRISFbits.TRISF0
#define SW_TOG1_LAT	LATGbits.LATG0
#define SW_TOG2_LAT	LATGbits.LATG1
#define SW_TOG3_LAT	LATFbits.LATF1
#define SW_TOG4_LAT	LATFbits.LATF0

// Rotaries
#define SW_ROT1_TRIS	TRISCbits.TRISC4
#define SW_ROT2_TRIS	TRISCbits.TRISC3
#define SW_ROT3_TRIS	TRISCbits.TRISC2
#define SW_TROT1_TRIS	TRISCbits.TRISC1
#define SW_TROT2_TRIS	TRISEbits.TRISE7
#define SW_ROT1_ANSEL	ANSELCbits.ANSC4
#define SW_ROT2_ANSEL	ANSELCbits.ANSC3
#define SW_ROT3_ANSEL	ANSELCbits.ANSC2
#define SW_TROT1_ANSEL	ANSELCbits.ANSC1
#define SW_TROT2_ANSEL	ANSELEbits.ANSE7
#define SW_ROT1_CSS	ADCCSS1bits.CSS19
#define SW_ROT2_CSS	ADCCSS1bits.CSS20
#define SW_ROT3_CSS	ADCCSS1bits.CSS21
#define SW_TROT1_CSS	ADCCSS1bits.CSS22
#define SW_TROT2_CSS	ADCCSS1bits.CSS15

// Touchscreen
#define LCD_XP_TRIS	TRISGbits.TRISG15
#define LCD_XN_TRIS	TRISEbits.TRISE5
#define LCD_YP_TRIS	TRISAbits.TRISA5
#define LCD_YN_TRIS	TRISEbits.TRISE6
#define LCD_XP_ANSEL	ANSELGbits.ANSG15
#define LCD_XN_ANSEL	ANSELEbits.ANSE5
#define LCD_YP_ANSEL	ANSELAbits.ANSA5
#define LCD_YN_ANSEL	ANSELEbits.ANSE6
#define LCD_XP_CSS	ADCCSS1bits.CSS23
#define LCD_XN_CSS	ADCCSS1bits.CSS17
#define LCD_YP_CSS	ADCCSS1bits.CSS34
#define LCD_YN_CSS	ADCCSS1bits.CSS16

void update_sw_values();
void delay(uint32_t mil);

//********************************************************
//********************************************************

// RA8875 Stuff!!

void PLL_init(void);
void writeCommand(uint8_t command);
void writeData(uint8_t data);

// Define statements for registers and other info
// Colors (RGB565)
#define	RA8875_BLACK            0x0000
#define	RA8875_BLUE             0x001F
#define	RA8875_RED              0xF800
#define	RA8875_GREEN            0x07E0
#define RA8875_CYAN             0x07FF
#define RA8875_MAGENTA          0xF81F
#define RA8875_YELLOW           0xFFE0  
#define RA8875_WHITE            0xFFFF

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


#define RA8875_ELLIPSE                0xA0
#define RA8875_ELLIPSE_STATUS         0x80

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

#endif	/* WHEEL_H */

