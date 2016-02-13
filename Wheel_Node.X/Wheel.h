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
#include "RA8875_Driver.h"

#define TERMINATING 1
#define INTERNAL_CLK 1

// Indicator LED's
#define PIC_FN_TRIS 	TRISBbits.TRISB6
#define PIC_MODE_TRIS 	TRISBbits.TRISB7
#define PIC_FN_LAT 		LATBbits.LATB6
#define PIC_MODE_LAT 	LATBbits.LATB7

#define LCD_CS_TRIS 	TRISBbits.TRISB8
#define LCD_CS_LAT		LATBbits.LATB8
#define LCD_BACKLITE_TRIS	TRISEbits.TRISE8
#define LCD_BACKLITE_LAT	LATEbits.LATE8
#define LCD_RST_TRIS 	TRISBbits.TRISB11
#define LCD_RST_LAT		LATBbits.LATB11

// SPI stuff
#define SPI_BUSY 		SPI1STATbits.SPIBUSY
#define SPI_BUFFER		SPI1BUF

// Momentary Switches
#define SW_MOMENT1_TRIS	TRISDbits.TRISD5
#define SW_MOMENT2_TRIS	TRISDbits.TRISD4
#define SW_MOMENT3_TRIS	TRISDbits.TRISD13
#define SW_MOMENT4_TRIS	TRISDbits.TRISD12
#define SW_MOMENT1_PORT	PORTDbits.RD5
#define SW_MOMENT2_PORT	PORTDbits.RD4
#define SW_MOMENT3_PORT	PORTDbits.RD13
#define SW_MOMENT4_PORT	PORTDbits.RD12

// Toggle Switches
#define SW_TOG1_TRIS	TRISGbits.TRISG0
#define SW_TOG2_TRIS	TRISGbits.TRISG1
#define SW_TOG3_TRIS	TRISFbits.TRISF1
#define SW_TOG4_TRIS	TRISFbits.TRISF0
#define SW_TOG1_PORT	PORTGbits.RG0
#define SW_TOG2_PORT	PORTGbits.RG1
#define SW_TOG3_PORT	PORTFbits.RF1
#define SW_TOG4_PORT	PORTFbits.RF0

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
#define SW_ROT1_CSS		ADCCSS1bits.CSS19
#define SW_ROT2_CSS		ADCCSS1bits.CSS20
#define SW_ROT3_CSS		ADCCSS1bits.CSS21
#define SW_TROT1_CSS	ADCCSS1bits.CSS22
#define SW_TROT2_CSS	ADCCSS1bits.CSS15

// Touchscreen
#define LCD_XP_TRIS		TRISGbits.TRISG15
#define LCD_XN_TRIS		TRISEbits.TRISE5
#define LCD_YP_TRIS		TRISAbits.TRISA5
#define LCD_YN_TRIS		TRISEbits.TRISE6
#define LCD_XP_ANSEL	ANSELGbits.ANSG15
#define LCD_XN_ANSEL	ANSELEbits.ANSE5
#define LCD_YP_ANSEL	ANSELAbits.ANSA5
#define LCD_YN_ANSEL	ANSELEbits.ANSE6
#define LCD_XP_CSS		ADCCSS1bits.CSS23
#define LCD_XN_CSS		ADCCSS1bits.CSS17
#define LCD_YP_CSS		ADCCSS1bits.CSS34
#define LCD_YN_CSS		ADCCSS1bits.CSS16

void update_sw_values();
void delay(uint32_t mil);

#endif	/* WHEEL_H */
