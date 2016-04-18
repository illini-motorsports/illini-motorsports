/**
 * Wheel Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#ifndef WHEEL_H
#define WHEEL_H

#include <sys/types.h>
#include "../FSAE_32/FSAE_config_32.h"
#include "../FSAE_32/FSAE_CAN_32.h"
#include "../FSAE_32/FSAE_adc_32.h"
#include "../FSAE.X/CAN.h"
#include "RA8875_driver.h"
#include "FSAE_LCD.h"

// Enable programmable termination
#define TERMINATING 1

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 0

#define LCD_CS_TRIS TRISGbits.TRISG9
#define LCD_CS_LAT  LATGbits.LATG9
#define LCD_RST_TRIS TRISBbits.TRISB4
#define LCD_RST_LAT  LATBbits.LATB4

#define ROT1_TRIS 	TRISCbits.TRISC4
#define ROT2_TRIS 	TRISCbits.TRISC3
#define ROT3_TRIS 	TRISCbits.TRISC2
#define TROT1_TRIS 	TRISCbits.TRISC1
#define TROT2_TRIS	TRISEbits.TRISE7
#define ROT1_ANSEL	ANSELCbits.ANSC4
#define ROT2_ANSEL	ANSELCbits.ANSC3
#define ROT3_ANSEL	ANSELCbits.ANSC2
#define TROT1_ANSEL	ANSELCbits.ANSC1
#define TROT2_ANSEL ANSELEbits.ANSE7
#define ROT1_CSS		ADCCSS1bits.CSS19
#define ROT2_CSS		ADCCSS1bits.CSS20
#define ROT3_CSS		ADCCSS1bits.CSS21
#define TROT1_CSS		ADCCSS1bits.CSS22
#define TROT2_CSS		ADCCSS1bits.CSS15
#define ROT1_CHN		19
#define ROT2_CHN		20
#define ROT3_CHN		21
#define TROT1_CHN		22
#define TROT2_CHN		15

#define ROT_RANGE_LOW 0
#define ROT_RANGE_0 409
#define ROT_RANGE_1 819
#define ROT_RANGE_2 1228
#define ROT_RANGE_3 1638
#define ROT_RANGE_4 2048
#define ROT_RANGE_5 2457
#define ROT_RANGE_6 2867
#define ROT_RANGE_7 3276
#define ROT_RANGE_8 3686
#define ROT_RANGE_HIGH 4096

#define SW1_TRIS 	TRISGbits.TRISG0
#define SW2_TRIS 	TRISGbits.TRISG1
#define SW3_TRIS 	TRISFbits.TRISF1
#define SW4_TRIS 	TRISFbits.TRISF0
#define MOM1_TRIS	TRISDbits.TRISD5
#define MOM2_TRIS	TRISDbits.TRISD4
#define MOM3_TRIS	TRISDbits.TRISD13
#define MOM4_TRIS	TRISDbits.TRISD12
#define SW1_PORT 	PORTGbits.RG0
#define SW2_PORT 	PORTGbits.RG1
#define SW3_PORT 	PORTFbits.RF1
#define SW4_PORT 	PORTFbits.RF0
#define MOM1_PORT	PORTDbits.RD5
#define MOM2_PORT	PORTDbits.RD4
#define MOM3_PORT	PORTDbits.RD13
#define MOM4_PORT	PORTDbits.RD12
#define SW1_BIT		0b1
#define SW2_BIT		0b10
#define SW3_BIT		0b100
#define SW4_BIT		0b1000
#define MOM1_BIT	0b1
#define MOM2_BIT	0b10
#define MOM3_BIT	0b100
#define MOM4_BIT	0b1000

#define CAN_SW_STATE_FREQ 	100
#define CAN_SW_ADL_FREQ 		500
#define CAN_DIAG_FREQ 			1000	

volatile uint32_t millis;

volatile uint8_t rotary[3], tRotary[2], swBitmask, momBitmask;
// Function definitions
void main(void);
void delay(uint32_t num);
void process_CAN_msg(CAN_message msg);
double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl);
void CANswitchStates(void);
void CANswitchADL(void);
void CANdiag(void);
void initADCWheel(void);
void updateSwVals(void);
uint8_t getRotaryPosition(uint32_t adcValue);

#endif /* WHEEL_H */
