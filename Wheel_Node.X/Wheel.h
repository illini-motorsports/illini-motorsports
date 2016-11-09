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
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"
#include "RA8875_driver.h"
#include "FSAE_LCD.h"

// Enable programmable termination
#define TERMINATING 0

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 0

#define LCD_CS_TRIS	TRISGbits.TRISG9
#define LCD_CS_LAT	LATGbits.LATG9
#define LCD_RST_TRIS	TRISBbits.TRISB4
#define LCD_RST_LAT	LATBbits.LATB4

#define ROT1_TRIS	TRISCbits.TRISC4
#define ROT2_TRIS 	TRISCbits.TRISC3
#define ROT3_TRIS 	TRISCbits.TRISC2
#define TROT1_TRIS 	TRISCbits.TRISC1
#define TROT2_TRIS	TRISEbits.TRISE7
#define ROT1_ANSEL	ANSELCbits.ANSC4
#define ROT2_ANSEL	ANSELCbits.ANSC3
#define ROT3_ANSEL	ANSELCbits.ANSC2
#define TROT1_ANSEL	ANSELCbits.ANSC1
#define TROT2_ANSEL 	ANSELEbits.ANSE7
#define ROT1_CSS	ADCCSS1bits.CSS19
#define ROT2_CSS	ADCCSS1bits.CSS20
#define ROT3_CSS	ADCCSS1bits.CSS21
#define TROT1_CSS	ADCCSS1bits.CSS22
#define TROT2_CSS	ADCCSS1bits.CSS15
#define ROT1_CHN	19
#define ROT2_CHN	20
#define ROT3_CHN	21
#define TROT1_CHN	22
#define TROT2_CHN	15

#define ROT_RANGE_LOW	0
#define ROT_RANGE_0	409
#define ROT_RANGE_1 	819
#define ROT_RANGE_2 	1228
#define ROT_RANGE_3 	1638
#define ROT_RANGE_4 	2048
#define ROT_RANGE_5 	2457
#define ROT_RANGE_6 	2867
#define ROT_RANGE_7 	3276
#define ROT_RANGE_8 	3686
#define ROT_RANGE_HIGH	4096

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

#define CAN_SW_STATE_FREQ 	100
#define CAN_SW_ADL_FREQ 	500
#define CAN_DIAG_FREQ 		1000

//PDM Bitmask bits
#define STR_ENBL_BIT		0x10
#define BVBAT_ENBL_BIT		0x20
#define B5v5_ENBL_BIT		0x40
#define PDLD_ENBL_BIT		0x80
#define PDLU_ENBL_BIT		0x100
#define AUX_ENBL_BIT		0x200
#define FAN_ENBL_BIT		0x400
#define WTR_ENBL_BIT		0x800
#define ECU_ENBL_BIT		0x1000
#define FUEL_ENBL_BIT		0x2000
#define INJ_ENBL_BIT		0x4000
#define IGN_ENBL_BIT		0x8000

#define STR2_PEAKM_BIT		0x4
#define STR1_PEAKM_BIT		0x8
#define STR0_PEAKM_BIT		0x10
#define BVBAT_PEAKM_BIT		0x20
#define B5v5_PEAKM_BIT		0x40
#define PDLD_PEAKM_BIT		0x80
#define PDLU_PEAKM_BIT		0x100
#define AUX_PEAKM_BIT		0x200
#define FAN_PEAKM_BIT		0x400
#define WTR_PEAKM_BIT		0x800
#define ECU_PEAKM_BIT		0x1000
#define FUEL_PEAKM_BIT		0x2000
#define INJ_PEAKM_BIT		0x4000
#define IGN_PEAKM_BIT		0x8000

#define KILL_PDM_SW_BIT		0x8
#define ACT_DN_PDM_SW_BIT	0x10
#define ACT_UP_PDM_SW_BIT	0x20
#define ON_PDM_SW_BIT		0x40
#define STR_PDM_SW_BIT		0x80

volatile uint32_t millis;

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
void init_spi();

#endif /* WHEEL_H */
