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
#include "../FSAE.X/FSAE_tlc5955.h"
#include "../FSAE.X/CAN.h"
#include "RA8875_driver.h"
#include "FSAE_LCD.h"

#define LCD_CS_TRIS     TRISCbits.TRISC14
#define LCD_CS_LAT      LATCbits.LATC14
#define LCD_CS_LATBITS  (uint32_t*) (&LATCbits)
#define LCD_CS_LATNUM   14
#define LCD_RST_TRIS    TRISCbits.TRISC13
#define LCD_RST_LAT     LATCbits.LATC13
#define LCD_PWM_TRIS    TRISEbits.TRISE3
#define LCD_PWM_LAT     LATEbits.LATE3

#define ROT1_TRIS     TRISFbits.TRISF12
#define ROT1_ANSEL    ANSELFbits.ANSF12
#define ROT1_CSS      ADCCSS1bits.CSS31
#define ROT1_CHN      31
#define ROT2_TRIS     TRISFbits.TRISF13
#define ROT2_ANSEL    ANSELFbits.ANSF13
#define ROT2_CSS      ADCCSS1bits.CSS8
#define ROT2_CHN      8
#define ROT3_TRIS     TRISBbits.TRISB14
#define ROT3_ANSEL    ANSELBbits.ANSB14
#define ROT3_CSS      ADCCSS1bits.CSS9
#define ROT3_CHN      9
#define TROT1_TRIS    TRISBbits.TRISB15
#define TROT1_ANSEL   ANSELBbits.ANSB15
#define TROT1_CSS     ADCCSS1bits.CSS10
#define TROT1_CHN     10
#define TROT2_TRIS    TRISEbits.TRISE8
#define TROT2_ANSEL   ANSELEbits.ANSE8
#define TROT2_CSS     ADCCSS1bits.CSS25
#define TROT2_CHN     25

#define ROT_RANGE_LOW   0
#define ROT_RANGE_0     409
#define ROT_RANGE_1     819
#define ROT_RANGE_2     1228
#define ROT_RANGE_3     1638
#define ROT_RANGE_4     2048
#define ROT_RANGE_5     2457
#define ROT_RANGE_6     2867
#define ROT_RANGE_7     3276
#define ROT_RANGE_8     3686
#define ROT_RANGE_HIGH  4096

#define SW1_TRIS   TRISGbits.TRISG14
#define SW1_PORT   PORTGbits.RG14
#define SW2_TRIS   TRISEbits.TRISE1
#define SW2_PORT   PORTEbits.RE1
#define SW3_TRIS   TRISBbits.TRISB3
#define SW3_PORT   PORTBbits.RB3
#define SW4_TRIS   TRISBbits.TRISB2
#define SW4_PORT   PORTBbits.RB2

#define MOM1_TRIS  TRISEbits.TRISE0
#define MOM1_PORT  PORTEbits.RE0
#define MOM2_TRIS  TRISAbits.TRISA7
#define MOM2_PORT  PORTAbits.RA7
#define MOM3_TRIS  TRISEbits.TRISE9
#define MOM3_PORT  PORTEbits.RE9
#define MOM4_TRIS  TRISBbits.TRISB5
#define MOM4_PORT  PORTBbits.RB5

#define CAN_SW_STATE_FREQ 100
#define CAN_SW_ADL_FREQ   500
#define CAN_DIAG_FREQ     1000

volatile uint32_t millis;

void main(void);
void delay(uint32_t num);
void process_CAN_msg(CAN_message msg);
double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl);
void CANswitchStates(void);
void CANswitchADL(void);
void CANdiag(void);
void updateSwVals(void);
uint8_t getRotaryPosition(uint32_t adcValue);

#endif /* WHEEL_H */
