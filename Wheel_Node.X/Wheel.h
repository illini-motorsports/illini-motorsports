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

#define ROT0_TRIS       TRISFbits.TRISF12
#define ROT0_ANSEL      ANSELFbits.ANSF12
#define ROT0_CSS        ADCCSS1bits.CSS31
#define ROT0_CHN        31
#define ROT2_TRIS       TRISFbits.TRISF13
#define ROT2_ANSEL      ANSELFbits.ANSF13
#define ROT2_CSS        ADCCSS1bits.CSS8
#define ROT2_CHN        8
#define ROT2_TRG        ADCTRG3bits.TRGSRC8
#define ROT1_TRIS       TRISBbits.TRISB14
#define ROT1_ANSEL      ANSELBbits.ANSB14
#define ROT1_CSS        ADCCSS1bits.CSS9
#define ROT1_CHN        9
#define ROT1_TRG        ADCTRG3bits.TRGSRC9
#define TROT0_TRIS      TRISBbits.TRISB15
#define TROT0_ANSEL     ANSELBbits.ANSB15
#define TROT0_CSS       ADCCSS1bits.CSS10
#define TROT0_CHN       10
#define TROT0_TRG       ADCTRG3bits.TRGSRC10
#define TROT1_TRIS      TRISEbits.TRISE8
#define TROT1_ANSEL     ANSELEbits.ANSE8
#define TROT1_CSS       ADCCSS1bits.CSS25
#define TROT1_CHN       25

#define SW0_TRIS        TRISGbits.TRISG14
#define SW0_PORT        PORTGbits.RG14
#define SW1_TRIS        TRISEbits.TRISE1
#define SW1_PORT        PORTEbits.RE1
#define SW2_TRIS        TRISBbits.TRISB3
#define SW2_PORT        PORTBbits.RB3
#define SW2_ANSEL       ANSELBbits.ANSB3
#define SW3_TRIS        TRISBbits.TRISB2
#define SW3_PORT        PORTBbits.RB2
#define SW3_ANSEL       ANSELBbits.ANSB2

#define MOM0_TRIS       TRISEbits.TRISE0
#define MOM0_PORT       PORTEbits.RE0
#define MOM1_TRIS       TRISAbits.TRISA7
#define MOM1_PORT       PORTAbits.RA7
#define MOM2_TRIS       TRISEbits.TRISE9
#define MOM2_PORT       PORTEbits.RE9
#define MOM2_ANSEL      ANSELEbits.ANSE9
#define MOM3_TRIS       TRISBbits.TRISB5
#define MOM3_PORT       PORTBbits.RB5
#define MOM3_ANSEL      ANSELBbits.ANSB5

#define CAN_SW_STATE_FREQ 100
#define CAN_SW_ADL_FREQ   500
#define CAN_DIAG_FREQ     1000
#define TEMP_SAMP_INTV    333

const uint16_t shiftRPM[7] = {
  13000,  // Neutral for testing
  12444,  // First
  11950,  // Second
  12155,  // Third
  12100,  // Fourth
  11150,  // Fifth
  13000   // Sixth
};

volatile uint32_t millis;
uint8_t warnCount;

void main(void);
void init_gpio_wheel(void);
void delay(uint32_t num);
void process_CAN_msg(CAN_message msg);
double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl);
void CANswitchStates(void);
void CANswitchADL(void);
void CANdiag(void);
void updateSwVals(void);
uint8_t getRotaryPosition(uint32_t adcValue);
void checkChangeScreen(void);
void sample_temp(void);
void updateDataItem(volatile dataItem * data, double value);

#endif /* WHEEL_H */
