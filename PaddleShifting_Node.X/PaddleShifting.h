/**
 * PaddleShifting Header
 *
 * Processor:    PIC18F46K80
 * Complier:     Microchip C18
 * Author:       Andrew Mass
 * Date:         2015-2016
 */

#ifndef PADDLESHIFTING_H
#define PADDLESHIFTING_H

#include "p18f46k80.h"
#include "GenericTypeDefs.h"
#include "adc.h"
#include "../ECAN.X/ECAN.h"
#include "../FSAE.X/FSAE.h"
#include "../FSAE.X/CAN.h"

// State variable definitions
#define SHIFT_UP_SW (!SHIFT_UP_PORT)
#define SHIFT_DN_SW (!SHIFT_DN_PORT)
#define SHIFT_NT_BT (!SHIFT_NT_PORT)

// Definitions for gear position variable
#define GEAR_NEUT 0
#define GEAR_FAIL 7

// Timing definitions (ms)
#define UPSHIFT_DUR    100
#define DNSHIFT_DUR    100
#define LOCKOUT_DUR    50
#define DIAG_MSG_SEND  100
#define TEMP_SAMP_INTV 1000
#define GEAR_SAMP_INTV 10

// Pin definitions for TERM signal
#define TERM_TRIS TRISCbits.TRISC6
#define TERM_LAT	LATCbits.LATC6

// Pin definitions for TEMP signal
#define TEMP_TRIS TRISAbits.TRISA2

// Pin definitions for GEAR_POS signal
#define GEAR_POS_TRIS TRISAbits.TRISA1
#define GEAR_POS_PORT PORTAbits.RA1

// Pin definitions for SHIFT signals
#define SHIFT_UP_TRIS TRISDbits.TRISD4
#define SHIFT_UP_PORT PORTDbits.RD4
#define SHIFT_DN_TRIS TRISDbits.TRISD5
#define SHIFT_DN_PORT PORTDbits.RD5
#define SHIFT_NT_TRIS TRISDbits.TRISD6
#define SHIFT_NT_PORT PORTDbits.RD6

// Pin definitions for ACT signals
#define ACT_UP_TRIS TRISBbits.TRISB0
#define ACT_UP_LAT  LATBbits.LATB0
#define ACT_UP_PORT PORTBbits.RB0
#define ACT_DN_TRIS TRISBbits.TRISB1
#define ACT_DN_LAT  LATBbits.LATB1
#define ACT_DN_PORT PORTBbits.RB1

// ADC Channel definitions
#define ADC_GEAR_CHN ADC_CH1
#define ADC_TEMP_CHN ADC_CH2

// Function definitions
void high_isr(void);
uint16_t sample(const uint8_t ch);
void process_upshift_press(void);
void process_downshift_press(void);

#endif /* PADDLESHIFTING_H */
