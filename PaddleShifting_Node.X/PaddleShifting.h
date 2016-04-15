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
#include "../FSAE.X/errno.h"

// State variable definitions
#define SHIFT_UP_SW (!SHIFT_UP_PORT)
#define SHIFT_DN_SW (!SHIFT_DN_PORT)
#define SHIFT_NT_BT (!SHIFT_NT_PORT)

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Timing definitions (ms)
#define LOCKOUT_DUR    200 //TODO: Tune this value
#define DIAG_MSG_SEND  500
#define TEMP_SAMP_INTV 333
#define GEAR_SAMP_INTV 10
#define MAX_SHIFT_DUR  250 //TODO: Tune this value
#define RELAX_WAIT     20  //TODO: Tune this value
#define IGN_CUT_WAIT   10  //TODO: Tune this value
#define UP_SHIFT_DUR   150 //TODO: Tune this value
#define DN_SHIFT_DUR   125 //TODO: Tune this value
#define NT_SHIFT_DUR   50  //TODO: Tune this value
#define CAN_STATE_WAIT 1000

// Definitions for error conditions
//TODO: Decide on these values
#define LOW_VOLT    1000 // 10.00V
#define LOWER_VOLT  900  // 9.00V
#define IGN_CUT_RPM 2500 // 2500rpm
#define IGN_CUT_TPS 750  // 75.0%

// Thresholds
#define RPM_ON_THRESHOLD 600.0 // rpm
#define MAX_RETRY        2

// Miscellaneous definitions
#define IGN_CUT_SPOOF 10000 // Value for "spoofed" gear shift force sensor
#define ACT_ON  0
#define ACT_OFF 1

// Definitions for gear position variable
#define GEAR_NEUT 0
#define GEAR_FAIL 7

// Definitions for shift enum
#define SHIFT_ENUM_UP 0
#define SHIFT_ENUM_DN 1
#define SHIFT_ENUM_NT 2

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
uint8_t check_shift_conditions(uint8_t shift_enum);
void do_shift(uint8_t shift_enum);
void do_shift_gear_fail(uint8_t shift_enum);
void sample_gear(void);
void sample_temp(void);
void send_diag_can(uint8_t override);
void relax_wait(void);
void main_loop_misc(void);

#endif /* PADDLESHIFTING_H */
