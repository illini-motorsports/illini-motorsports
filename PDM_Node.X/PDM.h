/**
 * PDM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef PDM_H
#define PDM_H

#include <sys/types.h>
#include "../FSAE_32/FSAE_config_32.h"

// Enable programmable termination
#define TERMINATING 1

// Defines for MOSFET control
#define PWR_ON  0
#define PWR_OFF 1

// Indices for arrays containing all the loads
#define IGN_IDX   0
#define INJ_IDX   1
#define FUEL_IDX  2
#define ECU_IDX   3
#define WTR_IDX   4
#define FAN_IDX   5
#define AUX_IDX   6
#define PDLU_IDX  7
#define PDLD_IDX  8
#define B5V5_IDX  9
#define BVBAT_IDX 10
#define STR_IDX   11
#define STR0_IDX  11
#define STR1_IDX  12
#define STR2_IDX  13

// Pin definitions for EN signal bus
#define EN_IGN_LAT    LATDbits.LATD15
#define EN_INJ_LAT    LATCbits.LATC15
#define EN_FUEL_LAT   LATFbits.LATF3
#define EN_ECU_LAT    LATFbits.LATF2
#define EN_WTR_LAT    LATFbits.LATF8
#define EN_FAN_LAT    LATAbits.LATA3
#define EN_AUX_LAT    LATAbits.LATA4
#define EN_PDLU_LAT   LATFbits.LATF4
#define EN_PDLD_LAT   LATFbits.LATF5
#define EN_B5V5_LAT   LATAbits.LATA14
#define EN_BVBAT_LAT  LATAbits.LATA15
#define EN_STR_LAT    LATDbits.LATD9
#define EN_IGN_PORT   PORTDbits.PORTD15
#define EN_INJ_PORT   PORTCbits.PORTC15
#define EN_FUEL_PORT  PORTFbits.PORTF3
#define EN_ECU_PORT   PORTFbits.PORTF2
#define EN_WTR_PORT   PORTFbits.PORTF8
#define EN_FAN_PORT   PORTAbits.PORTA3
#define EN_AUX_PORT   PORTAbits.PORTA4
#define EN_PDLU_PORT  PORTFbits.PORTF4
#define EN_PDLD_PORT  PORTFbits.PORTF5
#define EN_B5V5_PORT  PORTAbits.PORTA14
#define EN_BVBAT_PORT PORTAbits.PORTA15
#define EN_STR_PORT   PORTDbits.PORTD9
#define EN_IGN_TRIS   TRISDbits.TRISD15
#define EN_INJ_TRIS   TRISCbits.TRISC15
#define EN_FUEL_TRIS  TRISFbits.TRISF3
#define EN_ECU_TRIS   TRISFbits.TRISF2
#define EN_WTR_TRIS   TRISFbits.TRISF8
#define EN_FAN_TRIS   TRISAbits.TRISA3
#define EN_AUX_TRIS   TRISAbits.TRISA4
#define EN_PDLU_TRIS  TRISFbits.TRISF4
#define EN_PDLD_TRIS  TRISFbits.TRISF5
#define EN_B5V5_TRIS  TRISAbits.TRISA14
#define EN_BVBAT_TRIS TRISAbits.TRISA15
#define EN_STR_TRIS   TRISDbits.TRISD9

// Pin definitions for !CS signal bus
#define CS_IGN_LAT    LATEbits.LATE8
#define CS_INJ_LAT    LATEbits.LATE9
#define CS_FUEL_LAT   LATBbits.LATB5
#define CS_ECU_LAT    LATBbits.LATB4
#define CS_WTR_LAT    LATBbits.LATB3
#define CS_FAN_LAT    LATBbits.LATB2
#define CS_AUX_LAT    LATBbits.LATB6
#define CS_PDLU_LAT   LATBbits.LATB7
#define CS_PDLD_LAT   LATAbits.LATA9
#define CS_B5V5_LAT   LATAbits.LATA10
#define CS_BVBAT_LAT  LATBbits.LATB8
#define CS_STR0_LAT   LATAbits.LATA1
#define CS_STR1_LAT   LATFbits.LATF13
#define CS_STR2_LAT   LATDbits.LATD14
#define CS_IGN_PORT    PORTEbits.PORTE8
#define CS_INJ_PORT    PORTEbits.PORTE9
#define CS_FUEL_PORT   PORTBbits.PORTB5
#define CS_ECU_PORT    PORTBbits.PORTB4
#define CS_WTR_PORT    PORTBbits.PORTB3
#define CS_FAN_PORT    PORTBbits.PORTB2
#define CS_AUX_PORT    PORTBbits.PORTB6
#define CS_PDLU_PORT   PORTBbits.PORTB7
#define CS_PDLD_PORT   PORTAbits.PORTA9
#define CS_B5V5_PORT   PORTAbits.PORTA10
#define CS_BVBAT_PORT  PORTBbits.PORTB8
#define CS_STR0_PORT   PORTAbits.PORTA1
#define CS_STR1_PORT   PORTFbits.PORTF13
#define CS_STR2_PORT   PORTDbits.PORTD14
#define CS_IGN_TRIS    TRISEbits.TRISE8
#define CS_INJ_TRIS    TRISEbits.TRISE9
#define CS_FUEL_TRIS   TRISBbits.TRISB5
#define CS_ECU_TRIS    TRISBbits.TRISB4
#define CS_WTR_TRIS    TRISBbits.TRISB3
#define CS_FAN_TRIS    TRISBbits.TRISB2
#define CS_AUX_TRIS    TRISBbits.TRISB6
#define CS_PDLU_TRIS   TRISBbits.TRISB7
#define CS_PDLD_TRIS   TRISAbits.TRISA9
#define CS_B5V5_TRIS   TRISAbits.TRISA10
#define CS_BVBAT_TRIS  TRISBbits.TRISB8
#define CS_STR0_TRIS   TRISAbits.TRISA1
#define CS_STR1_TRIS   TRISFbits.TRISF13
#define CS_STR2_TRIS   TRISDbits.TRISD14

// Function definitions
void main(void);
void send_rheo(uint16_t msg);
void send_all_rheo(uint16_t msg);

#endif /* PDM_H */
