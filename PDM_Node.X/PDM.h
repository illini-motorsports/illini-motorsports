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
#include "../FSAE_32/FSAE_CAN_32.h"
#include "../FSAE.X/CAN.h"

// Enable programmable termination
#define TERMINATING 1

// Defines for MOSFET control
#define PWR_ON  1
#define PWR_OFF 0

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
#define EN_IGN_PORT   PORTDbits.RD15
#define EN_INJ_PORT   PORTCbits.RC15
#define EN_FUEL_PORT  PORTFbits.RF3
#define EN_ECU_PORT   PORTFbits.RF2
#define EN_WTR_PORT   PORTFbits.RF8
#define EN_FAN_PORT   PORTAbits.RA3
#define EN_AUX_PORT   PORTAbits.RA4
#define EN_PDLU_PORT  PORTFbits.RF4
#define EN_PDLD_PORT  PORTFbits.RF5
#define EN_B5V5_PORT  PORTAbits.RA14
#define EN_BVBAT_PORT PORTAbits.RA15
#define EN_STR_PORT   PORTDbits.RD9
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
#define CS_IGN_LAT     LATEbits.LATE8
#define CS_INJ_LAT     LATEbits.LATE9
#define CS_FUEL_LAT    LATBbits.LATB5
#define CS_ECU_LAT     LATBbits.LATB4
#define CS_WTR_LAT     LATBbits.LATB3
#define CS_FAN_LAT     LATBbits.LATB2
#define CS_AUX_LAT     LATBbits.LATB6
#define CS_PDLU_LAT    LATBbits.LATB7
#define CS_PDLD_LAT    LATAbits.LATA9
#define CS_B5V5_LAT    LATAbits.LATA10
#define CS_BVBAT_LAT   LATBbits.LATB8
#define CS_STR0_LAT    LATAbits.LATA1
#define CS_STR1_LAT    LATFbits.LATF13
#define CS_STR2_LAT    LATDbits.LATD14
#define CS_IGN_PORT    PORTEbits.RE8
#define CS_INJ_PORT    PORTEbits.RE9
#define CS_FUEL_PORT   PORTBbits.RB5
#define CS_ECU_PORT    PORTBbits.RB4
#define CS_WTR_PORT    PORTBbits.RB3
#define CS_FAN_PORT    PORTBbits.RB2
#define CS_AUX_PORT    PORTBbits.RB6
#define CS_PDLU_PORT   PORTBbits.RB7
#define CS_PDLD_PORT   PORTAbits.RA9
#define CS_B5V5_PORT   PORTAbits.RA10
#define CS_BVBAT_PORT  PORTBbits.RB8
#define CS_STR0_PORT   PORTAbits.RA1
#define CS_STR1_PORT   PORTFbits.RF13
#define CS_STR2_PORT   PORTDbits.RD14
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

// Pin definitions for !SW signal bus
#define SW1_PORT   PORTDbits.RD12
#define SW2_PORT   PORTDbits.RD13
#define SW3_PORT   PORTDbits.RD4
#define SW4_PORT   PORTDbits.RD5
#define SW5_PORT   PORTFbits.RF0
#define SW6_PORT   PORTFbits.RF1
#define SW7_PORT   PORTGbits.RG1
#define SW8_PORT   PORTGbits.RG0
#define SW9_PORT   PORTAbits.RA6
#define KILL_PORT  PORTAbits.RA7
#define SW1_TRIS   TRISDbits.TRISD12
#define SW2_TRIS   TRISDbits.TRISD13
#define SW3_TRIS   TRISDbits.TRISD4
#define SW4_TRIS   TRISDbits.TRISD5
#define SW5_TRIS   TRISFbits.TRISF0
#define SW6_TRIS   TRISFbits.TRISF1
#define SW7_TRIS   TRISGbits.TRISG1
#define SW8_TRIS   TRISGbits.TRISG0
#define SW9_TRIS   TRISAbits.TRISA6
#define KILL_TRIS  TRISAbits.TRISA7

// Switch state definitions
#define STR_SW    (!SW1_PORT)
#define ON_SW     (!SW2_PORT)
#define ACT_UP_SW (!SW3_PORT)
#define ACT_DN_SW (!SW4_PORT)
#define KILL_SW   (!KILL_PORT)

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Load state definitions
#define IGN_EN   (EN_IGN_PORT == PWR_ON)
#define INJ_EN   (EN_INJ_PORT == PWR_ON)
#define FUEL_EN  (EN_FUEL_PORT == PWR_ON)
#define ECU_EN   (EN_ECU_PORT == PWR_ON)
#define WTR_EN   (EN_WTR_PORT == PWR_ON)
#define FAN_EN   (EN_FAN_PORT == PWR_ON)
#define AUX_EN   (EN_AUX_PORT == PWR_ON)
#define PDLU_EN  (EN_PDLU_PORT == PWR_ON)
#define PDLD_EN  (EN_PDLD_PORT == PWR_ON)
#define B5V5_EN  (EN_B5V5_PORT == PWR_ON)
#define BVBAT_EN (EN_BVBAT_PORT == PWR_ON)
#define STR_EN   (EN_STR_PORT == PWR_ON)

// Thresholds
#define RPM_ON_THRESHOLD 600.0 // rpm
#define FAN_THRESHOLD_H  90.0  // C
#define FAN_THRESHOLD_L  84.0  // C

// Timing constants (ms)
#define FUEL_PRIME_DUR     1000
#define BASIC_CONTROL_WAIT 1000
#define STR_MAX_DUR        4000

// Function definitions
void main(void);
void process_CAN_msg(CAN_message msg);
void send_rheo(uint16_t msg);
void send_all_rheo(uint16_t msg);

#endif /* PDM_H */
