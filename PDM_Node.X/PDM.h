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
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_ad7490.h"
#include "../FSAE.X/CAN.h"

// Definitions for MOSFET control
#define PWR_ON  1
#define PWR_OFF 0

// Definitions for overcurrent state
#define NO_OVERCRT    0
#define OVERCRT       1
#define OVERCRT_RESET 2

// The number of MOSFET control circuits
#define NUM_CTL 11

// The number of load circuits
#define NUM_LOADS 12

// Indices for arrays containing all the loads
#define FUEL_IDX  0
#define IGN_IDX   1
#define INJ_IDX   2
#define ABS_IDX   3
#define PDLU_IDX  4
#define PDLD_IDX  5
#define FAN_IDX   6
#define WTR_IDX   7
#define ECU_IDX   8
#define AUX_IDX   9
#define BVBAT_IDX 10
#define STR_IDX   11

// Thresholds
#define RPM_ON_THRESHOLD 200.0 // rpm
#define FAN_THRESHOLD_H  90.0  // C
#define FAN_THRESHOLD_L  84.0  // C
#define OVERCRT_DETECT   0.01  // A

// Timing constants (ms)

#define FUEL_PRIME_DUR     500
#define STR_MAX_DUR        4000
#define PDL_MAX_DUR        500
#define OVERRIDE_SW_WAIT   5000
#define BASIC_CONTROL_WAIT 1000

#define TEMP_SAMP_INTV     333
#define EXT_ADC_SAMP_INTV  10

#define DEBOUNCE_WAIT      10
#define OVERCRT_RESET_WAIT 250
#define OVERCRT_CHK_INTV   10

#define DIAG_SEND          1000
#define DIAG_STATE_SEND    500
#define RAIL_VOLT_SEND     10
#define LOAD_CUR_SEND      10
#define CUTOFF_VAL_SEND    1000
#define OVERCRT_COUNT_SEND 500

#define FUEL_PEAK_DUR      150
#define WTR_PEAK_DUR       250
#define FAN_PEAK_DUR       2000
#define ECU_PEAK_DUR       2

// Raw (bouncy) switch state definitions
#define STR_SW_RAW    (!SW1_PORT)
#define ON_SW_RAW     (SW2_PORT)
#define ACT_UP_SW_RAW (!SW3_PORT)
#define ACT_DN_SW_RAW (!SW4_PORT)
#define ABS_SW_RAW    (SW5_PORT)
#define AUX1_SW_RAW   (SW6_PORT)
#define AUX2_SW_RAW   (!SW7_PORT)
#define KILL_SW_RAW   (KILL_PORT)

// Debounced switch state definitions (use these for everything)
#define STR_SW (((switch_debounced >> 7) & 0x1))
#define ON_SW (((switch_debounced >> 6) & 0x1))
#define ACT_UP_SW (((switch_debounced >> 5) & 0x1))
#define ACT_DN_SW (((switch_debounced >> 4) & 0x1))
#define KILL_SW (((switch_debounced >> 3) & 0x1))
#define ABS_SW (((switch_debounced >> 2) & 0x1))
#define AUX1_SW (((switch_debounced >> 1) & 0x1))
#define AUX2_SW (((switch_debounced >> 0) & 0x1))

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Load state definitions
#define FUEL_EN  (EN_FUEL_PORT == PWR_ON)
#define IGN_EN   (EN_IGN_PORT == PWR_ON)
#define INJ_EN   (EN_INJ_PORT == PWR_ON)
#define ABS_EN   (EN_ABS_PORT == PWR_ON)
#define PDLU_EN  (EN_PDLU_PORT == PWR_ON)
#define PDLD_EN  (EN_PDLD_PORT == PWR_ON)
#define FAN_EN   (EN_FAN_PORT == PWR_ON)
#define WTR_EN   (EN_WTR_PORT == PWR_ON)
#define ECU_EN   (EN_ECU_PORT == PWR_ON)
#define AUX_EN   (EN_AUX_PORT == PWR_ON)
#define BVBAT_EN (EN_BVBAT_PORT == PWR_ON)
#define STR_EN   (EN_STR_PORT == PWR_ON)

// Pin definitions for EN signal bus
#define EN_FUEL_LAT     LATBbits.LATB3
#define EN_IGN_LAT      LATDbits.LATD5
#define EN_INJ_LAT      LATCbits.LATC14
#define EN_ABS_LAT      LATBbits.LATB14
#define EN_PDLU_LAT     LATEbits.LATE2
#define EN_PDLD_LAT     LATGbits.LATG12
#define EN_FAN_LAT      LATDbits.LATD13
#define EN_WTR_LAT      LATBbits.LATB7
#define EN_ECU_LAT      LATEbits.LATE4
#define EN_AUX_LAT      LATAbits.LATA9
#define EN_BVBAT_LAT    LATAbits.LATA15
#define EN_STR_LAT      LATAbits.LATA3
#define EN_FUEL_PORT    PORTBbits.RB3
#define EN_IGN_PORT     PORTDbits.RD5
#define EN_INJ_PORT     PORTCbits.RC14
#define EN_ABS_PORT     PORTBbits.RB14
#define EN_PDLU_PORT    PORTEbits.RE2
#define EN_PDLD_PORT    PORTGbits.RG12
#define EN_FAN_PORT     PORTDbits.RD13
#define EN_WTR_PORT     PORTBbits.RB7
#define EN_ECU_PORT     PORTEbits.RE4
#define EN_AUX_PORT     PORTAbits.RA9
#define EN_BVBAT_PORT   PORTAbits.RA15
#define EN_STR_PORT     PORTAbits.RA3
#define EN_FUEL_TRIS    TRISBbits.TRISB3
#define EN_IGN_TRIS     TRISDbits.TRISD5
#define EN_INJ_TRIS     TRISCbits.TRISC14
#define EN_ABS_TRIS     TRISBbits.TRISB14
#define EN_PDLU_TRIS    TRISEbits.TRISE2
#define EN_PDLD_TRIS    TRISGbits.TRISG12
#define EN_FAN_TRIS     TRISDbits.TRISD13
#define EN_WTR_TRIS     TRISBbits.TRISB7
#define EN_ECU_TRIS     TRISEbits.TRISE4
#define EN_AUX_TRIS     TRISAbits.TRISA9
#define EN_BVBAT_TRIS   TRISAbits.TRISA15
#define EN_STR_TRIS     TRISAbits.TRISA3
#define EN_FUEL_ANSEL   ANSELBbits.ANSB3
#define EN_ABS_ANSEL    ANSELBbits.ANSB14
#define EN_WTR_ANSEL    ANSELBbits.ANSB7
#define EN_ECU_ANSEL    ANSELEbits.ANSE4
#define EN_AUX_ANSEL    ANSELAbits.ANSA9

// Pin definitions for !CS signal bus
#define CS_FUEL_LAT     LATBbits.LATB2
#define CS_IGN_LAT      LATDbits.LATD4
#define CS_INJ_LAT      LATCbits.LATC13
#define CS_ABS_LAT      LATBbits.LATB13
#define CS_PDLU_LAT     LATGbits.LATG13
#define CS_PDLD_LAT     LATGbits.LATG14
#define CS_FAN_LAT      LATDbits.LATD12
#define CS_WTR_LAT      LATBbits.LATB6
#define CS_ECU_LAT      LATEbits.LATE3
#define CS_AUX_LAT      LATAbits.LATA10
#define CS_BVBAT_LAT    LATAbits.LATA4
#define CS_FUEL_TRIS    TRISBbits.TRISB2
#define CS_IGN_TRIS     TRISDbits.TRISD4
#define CS_INJ_TRIS     TRISCbits.TRISC13
#define CS_ABS_TRIS     TRISBbits.TRISB13
#define CS_PDLU_TRIS    TRISGbits.TRISG13
#define CS_PDLD_TRIS    TRISGbits.TRISG14
#define CS_FAN_TRIS     TRISDbits.TRISD12
#define CS_WTR_TRIS     TRISBbits.TRISB6
#define CS_ECU_TRIS     TRISEbits.TRISE3
#define CS_AUX_TRIS     TRISAbits.TRISA10
#define CS_BVBAT_TRIS   TRISAbits.TRISA4

// Pin definitions for !SW signal bus
#define SW1_PORT   PORTBbits.RB4
#define SW2_PORT   PORTEbits.RE9
#define SW3_PORT   PORTEbits.RE8
#define SW4_PORT   PORTAbits.RA0
#define SW5_PORT   PORTGbits.RG9
#define SW6_PORT   PORTGbits.RG8
#define SW7_PORT   PORTGbits.RG7
#define SW8_PORT   PORTCbits.RC4
#define SW9_PORT   PORTCbits.RC3
#define KILL_PORT  PORTCbits.RC2
#define SW1_TRIS   TRISBbits.TRISB4
#define SW2_TRIS   TRISEbits.TRISE9
#define SW3_TRIS   TRISEbits.TRISE8
#define SW4_TRIS   TRISAbits.TRISA0
#define SW5_TRIS   TRISGbits.TRISG9
#define SW6_TRIS   TRISGbits.TRISG8
#define SW7_TRIS   TRISGbits.TRISG7
#define SW8_TRIS   TRISCbits.TRISC4
#define SW9_TRIS   TRISCbits.TRISC3
#define KILL_TRIS  TRISCbits.TRISC2
#define SW1_ANSEL  ANSELBbits.ANSB4
#define SW2_ANSEL  ANSELEbits.ANSE9
#define SW3_ANSEL  ANSELEbits.ANSE8
#define SW4_ANSEL  ANSELAbits.ANSA0
#define SW5_ANSEL  ANSELGbits.ANSG9
#define SW6_ANSEL  ANSELGbits.ANSG8
#define SW7_ANSEL  ANSELGbits.ANSG7
#define SW8_ANSEL  ANSELCbits.ANSC4
#define SW9_ANSEL  ANSELCbits.ANSC3
#define KILL_ANSEL ANSELCbits.ANSC2

// MOSFET Current Ratios
#define CUR_RATIO   8800.0

// Current Scalar Inverse
#define SCL_INV      1000.0
#define SCL_INV_LRG  100.0
#define SCL_INV_CUT  400.0

// Initial overcurrent thresholds to use for all the loads
const double load_cutoff[NUM_CTL] = {
  10.0,  // FUEL
  100.0, // IGN
  100.0, // INJ
  50.0,  // ABS
  60.0,  // PDLU
  60.0,  // PDLD
  20.0,  // FAN
  15.0,  // WTR
  20.0,  // ECU
  2.0,   // AUX
  10.0   // BVBAT
};

// Initial peak-mode overcurrent thresholds to use for all the loads
const double load_peak_cutoff[NUM_CTL] = {
  40.0,  // FUEL
  0.0,   // IGN
  0.0,   // INJ
  0.0,   // ABS
  0.0,   // PDLU
  0.0,   // PDLD
  100.0, // FAN
  40.0,  // WTR
  60.0,  // ECU
  0.0,   // AUX
  0.0    // BVBAT
};

// External ADC channel indices for all the loads
const uint8_t ADC_CHN[NUM_LOADS] = {
  7,  // FUEL
  6,  // IGN
  4,  // INJ
  5,  // ABS
  8,  // PDLU
  9,  // PDLD
  10, // FAN
  11, // WTR
  3,  // ECU
  2,  // AUX
  1,  // BVBAT
  0   // STR
};

/**
 * Function definitions
 */

void main(void);

// Logic functions
void process_CAN_msg(CAN_message msg);
void debounce_switches(void);
void check_load_overcurrent(void);

// ADC sample functions
void sample_temp(void);
void sample_ext_adc(void);

// CAN message sending functions
void send_diag_can(void);
void send_diag_state_can(uint8_t override);
void send_rail_volt_can(void);
void send_load_current_can(void);
void send_cutoff_values_can(uint8_t override);
void send_overcrt_count_can(uint8_t override);

// Utility functions
double wpr_to_res(uint8_t wpr);
uint8_t res_to_wpr(double res);
uint8_t load_enabled(uint8_t load_idx);
void set_load(uint8_t load_idx, uint8_t load_state);

// Rheostat functions
void set_rheo(uint8_t load_idx, uint8_t val);
void send_all_rheo(uint16_t msg);
void init_rheo(void);

#endif /* PDM_H */
