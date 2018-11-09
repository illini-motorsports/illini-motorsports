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
#include "../FSAE.X/FSAE_spi.h"
#include "../FSAE.X/FSAE_ad7490.h"
#include "../FSAE.X/FSAE_mcp23s17.h"
#include "../FSAE.X/FSAE_rheo.h"
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/errno.h"
#include "../FSAE.X/FSAE_pwm.h"

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
#define RPM_CRIT_CHECK   1000.0 // rpm
#define FAN_THRESHOLD_H  80.0  // C
#define FAN_THRESHOLD_L  70.0  // C
#define OVERCRT_DETECT   0.01  // A
#define CRIT_VOLTAGE     10000 // 10.0 V
#define CRIT_OILPRES     1.2   // bar
#define CRIT_OILTEMP     140.0 // C
#define CRIT_ENGTEMP     110.0 // C

// Timing constants (ms)
#define FUEL_PRIME_DUR     1000
#define STR_MAX_DUR        4000
#define PDL_MAX_DUR        500
#define OVERRIDE_SW_WAIT   5000
#define BASIC_CONTROL_WAIT 1000
#define MAX_IDLE_TIME      300000 // 5 mins

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

#define CRIT_VOLT_WAIT     1000
#define CRIT_OILPRES_WAIT  500
#define CRIT_OILTEMP_WAIT  5000
#define CRIT_ENGTEMP_WAIT  5000

// Raw (bouncy) switch state definitions
#define STR_SW_RAW    (!SW1_PORT)
#define ON_SW_RAW     (SW2_PORT)
#define ACT_UP_SW_RAW (!SW4_PORT)
#define ACT_DN_SW_RAW (!SW3_PORT)
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
#define EN_PDLU_LAT     LATGbits.LATG12
#define EN_PDLD_LAT     LATEbits.LATE2
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
#define EN_PDLU_PORT    PORTGbits.RG12
#define EN_PDLD_PORT    PORTEbits.RE2
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
#define EN_PDLU_TRIS    TRISGbits.TRISG12
#define EN_PDLD_TRIS    TRISEbits.TRISE2
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
#define CS_PDLU_LAT     LATGbits.LATG14
#define CS_PDLD_LAT     LATGbits.LATG13
#define CS_FAN_LAT      LATDbits.LATD12
#define CS_WTR_LAT      LATBbits.LATB6
#define CS_ECU_LAT      LATEbits.LATE3
#define CS_AUX_LAT      LATAbits.LATA10
#define CS_BVBAT_LAT    LATAbits.LATA4
#define CS_FUEL_TRIS    TRISBbits.TRISB2
#define CS_IGN_TRIS     TRISDbits.TRISD4
#define CS_INJ_TRIS     TRISCbits.TRISC13
#define CS_ABS_TRIS     TRISBbits.TRISB13
#define CS_PDLU_TRIS    TRISGbits.TRISG14
#define CS_PDLD_TRIS    TRISGbits.TRISG13
#define CS_FAN_TRIS     TRISDbits.TRISD12
#define CS_WTR_TRIS     TRISBbits.TRISB6
#define CS_ECU_TRIS     TRISEbits.TRISE3
#define CS_AUX_TRIS     TRISAbits.TRISA10
#define CS_BVBAT_TRIS   TRISAbits.TRISA4

#define CS_FUEL_LATBITS   (uint32_t*) (&LATBbits)
#define CS_IGN_LATBITS    (uint32_t*) (&LATDbits)
#define CS_INJ_LATBITS    (uint32_t*) (&LATCbits)
#define CS_ABS_LATBITS    (uint32_t*) (&LATBbits)
#define CS_PDLU_LATBITS   (uint32_t*) (&LATGbits)
#define CS_PDLD_LATBITS   (uint32_t*) (&LATGbits)
#define CS_FAN_LATBITS    (uint32_t*) (&LATDbits)
#define CS_WTR_LATBITS    (uint32_t*) (&LATBbits)
#define CS_ECU_LATBITS    (uint32_t*) (&LATEbits)
#define CS_AUX_LATBITS    (uint32_t*) (&LATAbits)
#define CS_BVBAT_LATBITS  (uint32_t*) (&LATAbits)
#define CS_FUEL_LATNUM    2
#define CS_IGN_LATNUM     4
#define CS_INJ_LATNUM     13
#define CS_ABS_LATNUM     13
#define CS_PDLU_LATNUM    14
#define CS_PDLD_LATNUM    13
#define CS_FAN_LATNUM     12
#define CS_WTR_LATNUM     6
#define CS_ECU_LATNUM     3
#define CS_AUX_LATNUM     10
#define CS_BVBAT_LATNUM   4

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

// AD7490 CS Pin definitions
// Cast to uint32 pointer to more easily pass into functions
#define CS_AD7490_LATBITS   ((uint32_t*) (&LATAbits))
#define CS_AD7490_LATNUM    1

// MCP23s17 CS Pin definitions
#define CS_GPIO_LAT         LATEbits.LATE7
#define CS_GPIO_TRIS        TRISEbits.TRIS7
#define CS_GPIO_LATBITS     ((uint32_t*) (&LATEbits))
#define CS_GPIO_LATNUM      7

// Current Scalar Inverse
#define SCL_INV      1000.0
#define SCL_INV_LRG  100.0
#define SCL_INV_CUT  400.0

// Misc Magic Numbers
#define EXT_ADC_NUM_STEPS   4095.0
#define EXT_ADC_VOLT_RANGE  5.0
#define VFB_CUTOFF          4.7

// Initial overcurrent thresholds to use for all the loads
const double load_cutoff[NUM_CTL] = {
  10.0,  // FUEL
  60.0,  // IGN
  60.0,  // INJ
  100.0, // ABS
  60.0,  // PDLU
  60.0,  // PDLD
  20.0,  // FAN
  15.0,  // WTR
  20.0,  // ECU
  2.0,   // AUX
  20.0   // BVBAT
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

// Duration to remain in peak-mode for all the loads
const uint32_t load_peak_duration[NUM_LOADS] = {
  150,  // FUEL
  0,    // IGN
  0,    // INJ
  0,    // ABS
  0,    // PDLU
  0,    // PDLD
  2000, // FAN
  250,  // WTR
  10,   // ECU
  0,    // AUX
  0     // BVBAT
};

// External ADC channel indices for all the loads
const uint8_t ADC_CHN[NUM_LOADS] = {
  7,  // FUEL (LK1)
  6,  // IGN  (LK2)
  4,  // INJ  (LK3)
  5,  // ABS  (L1)
  9,  // PDLU (L3)
  8,  // PDLD (L2)
  10, // FAN  (L4)
  11, // WTR  (L5)
  3,  // ECU  (L6)
  2,  // AUX  (L7)
  1,  // BVBAT(LB1)
  0   // STR  (LH1)
};

const uint16_t load_current_ratios[NUM_LOADS] = {
  5300,  // FUEL (AUIR3314)
  5300,  // IGN  (AUIR3314)
  5300,  // INJ  (AUIR3314)
  8800,  // ABS  (AUIR3313)
  8800,  // PDLU (AUIR3313)
  8800,  // PDLD (AUIR3313)
  8800,  // FAN  (AUIR3313)
  5300,  // WTR  (AUIR3314)
  8800,  // ECU  (AUIR3313)
  2800,  // AUX  (AUIR3315)
  5300,  // BVBAT(AUIR3314)
  30000  // STR  (BTS555)
};

// External GPIO channel indices for all the loads
const uint8_t GPIO_CHN[NUM_LOADS] = {
  4,  // FUEL
  5,  // IGN
  6,  // INJ
  7,  // ABS
  9,  // PDLU
  8,  // PDLD
  10, // FAN
  11, // WTR
  12, // ECU
  13, // AUX
  14, // BVBAT
  15  // STR
};

/**
 * Function definitions
 */

void main(void);

// Logic functions
void process_CAN_msg(CAN_message msg);
void debounce_switches(void);
void check_peak_timer(void);
void check_load_overcurrent(void);
void prevent_engine_blowup(void);

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
void enable_load(uint8_t load_idx);
void disable_load(uint8_t load_idx);
void set_load(uint8_t load_idx, uint8_t condition);
double wpr_to_res(uint8_t wpr);
uint8_t res_to_wpr(double res);
double adc_to_volt(uint16_t adc_val);
uint8_t load_enabled(uint8_t load_idx);
void set_en_load(uint8_t load_idx, uint8_t load_state);

// Initialize Rheostats
void init_rheostats(void);

#endif /* PDM_H */
