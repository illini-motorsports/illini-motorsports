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
#include "../FSAE_32/FSAE_adc_32.h"
#include "../FSAE_32/FSAE_nvm_32.h"
#include "../FSAE.X/CAN.h"

// Enable programmable termination
#define TERMINATING 1

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 0

// Determines whether SYSCLK / 10 is driven out on RF0
#define REFCLKO 0

// Defines for MOSFET control
#define PWR_ON  1
#define PWR_OFF 0

// Define for the number of MOSFET control circuits
#define NUM_LOADS 14

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

// Thresholds
#define RPM_ON_THRESHOLD 200.0 // rpm
#define FAN_THRESHOLD_H  90.0  // C
#define FAN_THRESHOLD_L  84.0  // C

// Timing constants (ms)

#define FUEL_PRIME_DUR     500
#define STR_MAX_DUR        4000
#define STR_PULSE_DUR      100
#define MAX_PDL_DUR        500

#define BASIC_CONTROL_WAIT 1000
#define TEMP_SAMP_INTV     333
#define DIAG_MSG_SEND      1000
#define LOAD_CUR_SEND      10
#define RAIL_VOLT_SEND     50
#define CUTOFF_VAL_SEND    1000
#define LOAD_STATUS_SEND   500

#define FUEL_PEAK_DUR      100
#define WTR_PEAK_DUR       100
#define FAN_PEAK_DUR       100
#define ECU_PEAK_DUR       20

// Switch state definitions
#define STR_SW    (!SW1_PORT)
#define ON_SW     (SW2_PORT)
#define ACT_UP_SW (SW3_PORT)
#define ACT_DN_SW (SW4_PORT)
#define KILL_SW   (SW5_PORT)

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Load state definitions
//#define IGN_EN   (EN_IGN_PORT == PWR_ON)
#define IGN_EN   (ign_enabled)
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
#define STR_EN   (EN_STR_PORT == PWR_ON || str_pulse_flag)

// Definitions for interval override control
#define OVERRIDE    1
#define NO_OVERRIDE 0

// Pin definitions for EN signal bus
#define EN_IGN_LAT    LATDbits.LATD15
#define EN_INJ_LAT    LATCbits.LATC15
#define EN_FUEL_LAT   LATGbits.LATG0
#define EN_ECU_LAT    LATFbits.LATF2
#define EN_WTR_LAT    LATFbits.LATF8
#define EN_FAN_LAT    LATAbits.LATA3
#define EN_AUX_LAT    LATAbits.LATA4
#define EN_PDLU_LAT   LATFbits.LATF4
#define EN_PDLD_LAT   LATFbits.LATF5
#define EN_B5V5_LAT   LATAbits.LATA14
#define EN_BVBAT_LAT  LATAbits.LATA15
#define EN_STR_LAT    LATDbits.LATD9
//#define EN_IGN_PORT   PORTDbits.RD15
#define EN_INJ_PORT   PORTCbits.RC15
#define EN_FUEL_PORT  PORTGbits.RG0
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
#define EN_FUEL_TRIS  TRISGbits.TRISG0
#define EN_ECU_TRIS   TRISFbits.TRISF2
#define EN_WTR_TRIS   TRISFbits.TRISF8
#define EN_FAN_TRIS   TRISAbits.TRISA3
#define EN_AUX_TRIS   TRISAbits.TRISA4
#define EN_PDLU_TRIS  TRISFbits.TRISF4
#define EN_PDLD_TRIS  TRISFbits.TRISF5
#define EN_B5V5_TRIS  TRISAbits.TRISA14
#define EN_BVBAT_TRIS TRISAbits.TRISA15
#define EN_STR_TRIS   TRISDbits.TRISD9

// Pin defnition for unused EN_FUEL signal (due to 1V silicon bug)
#define EN_FUEL_UNUSED_TRIS TRISFbits.TRISF3

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
//#define SW8_PORT   PORTGbits.RG0
#define SW9_PORT   PORTAbits.RA6
#define KILL_PORT  PORTAbits.RA7
#define SW1_TRIS   TRISDbits.TRISD12
#define SW2_TRIS   TRISDbits.TRISD13
#define SW3_TRIS   TRISDbits.TRISD4
#define SW4_TRIS   TRISDbits.TRISD5
#define SW5_TRIS   TRISFbits.TRISF0
#define SW6_TRIS   TRISFbits.TRISF1
#define SW7_TRIS   TRISGbits.TRISG1
//#define SW8_TRIS   TRISGbits.TRISG0
#define SW9_TRIS   TRISAbits.TRISA6
#define KILL_TRIS  TRISAbits.TRISA7

// Pin definitions for ADC signal bus
#define ADC_IGN_TRIS    TRISGbits.TRISG15
#define ADC_INJ_TRIS    TRISAbits.TRISA5
#define ADC_FUEL_TRIS   TRISEbits.TRISE5
#define ADC_ECU_TRIS    TRISEbits.TRISE6
#define ADC_WTR_TRIS    TRISEbits.TRISE7
#define ADC_FAN_TRIS    TRISCbits.TRISC1
#define ADC_AUX_TRIS    TRISCbits.TRISC2
#define ADC_PDLU_TRIS   TRISCbits.TRISC3
#define ADC_PDLD_TRIS   TRISCbits.TRISC4
#define ADC_B5V5_TRIS   TRISGbits.TRISG6
#define ADC_BVBAT_TRIS  TRISGbits.TRISG7
#define ADC_STR0_TRIS   TRISGbits.TRISG8
#define ADC_STR1_TRIS   TRISGbits.TRISG9
#define ADC_STR2_TRIS   TRISAbits.TRISA0
#define ADC_3V3_TRIS    TRISFbits.TRISF12
#define ADC_5V_TRIS     TRISBbits.TRISB12
#define ADC_5V5_TRIS    TRISBbits.TRISB13
#define ADC_12V_TRIS    TRISBbits.TRISB14
#define ADC_VBAT_TRIS   TRISBbits.TRISB15
#define ADC_PTEMP_TRIS   TRISBbits.TRISB11
#define ADC_IGN_ANSEL   ANSELGbits.ANSG15
#define ADC_INJ_ANSEL   ANSELAbits.ANSA5
#define ADC_FUEL_ANSEL  ANSELEbits.ANSE5
#define ADC_ECU_ANSEL   ANSELEbits.ANSE6
#define ADC_WTR_ANSEL   ANSELEbits.ANSE7
#define ADC_FAN_ANSEL   ANSELCbits.ANSC1
#define ADC_AUX_ANSEL   ANSELCbits.ANSC2
#define ADC_PDLU_ANSEL  ANSELCbits.ANSC3
#define ADC_PDLD_ANSEL  ANSELCbits.ANSC4
#define ADC_B5V5_ANSEL  ANSELGbits.ANSG6
#define ADC_BVBAT_ANSEL ANSELGbits.ANSG7
#define ADC_STR0_ANSEL  ANSELGbits.ANSG8
#define ADC_STR1_ANSEL  ANSELGbits.ANSG9
#define ADC_STR2_ANSEL  ANSELAbits.ANSA0
#define ADC_3V3_ANSEL   ANSELFbits.ANSF12
#define ADC_5V_ANSEL    ANSELBbits.ANSB12
#define ADC_5V5_ANSEL   ANSELBbits.ANSB13
#define ADC_12V_ANSEL   ANSELBbits.ANSB14
#define ADC_VBAT_ANSEL  ANSELBbits.ANSB15
#define ADC_PTEMP_ANSEL  ANSELBbits.ANSB11
#define ADC_IGN_CSS     ADCCSS1bits.CSS23
#define ADC_INJ_CSS     ADCCSS2bits.CSS34
#define ADC_FUEL_CSS    ADCCSS1bits.CSS17
#define ADC_ECU_CSS     ADCCSS1bits.CSS16
#define ADC_WTR_CSS     ADCCSS1bits.CSS15
#define ADC_FAN_CSS     ADCCSS1bits.CSS22
#define ADC_AUX_CSS     ADCCSS1bits.CSS21
#define ADC_PDLU_CSS    ADCCSS1bits.CSS20
#define ADC_PDLD_CSS    ADCCSS1bits.CSS19
#define ADC_B5V5_CSS    ADCCSS1bits.CSS14
#define ADC_BVBAT_CSS   ADCCSS1bits.CSS13
#define ADC_STR0_CSS    ADCCSS1bits.CSS12
#define ADC_STR1_CSS    ADCCSS1bits.CSS11
#define ADC_STR2_CSS    ADCCSS1bits.CSS24
#define ADC_3V3_CSS     ADCCSS1bits.CSS31
#define ADC_5V_CSS      ADCCSS1bits.CSS7
#define ADC_5V5_CSS     ADCCSS1bits.CSS8
#define ADC_12V_CSS     ADCCSS1bits.CSS9
#define ADC_VBAT_CSS    ADCCSS1bits.CSS10
#define ADC_PTEMP_CSS    ADCCSS1bits.CSS6
#define ADC_JTEMP_CSS   ADCCSS2bits.CSS44
//#define ADC_IGN_TRG     ADCTRG1bits.TRGSRC23
//#define ADC_FUEL_TRG    ADCTRG1bits.TRGSRC17
//#define ADC_ECU_TRG     ADCTRG1bits.TRGSRC16
//#define ADC_WTR_TRG     ADCTRG1bits.TRGSRC15
//#define ADC_FAN_TRG     ADCTRG1bits.TRGSRC22
//#define ADC_AUX_TRG     ADCTRG1bits.TRGSRC21
//#define ADC_PDLU_TRG    ADCTRG1bits.TRGSRC20
//#define ADC_PDLD_TRG    ADCTRG1bits.TRGSRC19
//#define ADC_B5V5_TRG    ADCTRG1bits.TRGSRC14
//#define ADC_BVBAT_TRG   ADCTRG3bits.TRGSRC13
//#define ADC_STR0_TRG    ADCTRG4bits.TRGSRC12
#define ADC_STR1_TRG    ADCTRG3bits.TRGSRC11
//#define ADC_STR2_TRG    ADCTRG1bits.TRGSRC24
//#define ADC_3V3_TRG     ADCTRG4bits.TRGSRC31
#define ADC_5V_TRG      ADCTRG2bits.TRGSRC7
#define ADC_5V5_TRG     ADCTRG3bits.TRGSRC8
#define ADC_12V_TRG     ADCTRG3bits.TRGSRC9
#define ADC_VBAT_TRG    ADCTRG3bits.TRGSRC10
#define ADC_PTEMP_TRG    ADCTRG2bits.TRGSRC6
//#define ADC_JTEMP_TRG   ADCTRG3bits.TRGSRC44

// ADC channel definitions
#define ADC_IGN_CHN     23
#define ADC_INJ_CHN     34
#define ADC_FUEL_CHN    17
#define ADC_ECU_CHN     16
#define ADC_WTR_CHN     15
#define ADC_FAN_CHN     22
#define ADC_AUX_CHN     21
#define ADC_PDLU_CHN    20
#define ADC_PDLD_CHN    19
#define ADC_B5V5_CHN    14
#define ADC_BVBAT_CHN   13
#define ADC_STR0_CHN    12
#define ADC_STR1_CHN    11
#define ADC_STR2_CHN    24
#define ADC_3V3_CHN     31
#define ADC_5V_CHN      7
#define ADC_5V5_CHN     8
#define ADC_12V_CHN     9
#define ADC_VBAT_CHN    10
#define ADC_PTEMP_CHN    6
#define ADC_JTEMP_CHN   44

// MOSFET Current Ratios
#define IGN_RATIO   5300.0
#define INJ_RATIO   2800.0
#define FUEL_RATIO  8800.0
#define ECU_RATIO   8800.0
#define WTR_RATIO   5300.0
#define FAN_RATIO   8800.0
#define AUX_RATIO   5300.0
#define PDLU_RATIO  8800.0
#define PDLD_RATIO  8800.0
#define B5V5_RATIO  2800.0
#define BVBAT_RATIO 2800.0
#define STR0_RATIO  8800.0
#define STR1_RATIO  8800.0
#define STR2_RATIO  8800.0

// Load Scalar Inverse
#define IGN_SCLINV   1000.0
#define INJ_SCLINV   1000.0
#define FUEL_SCLINV  1000.0
#define ECU_SCLINV   1000.0
#define WTR_SCLINV   1000.0
#define FAN_SCLINV   1000.0
#define AUX_SCLINV   1000.0
#define PDLU_SCLINV  1000.0
#define PDLD_SCLINV  1000.0
#define B5V5_SCLINV  1000.0
#define BVBAT_SCLINV 1000.0
#define STR0_SCLINV  100.0
#define STR1_SCLINV  100.0
#define STR2_SCLINV  100.0
#define STR_SCLINV   100.0
#define TOTAL_SCLINV 100.0
#define CUT_SCLINV   400.0

// Constant used to check whether the NVM has been initialized
#define NVM_WPR_CONSTANT 0xDEADBEEF

/**
 * Functions to convert a wiper value to the expected resistance of the rheostat
 * and vice versa.
 *
 * Luckily, the conversion is mostly linear, so a linear regression approximates
 * the value well. These values were determined experimentally and should give a
 * good general idea of the FB pin resistance, but there will be some error. The
 * error between measured and calculated current cut-off due to this error is
 * limited to ~0.5A worst case, and this is at the high end of the cut-off range.
 * So, the error should not be a problem. Refer to the "Digital Rheostat V+
 * Selection" tab of the "PCB Info" document for more info.
 */
#define WPR_TO_RES(wpr) ((19.11639223 * (wpr)) + 256.6676635)
#define RES_TO_WPR(res) (0.0523111 * ((res) - 256.6676635))

/**
 * Struct representing the layout of wiper value data in non-volatile memory. The
 * key value is a unique constant that we can use to check whether the NVM has
 * been initialized by the PDM.
 */
typedef struct {
  uint32_t key;

  uint8_t ign_wpr_val;
  uint8_t inj_wpr_val;
  uint8_t fuel_wpr_val;
  uint8_t ecu_wpr_val;
  uint8_t wtr_wpr_val;
  uint8_t fan_wpr_val;
  uint8_t aux_wpr_val;
  uint8_t pdlu_wpr_val;
  uint8_t pdld_wpr_val;
  uint8_t b5v5_wpr_val;
  uint8_t bvbat_wpr_val;
  uint8_t str0_wpr_val;
  uint8_t str1_wpr_val;
  uint8_t str2_wpr_val;

  uint8_t fuel_peak_wpr_val;
  uint8_t wtr_peak_wpr_val;
  uint8_t fan_peak_wpr_val;
} Wiper_nvm_data;

// Function definitions
void main(void);
void process_CAN_msg(CAN_message msg);
void send_diag_can(void);
void sample_temp(void);
void send_load_current_can(void);
void send_rail_volt_can(void);
void send_cutoff_values_can(uint8_t override);
void send_load_status_can(uint8_t override);
void set_rheo(uint8_t load_idx, uint8_t val);
void send_all_rheo(uint16_t msg);
void init_adc_pdm(void);
void set_current_cutoff(uint8_t load_idx, uint8_t peak_mode, double cutoff);

#endif /* PDM_H */
