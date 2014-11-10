/*
 * PDM Node Main File Header
 *
 * File Name:       PDM.h
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18
 * Author:          George Schwieters
 * Author:          Andrew Mass
 * Created:         2013-2014
 */

#ifndef PDM_H
#define PDM_H

#include "adc.h"

/*
 * Code Control
 */

//#define INTERNAL
#define MCHP_C18
#define CAN_KILL
#define CRIT_KILL
#define MAX_START

/*
 * Magic Numbers
 */

#define PWR_OFF 0
#define PWR_ON 1
#define NUM_LOADS 8

// Timing (ms)
#define PRIME_WAIT 500
#define CAN_PERIOD 500
#define CRIT_WAIT 2000
#define CRIT_WAIT_CAN 10000
#define START_WAIT 4000

// Peak current wait times (ms)
#define IGN_PEAK_WAIT 500
#define FUEL_PEAK_WAIT 500
#define WATER_PEAK_WAIT 500
#define START_PEAK_WAIT 500
#define FAN_PEAK_WAIT 500

// Critical error conditions
#define VOLTAGE_CRIT 1150       // 11.5 V
#define OIL_PRESS_CRIT_L 160    // 16.0 psi
#define OIL_PRESS_CRIT_H 250    // 25.0 psi
#define ENGINE_TEMP_CRIT 1150   // 115.0 C
#define OIL_TEMP_CRIT 2100      // 210.0 C

// Warning error conditions
#define VOLTAGE_WARN 1250       // 12.5 V
#define ENGINE_TEMP_WARN 1000   // 100.0 C
#define OIL_TEMP_WARN 1800      // 180.0 C

// Thresholds
#define RPM_ON_THRESHOLD 600    // 600 rpm
#define RPM_THRESHOLD_H 4000    // 4,000 rpm
#define RPM_THRESHOLD_L 1000    // 1,000 rpm
#define FAN_THRESHOLD_H 900     // 90.0 C
#define FAN_THRESHOLD_L 840     // 84.0 C

// Indices for arrays
#define IGN_val     0
#define FUEL_val    1
#define WATER_val   2
#define START_val   3
#define FAN_val     4
#define PCB_val     5
#define AUX_val     6
#define ECU_val     7
#define START_val_2 8
#define START_val_3 9

// Analog channel definitions
#define IGN_ch      ADC_CH2
#define FUEL_ch     ADC_CH3
#define WATER_ch    ADC_CH1
#define PCB_ch      ADC_CH0
#define ECU_ch      ADC_CH8
#define AUX_ch      ADC_CH9
#define FAN_ch      ADC_CH4
#define START_ch_3  ADC_CH7
#define START_ch_2  ADC_CH6
#define START_ch    ADC_CH5

/*
 * Pin Definitions
 */

#define IGN_LAT         LATDbits.LATD0
#define IGN_P_LAT       LATDbits.LATD1
#define FUEL_LAT        LATDbits.LATD2
#define FUEL_P_LAT      LATDbits.LATD3
#define FAN_LAT         LATDbits.LATD4
#define FAN_P_LAT       LATDbits.LATD5
#define START_LAT       LATDbits.LATD6
#define START_P_LAT     LATDbits.LATD7

#define AUX_LAT         LATCbits.LATC2
#define ECU_LAT         LATCbits.LATC3
#define PCB_LAT         LATCbits.LATC4
#define TERM_LAT        LATCbits.LATC5
#define WATER_LAT       LATCbits.LATC6
#define WATER_P_LAT     LATCbits.LATC7

#define IGN_PORT        PORTDbits.RD0
#define IGN_P_PORT      PORTDbits.RD1
#define FUEL_PORT       PORTDbits.RD2
#define FUEL_P_PORT     PORTDbits.RD3
#define FAN_PORT        PORTDbits.RD4
#define FAN_P_PORT      PORTDbits.RD5
#define START_PORT      PORTDbits.RD6
#define START_P_PORT    PORTDbits.RD7

#define AUX_PORT        PORTCbits.RC2
#define ECU_PORT        PORTCbits.RC3
#define PCB_PORT        PORTCbits.RC4
#define WATER_PORT      PORTCbits.RC6
#define WATER_P_PORT    PORTCbits.RC7

#define START_SW_PORT   PORTBbits.RB5
#define ON_SW_PORT      PORTBbits.RB0

/*
 * Function Definitions
 */

void high_isr(void);
void killCar(void);
void sample(int *data, const unsigned char index, const unsigned char ch);

#endif
