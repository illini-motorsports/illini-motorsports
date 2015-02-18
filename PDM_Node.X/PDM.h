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
#define CRIT_KILL
#define MAX_START
#define BASIC_CONTROL 1

/*
 * Magic Numbers
 */

#define PWR_OFF 0
#define PWR_ON 1
#define NUM_LOADS 7

// Timing (ms)
#define PRIME_WAIT 500
#define CAN_PERIOD 500
#define START_WAIT 4000
#define BASIC_CONTROL_WAIT 1000

// Critical kill wait timers (ms)
#define CRIT_WAIT_VOLTAGE 2000
#define CRIT_WAIT_OIL_PRESS 500
#define CRIT_WAIT_TEMP 5000

// Peak current wait times (ms)
#define ECU_PEAK_WAIT 500
#define FUEL_PEAK_WAIT 500
#define WATER_PEAK_WAIT 500
#define START_PEAK_WAIT 500
#define FAN_PEAK_WAIT 500

// Critical error conditions
#define VOLTAGE_CRIT 1100       // 11.0 V
#define OIL_PRESS_CRIT_L 160    // 16.0 psi
#define OIL_PRESS_CRIT_H 250    // 25.0 psi
#define ENGINE_TEMP_CRIT 1150   // 115.0 C
#define OIL_TEMP_CRIT 1800      // 180.0 C

// Thresholds
#define RPM_ON_THRESHOLD 600    // 600 rpm
#define RPM_THRESHOLD_H 4000    // 4,000 rpm
#define RPM_THRESHOLD_L 1000    // 1,000 rpm
#define FAN_THRESHOLD_H 900     // 90.0
#define FAN_THRESHOLD_L 840     // 84.0

// Indices for arrays
#define ECU_val     0
#define FUEL_val    1
#define WATER_val   2
#define START_val   3
#define FAN_val     4
#define PCB_val     5
#define AUX_val     6
#define START_val_2 7
#define START_val_3 8

// Analog channel definitions
#define START_ch    ADC_CH0
#define START_ch_2  ADC_CH1
#define START_ch_3  ADC_CH3
#define WATER_ch    ADC_CH4
#define AUX_ch      ADC_CH5
#define FAN_ch      ADC_CH6
#define ECU_ch      ADC_CH7
#define PCB_ch      ADC_CH8
#define FUEL_ch     ADC_CH9

/*
 * Pin Definitions
 */

#define ECU_LAT         LATDbits.LATD0
#define ECU_P_LAT       LATDbits.LATD1
#define FUEL_LAT        LATDbits.LATD2
#define FUEL_P_LAT      LATDbits.LATD3
#define FAN_LAT         LATDbits.LATD4
#define FAN_P_LAT       LATDbits.LATD5
#define START_LAT       LATDbits.LATD6
#define START_P_LAT     LATDbits.LATD7
#define WATER_LAT       LATCbits.LATC6
#define WATER_P_LAT     LATCbits.LATC7

#define AUX_LAT         LATCbits.LATC2
#define PCB_LAT         LATCbits.LATC4
#define TERM_LAT        LATCbits.LATC5

#define ECU_PORT        PORTDbits.RD0
#define ECU_P_PORT      PORTDbits.RD1
#define FUEL_PORT       PORTDbits.RD2
#define FUEL_P_PORT     PORTDbits.RD3
#define FAN_PORT        PORTDbits.RD4
#define FAN_P_PORT      PORTDbits.RD5
#define START_PORT      PORTDbits.RD6
#define START_P_PORT    PORTDbits.RD7
#define WATER_PORT      PORTCbits.RC6
#define WATER_P_PORT    PORTCbits.RC7

#define AUX_PORT        PORTCbits.RC2
#define PCB_PORT        PORTCbits.RC4

#define START_SW_PORT   PORTBbits.RB5
#define ON_SW_PORT      PORTBbits.RB0

/*
 * Function Definitions
 */

void high_isr(void);
void killCar(void);
void sample(int *data, const unsigned char index, const unsigned char ch);

#endif
