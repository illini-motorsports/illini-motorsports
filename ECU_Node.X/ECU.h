/**
 * ECU Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#pragma once

#include <sys/types.h>
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/errno.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_spi.h"

// Thresholds
#define RPM_ON_THRESHOLD 200.0 // rpm

// Timing constants (ms)
#define TEMP_SAMP_INTV     333
#define DIAG_SEND          1000

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)
#define CRANK_PERIODS 44

// Pin definitions

#define VR1_PORT  PORTCbits.RC2
#define VR1_TRIS  TRISCbits.TRISC2
#define VR1_ANSEL ANSELCbits.ANSC2
#define VR2_PORT  PORTCbits.RC3
#define VR2_TRIS  TRISCbits.TRISC3
#define VR2_ANSEL ANSELCbits.ANSC3

uint8_t dummy = 0;
#define INJ1_LAT dummy
#define INJ2_LAT dummy
#define INJ3_LAT dummy
#define INJ4_LAT dummy
#define IGN1_LAT dummy
#define IGN2_LAT dummy
#define IGN3_LAT dummy
#define IGN4_LAT dummy

#define INJ1_EN_MASK 0b00000000000000000000000000000001
#define INJ1_DS_MASK 0b00000000000000000000000000000010
#define INJ2_EN_MASK 0b00000000000000000000000000000100
#define INJ2_DS_MASK 0b00000000000000000000000000001000
#define INJ3_EN_MASK 0b00000000000000000000000000010000
#define INJ3_DS_MASK 0b00000000000000000000000000100000
#define INJ4_EN_MASK 0b00000000000000000000000001000000
#define INJ4_DS_MASK 0b00000000000000000000000010000000
#define IGN1_EN_MASK 0b00000000000000000000000100000000
#define IGN1_DS_MASK 0b00000000000000000000001000000000
#define IGN2_EN_MASK 0b00000000000000000000010000000000
#define IGN2_DS_MASK 0b00000000000000000000100000000000
#define IGN3_EN_MASK 0b00000000000000000001000000000000
#define IGN3_DS_MASK 0b00000000000000000010000000000000
#define IGN4_EN_MASK 0b00000000000000000100000000000000
#define IGN4_DS_MASK 0b00000000000000001000000000000000

/**
 * Function definitions
 */

void main(void);

// Logic functions
void process_CAN_msg(CAN_message msg);
void kill_engine(uint16_t errno);

// ADC sample functions
void sample_temp(void);

// CAN message sending functions
void send_diag_can(void);

// Utility functions
void init_adc_ecu(void);
