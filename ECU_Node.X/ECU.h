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

// Pin definitions

#define VR1_PORT  PORTCbits.RC2
#define VR1_TRIS  TRISCbits.TRISC2
#define VR1_ANSEL ANSELCbits.ANSC2
#define VR2_PORT  PORTCbits.RC3
#define VR2_TRIS  TRISCbits.TRISC3
#define VR2_ANSEL ANSELCbits.ANSC3

/**
 * Function definitions
 */

void main(void);

// Logic functions
void process_CAN_msg(CAN_message msg);

// ADC sample functions
void sample_temp(void);

// CAN message sending functions
void send_diag_can(void);

// Utility functions
void init_adc_ecu(void);
