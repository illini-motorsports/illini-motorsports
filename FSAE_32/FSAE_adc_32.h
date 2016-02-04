/**
 * FSAE Library 32bit ADC Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_adc_32_H
#define FSAE_adc_32_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config_32.h"

// ANSEL Settings
#define AN_INPUT  1
#define DIG_INPUT 0

// ADCTRGx Settings
#define NO_TRIGGER   0b00
#define SOFTWARE     0b01
#define SCAN_TRIGGER 0b11

// Pin definitions for temperature sensor
#define TEMP_TRIS  TRISBbits.TRISB11
#define TEMP_ANSEL ANSELBbits.ANSB11
#define TEMP_CSS   ADCCSS1bits.CSS6

// Function definitions
void init_adc(void (*specific_init)(void));

#endif /* FSAE_adc_32_H */
