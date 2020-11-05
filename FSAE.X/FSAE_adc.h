/**
 * FSAE Library ADC Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_adc_H
#define FSAE_adc_H

#include "FSAE_config.h"
#include <sys/types.h>
#include <xc.h>

// ADCTRGx Settings
#define NO_TRIGGER 0b00000
#define SOFTWARE 0b00001
#define SCAN_TRIGGER 0b11 // TODO: This is not right

// Pin definitions for PCB temperature sensor
#define ADC_PTEMP_CHN 6
#define ADC_PTEMP_TRIS TRISBbits.TRISB11
#define ADC_PTEMP_ANSEL ANSELBbits.ANSB11
#define ADC_PTEMP_CSS ADCCSS1bits.CSS6
#define ADC_PTEMP_TRG ADCTRG2bits.TRGSRC6

// Pin definitions for junction temperature sensor
#define ADC_JTEMP_CHN 44
#define ADC_JTEMP_CSS ADCCSS2bits.CSS44
//#define ADC_JTEMP_TRG   ADCTRG3bits.TRGSRC44

// Function definitions
void init_adc(void (*specific_init)(void));

#endif /* FSAE_adc_H */
