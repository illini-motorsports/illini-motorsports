/* 
 * File:   SGH.h
 * Author: Jacob Drewniak
 *
 * Created on May 1, 2018, 11:56 PM
 */

#ifndef SGH_H
#define SGH_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/FSAE_spi.h"
#include "../FSAE.X/FSAE_ad7490.h"
#include "../FSAE.X/FSAE_mcp23s17.h"
#include "../FSAE.X/FSAE_max31855.h"

// ***************************
// High Speed ADC Defs
// ***************************
// SPI Defs
#define CS_HS_ADC_0 0

// ***************************
// GPIO Defs
// ***************************
// Pin numbers are created by: 16*CHIP_NUM + 8*(BANK=='B') + PIN_NUM
#define ADC_0_CS_TRIS           TRISAbits.TRISA10
#define ADC_0_CS_LAT            LATAbits.LATA10
#define ADC_0_CS_LATBITS        (uint32_t*) (&LATAbits)
#define ADC_0_CS_LATNUM         10

#define GAIN                    304.03
#define GAUGE_FACTOR            2.155
#define VDD                     12
#define POISSON                 0.30

#define TEMP_SAMP_INTV    333

uint8_t analogMappings[8] = {4,5,7,6,1,0,2,3};

void main(void);
void update_analog_channels(void);
void strain_calc(void);

void init_adcs();

void CANdiag(void);
void CANAnalogChannels(void);

void sample_temp(void);

#endif /* SGH_H */

