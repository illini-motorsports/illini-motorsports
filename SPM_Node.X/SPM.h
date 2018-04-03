#ifndef SPM_H
#define SPM_H

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
#define CS_HS_ADC_1 1

// ***************************
// GPIO Defs
// ***************************
// Pin numbers are created by: 16*CHIP_NUM + 8*(BANK=='B') + PIN_NUM
#define ADC_0_CS_TRIS           TRISAbits.TRISA9
#define ADC_1_CS_TRIS           TRISAbits.TRISA10
#define ADC_0_CS_LAT            LATAbits.LATA9
#define ADC_1_CS_LAT            LATAbits.LATA10
#define ADC_0_CS_LATBITS        (uint32_t*) (&LATAbits)
#define ADC_1_CS_LATBITS        (uint32_t*) (&LATAbits)
#define ADC_0_CS_LATNUM         9
#define ADC_1_CS_LATNUM         10

#define ANALOG_CAN_SCL          1000

#define GPIO_CS_TRIS            TRISAbits.TRISA15
#define GPIO_CS_LAT             LATAbits.LATA15
#define GPIO_CS_LATBITS         (uint32_t*) (&LATAbits)
#define GPIO_CS_LATNUM          15

uint8_t analogMappings[32] = {11,10,9,8,3,2,1,0,12,13,14,15,5,4,7,6,23,22,21,20,19,18,17,16,28,29,30,31,27,26,25};

void main(void);
void update_analog_channels(void);
void update_digital_channels(void);

void init_gpio();
void init_adcs();

void CANAnalogChannels(void);

#endif /* SPM_H */
