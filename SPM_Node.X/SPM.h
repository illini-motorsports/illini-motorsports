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
// Channel Defs

#define FREQ_BYP_0  4
#define FREQ_DIVA_0 3
#define FREQ_DIVB_0 2
#define FREQ_DIVC_0 1
#define FREQ_DIVD_0 0
#define FREQ_BYP_1  8
#define FREQ_DIVA_1 9
#define FREQ_DIVB_1 7
#define FREQ_DIVC_1 6
#define FREQ_DIVD_1 5
#define FREQ_BYP_2  14
#define FREQ_DIVA_2 13
#define FREQ_DIVB_2 12
#define FREQ_DIVC_2 11
#define FREQ_DIVD_2 10
#define FREQ_BYP_3  3
#define FREQ_DIVA_3 15
#define FREQ_DIVB_3 2
#define FREQ_DIVC_3 1
#define FREQ_DIVD_3 0

// ***************************
// GPIO Defs
// ***************************
// Pin numbers are created by: 16*CHIP_NUM + 8*(BANK=='B') + PIN_NUM
#define AD7490_0_CS_TRIS      TRISAbits.TRISA9
#define AD7490_1_CS_TRIS      TRISAbits.TRISA10
#define AD7490_0_CS_LAT       LATAbits.LATA9
#define AD7490_1_CS_LAT       LATAbits.LATA10
#define AD7490_0_CS_LATBITS   (uint32_t*) (&LATAbits)
#define AD7490_1_CS_LATBITS   (uint32_t*) (&LATAbits)
#define AD7490_0_CS_LATNUM    9
#define AD7490_1_CS_LATNUM    10

#define AD7680_0_CS_TRIS      TRISEbits.TRISE6
#define AD7680_1_CS_TRIS      TRISEbits.TRISE7
#define AD7680_2_CS_TRIS      TRISAbits.TRISA5
#define AD7680_3_CS_TRIS      TRISGbits.TRISG15
#define AD7680_0_CS_LAT       LATEbits.LATE6
#define AD7680_1_CS_LAT       LATEbits.LATE7
#define AD7680_2_CS_LAT       LATAbits.LATA5
#define AD7680_3_CS_LAT       LATGbits.LATG15

#define ANALOG_CAN_SCL        1000
#define THERMOCOUPLE_CAN_SCL  4
#define JUNCTION_CAN_SCL      16

#define MCP23S17_0_CS_TRIS    TRISAbits.TRISA15
#define MCP23S17_1_CS_TRIS    TRISAbits.TRISA4
#define MCP23S17_2_CS_TRIS    TRISAbits.TRISA3
#define MCP23S17_0_CS_LAT     LATAbits.LATA15
#define MCP23S17_1_CS_LAT     LATAbits.LATA4
#define MCP23S17_2_CS_LAT     LATAbits.LATA3
#define MCP23S17_0_CS_LATBITS	(uint32_t*) (&LATAbits)
#define MCP23S17_1_CS_LATBITS (uint32_t*) (&LATAbits)
#define MCP23S17_2_CS_LATBITS (uint32_t*) (&LATAbits)
#define MCP23S17_0_CS_LATNUM	15
#define MCP23S17_1_CS_LATNUM	4
#define MCP23S17_2_CS_LATNUM	3

#define MAX31855_0_CS_TRIS       TRISAbits.TRISA0
#define MAX31855_1_CS_TRIS       TRISEbits.TRISE8
#define MAX31855_2_CS_TRIS       TRISEbits.TRISE9
#define MAX31855_3_CS_TRIS       TRISBbits.TRISB4
#define MAX31855_4_CS_TRIS       TRISBbits.TRISB3
#define MAX31855_5_CS_TRIS       TRISBbits.TRISB2
#define MAX31855_0_CS_LAT        LATAbits.LATA0
#define MAX31855_1_CS_LAT        LATEbits.LATE8
#define MAX31855_2_CS_LAT        LATEbits.LATE9
#define MAX31855_3_CS_LAT        LATBbits.LATB4
#define MAX31855_4_CS_LAT        LATBbits.LATB3
#define MAX31855_5_CS_LAT        LATBbits.LATB2
#define MAX31855_0_CS_LATBITS    (uint32_t*) (&LATAbits)
#define MAX31855_1_CS_LATBITS    (uint32_t*) (&LATEbits)
#define MAX31855_2_CS_LATBITS    (uint32_t*) (&LATEbits)
#define MAX31855_3_CS_LATBITS    (uint32_t*) (&LATBbits)
#define MAX31855_4_CS_LATBITS    (uint32_t*) (&LATBbits)
#define MAX31855_5_CS_LATBITS    (uint32_t*) (&LATBbits)
#define MAX31855_0_CS_LATNUM     0
#define MAX31855_1_CS_LATNUM     8
#define MAX31855_2_CS_LATNUM     9
#define MAX31855_3_CS_LATNUM     4
#define MAX31855_4_CS_LATNUM     3
#define MAX31855_5_CS_LATNUM     2



uint8_t analogMappings[32] = {11,10,9,8,7,6,5,4,3,2,1,0,12,13,14,15,23,22,21,20,19,18,17,16,28,29,30,31,27,26,25,24};
uint8_t pgaMappings[12] = {4,5,6,9,8,7,12,11,10,15,14,13};

// Frequency output pins
#define FREQ_IN_0_TRIS TRISGbits.TRISG9
#define FREQ_IN_2_TRIS TRISFbits.TRISF1
#define FREQ_IN_3_TRIS TRISFbits.TRISF5

void main(void);
void update_analog_channels(void);
void update_thermocouples(void);
void update_digital_channels(void);

void init_gpio_ext();
void init_adcs();
void init_tcouples();
void set_pga(uint8_t chan, uint8_t level);
void set_freq_div(uint8_t chan, uint8_t div);
uint16_t ad7680_read_spi();

void CANAnalogChannels(void);
void CANThermocouples(void);

uint16_t set_bit_val(uint16_t current, uint8_t pos, uint8_t val);

#endif /* SPM_H */
