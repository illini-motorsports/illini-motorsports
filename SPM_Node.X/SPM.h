#ifndef SPM_H
#define SPM_H

#include "../FSAE.X/CAN.h"
#include "../FSAE.X/FSAE_ad7490.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_max31855.h"
#include "../FSAE.X/FSAE_mcp23s17.h"
#include "../FSAE.X/FSAE_spi.h"
#include <sys/types.h>

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
#define ADC_0_CS_TRIS TRISAbits.TRISA9
#define ADC_1_CS_TRIS TRISAbits.TRISA10
#define ADC_2_CS_TRIS TRISCbits.TRISC3
#define ADC_3_CS_TRIS TRISCbits.TRISC4
#define ADC_0_CS_LAT LATAbits.LATA9
#define ADC_1_CS_LAT LATAbits.LATA10
#define ADC_2_CS_LAT LATCbits.LATC3
#define ADC_3_CS_LAT LATCbits.LATC4
#define ADC_0_CS_LATBITS (uint32_t *)(&LATAbits)
#define ADC_1_CS_LATBITS (uint32_t *)(&LATAbits)
#define ADC_2_CS_LATBITS (uint32_t *)(&LATCbits)
#define ADC_3_CS_LATBITS (uint32_t *)(&LATCbits)
#define ADC_0_CS_LATNUM 9
#define ADC_1_CS_LATNUM 10
#define ADC_2_CS_LATNUM 3
#define ADC_3_CS_LATNUM 4

#define ANALOG_CAN_SCL 1000

#define GPIO_CS_TRIS TRISAbits.TRISA15
#define GPIO_CS_LAT LATAbits.LATA15
#define GPIO_CS_LATBITS (uint32_t *)(&LATAbits)
#define GPIO_CS_LATNUM 15

#define THERMOCOUPLE_CS_LAT1 LATEbits.LATE8
#define THERMOCOUPLE_CS_LAT2 LATEbits.LATE9
#define THERMOCOUPLE_CS_LAT3 LATBbits.LATB4
#define THERMOCOUPLE_CS_LAT4 LATBbits.LATB2

#define THERMOCOUPLE_CS_TRIS1 TRISEbits.TRISE8
#define THERMOCOUPLE_CS_TRIS2 TRISEbits.TRISE9
#define THERMOCOUPLE_CS_TRIS3 TRISBbits.TRISB4
#define THERMOCOUPLE_CS_TRIS4 TRISBbits.TRISB2

#define THERMOCOUPLE_CS_LATBITS1 (uint32_t *)(&LATEbits)
#define THERMOCOUPLE_CS_LATBITS2 (uint32_t *)(&LATEbits)
#define THERMOCOUPLE_CS_LATBITS3 (uint32_t *)(&LATBbits)
#define THERMOCOUPLE_CS_LATBITS4 (uint32_t *)(&LATBbits)

#define THERMOCOUPLE_CS_LATNUM1 8
#define THERMOCOUPLE_CS_LATNUM2 9
#define THERMOCOUPLE_CS_LATNUM3 4
#define THERMOCOUPLE_CS_LATNUM4 2

#define TEMP_SAMP_INTV 333
#define CAN_ANALOG_INTV 50
#define CAN_DIAG_INTV 250
#define TIRE_DIAMETER_PER_TR 0.267035
#define SPI_DIFF_TIME 0.0000836

uint8_t analogMappings[32] = {27, 26, 0,  28, 7,  6,  4,  25, 24, 0,  12,
                              13, 14, 15, 5,  4,  7,  6,  23, 22, 21, 20,
                              19, 18, 17, 16, 11, 10, 25, 5,  9,  8};

double tempRange[20] = {-40, -30, -20, -10, 0,   10,  20,  30,  40,  50,
                        60,  70,  80,  90,  100, 110, 120, 130, 140, 150};
double resRange[20] = {45313, 26114, 15462, 9397, 5896, 3792, 2500,
                       1707,  1175,  834,   596,  436,  323,  243,
                       187,   144,   113,   89,   71,   57};

void main(void);
void update_analog_channels(void);
void update_digital_channels(void);
void update_strain_channels(void);
void read_thermocouples(void);

void init_gpio();
void init_adcs();
void init_thermocouples();

void CANAnalogChannels(void);
void CANDiag(void);

void sample_temp(void);
void process_CAN_msg(CAN_message msg);

void calc_wheel_speeds(void);
int32_t linearizeThermistor(double inTherm);

#endif /* SPM_H */