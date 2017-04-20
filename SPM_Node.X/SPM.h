#ifndef SPM_H
#define SPM_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/FSAE_spi.h"
#include "../FSAE.X/FSAE_ad7490.h"

// Enable programmable termination
//#define TERMINATING 0 //Redefinition issue

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 0

// ***************************
// High Speed ADC Defs
// ***************************
// SPI Defs
#define CS_HS_ADC_0 0
#define CS_HS_ADC_1 1
// Channel Defs

// ***************************
// Low Speed ADC Defs
// ***************************
// SPI Defs
#define CS_LS_ADC_0 0
#define CS_LS_ADC_1 1
#define CS_LS_ADC_2 2
#define CS_LS_ADC_3 3

// ***************************
// GPIO Defs
// ***************************
// SPI Defs
#define CS_GPIO_0 0
#define CS_GPIO_1 1
#define CS_GPIO_2 2

// Pin numbers are created by: 16*CHIP_NUM + 8*(BANK=='B') + PIN_NUM
#define FREQ_BYP_0 4
#define FREQ_DIVA_0 3
#define FREQ_DIVB_0 2
#define FREQ_DIVC_0 1
#define FREQ_DIVD_0 0
#define FREQ_BYP_1 8
#define FREQ_DIVA_1 9
#define FREQ_DIVB_1 7
#define FREQ_DIVC_1 6
#define FREQ_DIVD_1 5
#define FREQ_BYP_2 14
#define FREQ_DIVA_2 13
#define FREQ_DIVB_2 12
#define FREQ_DIVC_2 11
#define FREQ_DIVD_2 10
#define FREQ_BYP_3 19
#define FREQ_DIVA_3 15
#define FREQ_DIVB_3 18
#define FREQ_DIVC_3 17
#define FREQ_DIVD_3 16
#define PGA_0_0 20
#define PGA_0_1 21
#define PGA_0_2 22
#define PGA_1_0 25
#define PGA_1_1 24
#define PGA_1_2 23
#define PGA_2_0 28
#define PGA_2_1 27
#define PGA_2_2 26
#define PGA_3_0 31
#define PGA_3_1 30
#define PGA_3_2 29
#define DIGIN_0 32
#define DIGIN_1 33
#define DIGIN_2 34
#define DIGIN_3 35
#define DIGIN_4 36
#define DIGIN_5 37
#define DIGIN_6 38
#define DIGIN_7 39
#define DIGIN_8 40
#define DIGIN_9 41
#define DIGIN_10 42
#define DIGIN_11 43
#define DIGIN_12 44
#define DIGIN_13 45
#define DIGIN_14 46
#define DIGIN_15 47

uint8_t analogMappings[32] = {11,10,9,8,7,6,5,4,3,2,1,0,12,13,14,15,23,22,21,20,19,18,17,16,28,29,30,31,27,26,25,24};

void main(void);

#endif /* SPM_H */
