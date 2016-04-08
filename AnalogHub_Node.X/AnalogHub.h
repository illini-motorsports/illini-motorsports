/**
 * AnalogHub Header
 *
 * Processor:    PIC18F46K80
 * Complier:     Microchip C18
 * Author:       Andrew Mass
 * Date:         2015-2016
 */

#ifndef ANALOGHUB_H
#define ANALOGHUB_H

#include "p18f46k80.h"
#include "GenericTypeDefs.h"
#include "adc.h"
#include "../ECAN.X/ECAN.h"
#include "../FSAE.X/FSAE.h"
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/errno.h"

// Timing definitions (ms)
#define DIAG_MSG_SEND  10
#define FAST_MSG_SEND  2
#define MED_MSG_SEND   20
#define SLOW_MSG_SEND  500

// ADC channel definitions for FRONT ADC channels
#define ADC_SPFL_CHN  10
#define ADC_SPFR_CHN  9
#define ADC_BPF_CHN   7
#define ADC_BPR_CHN   8
#define ADC_STRP_CHN  5
#define ADC_APPS0_CHN 2
#define ADC_APPS1_CHN 1
#define ADC_PTDP_CHN  6

// ADC channel definitions for REAR ADC channels
#define ADC_SPRL_CHN 8
#define ADC_SPRR_CHN 7
#define ADC_EOS_CHN  10
#define ADC_BCD_CHN  3
#define ADC_CTRI_CHN 9
#define ADC_CTRO_CHN 6
#define ADC_CTSP_CHN 5
#define ADC_FTFT_CHN 1
#define ADC_CPSP_CHN 4
#define ADC_FPFT_CHN 0
#define ADC_MCD_CHN  2

// Pin definitions for TERM signal
#define TERM_TRIS TRISCbits.TRISC6
#define TERM_LAT  LATCbits.LATC6

//TODO: Pin definitions for FRONT ADC channels

//TODO: Pin definitions for RADIO channels

// Function definitions
void high_isr(void);
uint16_t sample(const uint8_t ch);
void send_diag_can(void);
void send_fast_can(void);
void send_med_can(void);
void send_slow_can(void);

#endif /* ANALOGHUB_H */
