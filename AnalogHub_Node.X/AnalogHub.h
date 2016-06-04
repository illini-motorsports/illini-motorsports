/**
 * AnalogHub Header
 *
 * Processor:    PIC18F46K80
 * Compiler:     Microchip C18
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
#define DIAG_MSG_SEND  1000
#define FAST_MSG_SEND  2
#define MED_MSG_SEND   20
#define SLOW_MSG_SEND  500

// ADC channel definitions for FRONT ADC channels
#define ADC_SPFL_CHN  ADC_CH10
#define ADC_SPFR_CHN  ADC_CH9
#define ADC_BPF_CHN   ADC_CH7
#define ADC_BPR_CHN   ADC_CH8
#define ADC_STRP_CHN  ADC_CH5
#define ADC_APPS0_CHN ADC_CH2
#define ADC_APPS1_CHN ADC_CH1
#define ADC_PTDP_CHN  ADC_CH6

// ADC channel definitions for REAR ADC channels
#define ADC_SPRL_CHN ADC_CH8
#define ADC_SPRR_CHN ADC_CH7
#define ADC_EOS_CHN  ADC_CH10
#define ADC_BCD_CHN  ADC_CH3
#define ADC_CTRI_CHN ADC_CH9
#define ADC_CTRO_CHN ADC_CH6
#define ADC_CTSP_CHN ADC_CH5
#define ADC_CPSP_CHN ADC_CH4
#define ADC_MCD_CHN  ADC_CH2

// Pin definitions for TERM signal
#define TERM_TRIS TRISCbits.TRISC6
#define TERM_LAT  LATCbits.LATC6

// Pin definitions for FRONT ADC channels
#define ADC_SPFL_TRIS  TRISBbits.TRISB0
#define ADC_SPFR_TRIS  TRISBbits.TRISB4
#define ADC_BPF_TRIS   TRISEbits.TRISE2
#define ADC_BPR_TRIS   TRISBbits.TRISB1
#define ADC_STRP_TRIS  TRISEbits.TRISE0
#define ADC_APPS0_TRIS TRISAbits.TRISA2
#define ADC_APPS1_TRIS TRISAbits.TRISA1
#define ADC_PTDP_TRIS  TRISEbits.TRISE1
#define ADC_UAN0_TRIS  TRISAbits.TRISA0
#define ADC_UAN0_LAT   LATAbits.LATA0

// Pin definitions for REAR ADC channels
#define ADC_SPRL_TRIS TRISBbits.TRISB1
#define ADC_SPRR_TRIS TRISEbits.TRISE2
#define ADC_EOS_TRIS  TRISBbits.TRISB0
#define ADC_BCD_TRIS  TRISAbits.TRISA3
#define ADC_CTRI_TRIS TRISBbits.TRISB4
#define ADC_CTRO_TRIS TRISEbits.TRISE1
#define ADC_CTSP_TRIS TRISEbits.TRISE0
#define ADC_CPSP_TRIS TRISAbits.TRISA5
#define ADC_MCD_TRIS  TRISAbits.TRISA2
#define ADC_UAN1_TRIS  TRISAbits.TRISA1
#define ADC_UAN1_LAT   LATAbits.LATA1
#define ADC_UAN0_TRIS  TRISAbits.TRISA0
#define ADC_UAN0_LAT   LATAbits.LATA0

// Pin definitions for RADIO channels
#define RADIO0_TRIS TRISAbits.TRISA5
#define RADIO1_TRIS TRISAbits.TRISA3
#define RADIO0_LAT LATAbits.LATA5
#define RADIO1_LAT LATAbits.LATA3

// Function definitions
void high_isr(void);
uint16_t sample(const uint8_t ch);
void send_diag_can(void);
void send_fast_can(void);
void send_med_can(void);
void send_slow_can(void);

#endif /* ANALOGHUB_H */
