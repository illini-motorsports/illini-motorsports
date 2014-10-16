
/*
 *                  Analog Node Main File Header
 *
 * File Name:       CANHub.h
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 * Created:         2012-2013
 */

#ifndef CANHUB_H
#define CANHUB_H

#include "adc.h"

/*
 * Code Control
 */

//#define INTERNAL
//#define DEBUGGING
#define MCHP_C18

/*
 * Magic Numbers
 */

#define INPUT 1
#define OUTPUT 0
#define INTEL 2     // least significant byte comes first
#define MOTOROLA 3  // most significant byte comes first

// Set the emission period (in ms)
// both must be a power of two for consistent sampling
#define FAST_SAMPLE 2
#define SLOW_SAMPLE 128

/*
 * Pin Defintions
 */

#define TERM_LAT LATCbits.LATC6

/*
 * Front Hub Specific Definitions
 */
#ifdef FRONT

// define msg IDs
#define FAST_ID 0x060
#define SLOW_ID 0x110

// define number of sensors
#define FAST_NUM 2
#define SLOW_NUM 2

// define sensor byte locations
#define SUS_R_BYTE 0
#define SUS_L_BYTE 2
#define BRAKE_F_P_BYTE 0
#define BRAKE_R_P_BYTE 2
#define STEER_BYTE 4

// define sensor pin location
#define SUS_R ADC_CH3
#define SUS_L ADC_CH1
#define BRAKE_R_P ADC_CH0
#define BRAKE_F_P ADC_CH2
#define STEER ADC_CH4

#define RADIO_LAT_0 LATEbits.LATE0
#define RADIO_LAT_1 LATEbits.LATE1
#define RADIO_TRIS_0 TRISEbits.TRISE0
#define RADIO_TRIS_1 TRISEbits.TRISE1

#define RADIO_SW_ID 0x500L
#define RADIO_SW_BYTE0_ID 0x00
#define RADIO_SW_BYTE 2

/*
 * Rear Hub Specific Definitions
 */
#elif REAR

// define msg IDs
#define FAST_ID 0x050
#define SLOW_ID 0x100
#define Y_ID 0x070
#define X_ID 0x080

// define number of slow sampled sensors
#define FAST_NUM 2
#define SLOW_NUM 0

// define sensor byte locations
#define SUS_R_BYTE 0
#define SUS_L_BYTE 2

// define sensor pin location
#define SUS_R ADC_CH2
#define SUS_L ADC_CH1

// Motec ADL
#define X_OFFSET 0x8000
#define Y_OFFSET 0x8000
#define X_BYTE 4
#define Y_BYTE 4
#define ADL_DLC 8
#define ADL_SAMPLE 200
#define Y_ID 0x070
#define X_ID 0x080

typedef struct {
    unsigned X_accel : 1;
    unsigned Y_accel : 1;
} FLAGS;

#endif

void init_unused_pins(void);
void high_isr(void);
void sample(unsigned char *data, const unsigned char byte, const unsigned char ch);
void process_resend(const unsigned char *data, unsigned char *msg,
        const unsigned char byte, const int offset, const unsigned char ADL_ch,
        const unsigned char order);

#endif
