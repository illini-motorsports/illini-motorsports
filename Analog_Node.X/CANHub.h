/******************************************************************************
 *
 *                  Analog Hub C Main Code Header
 *
 ******************************************************************************
 * FileName:        CANHub.h
 * Dependencies:    none
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18

*******************************************************************************
	USER REVISON HISTORY
//
//
//
//

*******************************************************************************/

#ifndef CANHUB_H
#define CANHUB_H


/***********************************************/
/*  User Structures & Defines                  */
/***********************************************/

//#define INTERNAL
//#define DEBUGGING
#define MCHP_C18
#define INPUT 1
#define OUTPUT 0
#define INTEL 2		// least significant byte comes first
#define MOTOROLA 3	// most significant byte comes first

// Set the emission period (in ms)
// both must be a power of two for consistent sampling
#define FAST_SAMPLE 2
#define SLOW_SAMPLE 128

#define TERM_LAT LATCbits.LATC6

// Hub location specific code configuration
#ifdef FRONT

// define msg IDs
#define FAST_ID	0x060
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
#define ADL_ID 0x500
#define ADL_DLC 8
#define ADL_SAMPLE 200
#define Y_ID 0x070
#define X_ID 0x080
#define ADL1 2
#define ADL2 4
#define ADL3 6
#define ADL4 2
#define ADL5 4
#define ADL6 6
#define ADL7 2
#define ADL8 4

typedef struct {
	unsigned X_accel:1;
	unsigned Y_accel:1;
} FLAGS;

#endif


/***********************************************/
/*  User Function Prototypes                   */
/***********************************************/

void init_unused_pins(void);
void high_isr(void);
void sample(BYTE *data, const BYTE byte, const BYTE ch);
void process_resend(const BYTE *data, BYTE *msg, const BYTE byte, const int offset, const BYTE ADL_ch, const BYTE order);

#endif
