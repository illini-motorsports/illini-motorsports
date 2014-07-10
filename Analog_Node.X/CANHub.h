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
#define FAST_SAMPLE 1
#define SLOW_SAMPLE 128

#define TERM_LAT LATCbits.LATC6

// Hub location specific code configuration
#ifdef FRONT

// define msg IDs
#define FAST_ID	0x060
#define SLOW_ID_0 0x110
#define SLOW_ID_1 0x120

// define number of sensors
//#define FAST_NUM 2
//#define SLOW_NUM_0 3
//#define SLOW_NUM_1 2
#define FAST_NUM 0
#define SLOW_NUM_0 0
#define SLOW_NUM_1 0

// define sensor byte locations
#define SUS_R_BYTE 0
#define SUS_L_BYTE 2
#define BRAKE_R_P_BYTE 0
#define BRAKE_F_P_BYTE 2
#define STEER_BYTE 4
#define BRAKE_R_T_BYTE 0
#define BRAKE_L_T_BYTE 2

// define sensor pin location
#define SUS_R ADC_CH0
#define SUS_L ADC_CH1
#define BRAKE_R_P ADC_CH2
#define BRAKE_F_P ADC_CH3
#define STEER ADC_CH4
#define BRAKE_R_T ADC_CH5
#define BRAKE_L_T ADC_CH6

#elif REAR

// define msg IDs
#define FAST_ID 0x050
#define SLOW_ID_0 0x100
#define Y_ID 0x070
#define X_ID 0x080

// define number of slow sampled sensors
//#define FAST_NUM 2
//#define SLOW_NUM_0 2
#define FAST_NUM 0
#define SLOW_NUM_0 0

// define sensor byte locations
#define SUS_L_BYTE 0
#define SUS_R_BYTE 2
#define BRAKE_R_T_BYTE 0
#define BRAKE_L_T_BYTE 2

// define sensor pin location
#define SUS_R ADC_CH0
#define SUS_L ADC_CH1
#define BRAKE_R_T ADC_CH5
#define BRAKE_L_T ADC_CH6

#endif

typedef struct {
	unsigned X_accel:1;
	unsigned Y_accel:1;
} FLAGS;


/***********************************************/
/*  User Function Prototypes                   */
/***********************************************/

void init_unused_pins(void);
void high_isr(void);
void sample(BYTE *data, const BYTE byte, const BYTE ch);
void process_resend(const BYTE *data, BYTE *msg, const BYTE byte, const int offset, const BYTE ADL_ch, const BYTE order);

#endif
