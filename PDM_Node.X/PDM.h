/******************************************************************************
 *
 *                  PDM Node C Main Code Header
 *
 ******************************************************************************
 * FileName:        PDM.h
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

#ifndef PDM_H
#define PDM_H


/***********************************************/
/*  User Structures & Defines                  */
/***********************************************/

//#define INTERNAL
//#define DEBUGGING
//#define PARANOID_MODE
//#define OVERCURRENT_HANDLING

#define MCHP_C18
#define INPUT 1
#define OUTPUT 0
#define PWR_OFF 0
#define PWR_ON 1

#define PDM_ID 0x300L
#define ET_ID 0x201L
#define OT_ID 0x200L
#define OP_ID 0x200L
#define RPM_ID 0x200L
#define FAN_SW_ID 0x500L

#define CAN_PERIOD 500

#define OP_BYTE 4
#define OT_BYTE 6
#define ET_BYTE 0
#define RPM_BYTE 0
#define FAN_SW_BYTE 4
#define FAN_SW_BYTE0_ID 0x00

#define OP_THRESHOLD_L 160		// 16.0
#define OP_THRESHOLD_H 250		// 25.0
#define OT_THRESHOLD 2100		// 210.0
#define ET_THRESHOLD 1150		// 115.0
#define RPM_THRESHOLD_H 4000	// 4,000
#define RPM_THRESHOLD_L 1000	// 1,000

#define ON_THRESHOLD 600

#define FAN_THRESHOLD_H 900	// 90.0
#define FAN_THRESHOLD_L 840	// 84.0

#define PRIME_WAIT 500

#define IGN_val 0
#define FUEL_val 1
#define WATER_val 2
#define START_val 3
#define FAN_val 4
#define PCB_val 5
#define AUX_val 6
#define ECU_val 7
#define START_val_2 8
#define START_val_3 9

#define IGN_ch ADC_CH2
#define FUEL_ch ADC_CH3
#define WATER_ch ADC_CH1
#define PCB_ch ADC_CH0
#define ECU_ch ADC_CH8
#define AUX_ch ADC_CH9
#define FAN_ch ADC_CH4
#define START_ch_3 ADC_CH7
#define START_ch_2 ADC_CH6
#define START_ch ADC_CH5

// times in ms
#define ERROR_WAIT 500
#define ERROR_LIMIT 4
#define ERROR_RESET 120000
#define CAN_PER 500

#define NUM_LOADS 8
#define NON_INDUCTIVE 3

#define MIN_MA_OVERCURRENT 50

// main function flags and other information
typedef union {
    struct {
		unsigned char bits;
    };
    struct {
		unsigned Fuel:1;
		unsigned Ignition:1;
		unsigned Starter:1;
		unsigned Water:1;
		unsigned Fan:1;
		unsigned PCB:1;
		unsigned ECU:1;
		unsigned AUX:1;
    };
} Error_Status;

#define	IGN_LAT		LATDbits.LATD0
#define	IGN_P_LAT	LATDbits.LATD1
#define	FUEL_LAT	LATDbits.LATD2
#define	FUEL_P_LAT	LATDbits.LATD3
#define	FAN_LAT		LATDbits.LATD4
#define	FAN_P_LAT	LATDbits.LATD5
#define	START_LAT	LATDbits.LATD6
#define START_P_LAT	LATDbits.LATD7

#define AUX_LAT		LATCbits.LATC2
#define ECU_LAT		LATCbits.LATC3
#define PCB_LAT		LATCbits.LATC4
#define WATER_LAT	LATCbits.LATC6
#define WATER_P_LAT	LATCbits.LATC7
#define TERM_LAT	LATCbits.LATC5

#define	IGN_PORT	PORTDbits.RD0
#define	IGN_P_PORT	PORTDbits.RD1
#define	FUEL_PORT	PORTDbits.RD2
#define	FUEL_P_PORT	PORTDbits.RD3
#define	FAN_PORT	PORTDbits.RD4
#define	FAN_P_PORT	PORTDbits.RD5
#define	START_PORT	PORTDbits.RD6
#define START_P_PORT	PORTDbits.RD7

#define AUX_PORT	PORTCbits.RC2
#define ECU_PORT	PORTCbits.RC3
#define PCB_PORT	PORTCbits.RC4
#define WATER_PORT	PORTCbits.RC6
#define WATER_P_PORT	PORTCbits.RC7

#define START_SW	PORTBbits.RB5
#define ON_SW		PORTBbits.RB0


/***********************************************/
/*  User Function Prototypes                   */
/***********************************************/

void high_isr(void);
void sample(int *data, const BYTE index, const BYTE ch);
BYTE preventEngineBlowup(unsigned long * oil_press_tmr, unsigned long * oil_temp_tmr, unsigned long * water_temp_tmr);
void checkWaterTemp(BYTE * turn_on);

#endif
