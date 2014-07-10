/******************************************************************************
 *
 *					Sensor Hub C Main Code
 *
 ******************************************************************************
 * FileName:		CANHub.c
 * Dependencies:	p18F46K80.h,
 *					timers.h,
 *					adc.h,
 *					ECAN.h and
 *					CANfHub.h
 * Processor:		PIC18F46K80
 * Complier:		Microchip C18
 * Version:			1.00
 * Author:			George Schwieters
 * Created:			2012-2013

*******************************************************************************
	USER REVISON HISTORY
//
// 03/08/13 - added compilier optimizations to intial release of code and
//			rough documentation
// 03/29/13 - added accel data resend code to interface with Motec
// 04/18/13	- added function for resending data for Motec
//

*******************************************************************************/

/***********************************************/
/*  Header Files                               */
/***********************************************/

#include "p18F46K80.h"
#include "timers.h"
#include "adc.h"
#include "ECAN.h"
#include "CANHub.h"


/***********************************************/
/*  PIC18F46K80 Configuration Bits Settings    */
/***********************************************/

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#ifdef INTERNAL
#pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#else
#pragma config FOSC = HS1       // Oscillator (HS oscillator (Medium power, 4 MHz - 16 MHz))
#endif

#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
#pragma config BOREN = OFF      // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)


/***********************************************/
/*  Global Variable Declarations               */
/***********************************************/

static unsigned int millis;	// millisecond count


/***********************************************/
/*  Interrupts                                 */
/***********************************************/

/*********************************************************************************
  Interrupt Function:
    void high_isr(void)
  Summary:
    Function to service interrupts
  Conditions:
	* Timer0 module must be setup along with the oscillator being used
	* ECAN must be configured
	* A/D converter must be configured
  Input:
    none
  Return Values:
    none
  Side Effects:
	Transmits messages on CAN
	Reloads the timer0 registers and increments millis
	Resets the interrupt flags after servicing them
  Description:

  *********************************************************************************/
#pragma code high_vector = 0x08
void high_vector(void) {
	_asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr (void) {

	BYTE data[8];	// holds CAN data bytes

	// check for timer0 rollover indicating a millisecond has passed
	if (INTCONbits.TMR0IF) {
		INTCONbits.TMR0IF = 0;
		WriteTimer0(0x82);		// load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
		millis++;
	}

#if (FAST_NUM != 0)
	// wait until millisecond count is a multiple of the sample parameter for
	// high frequency sampled sensors
	if(!(millis % FAST_SAMPLE)) {

		// sample suspension pots
		sample(data, SUS_L_BYTE, SUS_L);
		sample(data, SUS_R_BYTE, SUS_R);

		// send the sampled values out on CAN
		while(!ECANSendMessage(FAST_ID, data, FAST_NUM * 2, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_3));
	}
#endif

#if (SLOW_NUM != 0)
	// wait until millisecond count is a multiple of the sample parameter for
	// low frequency sampled sensors
	if(!(millis % SLOW_SAMPLE)) {

#ifdef FRONT
		// sample the sensors
		sample(data, BRAKE0_BYTE, BRAKE0);
		sample(data, BRAKE1_BYTE, BRAKE1);
		sample(data, STEER_BYTE, STEER);
#elif REAR

#endif

		// send the sampled values out on CAN
		while(!ECANSendMessage(SLOW_ID, data, SLOW_NUM * 2, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_2));
	}
#endif

	return;
}


/***********************************************/
/*  Main Program                               */
/***********************************************/

void main(void) {

	/*************************
	 * Variable Declarations *
	 *************************/
#ifdef MOTEC_RESEND
	unsigned long id;			// holds CAN msgID
	BYTE data_r[8];				// holds CAN data bytes
	BYTE dataLen;				// holds number of CAN data bytes
	ECAN_RX_MSG_FLAGS flags;	// holds information about recieved message
	FLAGS Recieved;
	BYTE msg[8];
#endif

	init_unused_pins();			// assert values to unused pins

	/*********************
	 * Oscillator Set-Up *
	 *********************/
#ifdef INTERNAL
	// OSCTUNE
	OSCTUNEbits.INTSRC = 0;		// Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
	OSCTUNEbits.PLLEN = 1;		// Frequency Multiplier PLL Select (1 to enable)
	OSCTUNEbits.TUN5 = 0;		// Fast RC Oscillator Frequency Tuning (seems to be 2's comp encoding)
	OSCTUNEbits.TUN4 = 0;		// 011111 = max
	OSCTUNEbits.TUN3 = 0;		// ... 000001
	OSCTUNEbits.TUN2 = 0;		// 000000 = center (running at calibrated frequency)
	OSCTUNEbits.TUN1 = 0;		// 111111 ...
	OSCTUNEbits.TUN0 = 0;		// 100000

	// OSCCCON
	OSCCONbits.IDLEN = 1;		// Idle Enable Bit (1 to enter idle mode after SLEEP instruction else sleep mode is entered)
	OSCCONbits.IRCF2 = 1;		// Internal Oscillator Frequency Select Bits
	OSCCONbits.IRCF1 = 1;		// When using HF, settings are:
	OSCCONbits.IRCF0 = 1;		// 111 - 16 MHz, 110 - 8MHz (default), 101 - 4MHz, 100 - 2 MHz, 011 - 1 MHz
	OSCCONbits.SCS1 = 0;
	OSCCONbits.SCS0 = 0;

	// OSCCON2
	OSCCON2bits.MFIOSEL = 0;

	while(!OSCCONbits.HFIOFS);	// wait for stable clock

#else
	// OSCTUNE
	OSCTUNEbits.INTSRC = 0;		// Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
	OSCTUNEbits.PLLEN = 1;		// Frequency Multiplier PLL Select (1 to enable)

	// OSCCCON
	OSCCONbits.SCS1 = 0;		// select configuration chosen oscillator
	OSCCONbits.SCS0 = 0;		// SCS = 00

	// OSCCON2
	OSCCON2bits.MFIOSEL = 0;

	while(!OSCCONbits.OSTS);		// wait for stable external clock
#endif

	/**********************
	 * Peripherals Setup  *
	 **********************/

	// turn on and configure the TIMER1 oscillator
	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
	WriteTimer0(0x82);		// load timer register
	millis = 0;				// clear milliseconds count
	INTCONbits.TMR0IE = 1;	// turn on timer0 interupts

	// turn on and configure the A/D converter module
	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_4_TAD, ADC_CH0 & ADC_INT_OFF, ADC_REF_VDD_VDD & ADC_REF_VDD_VSS & ADC_NEG_CH0);
	ANCON0 = 0x1F;		// AN0 - 4 are analog
	ANCON1 = 0x00;		// rest are digital
	TRISAbits.TRISA0 = INPUT;
	TRISAbits.TRISA1 = INPUT;
	TRISAbits.TRISA2 = INPUT;
	TRISAbits.TRISA3 = INPUT;
	TRISAbits.TRISA5 = INPUT;

    ECANInitialize();		// setup ECAN

	// interrupts setup
	INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
	INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
	RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

#ifdef MOTEC_RESEND
	Recieved.X_accel = FALSE;
	Recieved.Y_accel = FALSE;
#endif


/***************end setup; begin main loop************************************/


	// all A/D operations are dealt with in the ISR
	// that's triggered by the 1 ms rollover timer
	while (1) {
#ifdef MOTEC_RESEND
		// poll for an acceleromter message (other messages are filtered out)
		while(!ECANReceiveMessage(&id, data_r, &dataLen, &flags));

		// check which accelerometer message was recieved
		if(id == Y_ID && !Recieved.Y_accel) {
			// process CAN bus data and put in ADL format
			process_resend(data_r, msg, Y_BYTE, Y_OFFSET, ADL2, INTEL);
			Recieved.Y_accel = TRUE;
		}
		else if(id == X_ID && !Recieved.X_accel) {
			// process CAN bus data and put in ADL format
			process_resend(data_r, msg, X_BYTE, X_OFFSET, ADL1, INTEL);
			Recieved.X_accel = TRUE;
		}

		// resend out data
		if(Recieved.X_accel && Recieved.Y_accel) {
			while(!ECANSendMessage(RESEND_ID, msg, ADL_DLC, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1));
			Recieved.X_accel = FALSE;
			Recieved.Y_accel = FALSE;
		}
#endif
	}

	return;
}

/***********************************************/
/*  User Functions                             */
/***********************************************/


/******************************************************************************
  Function:
	void init_unused_pins(void)
  Summary:
	Function to assert unused pins to eliminate sources of noise and reduce MCU
	power draw.
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    Certain pins will be set low and configured as outputs
  Description:

  ****************************************************************************/
void init_unused_pins(void) {

	// first configure to outputs
//	TRISAbits.TRISA0 = OUTPUT; AN0
//	TRISAbits.TRISA1 = OUTPUT; AN1
//	TRISAbits.TRISA2 = OUTPUT; AN2
//	TRISAbits.TRISA3 = OUTPUT; AN3
//	TRISAbits.TRISA5 = OUTPUT; AN4
//	TRISAbits.TRISA6 = OUTPUT; OSC2
//	TRISAbits.TRISA7 = OUTPUT; OSC1

	TRISBbits.TRISB0 = OUTPUT;
	TRISBbits.TRISB1 = OUTPUT;
//	TRISBbits.TRISB2 = OUTPUT; CANTX
//	TRISBbits.TRISB3 = OUTPUT; CANRX
	TRISBbits.TRISB4 = OUTPUT;
	TRISBbits.TRISB5 = OUTPUT;
	TRISBbits.TRISB6 = OUTPUT;
	TRISBbits.TRISB7 = OUTPUT;

	TRISCbits.TRISC0 = OUTPUT;
	TRISCbits.TRISC1 = OUTPUT;
	TRISCbits.TRISC2 = OUTPUT;
	TRISCbits.TRISC3 = OUTPUT;
	TRISCbits.TRISC4 = OUTPUT;
	TRISCbits.TRISC5 = OUTPUT;
	TRISCbits.TRISC6 = OUTPUT;
	TRISCbits.TRISC7 = OUTPUT;

	TRISDbits.TRISD0 = OUTPUT;
	TRISDbits.TRISD1 = OUTPUT;
	TRISDbits.TRISD2 = OUTPUT;
	TRISDbits.TRISD3 = OUTPUT;
	TRISDbits.TRISD4 = OUTPUT;
	TRISDbits.TRISD5 = OUTPUT;
	TRISDbits.TRISD6 = OUTPUT;
	TRISDbits.TRISD7 = OUTPUT;

	TRISEbits.TRISE0 = OUTPUT;
	TRISEbits.TRISE1 = OUTPUT;
	TRISEbits.TRISE2 = OUTPUT;
//	TRISEbits.TRISE3 = OUTPUT; MCLR

	// then set pins low
//	LATAbits.LATA0 = 0; AN0
//	LATAbits.LATA1 = 0; AN1
//	LATAbits.LATA2 = 0; AN2
//	LATAbits.LATA3 = 0; AN3
//	LATAbits.LATA5 = 0; AN4
//	LATAbits.LATA6 = 0; OSC2
//	LATAbits.LATA7 = 0; OSC1

	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 0;
//	LATBbits.LATB2 = 0; CANTX
//	LATBbits.LATB3 = 0; CANRX
	LATBbits.LATB4 = 0;
	LATBbits.LATB5 = 0;
	LATBbits.LATB6 = 0;
	LATBbits.LATB7 = 0;

	LATCbits.LATC0 = 0;
	LATCbits.LATC1 = 0;
	LATCbits.LATC2 = 0;
	LATCbits.LATC3 = 0;
	LATCbits.LATC4 = 0;
	LATCbits.LATC5 = 0;
	LATCbits.LATC6 = 0;
	LATCbits.LATC7 = 0;

	LATDbits.LATD0 = 0;
	LATDbits.LATD1 = 0;
	LATDbits.LATD2 = 0;
	LATDbits.LATD3 = 0;
	LATDbits.LATD4 = 0;
	LATDbits.LATD5 = 0;
	LATDbits.LATD6 = 0;
	LATDbits.LATD7 = 0;

	LATEbits.LATE0 = 0;
	LATEbits.LATE1 = 0;
	LATEbits.LATE2 = 0;
//	LATEbits.LATE3 = 0; MCLR

	return;
}

/******************************************************************************
  Function:
	void sample(BYTE *data, const BYTE byte, const BYTE ch)
  Summary:
	Function to read the analog voltage of a pin and then put the value into the
	data array that will be transmited over CAN
  Conditions:
    A/D Converter must be configured
  Input:
	data - pointer to array of data bytes
    ch - which pin to sample
	byte - where to write the data in the passed array
  Return Values:
    none
  Side Effects:
    writes to data array
  Description:

  ****************************************************************************/
void sample(BYTE *data, const BYTE byte, const BYTE ch) {

	SelChanConvADC(ch);	// configure which pin you want to read and start A/D converter

	while(BusyADC());	// wait for complete conversion

	// put result in data array in accordance with specified byte location
	((unsigned int *)data)[byte / 2] = (unsigned int)ReadADC();

	return;
}

/******************************************************************************
  Function:
	void process_resend(const BYTE *data, BYTE *msg,
						const BYTE byte, const int offset,
 						const BYTE ADL_ch, const BYTE order)
  Summary:
	Function to take in incoming data and reformat to be read by Motec as an ADL
	CAN message.
  Conditions:
	none
  Input:
	data - the data bytes from the recieved CAN message
	msg - the data bytes to be transmitted
	byte - the location of the recieved data within the recieved message
	offset - the offset to apply to the incoming data before getting transmitted
	ADL_ch - the ADL channel that we want the transmitted data to placed in
	order - the byte order of the incoming data (either INTEL or MOTOROLA)
  Return Values:
    none
  Side Effects:
	Writes to msg array
  Description:

  ****************************************************************************/
void process_resend(const BYTE *data, BYTE *msg, const BYTE byte, const int offset, const BYTE ADL_ch, const BYTE order) {

	BYTE temp;

	// check which byte order we are dealing with
	if(order == INTEL) {
		// LSB byte comes first
		((int *)msg)[ADL_ch / 2] = data[byte + 1] * 256 + data[byte] - offset;
		// swap bytes to get into Motorola order
		temp = msg[ADL_ch];
		msg[ADL_ch] = msg[ADL_ch + 1];
		msg[ADL_ch + 1] = temp;
	}
	else {
		// MSB byte comes first
		((int *)msg)[ADL_ch / 2] = data[byte + 1] + data[byte] * 256 - offset;
		// swap bytes to get into Motorola order
		temp = msg[ADL_ch];
		msg[ADL_ch] = msg[ADL_ch + 1];
		msg[ADL_ch + 1] = temp;
	}

	return;
}
