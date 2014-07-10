/******************************************************************************
 *
 *					MDD File I/O Main Code
 *
 ******************************************************************************
 * FileName:        DAQ.c
 * Dependencies:    FSIO.h,
 *					DAQ.h,
 *					p18F46K80.h,
 *					timers.h and
 *					capture.h
 * Processor:		PIC18F46K80
 * Complier:		Microchip C18
 * Version:			1.00
 * Author:			George Schwieters
 * Created:			2012-2013

*******************************************************************************
	USER REVISON HISTORY
//
// 03/07/13 - intial release with various compilier optimizations (consts, statics etc.)
//			and optimized coding i.e. unsigned whenever possible and 8-bit variables
//			as much as possible etc. Also added rough documentation.
// 03/08/13 - added engine on check for logging control
// 03/09/13 - added version control to source and header files on assembla
// 03/13/13 - the buffer full flag is now reset in in FSIO.c; timer1 rollover is
//			now handled with a pending flag and incrementing happens only when
//			servicing the FIFO CAN buffer; added debugging define control over
//			when to begin and stop logging
// 04/17/13	- added priority levels to interrupts so that the seconds timer and the
//			CCP2 module can interrupt the buffering process.
// 04/18/13	- added back engine on condition checking and put it in main function
// 04/19/13	- changed the way seconds increments
//

*******************************************************************************/

/***********************************************/
/*  Header Files                               */
/***********************************************/

#include "p18F46K80.h"
#include "timers.h"
#include "capture.h"
#include "DAQ.h"
#include "FSIO.h"
#include "ECAN.h"


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
#pragma config PWRTEN = OFF      // Power Up Timer (Disabled)
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

// Data buffers A and B for writing
#pragma udata large_udataA = 0x700		// this specifies the location of the buffers in memory
volatile BYTE WriteBufferA[BUFF_SIZE];	// create buffer using defined buffer size
#pragma udata large_udataB = 0x900
volatile BYTE WriteBufferB[BUFF_SIZE];
#pragma udata

// volatile indicates that the value changes within interrupts and is read outside
// of the ISR so cached values of the variable cannot be used reliably
volatile MAIN Main;					// struct that holds bits used for flags and other stuff
volatile BUFF_HOLDER Buff;			// struct used for holding the buffer pointers
volatile BYTE rpm_h;				// RPM high byte
volatile BYTE rpm_l;				// RPM low byte
static unsigned int int_temp;		// for swapping buffer lengths in ISR
static BYTE k;						// high priority ISR for loop counters
static unsigned int seconds;		// holds timer1 rollover count
static unsigned int timestamp[MSGS_READ];	// holds CCP2 timestamps
static unsigned int timestamp_2[MSGS_READ];	// holds CCP2 timestamps
static unsigned int stamp[MSGS_READ];		// holds CCP2 timestamps
static unsigned int stamp_2[MSGS_READ];		// holds CCP2 timestamps
#ifdef DEBUGGING
static unsigned int dropped;		// keep track of number of dropped messages
#endif


/***********************************************/
/*  Interrupts                                 */
/***********************************************/

#pragma code high_vector = 0x08
void high_vector(void) {
	_asm goto high_isr _endasm
}
#pragma code

#pragma code low_vector = 0x18
void interrupt_at_low_vector(void) {
	_asm goto low_isr _endasm
}
#pragma code

/*********************************************************************************
  Interrupt Function:
    void low_isr(void)
  Summary:
    Service all low priority interrupt flags: FIFO WM and error flag
  Conditions:
	* Timer1 module must be setup along with the oscillator being used
	* ECAN must be configured
	* CCP2 must be configure for CAN capture functionality
  Input:
    none
  Return Values:
    none
  Side Effects:
    Resets the interrupt flags after servicing them
	When swap is set buffer A length is reset and if needed buffer pointers and lengths are swapped
	Clears all recieve buffere when an overflow event occurs
  Description:

  *********************************************************************************/
#pragma interruptlow low_isr
void low_isr (void) {

	// service FIFO RX buffers
	if(PIR5bits.FIFOWMIF) {
		PIR5bits.FIFOWMIF = 0;

		// sometimes a FIFO interrupt is triggered before 4 messeages have arrived; da fuk?
		if (Main.NumRead != MSGS_READ) {
			return;
		}

		Main.NumRead = 0;

		// check swap flag
		if(Main.Swap) {

			Main.BufferALen = 0;	// reset buffer A length
			Main.Swap = FALSE;		// and clear swap flag

			// we have been caching data in buffer B if this expression is true
			if(Main.BufferBLen != 0) {
				swap_len();
				swap_buff();
			}
		}

		// process data in CAN FIFO
		service_FIFO();
	}

	// check for an error with the bus
	if(PIR5bits.ERRIF) {
		// recieve buffer overflow occured clear out all the buffers
		if(COMSTATbits.RXB1OVFL == 1) {
			PIR5bits.ERRIF = 0;
			COMSTATbits.RXB1OVFL = 0;
			PIR5bits.FIFOWMIF = 0;
			B0CONbits.RXFUL = 0;
			B1CONbits.RXFUL = 0;
			B2CONbits.RXFUL = 0;
			B3CONbits.RXFUL = 0;
			B4CONbits.RXFUL = 0;
			B5CONbits.RXFUL = 0;
			RXB0CONbits.RXFUL = 0;
			RXB1CONbits.RXFUL = 0;
#ifdef DEBUGGING
			dropped++;
#endif
		}
	}

	return;
}

/*********************************************************************************
  Interrupt Function:
    void high_isr(void)
  Summary:
    Service all high priority interrupt flags: timer1 rollover and CCP2 trigger
  Conditions:
	* Timer1 module must be setup along with the oscillator being used
	* ECAN must be configured
	* CCP2 must be configure for CAN capture functionality
  Input:
    none
  Return Values:
    none
  Side Effects:
    Resets the interrupt flags after servicing them
	Reloads the timer1 registers and incremenets seconds when rollover occurs
	Writes timestamps
  Description:

  *********************************************************************************/
#pragma interrupt high_isr
void high_isr(void) {

	// check for timer1 rollover
	if(PIR1bits.TMR1IF) {
		PIR1bits.TMR1IF = 0;
		// load timer value such that the most signifcant bit is set so it
		// takes exactly one second for a 32.768kHz crystal to trigger a rollover interrupt
		TMR1H = 0x80;	// WriteTimer1(0x8000);
		TMR1L = 0x00;	//
		seconds++;
	}

	// check for incoming message to give timestamp
	if(PIR3bits.CCP2IF) {
		PIR3bits.CCP2IF = 0;

		// read CAN capture register and place in timestamp array
		// keeping track of how many are in the array
		stamp[Main.MsgNum] = CCPR2H * 256 + CCPR2L;		// ReadCapture2();
		stamp_2[Main.MsgNum++] = seconds;

		// check if four messages have came in so far
		if(Main.MsgNum == 0) {
			// swap the data byte by byte
			for (k = 0; k < MSGS_READ; k++) {
				int_temp = stamp[k];
				stamp[k] = timestamp[k];
				timestamp[k] = int_temp;
				int_temp = stamp_2[k];
				stamp_2[k] = timestamp_2[k];
				timestamp_2[k] = int_temp;
			}
			Main.NumRead = MSGS_READ;
		}
	}

	return;
}

/***********************************************/
/*  Main Program                               */
/***********************************************/

void main (void) {

	/*************************
	 * Varaible Declarations *
	 *************************/
	BUFF_HOLDER * Buff_p = &Buff;	// pointer to buffer stuct
	FSFILE * pointer;				// pointer to open file
	char fname[13] = "0000.TXT";	// holds name of file
	const char write = 'w';			// for opening file (must use variable for passing value in PIC18 when not using pgm function)
	SearchRec rec;					// holds search parameters and found file info
	const BYTE attributes			// holds search parameters
        = ATTR_ARCHIVE
		| ATTR_READ_ONLY
		| ATTR_HIDDEN;
#ifdef DEBUGGING
	int count = 0;
#endif

	init_unused_pins();		// assert values to unused pins

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

	while(!OSCCONbits.OSTS);	// wait for stable external clock
#endif


	/*********************
	 * Peripherals Setup *
	 *********************/

    ANCON0 = 0x00;			// default all pins to digital
    ANCON1 = 0x00;			// default all pins to digital

    SD_CS_TRIS = OUTPUT;	// set card select pin to output
    SD_CS = 1;				// card deselected

	// turn on and configure the TIMER1 oscillator
	OpenTimer1(T1_16BIT_RW & T1_SOURCE_PINOSC & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_ON, 0x00);
	IPR1bits.TMR1IP = 1;	// high priority
	seconds = 0;			// clear seconds
	WriteTimer1(0x8000);	// load timer registers

	// turn on and configure capture module
	OpenCapture2(CAP_EVERY_FALL_EDGE & CAPTURE_INT_ON);
	IPR3bits.CCP2IP = 1;	// high priority

	// initialize MDD I/O but don't allow data collection yet
	// since file creation can take a while
	// and buffers will fill up immediately
	Buff.BufferA = WriteBufferA;
	Buff.BufferB = WriteBufferB;
	Main.BufferALen = MEDIA_SECTOR_SIZE;
	Main.BufferBLen = MEDIA_SECTOR_SIZE;
	Main.Swap = FALSE;
	Main.BufferAFull = TRUE;
	Main.MsgNum = 0;			// holds index for CCP2 values

	// setup engine on condition checking
	rpm_l = 0;
	rpm_h = 0;

	while(!MDD_MediaDetect());	// wait for card presence
	while(!FSInit());			// setup file system library

	ECANInitialize();			// setup ECAN module

	// configure interrupts
	INTCONbits.GIE = 1;			// Global Interrupt Enable (1 enables)
	INTCONbits.PEIE = 1;		// Peripheral Interrupt Enable (1 enables)
	RCONbits.IPEN = 1;			// Interrupt Priority Enable (1 enables)

/***************end setup; begin main loop************************************/

	while(1) {

		// check if we want to start data logging
#ifdef DEBUGGING
        if (count == 0 && !PIR5bits.ERRIF) {
#else
		if(rpm_h * 256 + rpm_l > RPM_COMP && !PIR5bits.ERRIF) {
#endif
			while(!MDD_MediaDetect());	// wait for card presence

			// file name loop
			while(1) {
				// look for file with proposed name
				if(FindFirst(fname, attributes, &rec))
					if(FSerror() == CE_FILE_NOT_FOUND)	// check type of error was not finding the file
						break;							// exit loop, file name is unique
					else
						funct_error();

				// change file name and retest
				if (fname[3] == '9') {
					fname[3] = '0';		// reset first number
					fname[2]++;			// incement other number
				}
				else
					fname[3]++;			// increment file number
			}

			// create csv data file
			pointer = FSfopen(fname, &write);
			if(pointer == NULL)
				funct_error();

			// disable interrupts when dealing with volatile ints
			INTCONbits.GIE = 0;			// Global Interrupt Enable (1 enables)
			INTCONbits.PEIE = 0;		// Peripheral Interrupt Enable (1 enables)

			// setup buffers and flags to begin data collection
			Main.BufferALen = 0;
			Main.BufferBLen = 0;

			// renable
			INTCONbits.GIE = 1;
			INTCONbits.PEIE = 1;

			Main.BufferAFull = FALSE;
#ifdef DEBUGGING
			dropped = 0;
#endif

			// logging loop
			while(1) {
				// write buffer A to file
				if (Main.BufferAFull) {
					if(FSfwrite(Buff_p, pointer, (MAIN *) &Main) != BUFF_SIZE)
						funct_error();
#ifdef DEBUGGING
					count++;
#endif
                }

				// check if we should stop logging
#ifdef DEBUGGING
				if(count == DEBUG_LEN || PIR5bits.ERRIF) {
#else
				if(rpm_h * 256 + rpm_l < RPM_COMP || PIR5bits.ERRIF) {
#endif
					// close csv data file
					if(FSfclose(pointer))
						funct_error();

					// disable interrupts when dealing with volatile ints
					INTCONbits.GIE = 0;			// Global Interrupt Enable (1 enables)
					INTCONbits.PEIE = 0;		// Peripheral Interrupt Enable (1 enables)

					// stop collecting data in the buffers
					Main.BufferALen = MEDIA_SECTOR_SIZE;
					Main.BufferBLen = MEDIA_SECTOR_SIZE;

					// renable
					INTCONbits.GIE = 1;
					INTCONbits.PEIE = 1;

					Main.Swap = FALSE;
					Main.BufferAFull = TRUE;
					rpm_l = 0;
					rpm_h = 0;

					break;
				}
			}// logging loop
		}// data acq start control
	}// data acq loop

	return;
}


/***********************************************/
/*  User Functions                             */
/***********************************************/


/******************************************************************************
  Function:
    void funct_error(void)
  Summary:
    Place to stay when error occurs
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    none
  Description:

  ****************************************************************************/
void funct_error(void) {
	while(1);		// stay here
	return;
}

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
	configuration registers will be modified
  Description:

  ****************************************************************************/
void init_unused_pins(void) {

	// first configure to outputs
	TRISAbits.TRISA0 = OUTPUT;
	TRISAbits.TRISA1 = OUTPUT;
	TRISAbits.TRISA2 = OUTPUT;
	TRISAbits.TRISA3 = OUTPUT;
//	TRISAbits.TRISA5 = OUTPUT; CS'
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

//	TRISCbits.TRISC0 = OUTPUT; SOSC0
//	TRISCbits.TRISC1 = OUTPUT; SOSC1
	TRISCbits.TRISC2 = OUTPUT;
//	TRISCbits.TRISC3 = OUTPUT; SCK
//	TRISCbits.TRISC4 = OUTPUT; SDI
//	TRISCbits.TRISC5 = OUTPUT; SDO
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
	LATAbits.LATA0 = 0;
	LATAbits.LATA1 = 0;
	LATAbits.LATA2 = 0;
	LATAbits.LATA3 = 0;
//	LATAbits.LATA5 = 0; CS'
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

//	LATCbits.LATC0 = 0; SOSC0
//	LATCbits.LATC1 = 0; SOSC1
	LATCbits.LATC2 = 0;
//	LATCbits.LATC3 = 0; SCK
//	LATCbits.LATC4 = 0; SDI
//	LATCbits.LATC5 = 0; SDO
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
	void service_FIFO(void)
  Summary:
	Function to clear the FIFO buffer of recieved messages and create a message array
	to be written.
  Conditions:
    ECAN must be configured correctly
  Input:
    none
  Return Values:
    none
  Side Effects:
    clears out the recieve buffers
	updates the current rpm global variables
  Description:

  ****************************************************************************/
void service_FIFO(void) {

	static BYTE i;				// for loop counters
	static BYTE j;
	static BYTE msg[14];		// holds entire CAN message including timstamp
	unsigned long id;			// holds CAN msgID
	BYTE data[8];				// holds CAN data bytes
	BYTE dataLen;				// holds number of CAN data bytes
	ECAN_RX_MSG_FLAGS flags;	// holds information about recieved message

	// loop through messages in CAN buffers
	for (i = 0; i < MSGS_READ; i++) {
		// get message from RX buffer
		ECANReceiveMessage(&id, data, &dataLen, &flags);

		// check if a message with RPM data has been recieved
		if(id == RPM_ID) {
			// update current RPM
			rpm_l = data[RPM_BYTE + 1];
			rpm_h = data[RPM_BYTE];
		}

		if(dataLen > 0) {

			// collect message data before sending to buffer
			for (j = 0; j < dataLen + ID_PLUS_TIME; j++) {
				// get message ID
				if (j < MSG_ID_LEN) {
					msg[j] = ((BYTE *) (&id))[j];
				}
				// get data bytes
				else if (j < MSG_ID_LEN + dataLen) {
					msg[j] = data[j - MSG_ID_LEN];
				}
				// get CCP2 timer1 value that forms 2 LSBs of the timestamp
				else if (j < MSG_ID_LEN + dataLen + (TIMESTAMP_LEN / 2)) {
					msg[j] = ((BYTE*) timestamp)[j - MSG_ID_LEN - dataLen + i * 2];
				}
				// get seconds value that forms 2 MSBs of the timsetamp
				else if (j < dataLen + ID_PLUS_TIME) {
					msg[j] = ((BYTE *) timestamp_2)[j - MSG_ID_LEN - dataLen - (TIMESTAMP_LEN / 2) + i * 2];
				}
			}

			// send messgage to buffer
			append_write_buffer(msg, dataLen + ID_PLUS_TIME);
		}
	}

	return;
}


/******************************************************************************
  Function:
	void append_write_buffer(static const BYTE * temp, static BYTE applen)
  Summary:
	Function to control how data is appended to the user data buffers
  Conditions:
    none
  Input:
    temp - data array that holds one whole CAN message with its timestamp
	applen - the length of the data array
  Return Values:
    none
  Side Effects:
	When A fills up the flag to write is raised
  Description:
	This function controls how data is written using buffer A as the writing buffer and
	B as a temp cache for when we are writing to the SD and A is already full. When B
	fills up we are still writing data and all the buffers are filled meaning
	further messages will be dropped.

  ****************************************************************************/
void append_write_buffer(static const BYTE * temp, static BYTE applen) {

	static BYTE offset;
	static BYTE holder;

	// can't initialize these in declaration it won't work for whatever fucking reason
	Main.Written = FALSE;
	offset = 0;

	// message is dropped
	if (applen > (2 * BUFF_SIZE - (Main.BufferALen + Main.BufferBLen))) {
#ifdef DEBUGGING
		dropped++;
#endif
		return;
	}

	// try writing to buffer A first
	if (!Main.BufferAFull) {
		// room for all of data to write
		if (BUFF_SIZE - Main.BufferALen > applen) {
			Main.Written = TRUE;
		}
		// not enough room for all the data
		else if (BUFF_SIZE - Main.BufferALen < applen) {
			Main.BufferAFull = TRUE;
			// recalculate writing parameters for partial write
			offset = applen - (BUFF_SIZE - Main.BufferALen);
		}
		// exactly enough room for the data
		else {
			Main.BufferAFull = TRUE;
			Main.Written = TRUE;
		}
		// add message to buffer
		buff_cat(Buff.BufferA, temp, &(Main.BufferALen), applen - offset, 0);
	}

	// write to buffer B if couldn't write any or all data to buffer A
	if (!Main.Written) {
		// only use offset if there has been a partial write to buffer A
		if (offset != 0) {
			holder = applen;
			applen = offset;
			offset = holder - offset;
		}
		// add message to buffer
		buff_cat(Buff.BufferB, temp, &(Main.BufferBLen), applen, offset);
	}

	return;
}

/******************************************************************************
  Function:
	void buff_cat(static BYTE *WriteBuffer, static const BYTE *writeData,
 					static unsigned int *bufflen, static const BYTE applen,
 					static const BYTE offset)
  Summary:
	Function write data to a user data buffer
  Conditions:
    none
  Input:
    WriteBuffer - pointer to the buffer we will write to
	writeData - pointer to the data array we will write
	bufflen - pointer to the variable holding the buffer length of the buffer we are writing to
	applen - the length of the data to be written
	offset - the index offset for the data to write
  Return Values:
    none
  Side Effects:
	adds writen message length to the given buffer length variable
	writes data to the given buffer
  Description:

  ****************************************************************************/
void buff_cat(static BYTE *WriteBuffer, static const BYTE *writeData, static unsigned int *bufflen, static const BYTE applen, static const BYTE offset) {

	static BYTE i;

	// increment through the data bytes sending them to the write buffer
	for(i = 0; i < applen; i++) {
		WriteBuffer[*bufflen + i] = writeData[i + offset];
	}

	// increment the data length count for the writing buffer
	*bufflen += (unsigned int) applen;

	return;
}

/******************************************************************************
  Function:
	void swap_len(void)
  Summary:
	Function to swap buffer length values
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    the values for buffer length will be modified
  Description:

  ****************************************************************************/
void swap_len(void) {

	static unsigned int temp;
	temp = Main.BufferALen;

	// swap lengths
	Main.BufferALen = Main.BufferBLen;
	Main.BufferBLen = temp;

	return;
}

/******************************************************************************
  Function:
	void swap_buff(void)
  Summary:
	Function to swap the pointers to the user data buffers
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    The pointers in the Buff struct are modified
  Description:

  ****************************************************************************/
void swap_buff(void) {

	static BYTE *temp;
	temp = Buff.BufferA;

	// swap pointers
	Buff.BufferA = Buff.BufferB;
	Buff.BufferB = temp;

	return;
}
