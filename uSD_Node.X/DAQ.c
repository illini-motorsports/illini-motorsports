
/*
 *					MDD File I/O Main Code
 *
 * File Name:		DAQ.c
 * Processor:		PIC18F46K80
 * Complier:		Microchip C18
 * Version:			2.00
 * Author:			George Schwieters
 * Created:			2012-2013
 */

#include "capture.h"
#include "DAQ.h"
#include "FSIO.h"
#include "ECAN.h"
#include "FSAE.h"


/*
 *  PIC18F46K80 Configuration Bits
 */

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


/*
 * Global Variables
 */

// USer data buffers A and B
#pragma udata large_udataA = 0x700		// this specifies the location of the buffers in memory
volatile unsigned char WriteBufferA[BUFF_SIZE];	// create buffer using defined buffer size
#pragma udata large_udataB = 0x900
volatile unsigned char WriteBufferB[BUFF_SIZE];
#pragma udata

// variables used inside and outside the ISRs
volatile static MAIN Main;					// struct that holds bits used for flags and other stuff
volatile static BUFF_HOLDER Buff;			// struct used for holding the buffer pointers
volatile static unsigned char rpm_h;		// RPM high byte
volatile static unsigned char rpm_l;		// RPM low byte

// variables used inside both ISRs
volatile static unsigned int timestamp[MSGS_READ];		// holds CCP2 timestamps
volatile static unsigned int timestamp_2[MSGS_READ];	// holds CCP2 timestamps

// variables used inside one of the ISRs
static unsigned char k;				// high priority ISR for loop counters
static unsigned int int_temp;		// for swapping buffer lengths in ISR
static unsigned int seconds;		// holds timer1 rollover count
static unsigned int stamp[MSGS_READ];		// holds CCP2 timestamps
static unsigned int stamp_2[MSGS_READ];		// holds CCP2 timestamps

static unsigned int rpmLast = 0;            // holds time (in seconds) of last rpm message
#ifdef DEBUGGING
static unsigned int dropped;		// keep track of number of dropped messages
#endif


/*
 * Interrupts
 */

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

/*
 *	void low_isr(void)
 *
 *	Description:	This interrupt will service all low priority interrupts. This includes
 *					servicing the ECAN FIFO buffer and handling ECAN errors. This is not
 *					a traditional interrupt due do the nature of the project. The interrupt is
 *					by no means short but it must be this way in order to be able to buffer
 *					data at all times even when the uSD card is being written to in the main
 *					loop.
 *	Input(s): none
 *	Reaturn Value(s): none
 *	Side Effects:	This will modify PIR5, B0CON, B1CON, B2CON, B3CON, B4CON, B5CON,
 *					COMSTAT, RXB0CON & RXB1CON. This will also modify the flags in Main
 */
#pragma interruptlow low_isr
void low_isr(void) {

	// service FIFO RX buffers
	if(PIR5bits.FIFOWMIF) {
		PIR5bits.FIFOWMIF = 0;

		// sometimes a FIFO interrupt is triggered before 4 messeages have arrived; da fuk?
		if (Main.NumRead != MSGS_READ) {
			return;
		}

		Main.NumRead = 0;

		CLI(); // begin critical section
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
		STI(); // end critical section

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

/*
 *	void high_isr(void)
 *
 *	Description:	This interrupt will service all high priority interrupts which includes
 *					timer 1 rollover and the capture 2 module.
 *	Input(s): none
 *	Reaturn Value(s): none
 *	Side Effects:	This will modify TMR1H, TMR1L, PIR1 & PIR3. Also seconds,
 *					stamp, stamp_2, time_stanp, time_stamp_2 & Main variables will be written to.
 */
#pragma interrupt high_isr
void high_isr(void) {

	// check for timer1 rollover
	if(PIR1bits.TMR1IF) {
		PIR1bits.TMR1IF = 0;
		// load timer value such that the most signifcant bit is set so it
		// takes exactly one second for a 32.768kHz crystal to trigger a rollover interrupt
		TMR1H = TMR1H_RELOAD;	// WriteTimer1(TMR1H_RELOAD * 256 + TMR1L_RELOAD);
		TMR1L = TMR1L_RELOAD;	//
		seconds++;

		// check if the engine has stopped responding
		if(rpmLast - seconds > 1){
			Main.EngTO = 1;
		}
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

void main (void) {

/*
 * Variable Declarations
 */
	BUFF_HOLDER * Buff_p = &Buff;	// pointer to buffer stuct
	FSFILE * pointer;				// pointer to open file
	char fname[13] = "0000.TXT";	// holds name of file
	const char write = 'w';			// for opening file (must use variable for passing value in PIC18 when not using pgm function)
	SearchRec rec;					// holds search parameters and found file info
	const unsigned char attributes			// holds search parameters
        = ATTR_ARCHIVE
		| ATTR_READ_ONLY
		| ATTR_HIDDEN;
#ifdef DEBUGGING
	int count = 0;
#endif

	// assert values to unused pins
	init_unused_pins();

/*
 * Variable Initialization
 */

	// initialize MDD I/O variables but don't allow data collection yet
	// since file creation can take a while
	// and buffers will fill up immediately
	Buff.BufferA = WriteBufferA;
	Buff.BufferB = WriteBufferB;
	Main.BufferALen = MEDIA_SECTOR_SIZE;
	Main.BufferBLen = MEDIA_SECTOR_SIZE;
	Main.Swap = FALSE;
	Main.BufferAFull = TRUE;
	Main.MsgNum = 0;			// holds index for CCP2 values
	Main.EngTO = 0;

	// setup engine on condition checking
	rpm_l = 0;
	rpm_h = 0;

	// clear timer
	seconds = 0;

/*
 * Peripheral Initialization
 */

	// can use internal or external
	init_oscillator();

    ANCON0 = 0x00;			// default all pins to digital
    ANCON1 = 0x00;			// default all pins to digital

    SD_CS_TRIS = OUTPUT;	// set card select pin to output
    SD_CS = 1;				// card deselected

#ifdef LOGGING_0
	TRISCbits.TRISC6 = OUTPUT;	// programmable termination
	TERM_LAT = FALSE;
#endif

	// setup seconds interrupt
	init_timer1();

	// turn on and configure capture module
	OpenCapture2(CAP_EVERY_FALL_EDGE & CAPTURE_INT_ON);
	IPR3bits.CCP2IP = 1;		// high priority

	while(!MDD_MediaDetect());	// wait for card presence
	while(!FSInit());			// setup file system library

	ECANInitialize();			// setup ECAN module

	// configure interrupts
	RCONbits.IPEN = 1;			// Interrupt Priority Enable (1 enables)
	STI();

/*
 * Main Loop
 */

	while(1) {

		// check if we want to start data logging
		CLI(); // begin critical section
#ifdef DEBUGGING
        if (count == 0 && !PIR5bits.ERRIF) {
#else
		if(rpm_h * 256 + rpm_l > RPM_COMP && !PIR5bits.ERRIF) {
#endif
			STI(); // end critical section
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

			CLI(); // begin critical section
			// setup buffers and flags to begin data collection
			Main.BufferALen = 0;
			Main.BufferBLen = 0;
			Main.BufferAFull = FALSE;
			STI(); // end critical section
#ifdef DEBUGGING
			dropped = 0;
#endif

			// logging loop
			while(1) {
				// write buffer A to file
				CLI; // begin critical section
				if (Main.BufferAFull) {
					STI(); // end critical section
					if(FSfwrite(Buff_p, pointer, (MAIN *) &Main) != BUFF_SIZE)
						funct_error();
#ifdef DEBUGGING
					count++;
#endif
                }
				STI(); // end critical section

				CLI(); // begin critical section
				// check if we should stop logging
#ifdef DEBUGGING
				if(count == DEBUG_LEN || PIR5bits.ERRIF) {
#else
				if(rpm_h * 256 + rpm_l < RPM_COMP || PIR5bits.ERRIF || Main.EngTO == 1) {
#endif
					STI(); // end critical section
					// close csv data file
					if(FSfclose(pointer))
						funct_error();

					CLI(); // begin critical section
					// stop collecting data in the buffers
					Main.BufferALen = MEDIA_SECTOR_SIZE;
					Main.BufferBLen = MEDIA_SECTOR_SIZE;
					Main.Swap = FALSE;
					Main.BufferAFull = TRUE;
					rpm_l = 0;
					rpm_h = 0;
					STI(); // end critical section

					break;
				}
				STI(); // end critical section
			}// logging loop
		}// data acq start control
		STI(); // end critical section
	}// data acq loop

	return;
}


/*
 *  Local Functions
 */

/*
 *	void funct_error(void)
 *
 *	Description:	This function will be called when a serious error is encountered
 *					When this occurs the program will stay here until reset.
 *	Input(s): none
 *	Return Value(s): none
 *	Side Effects: This halts main program execution.
 */
void funct_error(void) {
	CLI();			// stop interrupts
	while(1);		// stay here
	return;
}

/*
 *	void service_FIFO(void)
 *
 *	Description:	This function will read messages from the ECAN buffers
 *					and package the information with a timestamp. It also reads
 *					RPM data for logging initiation.
 *	Input(s): none
 *	Return Value(s): none
 *	Side Effects:	This will modify rpm_l & rpm_h. Also it clears out the ECAN recieve
 *					buffers.
 */
void service_FIFO(void) {

	static unsigned char i;		// for loop counters
	static unsigned char j;
	static unsigned char msg[14];	// holds entire CAN message including timstamp
	unsigned long id;			// holds CAN msgID
	unsigned char data[8];		// holds CAN data bytes
	unsigned char dataLen;		// holds number of CAN data bytes
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

			// record message time in seconds
			rpmLast = seconds;
		}

		// look for beacon event
		// if no beacon event then discard the message
		if(id == BEACON_ID) {
			if(dataLen != 6)
				dataLen = 0;
		}

		// ensure there's data to record
		if(dataLen > 0) {

			// collect message data before sending to buffer
			for (j = 0; j < dataLen + ID_PLUS_TIME; j++) {
				// get message ID
				if (j < MSG_ID_LEN) {
					msg[j] = ((unsigned char *) (&id))[j];
				}
				// get data bytes
				else if (j < MSG_ID_LEN + dataLen) {
					msg[j] = data[j - MSG_ID_LEN];
				}
				// get CCP2 timer1 value that forms 2 LSBs of the timestamp
				else if (j < MSG_ID_LEN + dataLen + (TIMESTAMP_LEN / 2)) {
					msg[j] = ((unsigned char*) timestamp)[j - MSG_ID_LEN - dataLen + i * 2];
				}
				// get seconds value that forms 2 MSBs of the timsetamp
				else if (j < dataLen + ID_PLUS_TIME) {
					msg[j] = ((unsigned char *) timestamp_2)[j - MSG_ID_LEN - dataLen - (TIMESTAMP_LEN / 2) + i * 2];
				}
			}

			// send messgage to buffer
			append_write_buffer(msg, dataLen + ID_PLUS_TIME);
		}
	}

	return;
}

/*
 *	void append_write_buffer(static const unsigned char * temp,
 *								static unsigned char applen)
 *
 *	Description:	This function will decide how to append data to the user buffers.
 *					It will try to put the entire message in BufferA if possible.
 *					Otherwise it will either put a partial message in BufferA
 *					and BufferB or the entire message in BufferB.
 *	Input(s):	temp - data array that holds one whole CAN message with its timestamp
 *				applen - the length of the data array
 *	Return Value(s): none
 *	Side Effects: This will modify Main.
 */
void append_write_buffer(static const unsigned char * temp, static unsigned char applen) {

	static unsigned char offset;
	static unsigned char holder;

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

/*
 *	void buff_cat(static unsigned char *WriteBuffer, static const unsigned char *writeData,
 *				static unsigned int *bufflen, static const unsigned char applen,
 *				static const unsigned char offset)
 *
 *	Description: This function will append data to the user buffers byte by byte.
 *	Input(s):	WriteBuffer - pointer to the buffer we will write to
 *				writeData - pointer to the data array we will write
 *				bufflen - pointer to the variable holding the buffer length of the buffer we are writing to
 *				applen - the length of the data to be written
 *				offset - the index offset for the data to write
 *	Return Value(s): none
 *	Side Effects: This will modify BufferA & BufferB.
 */
void buff_cat(static unsigned char *WriteBuffer, static const unsigned char *writeData,
				static unsigned int *bufflen, static const unsigned char applen,
				static const unsigned char offset) {

	static unsigned char i;

	// increment through the data bytes sending them to the write buffer
	for(i = 0; i < applen; i++) {
		WriteBuffer[*bufflen + i] = writeData[i + offset];
	}

	// increment the data length count for the writing buffer
	*bufflen += (unsigned int) applen;

	return;
}

/*
 *	void swap_len(void)
 *
 *	Description: Swaps the buffer length members of Main.
 *	Input(s): none
 *	Return Value(s): none
 *	Side Effects: This will modify Main.
 */
void swap_len(void) {

	static unsigned int temp;
	temp = Main.BufferALen;

	// swap lengths
	Main.BufferALen = Main.BufferBLen;
	Main.BufferBLen = temp;

	return;
}

/*
 *	void swap_buff(void)
 *
 *	Description: Swaps the buffer pointer members of Buff.
 *	Input(s): none
 *	Return Value(s): none
 *	Side Effects: This will modify Buff.
 */
void swap_buff(void) {

	static unsigned char *temp;
	temp = Buff.BufferA;

	// swap pointers
	Buff.BufferA = Buff.BufferB;
	Buff.BufferB = temp;

	return;
}
