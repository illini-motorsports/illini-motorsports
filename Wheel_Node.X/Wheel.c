/******************************************************************************
 *
 *					Wheel Node C Main Code
 *
 ******************************************************************************
 * FileName:		Wheel.c
 * Dependencies:	p18F46K80.h,
 *                  timers.h,
 *                  Wheel.h,
 *                  ECAN.h,
 *                  spi.h,
 *                  adc.h
 * Processor:		PIC18F46K80
 * Complier:		Microchip C18
 * Version:         1.00
 * Author:          George Schwieters
 * Created:         2013-2014

*******************************************************************************
	USER REVISON HISTORY
//
//
//

*******************************************************************************/

/***********************************************/
/*  Header Files                               */
/***********************************************/

#include "p18F46K80.h"
#include "timers.h"
#include "Wheel.h"
#include "ECAN.h"
#include "spi.h"
#include "adc.h"


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

volatile unsigned int millis;		// holds timer0 rollover count

unsigned int refreshTime[2], blinkTimer[2], holdTimer[2];
BYTE blinkStates[2], holdText[2], displayStates[2];

// ECAN variables
unsigned long id;           // holds CAN msgID
BYTE data[8];				// holds CAN data bytes
BYTE dataLen;				// holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags;	// holds information about recieved message

volatile int chan[NUM_CHAN];
volatile int gear;

BYTE d_place_arr[NUM_CHAN] =	{2,		// oil temperature
                                2,		// engine temperature
                                2,		// battery voltage
                                2,		// oil pressure
                                2,		// ground speed
                                0};		// engine RPM
BYTE num_arr[12] = {NUM_0, NUM_1, NUM_2, NUM_3, NUM_4, NUM_5, NUM_6, NUM_7, NUM_8, NUM_9, BLANK, CHAR_N};
BYTE text_arr[NUM_CHAN][3] =	{{BLANK, CHAR_O, CHAR_t},	// oil temperature
                                {BLANK, CHAR_E, CHAR_t},	// engine temmperature
                                {CHAR_b, CHAR_A, CHAR_t},	// battery voltage
                                {BLANK, CHAR_O, CHAR_P},	// oil pressure
                                {CHAR_S, CHAR_P, CHAR_d},	// ground speed
                                {CHAR_t, CHAR_A, CHAR_c}};	// engine RPM


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
  Input:
    none
  Return Values:
    none
  Side Effects:
	Transmits messages on CAN
	Reloads the timer0 registers and increments millis
	Resets the interrupt flags after servicing them
	Fills the array of channel data
  Description:

  *********************************************************************************/
#pragma code high_vector = 0x08
void high_vector(void) {
	_asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr(void) {

	// check for timer0 rollover indicating a millisecond has passed
	if (INTCONbits.TMR0IF) {
		INTCONbits.TMR0IF = FALSE;
		WriteTimer0(0x85);		// load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
		millis++;
	}
	// check for recieved CAN message
	if(PIR5bits.RXB1IF) {
		PIR5bits.RXB1IF = FALSE; // reset the flag
		// get data from recieve buffer
		ECANReceiveMessage(&id, data, &dataLen, &flags);
		bufferData();	// put data in an array
	}

	return;
}

void main(void) {

	/*************************
	 * Variable Declarations *
	 *************************/

    BYTE radio_sw[2], drs_over_sw[2], fan_over_sw[2], fuel_map_sw[2], paddle_l_sw[2], paddle_r_sw[2];
    BYTE ADLmsg[8];
	BYTE cycleStates[2], intensity;
	unsigned int bounceTimer[2];
	unsigned int CAN_tmr;


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

	// turn on and configure the A/D converter module
	OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_4_TAD, ADC_CH0 & ADC_INT_OFF, ADC_REF_VDD_VDD & ADC_REF_VDD_VSS & ADC_NEG_CH0);
	ANCON0 = 0b00100111;	// AN0 - 2 and AN5 are analog
	ANCON1 = 0x00;          // rest are digital
	TRISAbits.TRISA0 = INPUT;
	TRISAbits.TRISA1 = INPUT;
	TRISAbits.TRISA2 = INPUT;
	TRISAbits.TRISA5 = INPUT;

	// turn on and configure the TIMER1 oscillator
	OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
	WriteTimer0(0x82);		// load timer register
	millis = 0;				// clear milliseconds count
	INTCONbits.TMR0IE = 1;	// turn on timer0 interupts

    // SPI setup
    SSPSTATbits.CKE = 1;		// SPI Clock Select, 1 = transmit on active to idle
    SSPCON1bits.CKP = 0;		// Clock Polarity Select, 0 = low level is idle state
    SSPCON1bits.SSPM = 0b1010;	// Clk Frequecy (Note: FOSC = 64MHz)
    SSPCON1bits.SSPEN = 1;      // SPI Enable, 1 enables

    // SPI pin I/O setup
    TRISCbits.TRISC3 = OUTPUT;	// SCK
    TRISCbits.TRISC5 = OUTPUT;	// SDO
    TRISDbits.TRISD3 = OUTPUT;	// CS
    CS = 1;

	// driver set up
	intensity = 0x0F;
	driver_write(DISP_MODE, NORMAL);		// leave test mode
	driver_write(SHUTDOWN, SHUTDOWN_OFF);	// leave shutdown mode
	driver_write(INTENSITY, intensity);		// set brightness to highest
    driver_write(SCAN, FULL_SCAN);          // Set scan to all digits
    driver_write(DECODE, NO_DECODE);        // Decoding disabled

	// set displays to display zero
	write_gear(0);
	write_num(0, 2, LEFT);
	write_num(0, 2, RIGHT);

	// intialize states
	cycleStates[LEFT] = CYCLE_L;
	cycleStates[RIGHT] = CYCLE_R;
	holdText[LEFT] = holdText[RIGHT] = TRUE;
	refreshTime[LEFT] = refreshTime[RIGHT] = holdTimer[LEFT] = holdTimer[RIGHT] =
						blinkTimer[LEFT] = blinkTimer[RIGHT] = millis;
	displayStates[LEFT] = OIL_T;
	displayStates[RIGHT] = ENGINE_T;

	ECANInitialize();		// setup ECAN

    // interrupts setup
	INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
	INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
	RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

	TRISCbits.TRISC6 = OUTPUT;	// programmable termination
	TERM_LAT = FALSE;

	while(1) {

		// check for change in button state
		if(cycleStates[LEFT] != CYCLE_L & millis - bounceTimer[LEFT] > BOUNCE_TIME) {
			// save new state
			cycleStates[LEFT] = CYCLE_L;
			bounceTimer[LEFT] = millis;
			// only change display if button is low
			if(!cycleStates[LEFT]) {
				if(++displayStates[LEFT] == NUM_CHAN)
					displayStates[LEFT] = 0;
				// put the appropriate text on the displays and
				// get the current time for timing logic
				updateText(LEFT, displayStates);
				holdText[LEFT] = TRUE;
				blinkTimer[LEFT] = holdTimer[LEFT] = millis;
			}
		}
		if(cycleStates[RIGHT] != CYCLE_R  & millis - bounceTimer[RIGHT] > BOUNCE_TIME) {
			cycleStates[RIGHT] = CYCLE_R;
			bounceTimer[RIGHT] = millis;
			if(!cycleStates[RIGHT]) {
				if(++displayStates[RIGHT] == NUM_CHAN)
					displayStates[RIGHT] = 0;
				updateText(RIGHT, displayStates);
				holdText[RIGHT] = TRUE;
                blinkTimer[RIGHT] = holdTimer[RIGHT] = millis;
			}
		}

		// update left and right displays with text or numerical data
		updateDisp(LEFT);
		updateDisp(RIGHT);
		write_gear(gear);

        // radio button
        if(!RADIO) {
            radio_sw[0] = 0x13;
            radio_sw[1] = 0x88;
        }
        else
            *(int *)radio_sw = 0;
#if 0
        // paddle switches
        if(PADDLE_L) {
            paddle_l_sw[0] = 0x13;
            paddle_l_sw[1] = 0x88;
        }
        else
            *(int *)paddle_l_sw = 0;
        if(PADDLE_R) {
            paddle_r_sw[0] = 0x13;
            paddle_r_sw[1] = 0x88;
        }
        else
            *(int *)paddle_r_sw = 0;
#endif
        // DRS override switch
        if(DRS_OVER) {
            drs_over_sw[0] = 0x13;
            drs_over_sw[1] = 0x88;
        }
        else
            *(int *)drs_over_sw = 0;
        // fan override switch
        if(FAN_OVER) {
            fan_over_sw[0] = 0x13;
            fan_over_sw[1] = 0x88;
        }
        else
            *(int *)fan_over_sw = 0;
        // fuel map switch
        if(FUEL_MAP) {
            fuel_map_sw[0] = 0x13;
            fuel_map_sw[1] = 0x88;
        }
        else
            *(int *)fuel_map_sw = 0;

		if(millis - CAN_tmr > CAN_PER) {
			CAN_tmr = millis;
			// send out the first three sampled switches
			ADLmsg[0] = 0x00;
			ADLmsg[1] = 0x00;
			ADLmsg[ADL1] = radio_sw[0];
			ADLmsg[ADL1 + 1] = radio_sw[1];
			ADLmsg[ADL2] = fan_over_sw[0];
			ADLmsg[ADL2 + 1] = fan_over_sw[1];
			ADLmsg[ADL3] = fuel_map_sw[0];
			ADLmsg[ADL3 + 1] = fuel_map_sw[1];
			ECANSendMessage(ADLid, ADLmsg, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
			// send out first three rotary encoders
			ADLmsg[0] = 0x01;
			ADLmsg[1] = 0x00;
			ADLsample(ADLmsg, ADL4, LAUNCH_ROT);
			ADLsample(ADLmsg, ADL5, TRAC_ROT);
			ADLsample(ADLmsg, ADL6, DRS_ROT);
			ECANSendMessage(ADLid, ADLmsg, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
		}



	} // end main loop

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
	TRISAbits.TRISA3 = OUTPUT;
	TRISAbits.TRISA5 = OUTPUT;
//	TRISAbits.TRISA6 = OUTPUT; OSC2
//	TRISAbits.TRISA7 = OUTPUT; OSC1

	TRISBbits.TRISB0 = OUTPUT;
	TRISBbits.TRISB1 = OUTPUT;
//	TRISBbits.TRISB2 = OUTPUT; CANTX
//	TRISBbits.TRISB3 = OUTPUT; CANRX
//	TRISBbits.TRISB4 = OUTPUT; RB4
//	TRISBbits.TRISB5 = OUTPUT; RB5
	TRISBbits.TRISB6 = OUTPUT;
	TRISBbits.TRISB7 = OUTPUT;

//	TRISCbits.TRISC0 = OUTPUT; SOSCO
//	TRISCbits.TRISC1 = OUTPUT; SOSCI
	TRISCbits.TRISC2 = OUTPUT;
//	TRISCbits.TRISC3 = OUTPUT; SCK
//	TRISCbits.TRISC4 = OUTPUT; SDI
//	TRISCbits.TRISC5 = OUTPUT; SDO
//	TRISCbits.TRISC6 = OUTPUT; RC6
//	TRISCbits.TRISC7 = OUTPUT; RC7

	TRISDbits.TRISD0 = OUTPUT;
	TRISDbits.TRISD1 = OUTPUT;
	TRISDbits.TRISD2 = OUTPUT;
//	TRISDbits.TRISD3 = OUTPUT; RD3
//	TRISDbits.TRISD4 = OUTPUT; RD4
//	TRISDbits.TRISD5 = OUTPUT; RD5
// 	TRISDbits.TRISD6 = OUTPUT; RD6
	TRISDbits.TRISD7 = OUTPUT;

//	TRISEbits.TRISE0 = OUTPUT; AN5
//	TRISEbits.TRISE1 = OUTPUT; RE1
//	TRISEbits.TRISE2 = OUTPUT; RE2
//	TRISEbits.TRISE3 = OUTPUT; MCLR

	// then set pins low
//	LATAbits.LATA0 = 0;
//	LATAbits.LATA1 = 0;
//	LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
	LATAbits.LATA5 = 0;
//	LATAbits.LATA6 = 0;
//	LATAbits.LATA7 = 0;

	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 0;
//	LATBbits.LATB2 = 0;
//	LATBbits.LATB3 = 0;
//	LATBbits.LATB4 = 0;
//	LATBbits.LATB5 = 0;
	LATBbits.LATB6 = 0;
	LATBbits.LATB7 = 0;

//	LATCbits.LATC0 = 0;
//	LATCbits.LATC1 = 0;
	LATCbits.LATC2 = 0;
//	LATCbits.LATC3 = 0;
//	LATCbits.LATC4 = 0;
//	LATCbits.LATC5 = 0;
//	LATCbits.LATC6 = 0;
//	LATCbits.LATC7 = 0;

	LATDbits.LATD0 = 0;
	LATDbits.LATD1 = 0;
	LATDbits.LATD2 = 0;
//	LATDbits.LATD3 = 0;
//	LATDbits.LATD4 = 0;
//	LATDbits.LATD5 = 0;
//	LATDbits.LATD6 = 0;
	LATDbits.LATD7 = 0;

//	LATEbits.LATE0 = 0;
//	LATEbits.LATE1 = 0;
//	LATEbits.LATE2 = 0;
//	LATEbits.LATE3 = 0;

	return;
}

/******************************************************************************
  Function:
    void driver_write(BYTE addr, BYTE data)
  Summary:
	Function to send out data to a specified address to the MAXIM LED driver
  Conditions:
	* SPI module must be configured
  Input:
	addr - LED driver address to transmit
	data - LED driver data value to transmit
  Return Values:
    none
  Side Effects:
	none
  Description:

  ****************************************************************************/

void driver_write(BYTE addr, BYTE data) {

	// first select the driver
	CS = 0;
	// send out address byte then the data byte
	WriteSPI(addr);
	WriteSPI(data);
	// deselect the device
	CS = 1;

	return;
}

/******************************************************************************
  Function:
    void write_gear(BYTE gear)
  Summary:
	Function to write a gear value to the wheel's middle seven segment display
  Conditions:
	* SPI module must be configured
  Input:
	gear - numerical value of gear to display
  Return Values:
    none
  Side Effects:
	node
  Description:

  ****************************************************************************/

void write_gear(BYTE gear) {

	// convert gear value to data to be sent to display driver
	if(gear != 7)
		gear = num_arr[gear];
	else
		gear = num_arr[11];

	// write the gear position
	driver_write(DIG6, gear);

	return;
}

/******************************************************************************
  Function:
    void blank_display(BYTE side)
  Summary:
	Function to clear a display
  Conditions:
	* SPI display must be ocnfigured
  Input:
	side - specifies which three seven segment displays to clear
  Return Values:
    none
  Side Effects:
	none
  Description:

  ****************************************************************************/

void blank_display(BYTE side) {

	int i;

	// check which side to blank
	if(side == BOTH) {
		// loop through digits and blank them
		for(i = 1; i < 8; i++) {
			driver_write(i, BLANK);
		}
	}
	else if(side == LEFT) {
		for(i = 1; i < 4; i++) {
			driver_write(i, BLANK);
		}
	}
	// right side
	else {
		for(i = 4; i < 7; i++) {
			driver_write(i, BLANK);
		}
	}

	return;
}

/******************************************************************************
  Function:
    void write_num(int data, BYTE d_place, BYTE side)
  Summary:
	Function to write a number to a set of displays
  Conditions:
	* SPI module must be configured
  Input:
	d_place - where to place the decimal place
	side - which side to write string of numbers
	data - numerical value to write to the displays
  Return Values:
    none
  Side Effects:
	none
  Description:

  ****************************************************************************/

void write_num(int data, BYTE d_place, BYTE side) {

	BYTE num_0, num_1, num_2;

	// get individual digits of full number
	num_2 = data % 10;
	num_1 = (data % 100) / 10;
	num_0 = (data % 1000) / 100;

	// convert values to data bytes for display driver
	num_0 = num_arr[num_0];
	num_1 = num_arr[num_1];
	num_2 = num_arr[num_2];

	// add decimal points to numbers
	if(d_place) {
		if(d_place == 1) {
			num_0 |= NUM_DP;
		}
		else if(d_place == 2) {
			num_1 |= NUM_DP;
		}
		else {
			num_2 |= NUM_DP;
		}
	}

	// write the number to the indicated side
	if(!side) {
		driver_write(DIG2, num_0);
		driver_write(DIG1, num_1);
		driver_write(DIG0, num_2);
	}
	else {
		driver_write(DIG3, num_0);
		driver_write(DIG4, num_1);
		driver_write(DIG5, num_2);
	}

	return;
}

/******************************************************************************
  Function:
    void updateText(BYTE side, BYTE *state)
  Summary:
	Send text to a set of displays
  Conditions:
	* SPI module must be configured
  Input:
	side - whichside to put the text
	state - which type of data we are displaying
  Return Values:
    none
  Side Effects:
	none
  Description:

  ****************************************************************************/

void updateText(BYTE side, BYTE *state) {

	// update left or right display with text
	if(!side) {
		// send out characters one at a time
		driver_write(DIG2, text_arr[state[side]][0]);
		driver_write(DIG1, text_arr[state[side]][1]);
		driver_write(DIG0, text_arr[state[side]][2]);
	}
	else {
		driver_write(DIG3, text_arr[state[side]][0]);
		driver_write(DIG4, text_arr[state[side]][1]);
		driver_write(DIG5, text_arr[state[side]][2]);
	}

	return;
}

/******************************************************************************
  Function:
    void updateDisp(BYTE side)
  Summary:
	Function to determine what should be displayed on the displays based on timers
  Conditions:
	* Timer0 module must be setup along with the oscillator being used
	* SPI module must be configured
  Input:
	side - which side to update
  Return Values:
    none
  Side Effects:
	none
  Description:

  ****************************************************************************/

void updateDisp(BYTE side) {

	// check if we are holding text
	if(holdText[side]) {
		// check if the hold time has passed
		if(millis - holdTimer[side] > HOLD_TIME) {
			holdText[side] = FALSE;
		}
		// carry out blinking of text
		else if(millis - blinkTimer[side] > BLINK_TIME) {
			// redisplay the text
			if(blinkStates[side]) {
				blinkStates[side] = FALSE;
				updateText(side, displayStates);
				blinkTimer[side] = millis;
			}
			// blank the displays
			else {
				blinkStates[side] = TRUE;
				blank_display(side);
				blinkTimer[side] = millis;
			}
		}
	}
	// data is being displayed
	else {
		if(millis - refreshTime[side] > REFRESH_TIME) {
			refreshTime[side] = millis;
			write_num(chan[displayStates[side]], d_place_arr[displayStates[side]], side);
		}
	}

	return;
}

/******************************************************************************
  Function:
    void bufferData(void)
  Summary:
	Function to take CAN data bytes and assign them to the appropriate part of the
	channel data array
  Conditions:
	none
  Input:
	none
  Return Values:
    none
  Side Effects:
	chan array is modified
  Description:

  ****************************************************************************/

void bufferData(void) {

	if(id == ECU_ID_0) {
		((BYTE*) &(chan[RPM]))[0] = data[RPM_BYTE + 1];
		((BYTE*) &(chan[RPM]))[1] = data[RPM_BYTE];
		((BYTE*) &(chan[OIL_P]))[0] = data[OIL_P_BYTE + 1];
		((BYTE*) &(chan[OIL_P]))[1] = data[OIL_P_BYTE];
		((BYTE*) &(chan[OIL_T]))[0] = data[OIL_T_BYTE + 1];
		((BYTE*) &(chan[OIL_T]))[1] = data[OIL_T_BYTE];
		chan[RPM] = chan[RPM] / 100;
	}
	else if(id == ECU_ID_1) {
		((BYTE*) &(chan[VOLTAGE]))[0] = data[VOLTAGE_BYTE + 1];
		((BYTE*) &(chan[VOLTAGE]))[1] = data[VOLTAGE_BYTE];
		((BYTE*) &(chan[ENGINE_T]))[0] = data[ENGINE_T_BYTE + 1];
		((BYTE*) &(chan[ENGINE_T]))[1] = data[ENGINE_T_BYTE];
		chan[VOLTAGE] = chan[VOLTAGE] / 10;
	}
	else if(id == ECU_ID_4) {
		((BYTE*) &(chan[SPEED]))[0] = data[GDN_SPD_BYTE + 1];
		((BYTE*) &(chan[SPEED]))[1] = data[GDN_SPD_BYTE];
		((BYTE*) &gear)[0] = data[GEAR_BYTE + 1];
		((BYTE*) &gear)[1] = data[GEAR_BYTE];
	}

	return;
}

/******************************************************************************
  Function:
    void ADLsample(BYTE *data, const BYTE ADLoffset, const BYTE ch)
  Summary:
	Function to sample an analog pin and place the value in an array
  Conditions:
    * A/D Converter must be configured
    * Analog pins must be initilized
  Input:
	*data - pointer to data array
	ADLoffset - where in the data array to place the data to conform to ADL
	protcol
	ch - the analog channel (pin) to sample
  Return Values:
    none
  Side Effects:
	sampled value is added to the data array passed
  Description:

  ****************************************************************************/
void ADLsample(BYTE *data, const BYTE ADLoffset, const BYTE ch) {

    unsigned int temp;
	SelChanConvADC(ch);	// configure which pin you want to read and start A/D converter

	while(BusyADC());	// wait for complete conversion

	// put result in data array in accordance with specified byte location
    temp = (unsigned int)ReadADC();
	modifyRotary(&temp);
    data[ADLoffset] = ((BYTE *)&temp)[1];
    data[ADLoffset + 1] = ((BYTE *)&temp)[0];

	return;
}

/******************************************************************************
  Function:
	void modifyRotary(int * sample)
  Summary:

  Conditions:

  Input:

  Return Values:
    none
  Side Effects:

  Description:

  ****************************************************************************/
void modifyRotary(unsigned int * sample) {
	// depending on the value of the sampled rotary postion
	// we will assign a new position that matches the pysical position
	if(*sample >= RANGE_LOW && *sample < RANGE_0)
		*sample = POS_6;
	else if(*sample >= RANGE_0 && *sample < RANGE_1)
		*sample = POS_5;
	else if(*sample >= RANGE_1 && *sample < RANGE_2)
		*sample = POS_4;
	else if(*sample >= RANGE_2 && *sample < RANGE_3)
		*sample = POS_3;
	else if(*sample >= RANGE_3 && *sample < RANGE_4)
		*sample = POS_7;
	else if(*sample >= RANGE_4 && *sample < RANGE_5)
		*sample = POS_2;
	else if(*sample >= RANGE_5 && *sample < RANGE_6)
		*sample = POS_1;
	else if(*sample >= RANGE_6 && *sample < RANGE_7)
		*sample = POS_0;
	else if(*sample >= RANGE_7 && *sample < RANGE_8)
		*sample = POS_9;
	else if(*sample >= RANGE_8 && *sample <= RANGE_HIGH)
		*sample = POS_8;
	return;
}