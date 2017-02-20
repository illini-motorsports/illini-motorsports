/**
 * DynoHub
 * 
 * Processor:	   PIC18F46K80
 * Complier:	   Microchip C18
 * Author:       Andrew Mass, George Schwieters
 * Created:			 2015-2016
 */

#include "DynoHub.h"

/***********************************************/
/* PIC18F46K80 Configuration Bits Settings     */
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
/* Global Variable Declarations                */
/***********************************************/

volatile unsigned long millis;  // holds timer0 rollover count
volatile unsigned int rpm;      // Holds engine RPM data from CAN

// Holds current value of cylinder lambda sensors
volatile unsigned int lambda0, lambda1, lambda2, lambda3;

// ECAN variables
unsigned long id;           // holds CAN msgID
BYTE data[8];				// holds CAN data bytes
BYTE dataLen;				// holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags;	// holds information about recieved message

/***********************************************/
/* Interrupts                                  */
/***********************************************/

/*********************************************************************************
  Interrupt Function:
    void high_isr(void)
  Summary:
    Function to service interrupts
  Conditions:
	Timer0 module must be setup along with the oscillator being used
	ECAN must be configured
  Input:
    none
  Return Values:
    none
  Side Effects:
	Reloads the timer0 registers and increments millis
	Resets the interrupt flags after servicing them
	Sets rpm
  Description:

  *********************************************************************************/
#pragma code high_vector = 0x08
void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr(void) {

    // Check for timer0 rollover indicating a millisecond has passed
    if (INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(0x85);    // Load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

	// check for recieved CAN message
	if(PIR5bits.RXB1IF) {
		PIR5bits.RXB1IF = FALSE; // reset the flag
		// get data from recieve buffer
		ECANReceiveMessage(&id, data, &dataLen, &flags);
		if(id == RPM_ID) {
			((BYTE*) &rpm)[0] = data[RPM_BYTE + 1];
            ((BYTE*) &rpm)[1] = data[RPM_BYTE];
		} else if (id == 0x460) {
            if(data[0] == 0) {
              lambda0 = ((((int)data[2]) << 8) & 0xFF00) | (data[1] & 0x00FF);
            }
        } else if (id == 0x461) {
            if(data[0] == 0) {
              lambda1 = ((((int)data[2]) << 8) & 0xFF000) | (data[1] & 0x00FF);
            }
        } else if (id == 0x462) {
            lambda2 = ((((int)data[1]) << 8) & 0xFF000) | (data[2] & 0x00FF);

        } else if (id == 0x463) {
            lambda3 = ((((int)data[1]) << 8) & 0xFF000) | (data[2] & 0x00FF);
          }
	}
}


/***********************************************/
/* Functions                                   */
/***********************************************/

// Sends data byte to address byte on the LED driver
void send(int address, int data) {
    CS = 0;
    WriteSPI(address);
    WriteSPI(data);
    CS = 1;
}

// Sets all LEDs to specified color
void set_all(int color) {
    send(DIG0, color);
    send(DIG1, color);
    send(DIG2, color);
    send(DIG3, color);
    send(DIG4, color);
    send(DIG5, color);
    send(DIG6, color);
    send(DIG7, color);
}

// Sets lights to NONE or REV_COLOR based on parameter
void set_lights(int max) {
  if(max > 0) send(DIG0, BLUE);
  else send(DIG0, NONE);

  if(max > 1) send(DIG1, BLUE);
  else send(DIG1, NONE); 

  if(max > 2) send(DIG2, BLUE);
  else send(DIG2, NONE);
  
  if(max > 3) send(DIG3, BLUE);
  else send(DIG3, NONE);
  
  if(max > 4) send(DIG4, BLUE);
  else send(DIG4, NONE);
  
  if(max > 5) send(DIG5, BLUE);
  else send(DIG5, NONE);
  
  if(max > 6) send(DIG6, BLUE);
  else send(DIG6, NONE);
  
  if(max > 7) send(DIG7, BLUE);
  else send(DIG7, NONE);
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
void init_unused_pins_dyno(void) {

	// first configure to outputs
	TRISAbits.TRISA0 = OUTPUT;
	TRISAbits.TRISA1 = OUTPUT;
	TRISAbits.TRISA2 = OUTPUT;
	TRISAbits.TRISA3 = OUTPUT;
	TRISAbits.TRISA5 = OUTPUT;
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
//	TRISDbits.TRISD3 = OUTPUT; RD3
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
	LATAbits.LATA5 = 0;
//	LATAbits.LATA6 = 0;
//	LATAbits.LATA7 = 0;

	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 0;
//	LATBbits.LATB2 = 0;
//	LATBbits.LATB3 = 0;
	LATBbits.LATB4 = 0;
	LATBbits.LATB5 = 0;
	LATBbits.LATB6 = 0;
	LATBbits.LATB7 = 0;

//	LATCbits.LATC0 = 0;
//	LATCbits.LATC1 = 0;
	LATCbits.LATC2 = 0;
//	LATCbits.LATC3 = 0;
//	LATCbits.LATC4 = 0;
//	LATCbits.LATC5 = 0;
	LATCbits.LATC6 = 0;
	LATCbits.LATC7 = 0;

	LATDbits.LATD0 = 0;
	LATDbits.LATD1 = 0;
	LATDbits.LATD2 = 0;
//	LATDbits.LATD3 = 0;
	LATDbits.LATD4 = 0;
	LATDbits.LATD5 = 0;
	LATDbits.LATD6 = 0;
	LATDbits.LATD7 = 0;

	LATEbits.LATE0 = 0;
	LATEbits.LATE1 = 0;
	LATEbits.LATE2 = 0;
//	LATEbits.LATE3 = 0;

	return;
}


/***********************************************/
/* Main Loop                                   */
/***********************************************/

void main(void) {

  /*************************
   * Variable Declarations *
   *************************/

  unsigned long blink_tmr, la_send_tmr = 0;

  lambda0 = 0;
  lambda1 = 0;
  lambda2 = 0;
  lambda3 = 0;
  
  init_oscillator();
  
  /*************************
   * Peripherals Setup     *
   *************************/
  
  ANCON0 = 0x00;    // Default all pins to digital
  ANCON1 = 0x00;    // Default all pins to digital
  
  // Turn on and configure the TIMER1 oscillator
  OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
  WriteTimer0(0x82);			// Load timer register
  millis = 0;					// Clear milliseconds count
  INTCONbits.TMR0IE = 1;		// Turn on timer0 interupts
  
  // SPI setup
  SSPSTATbits.CKE = 1;		// SPI Clock Select, 1 = transmit on active to idle
  SSPCON1bits.CKP = 0;		// Clock Polarity Select, 0 = low level is idle state
  SSPCON1bits.SSPM = 0b1010;	// Clk Frequecy (Note: FOSC = 64MHz)
  SSPCON1bits.SSPEN = 1;		// SPI Enable, 1 enables
  
  OpenSPI(SPI_FOSC_64, MODE_10, SMPEND);
  
  // SPI pin I/O setup
  TRISCbits.TRISC3 = OUTPUT;    // SCK
  TRISCbits.TRISC5 = OUTPUT;    // SDO
  TRISDbits.TRISD3 = OUTPUT;    // CS
  CS = 1;

	TRISCbits.TRISC6 = OUTPUT;	// programmable termination
	TERM_LAT = FALSE;

	ECANInitialize();		// setup ECAN

  // Interrupts setup
	INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
	INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
	RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

	init_unused_pins_dyno();

    
  /*************************
   * LED Driver Config     *
   *************************/
  
  send(SHUTDOWN, SHUTDOWN_OFF);   //Shutdown mode off
  send(DISP_MODE, NORMAL);		//Display mode normal
  send(DECODE, NO_DECODE);		//Decoding disabled
  send(SCAN, FULL_SCAN);			//Set scan to all digits
  send(INTENSITY, INTENSITY_MAX);	//Full intensity

  // Startup - Show full RED lights for a while before doing anything else
	blink_tmr = millis;
	while(1) {
		if(millis - blink_tmr < BLINK_TIME)
			set_all(RED);
		else if(millis - blink_tmr < BLINK_TIME * 2)
			set_all(RGB);
		else if(millis - blink_tmr < BLINK_TIME * 3)
			set_all(BLUE);
		else if(millis - blink_tmr < BLINK_TIME * 4)
			set_all(RED);
		else if(millis - blink_tmr < BLINK_TIME * 5)
			set_all(RGB);
		else if(millis - blink_tmr < BLINK_TIME * 6)
			set_all(BLUE);
		else if(millis - blink_tmr < BLINK_TIME * 7)
			set_all(RED);
		else if(millis - blink_tmr < BLINK_TIME * 8)
			set_all(RGB);
		else if(millis - blink_tmr < BLINK_TIME * 9)
			set_all(BLUE);
		else
			break;
	}

  while(1) {
    // Send out latest values of cylinder lambdas in ADL format
    if(millis - la_send_tmr >= 500) {
      unsigned char data[8] = {0};
      unsigned char data1[8] = {0};

      la_send_tmr = millis;

      // ADL 1, 2, 3
      ((unsigned int*) data)[0] = 0;
      ((unsigned int*) data)[1] = lambda0;
      ((unsigned int*) data)[2] = lambda1;
      ((unsigned int*) data)[3] = lambda2;
      ECANSendMessage(0x500, (BYTE *) data, 8, ECAN_TX_FLAGS);

      // ADL 4
      ((unsigned int*) data1)[0] = 0x1;
      ((unsigned int*) data1)[1] = lambda3;
      ECANSendMessage(0x500, (BYTE *) data1, 8, ECAN_TX_FLAGS);
    }

    // Sets certain lights to NONE or REV_COLOR based on rpm value        
    if(rpm >= REV_RANGE_LIMIT) {
			if(millis - blink_tmr < BLINK_TIME)
				set_all(REV_LIMIT_COLOR);
			else if(millis - blink_tmr < BLINK_TIME * 2)
				set_all(NONE);
			else
				blink_tmr = millis;
    } else if(rpm >= REV_RANGE_8) {
      set_lights(8);
    } else if(rpm >= REV_RANGE_7) {
      set_lights(7);
    } else if(rpm >= REV_RANGE_6) {
      set_lights(6);
    } else if(rpm >= REV_RANGE_5) {
      set_lights(5);
    } else if(rpm >= REV_RANGE_4) {
      set_lights(4);
    } else if(rpm >= REV_RANGE_3) {
      set_lights(3);
    } else if(rpm >= REV_RANGE_2) {
      set_lights(2);
    } else if(rpm >= REV_RANGE_1) {
      set_lights(1);
    } else {
      set_all(NONE);
    } // set LEDs based on RPM
  } // end main loop
}
