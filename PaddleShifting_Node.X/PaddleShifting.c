/**
 * PaddleShifting
 *
 * Processor:    PIC18F46K80
 * Complier:     Microchip C18
 * Author:       Andrew Mass
 * Created:      2015-2016
 */

#include "PaddleShifting.h"

/**
 * PIC18F46K80 Configuration Bits Settings
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

/**
 * Global Variables
 */

volatile uint32_t millis;  // Holds timer0 rollover count
volatile uint16_t rpm;     // Holds engine RPM data from CAN

// ECAN variables
uint32_t id;              // Holds CAN msgID
uint8_t data[8];				  // Holds CAN data bytes
uint8_t dataLen;				  // Holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags;	// Holds information about recieved message

/**
 * Interrupts
 */

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
    WriteTimer0(0x85);  // Load timer registers (0xFF (max val) - 0x7D (125) = 0x82)
    millis++;
  }

	// Check for received CAN message
	if(PIR5bits.RXB1IF) {
		PIR5bits.RXB1IF = 0; // Reset the flag

		// Get data from receive buffer
		ECANReceiveMessage(&id, data, &dataLen, &flags);
		if(id == MOTEC0_ID) {
			((uint8_t*) &rpm)[0] = data[ENG_RPM_BYTE + 1];
      ((uint8_t*) &rpm)[1] = data[ENG_RPM_BYTE];
		}
  }
}

void main(void) {
  init_oscillator();
  init_timer0();
  //init_timer1();
	init_unused_pins();

  /**
   * Peripherals Setup
   */

  ANCON0 = 0x00;    // Default all pins to digital
  ANCON1 = 0x00;    // Default all pins to digital

  // Programmable termination
  TERM_TRIS = OUTPUT;
  TERM_LAT = 0; // Not terminating

	ECANInitialize();

  // Interrupts setup
	INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
	INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
	RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

  // Main loop
  while(1);
}
