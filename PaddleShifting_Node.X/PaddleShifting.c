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
#pragma config FOSC = HS2       // Oscillator (HS oscillator (High power, 16MHz - 25MHz))
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

volatile uint32_t millis = 0;  // Holds timer0 rollover count
volatile uint32_t seconds = 0; // Holds timer1 rollover count

volatile uint16_t rpm = 0;     // Holds engine RPM data from CAN

volatile uint8_t prev_switch_up = 0; // Previous switch state of SHIFT_UP
volatile uint8_t prev_switch_dn = 0; // Previous switch state of SHIFT_DN

volatile uint8_t queue_up = 0; // Number of queued upshifts
volatile uint8_t queue_dn = 0; // Number of queued downshifts
volatile uint8_t queue_nt = 0; // Number of queued neutral shifts

volatile uint8_t gear = GEAR_FAIL; // Current gear

volatile uint32_t lockout_tmr = 0; // Holds millis value of last lockout set

// ECAN variables
uint32_t id;              // Holds CAN msgID
uint8_t data[8];				  // Holds CAN data bytes
uint8_t dataLen;				  // Holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags;	// Holds information about recieved message

void main(void) {
  /**
   * Declare local variables
   */
  uint32_t diag_send_tmr, temp_samp_tmr, gear_samp_tmr = 0;
  uint8_t data[8] = {0};
  int16_t temp = 0; // Holds temperature reading of PCB in units of [C/0.005]

  /**
   * General initialization
   */
  init_unused_pins();
  init_oscillator();
  init_timer0();
  init_timer1();

  /**
   * Initialize I/O pins
   */

  TEMP_TRIS = INPUT;
  TERM_TRIS = INPUT;
  GEAR_POS_TRIS = INPUT;
  SHIFT_UP_TRIS = INPUT;
  SHIFT_DN_TRIS = INPUT;
  SHIFT_NT_TRIS = INPUT;

  ACT_UP_TRIS = OUTPUT;
  ACT_UP_LAT = 0;
  ACT_DN_TRIS = OUTPUT;
  ACT_DN_LAT = 0;

  /**
   * Setup Peripherals
   */

  ANCON0 = 0b00000110; // AN1, AN2 analog, rest digital
  ANCON1 = 0x00;       // Default all pins to digital
  init_ADC();

  // Programmable termination
  TERM_TRIS = OUTPUT;
  TERM_LAT = 1; // Terminating

  ECANInitialize();

  // Interrupts setup
  INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
  INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
  RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

  // Main loop
  while(1) {

    /**
     * Sample GEAR_POS signal
     */
    if(millis - gear_samp_tmr >= GEAR_SAMP_INTV) {
      uint16_t gear_samp = sample(ADC_GEAR_CHN);

      /**
       * TODO: Apply linearization of sensor and CAN scalars to set value
       */
      gear_samp_tmr = millis;
    }

    /**
     * Sample TEMP signal
     */
    if(millis - temp_samp_tmr >= TEMP_SAMP_INTV) {
      uint16_t temp_samp = sample(ADC_TEMP_CHN);

      /**
       * Temp [C] = (Sample [V] - 0.75 [V]) / 10 [mV/C]
       * Temp [C] = ((5 * (temp_samp / 4095)) [V] - 0.75 [V]) / 0.01 [V/C]
       * Temp [C] = (5 * (temp_samp / 40.95)) - 75) [C]
       * Temp [C] = (temp_samp * 0.1221001221) - 75 [C]
       * Temp [C / 0.005] = 200 * ((temp_samp * 0.1221001221) - 75) [C / 0.005]
       * Temp [C / 0.005] = (temp_samp * 24.42002442) - 15000 [C / 0.005]
       */

      temp = (((double) temp_samp) * 24.42002442) - 15000.0;
      temp_samp_tmr = millis;
    }

    /**
     * Send diagnostic CAN message
     */
    if(millis - diag_send_tmr >= DIAG_MSG_SEND) {
      ((uint16_t*) data)[UPTIME_BYTE / 2] = seconds;
      ((int16_t*) data)[PCB_TEMP_BYTE / 2] = temp;
      data[GEAR_BYTE] = gear;
      data[QUEUE_NT_BYTE] = queue_nt;
      data[QUEUE_UP_BYTE] = queue_up;
      data[QUEUE_DN_BYTE] = queue_dn;

      ECANSendMessage(PADDLE0_ID, data, 8, ECAN_TX_FLAGS);
      diag_send_tmr = millis;
    }
  }
}

/**
 * void high_isr(void)
 *
 * Function to service high-priority interrupts
 */
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
    TMR0L = 0x84; // Adjusted from TMR0_RELOAD experimentally
    millis++;

    // Check for a new shift_up switch press
    if(SHIFT_UP_SW && !prev_switch_up) {
      process_upshift_press();
    }
    prev_switch_up = SHIFT_UP_SW;

    // Check for a new shift_dn switch press
    if(SHIFT_DN_SW && !prev_switch_dn) {
      process_downshift_press();
    }
    prev_switch_dn = SHIFT_DN_SW;
  }

  // Check for timer1 rollover
  if (PIR1bits.TMR1IF) {
    PIR1bits.TMR1IF = 0;
    TMR1H = TMR1H_RELOAD;
    TMR1L = TMR1L_RELOAD;
    seconds++;
  }

	// Check for received CAN message
	if (PIR5bits.RXB1IF) {
		PIR5bits.RXB1IF = 0; // Reset the flag

		// Get data from receive buffer
		ECANReceiveMessage(&id, data, &dataLen, &flags);
		if(id == MOTEC0_ID) {
			((uint8_t*) &rpm)[0] = data[ENG_RPM_BYTE + 1];
      ((uint8_t*) &rpm)[1] = data[ENG_RPM_BYTE];
		}
  }
}

/**
 * uint16_t sample(const uint8_t ch)
 *
 * This function reads the analog voltage of a pin and then returns the value
 *
 * @param ch - which pin to sample
 */
uint16_t sample(const uint8_t ch) {
  SelChanConvADC(ch); // Configure which pin you want to read and start A/D converter
  while(BusyADC()); // Wait for complete conversion
  return ReadADC();
}

/**
 * void process_upshift_press(void)
 *
 * After registering an upshift press, determine if we should increment the queue
 */
void process_upshift_press(void) {
  if (queue_nt == 1) {
    return;
  }

  if (millis - lockout_tmr < LOCKOUT_DUR) {
    return;
  }

  if (gear == GEAR_FAIL) {
    //TODO: Implement basic control
    return;
  }

  if (SHIFT_NT_BT) { // Pressed while holding the neutral button
    if (gear == GEAR_NEUT) {
      queue_nt = 0;
    } else if (gear == 1 || gear == 2) {
      queue_nt = 1;
      queue_up = 0;
      queue_dn = 0;
    } else {
      queue_dn = 1;
      lockout_tmr = millis;
    }
  } else { // Pressed while not holding the neutral button
    if (queue_dn > 0) {
      queue_dn = 0;
      queue_up = 0;
      return;
    }

    if (gear == GEAR_NEUT) {
      queue_dn = 1;
      lockout_tmr = millis;
      return;
    }

    if (queue_up < 6 - gear) {
      queue_up++;
      lockout_tmr = millis;
    }
  }
}

/**
 * void process_downshift_press(void)
 *
 * After registering an downshift press, determine if we should increment the queue
 */
void process_downshift_press(void) {
  if (queue_nt == 1) {
    return;
  }

  if (millis - lockout_tmr < LOCKOUT_DUR) {
    return;
  }

  if (gear == GEAR_FAIL) {
    //TODO: Implement basic control
    return;
  }

  if (SHIFT_NT_BT) { // Pressed while holding the neutral button
    if (gear == GEAR_NEUT) {
      queue_nt = 0;
    } else if (gear == 1 || gear == 2) {
      queue_nt = 1;
      queue_up = 0;
      queue_dn = 0;
    } else {
      queue_dn = 1;
      lockout_tmr = millis;
    }
  } else { // Pressed while not holding the neutral button
    if (queue_up > 0) {
      queue_dn = 0;
      queue_up = 0;
      return;
    }

    if (queue_dn + 1 < gear) {
      queue_dn++;
      lockout_tmr = millis;
    }
  }
}