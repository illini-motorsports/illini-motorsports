/**
 * AnalogHub
 *
 * Processor:    PIC18F46K80
 * Compiler:     Microchip C18
 * Author:       Andrew Mass
 * Created:      2015-2016
 */

#include "AnalogHub.h"

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

uint32_t fast_send_tmr, med_send_tmr, slow_send_tmr,
    diag_send_tmr = 0; // Various millis timers

#if FRONT
uint8_t radio_sw = 0; // State variable updated by PDM switch state CAN message
#endif

// ECAN variables
uint32_t id = 0;             // Holds CAN msgID
uint8_t data[8] = {0};			 // Holds CAN data bytes
uint8_t dataLen = 0;				 // Holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags = 0; // Holds information about recieved message

/**
 * void main(void)
 *
 * Main program execution function
 */
void main(void) {

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

  // Init CAN termination pin
  TERM_TRIS = INPUT;

  // Init RADIO pins
#if FRONT
  RADIO0_TRIS = INPUT;
  RADIO1_TRIS = INPUT;
#endif

#if FRONT // Init FRONT ADC pins
  ADC_SPFL_TRIS = INPUT;
  ADC_SPFR_TRIS = INPUT;
  ADC_BPF_TRIS = INPUT;
  ADC_BPR_TRIS = INPUT;
  ADC_STRP_TRIS = INPUT;
  ADC_APPS0_TRIS = INPUT;
  ADC_APPS1_TRIS = INPUT;
  ADC_PTDP_TRIS = INPUT;

  ADC_UAN0_TRIS = OUTPUT;
  ADC_UAN0_LAT = 0;
#elif REAR // Init REAR ADC pins
  ADC_SPRL_TRIS = INPUT;
  ADC_SPRR_TRIS = INPUT;
  ADC_EOS_TRIS = INPUT;
  ADC_BCD_TRIS = INPUT;
  ADC_CTRI_TRIS = INPUT;
  ADC_CTRO_TRIS = INPUT;
  ADC_CTSP_TRIS = INPUT;
  ADC_FTFT_TRIS = INPUT;
  ADC_CPSP_TRIS = INPUT;
  ADC_FPFT_TRIS = INPUT;
  ADC_MCD_TRIS = INPUT;
#endif

  /**
   * Setup Peripherals
   */

#if FRONT
  ANCON0 = 0b11100110; // All analog except for AN0, AN3, AN4
  ANCON1 = 0b00000111; // All digital except for AN8, AN9, AN10
#elif REAR
  ANCON0 = 0b11111111; // All analog
  ANCON1 = 0b00000111; // All digital except for AN8, AN9, AN10
#endif
  init_ADC();

  // Programmable termination
  TERM_TRIS = OUTPUT;
  TERM_LAT = 0; // Not terminating

  ECANInitialize();

  // Interrupts setup
  INTCONbits.GIE = 1;		// Global Interrupt Enable (1 enables)
  INTCONbits.PEIE = 1;	// Peripheral Interrupt Enable (1 enables)
  RCONbits.IPEN = 0;		// Interrupt Priority Enable (1 enables)

  // Main loop
  while(1) {

    // Sample and send fast speed sensor channels on CAN
    send_fast_can();

    // Sample and send medium speed sensor channels on CAN
    send_med_can();

    // Sample and send slow speed sensor channels on CAN
    send_slow_can();

    // Send diagnostic CAN message
    send_diag_can();

#if FRONT
    if (radio_sw) {
      RADIO0_TRIS = OUTPUT;
      RADIO1_TRIS = OUTPUT;
      RADIO0_LAT = 0;
      RADIO1_LAT = 0;
    } else {
      RADIO0_TRIS = INPUT;
      RADIO1_TRIS = INPUT;
    }
#endif
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

    switch (id) {

#if FRONT
      case WHEEL_ID + 1:
        // Process radio_sw CAN message
        radio_sw = data[BUTTON_BITS_BYTE] & RADIO_BTN_MASK;
        break;
#endif
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
 * void send_diag_can(void)
 *
 * Sends the diagnostic CAN message if the interval has passed.
 */
void send_diag_can(void) {
  if (millis - diag_send_tmr >= DIAG_MSG_SEND) {
    ((uint16_t*) data)[UPTIME_BYTE / 2] = seconds;

#if FRONT
    ECANSendMessage(ANALOG_FRONT_ID + 0x0, data, 2, ECAN_TX_FLAGS);
#elif REAR
    ECANSendMessage(ANALOG_REAR_ID + 0x0, data, 2, ECAN_TX_FLAGS);
#endif

    diag_send_tmr = millis;
  }
}

/**
 * void send_fast_can(void)
 *
 * Samples and sends fast speed sensor channels on CAN if the interval has passed
 *
 * TODO: Apply correct scaling rather than default of 0.0001 (V)
 */
void send_fast_can(void) {
  if (millis - fast_send_tmr >= FAST_MSG_SEND) {

#if FRONT // Sample and send FRONT fast speed sensor channels

    uint16_t spfl_samp = sample(ADC_SPFL_CHN);
    uint16_t spfl_volt = (uint16_t) (((((double) spfl_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t spfr_samp = sample(ADC_SPFR_CHN);
    uint16_t spfr_volt = (uint16_t) (((((double) spfr_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t bpf_samp = sample(ADC_BPF_CHN);
    uint16_t bpf_volt = (uint16_t) (((((double) bpf_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t bpr_samp = sample(ADC_BPR_CHN);
    uint16_t bpr_volt = (uint16_t) (((((double) bpr_samp) / 4095.0)
      * 5.0) * 10000.0);

    ((uint16_t*) data)[SPFL_BYTE / 2] = spfl_volt;
    ((uint16_t*) data)[SPFR_BYTE / 2] = spfr_volt;
    ((uint16_t*) data)[BPF_BYTE / 2] = bpf_volt;
    ((uint16_t*) data)[BPR_BYTE / 2] = bpr_volt;
    ECANSendMessage(ANALOG_FRONT_ID + 0x1, data, 8, ECAN_TX_FLAGS);

#elif REAR // Sample and send REAR fast speed sensor channels

    uint16_t sprl_samp = sample(ADC_SPRL_CHN);
    uint16_t sprl_volt = (uint16_t) (((((double) sprl_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t sprr_samp = sample(ADC_SPRR_CHN);
    uint16_t sprr_volt = (uint16_t) (((((double) sprr_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t eos_samp = sample(ADC_EOS_CHN);
    uint16_t eos_volt = (uint16_t) (((((double) eos_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t bcd_samp = sample(ADC_BCD_CHN);
    uint16_t bcd_volt = (uint16_t) (((((double) bcd_samp) / 4095.0)
      * 5.0) * 10000.0);

    ((uint16_t*) data)[SPRL_BYTE / 2] = sprl_volt;
    ((uint16_t*) data)[SPRR_BYTE / 2] = sprr_volt;
    ((uint16_t*) data)[EOS_BYTE / 2] = eos_volt;
    ((uint16_t*) data)[BCD_BYTE / 2] = bcd_volt;
    ECANSendMessage(ANALOG_REAR_ID + 0x1, data, 8, ECAN_TX_FLAGS);

#endif

    fast_send_tmr = millis;
  }
}

/**
 * void send_med_can(void)
 *
 * Samples and sends medium speed sensor channels on CAN if the interval has passed
 *
 * TODO: Apply correct scaling rather than default of 0.0001 (V)
 */
void send_med_can(void) {
  if (millis - med_send_tmr >= MED_MSG_SEND) {

#if FRONT // Sample and send FRONT medium speed sensor channels

    uint16_t strp_samp = sample(ADC_STRP_CHN);
    uint16_t strp_volt = (uint16_t) (((((double) strp_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t apps0_samp = sample(ADC_APPS0_CHN);
    uint16_t apps0_volt = (uint16_t) (((((double) apps0_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t apps1_samp = sample(ADC_APPS1_CHN);
    uint16_t apps1_volt = (uint16_t) (((((double) apps1_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t ptdp_samp = sample(ADC_PTDP_CHN);
    uint16_t ptdp_volt = (uint16_t) (((((double) ptdp_samp) / 4095.0)
      * 5.0) * 10000.0);

    ((uint16_t*) data)[STRP_BYTE / 2] = strp_volt;
    ((uint16_t*) data)[APPS0_BYTE / 2] = apps0_volt;
    ((uint16_t*) data)[APPS1_BYTE / 2] = apps1_volt;
    ((uint16_t*) data)[PTDP_BYTE / 2] = ptdp_volt;
    ECANSendMessage(ANALOG_FRONT_ID + 0x2, data, 8, ECAN_TX_FLAGS);

#elif REAR // Sample and send REAR medium speed sensor channels

    uint16_t cpsp_samp = sample(ADC_CPSP_CHN);
    uint16_t cpsp_volt = (uint16_t) (((((double) cpsp_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t fpft_samp = sample(ADC_FPFT_CHN);
    uint16_t fpft_volt = (uint16_t) (((((double) fpft_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t mcd_samp = sample(ADC_MCD_CHN);
    uint16_t mcd_volt = (uint16_t) (((((double) mcd_samp) / 4095.0)
      * 5.0) * 10000.0);

    ((uint16_t*) data)[CPSP_BYTE / 2] = cpsp_volt;
    ((uint16_t*) data)[FPFT_BYTE / 2] = fpft_volt;
    ((uint16_t*) data)[MCD_BYTE / 2] = mcd_volt;
    ECANSendMessage(ANALOG_REAR_ID + 0x3, data, 6, ECAN_TX_FLAGS);

#endif

    med_send_tmr = millis;
  }
}

/**
 * void send_slow_can(void)
 *
 * Samples and sends slow speed sensor channels on CAN if the interval has passed
 *
 * TODO: Apply correct scaling rather than default of 0.0001 (V)
 */
void send_slow_can(void) {
  if (millis - slow_send_tmr >= SLOW_MSG_SEND) {

#if REAR // Sample and send REAR slow speed sensor channels

    uint16_t ctri_samp = sample(ADC_CTRI_CHN);
    uint16_t ctri_volt = (uint16_t) (((((double) ctri_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t ctro_samp = sample(ADC_CTRO_CHN);
    uint16_t ctro_volt = (uint16_t) (((((double) ctro_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t ctsp_samp = sample(ADC_CTSP_CHN);
    uint16_t ctsp_volt = (uint16_t) (((((double) ctsp_samp) / 4095.0)
      * 5.0) * 10000.0);

    uint16_t ftft_samp = sample(ADC_FTFT_CHN);
    uint16_t ftft_volt = (uint16_t) (((((double) ftft_samp) / 4095.0)
      * 5.0) * 10000.0);

    ((uint16_t*) data)[CTRI_BYTE / 2] = ctri_volt;
    ((uint16_t*) data)[CTRO_BYTE / 2] = ctro_volt;
    ((uint16_t*) data)[CTSP_BYTE / 2] = ctsp_volt;
    ((uint16_t*) data)[FTFT_BYTE / 2] = ftft_volt;
    ECANSendMessage(ANALOG_REAR_ID + 0x2, data, 8, ECAN_TX_FLAGS);

#endif

    slow_send_tmr = millis;
  }
}
