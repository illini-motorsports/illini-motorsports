/**
 * FSAE Library
 *
 * Processor:   PIC18F46K80
 * Compiler:    Microchip C18
 * Author:      George Schwieters
 * Created:     2014-2015
 */

#include "FSAE.h"
#include "timers.h"
#include "adc.h"

#ifdef LOGGING
#include "Pins_Logger.h"
#elif ANALOGHUB
#include "Pins_AnalogHub.h"
#elif SHIFTLIGHTS
#include "Pins_ShiftLights.h"
#elif WHEEL
#include "Pins_Wheel.h"
#elif PDM
#include "Pins_PDM.h"
#elif TELEMETRY
#include "Pins_Telemetry.h"
#endif

/**
 * This function will disable all maskable interrupts.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify INTCON.
 */
void CLI(void) {
  INTCONbits.GIE = 0; // Global Interrupt Enable (1 enables)
  INTCONbits.PEIE = 0; // Peripheral Interrupt Enable (1 enables)
}

/**
 * This function will enable all maskable interrupts.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify INTCON.
 */
void STI(void) {
  INTCONbits.GIE = 1; // Global Interrupt Enable (1 enables)
  INTCONbits.PEIE = 1; // Peripheral Interrupt Enable (1 enables)
}

/**
 * This function will initialize the analog to digital converter.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify ADCON0, ADCON1 & ADCON2.
 */
void init_ADC(void) {
  OpenADC(ADC_FOSC_64 & ADC_RIGHT_JUST & ADC_4_TAD, ADC_CH0 & ADC_INT_OFF,
      ADC_REF_VDD_VDD & ADC_REF_VDD_VSS & ADC_NEG_CH0);
}

/**
 * This function will initialize Timer0 to take in the system clock and
 * interrupt once every millisecond.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify INTCON2, INTCON, TMR0L & T0CON.
 */
void init_timer0(void) {
  // Turn on and configure the TIMER0 oscillator
  OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
  WriteTimer0(TMR0_RELOAD); // Load timer register
  INTCON2bits.TMR0IP = 1; // High priority
}

/**
 * This function will initialize Timer1 to take in an external 32kHz cystal and
 * interrupt every second.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify T1CON, IPR1, TMR1L, TMR1H, PIE1 & PIR1.
 */
void init_timer1(void) {
  // Turn on and configure the TIMER1 oscillator
  OpenTimer1(T1_16BIT_RW & T1_SOURCE_PINOSC & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_ON, 0x00);
  WriteTimer1(TMR1_RELOAD); // Load timer registers
  IPR1bits.TMR1IP = 1; // High priority
}

/**
 * This function will configure the oscillator.
 *
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify OSCTUNE, OSCCON & OSCCON2.
 */
void init_oscillator(void) {

#ifdef INTERNAL
  // OSCTUNE
  OSCTUNEbits.INTSRC = 0; // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
  OSCTUNEbits.PLLEN = 1; // Frequency Multiplier PLL Select (1 to enable)
  OSCTUNEbits.TUN5 = 0; // Fast RC Oscillator Frequency Tuning (seems to be 2's comp encoding)
  OSCTUNEbits.TUN4 = 0; // 011111 = max
  OSCTUNEbits.TUN3 = 0; // ... 000001
  OSCTUNEbits.TUN2 = 0; // 000000 = center (running at calibrated frequency)
  OSCTUNEbits.TUN1 = 0; // 111111 ...
  OSCTUNEbits.TUN0 = 0; // 100000

  // OSCCCON
  OSCCONbits.IDLEN = 1; // Idle Enable Bit (1 to enter idle mode after SLEEP instruction else sleep mode is entered)
  OSCCONbits.IRCF2 = 1; // Internal Oscillator Frequency Select Bits
  OSCCONbits.IRCF1 = 1; // When using HF, settings are:
  OSCCONbits.IRCF0 = 1; // 111 - 16 MHz, 110 - 8MHz (default), 101 - 4MHz, 100 - 2 MHz, 011 - 1 MHz
  OSCCONbits.SCS1 = 0;
  OSCCONbits.SCS0 = 0;

  // OSCCON2
  OSCCON2bits.MFIOSEL = 0;

  while(!OSCCONbits.HFIOFS); // Wait for stable clock
}
#else
  // OSCTUNE
  OSCTUNEbits.INTSRC = 0; // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
  OSCTUNEbits.PLLEN = 1; // Frequency Multiplier PLL Select (1 to enable)

  // OSCCCON
  OSCCONbits.SCS1 = 0; // Select configuration chosen oscillator
  OSCCONbits.SCS0 = 0; // SCS = 00

  // OSCCON2
  OSCCON2bits.MFIOSEL = 0;

  while(!OSCCONbits.OSTS); // Wait for stable external clock
#endif
}

/**
 * This function will drive unused pins to logic low.
 * Input(s): none
 * Return Value(s): none
 * Side Effects: This will modify TRISA, TRISB, TRISC, TRISD, TRISE, LATA,
 *     LATB, LATC, LATD & LATE.
 */
void init_unused_pins(void) {
  // First configure to outputs then set the pin low
#ifndef GP_A0
  TRISAbits.TRISA0 = OUTPUT;
  LATAbits.LATA0 = 0;
#endif
#ifndef GP_A1
  TRISAbits.TRISA1 = OUTPUT;
  LATAbits.LATA1 = 0;
#endif
#ifndef GP_A2
  TRISAbits.TRISA2 = OUTPUT;
  LATAbits.LATA2 = 0;
#endif
#ifndef GP_A3
  TRISAbits.TRISA3 = OUTPUT;
  LATAbits.LATA3 = 0;
#endif
#ifndef GP_A5
  TRISAbits.TRISA5 = OUTPUT;
  LATAbits.LATA5 = 0;
#endif
#ifndef GP_A6
  TRISAbits.TRISA6 = OUTPUT;
  LATAbits.LATA6 = 0;
#endif
#ifndef GP_A7
  TRISAbits.TRISA7 = OUTPUT;
  LATAbits.LATA7 = 0;
#endif

#ifndef GP_B0
  TRISBbits.TRISB0 = OUTPUT;
  LATBbits.LATB0 = 0;
#endif
#ifndef GP_B1
  TRISBbits.TRISB1 = OUTPUT;
  LATBbits.LATB1 = 0;
#endif
#ifndef GP_B2
  TRISBbits.TRISB2 = OUTPUT;
  LATBbits.LATB2 = 0;
#endif
#ifndef GP_B3
  TRISBbits.TRISB3 = OUTPUT;
  LATBbits.LATB3 = 0;
#endif
#ifndef GP_B4
  TRISBbits.TRISB4 = OUTPUT;
  LATBbits.LATB4 = 0;
#endif
#ifndef GP_B5
  TRISBbits.TRISB5 = OUTPUT;
  LATBbits.LATB5 = 0;
#endif
#ifndef GP_B6
  TRISBbits.TRISB6 = OUTPUT;
  LATBbits.LATB6 = 0;
#endif
#ifndef GP_B7
  TRISBbits.TRISB7 = OUTPUT;
  LATBbits.LATB7 = 0;
#endif

#ifndef GP_C0
  TRISCbits.TRISC0 = OUTPUT;
  LATCbits.LATC0 = 0;
#endif
#ifndef GP_C1
  TRISCbits.TRISC1 = OUTPUT;
  LATCbits.LATC1 = 0;
#endif
#ifndef GP_C2
  TRISCbits.TRISC2 = OUTPUT;
  LATCbits.LATC2 = 0;
#endif
#ifndef GP_C3
  TRISCbits.TRISC3 = OUTPUT;
  LATCbits.LATC3 = 0;
#endif
#ifndef GP_C4
  TRISCbits.TRISC4 = OUTPUT;
  LATCbits.LATC4 = 0;
#endif
#ifndef GP_C5
  TRISCbits.TRISC5 = OUTPUT;
  LATCbits.LATC5 = 0;
#endif
#ifndef GP_C6
  TRISCbits.TRISC6 = OUTPUT;
  LATCbits.LATC6 = 0;
#endif
#ifndef GP_C7
  TRISCbits.TRISC7 = OUTPUT;
  LATCbits.LATC7 = 0;
#endif

#ifndef GP_D0
  TRISDbits.TRISD0 = OUTPUT;
  LATDbits.LATD0 = 0;
#endif
#ifndef GP_D1
  TRISDbits.TRISD1 = OUTPUT;
  LATDbits.LATD1 = 0;
#endif
#ifndef GP_D2
  TRISDbits.TRISD2 = OUTPUT;
  LATDbits.LATD2 = 0;
#endif
#ifndef GP_D3
  TRISDbits.TRISD3 = OUTPUT;
  LATDbits.LATD3 = 0;
#endif
#ifndef GP_D4
  TRISDbits.TRISD4 = OUTPUT;
  LATDbits.LATD4 = 0;
#endif
#ifndef GP_D5
  TRISDbits.TRISD5 = OUTPUT;
  LATDbits.LATD5 = 0;
#endif
#ifndef GP_D6
  TRISDbits.TRISD6 = OUTPUT;
  LATDbits.LATD6 = 0;
#endif
#ifndef GP_D7
  TRISDbits.TRISD7 = OUTPUT;
  LATDbits.LATD7 = 0;
#endif

#ifndef GP_E0
  TRISEbits.TRISE0 = OUTPUT;
  LATEbits.LATE0 = 0;
#endif
#ifndef GP_E1
  TRISEbits.TRISE1 = OUTPUT;
  LATEbits.LATE1 = 0;
#endif
#ifndef GP_E2
  TRISEbits.TRISE2 = OUTPUT;
  LATEbits.LATE2 = 0;
#endif
}
