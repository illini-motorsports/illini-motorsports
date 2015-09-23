/**
 * PDM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "PDM.h"

static volatile int seconds = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_unused_pins(); // Set unused I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(); // Initialize oscillator configuration bits
  //init_timer1(); // Initialize timer1
  asm volatile("ei"); // Enable interrupts

  // Initialize LED output pin
  TRISEbits.TRISE5 = OUTPUT;
  LATEbits.LATE5 = 0;
  
  // Main loop
  while(1) {
    LATEbits.LATE5 = LATEbits.LATE5 ? 0 : 1; // Invert LATE5 - Toggle the LED
  }
}

/**
 * TMR1 Interrupt Handler
 *
 * Fires once every second.
 */
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
  seconds++;
  LATEbits.LATE5 = LATEbits.LATE5 ? 0 : 1; // Invert LATE5 - Toggle the LED
  IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}