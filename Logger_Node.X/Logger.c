/**
 * Logger
 *
 * Processor:   PIC32MZ2048ECM064
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "Logger.h"

static volatile int seconds = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_unused_pins(); // Set unused I/O pins to low outputs
  init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(); // Initialize oscillator configuration bits
  init_can(); // Initialize CAN
  init_timer1(); // Initialize timer1

  // Initialize LED output pin
  TRISEbits.TRISE5 = OUTPUT;
  LATEbits.LATE5 = 1;

  // Initialize SYSCLK reference output pin
  TRISFbits.TRISF0 = OUTPUT;
  RPF0R = 0b1111; // Assign REFCLKO1 to RF0

  CanRxMessageBuffer* receive;
  CanTxMessageBuffer* transmit;

  // Main loop
  while(1) {

    if(seconds > 5) {
      transmit = (CanTxMessageBuffer*) (PA_TO_KVA1(C1FIFOUA1));
      transmit->messageWord[0] = 0;
      transmit->messageWord[1] = 0;
      transmit->messageWord[2] = 0;
      transmit->messageWord[3] = 0;

      transmit->CMSGSID.SID = 0x210;
      transmit->CMSGEID.DLC = 8;
      transmit->CMSGDATA0.Byte0 = 0xAA;
      transmit->CMSGDATA0.Byte1 = 0xBB;

      C1FIFOCON1bits.UINC = 1;
      C1FIFOCON1bits.TXREQ = 1;
    }

    // Keep polling until the FIFO isn't empty
    while(C1FIFOINT0bits.RXNEMPTYIF == 1) {
      receive = (CanRxMessageBuffer*) (PA_TO_KVA0(C1FIFOUA0));

      // Signal to the CAN module that we've processed a message
      C1FIFOCON0bits.UINC = 1;
    }
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

/**
 * CAN1 Interrupt Handler
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL6SRS))) can_inthnd(void) {
  IFS4bits.CAN1IF = 0;
}
