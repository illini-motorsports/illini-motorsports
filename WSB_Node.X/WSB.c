/**
 * WSB
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Ben Cross
 * Created:     2019-2020
 */

#include "WSB.h"

void main(void) {
  init_general();     // Set general runtime configuration bits
  init_gpio_pins();   // Set all I/O pins to low outputs
  init_oscillator(0); // Initialize oscillator configuration bits
  init_timer2();      // Initialize timer2 (millis)
  init_adc(NULL);     // Initialize ADC module
  init_termination(NOT_TERMINATING);
  init_can(); // Initialize CAN
  STI();      // Enable interrupts
}
