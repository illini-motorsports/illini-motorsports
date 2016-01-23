/**
 * PDM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "PDM.h"

static volatile uint32_t seconds = 0;
static volatile uint8_t res_flag = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(); // Initialize oscillator configuration bits
  //init_timer1(); // Initialize timer1
  //init_spi(); // Initialize SPI interface
  //asm volatile("ei"); // Enable interrupts

  // Main loop
  while(1);
}

/**
 * TMR1 Interrupt Handler
 *
 * Fires once every second.
 *
 * TODO: Fix for actual PDM code.
 */
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
  seconds++;
  LATEbits.LATE5 = LATEbits.LATE5 ? 0 : 1; // Invert LATE5 - Toggle the LED

  /*
  // Flip resistance every 3 seconds
  if(seconds % 3 == 0) {
    if(res_flag) {
      // Minimum resistance
      send_rheo(0x0000);
      res_flag = 0;
    } else {
      // Maximum resistance
      send_rheo(0x00FF);
      res_flag = 1;
    }
  }
   */

  IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}

/**
 * TODO: Fix for actual PDM code
 */
void send_rheo(uint16_t msg) {
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATGbits.LATG15 = 0; // CS selected
  SPI1BUF = msg;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATGbits.LATG15 = 1; // CS deselected
}