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
  init_adc(); // Initialize ADC module
  init_termination(); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  // Set EN pins to outputs
  EN_IGN_TRIS = OUTPUT;   
  EN_INJ_TRIS = OUTPUT;
  EN_FUEL_TRIS = OUTPUT;
  EN_ECU_TRIS = OUTPUT;  
  EN_WTR_TRIS = OUTPUT; 
  EN_FAN_TRIS = OUTPUT; 
  EN_AUX_TRIS = OUTPUT; 
  EN_PDLU_TRIS = OUTPUT; 
  EN_PDLD_TRIS = OUTPUT; 
  EN_B5V5_TRIS = OUTPUT; 
  EN_BVBAT_TRIS = OUTPUT; 
  EN_STR_TRIS = OUTPUT; 

  // Turn off all loads
  EN_IGN_LAT = PWR_OFF;   
  EN_INJ_LAT = PWR_OFF;
  EN_FUEL_LAT = PWR_OFF;
  EN_ECU_LAT = PWR_OFF;  
  EN_WTR_LAT = PWR_OFF; 
  EN_FAN_LAT = PWR_OFF; 
  EN_AUX_LAT = PWR_OFF; 
  EN_PDLU_LAT = PWR_OFF; 
  EN_PDLD_LAT = PWR_OFF; 
  EN_B5V5_LAT = PWR_OFF; 
  EN_BVBAT_LAT = PWR_OFF; 
  EN_STR_LAT = PWR_OFF; 

  // Set TRIS registers - !CS
  // Set all !CS signals high

  // Set all rheostats to 5k
  //TODO: Should actually set them to value stored in flash memory

  // Set TRIS registers - ADC
  // Set TRIS registers - !SW

  // Turn on engine-off loads

  asm volatile("ei"); // Enable interrupts

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
