/**
 * SIM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#include "Sim.h"

uint32_t sim_wait = 0;
uint32_t sim_wait_tgt = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  init_oscillator(0); // Initialize oscillator configuration bits

  STI(); // Enable interrupts

  uint32_t i,j,k;
  uint32_t med_wait, long_wait;

  // Set up simulation outputs
  TRISBbits.TRISB15 = OUTPUT;
  #define SIM_OUT LATBbits.LATB15
  TRISBbits.TRISB14 = OUTPUT;
  #define SIM_SYNC_OUT LATBbits.LATB14

  // Set up adjustment analog input
  TRISBbits.TRISB2 = INPUT;
  ANSELBbits.ANSB2 = AN_INPUT;
  AD1CAL1 = DEVADC1; AD1CAL2 = DEVADC2; AD1CAL3 = DEVADC3; AD1CAL4 = DEVADC4; AD1CAL5 = DEVADC5;
  AD1CON1 = 0x0;
  AD1CON2 = 0x0;
  AD1CON2bits.ADCSEL = 1;
  AD1CON2bits.ADCDIV = 4;
  AD1CON3 = 0x0;
  AD1GIRQEN1 = 0x0; AD1GIRQEN2 = 0x0;
  AD1CSS1 = 0x0; AD1CSS2 = 0x0;
  AD1CMPCON1 = 0x0; AD1CMPCON2 = 0x0; AD1CMPCON3 = 0x0; AD1CMPCON4 = 0x0; AD1CMPCON5 = 0x0; AD1CMPCON6 = 0x0;
  AD1FLTR1 = 0x0; AD1FLTR2 = 0x0; AD1FLTR3 = 0x0; AD1FLTR4 = 0x0; AD1FLTR5 = 0x0; AD1FLTR6 = 0x0;
  AD1TRG1 = 0x0; AD1TRG2 = 0x0; AD1TRG3 = 0x0;
  AD1TRG1bits.TRGSRC2 = 1; // Software trigger
  AD1CON1bits.ADCEN = 1;
  while (AD1CON2bits.ADCRDY == 0);

  uint32_t samp = 0;

  while (sim_wait == 0) {
    AD1CON3bits.GSWTRG = 1;
    if (AD1DSTAT1bits.ARDY2) {
      samp = AD1DATA2;
      if (samp > 50) {
        sim_wait = samp-50;
      }
    }
  }

  // Main loop
  while (1) {
    if (!(k % 10))
      AD1CON3bits.GSWTRG = 1;

    med_wait = sim_wait * 2;
    long_wait = sim_wait * 4;

    SIM_OUT = 0; // Falling edge #1
    for (i = 0; i < 21; i++) {
      // Repeat: Short gap, rising edge, short gap, falling edge
      for(j = 0; j < sim_wait; j++);
      SIM_OUT = 1;
      for(j = 0; j < sim_wait; j++);
      SIM_OUT = 0;

      // Simulate sync signal somewhere in the middle
      if (k % 2) {
        if (i == 15)
          SIM_SYNC_OUT = 1;
        else if (i == 18)
          SIM_SYNC_OUT = 0;
      }
    }

    // Med gap then rising edge
    for(j = 0; j < med_wait; j++);
    SIM_OUT = 1;

    // Long gap then repeat
    for(j = 0; j < long_wait; j++);
    ++k;

    // Sample new sim_wait_tgt
    if (AD1DSTAT1bits.ARDY2) {
      samp = AD1DATA2;
      if (samp > 50) {
        sim_wait_tgt = samp-50;
      }
    }
    
    // Slowly move sim_wait towards sim_wait_tgt
    if (sim_wait_tgt > sim_wait) {
      if (sim_wait_tgt - sim_wait > 25)
        ++sim_wait;
    } else if (sim_wait > sim_wait_tgt) {
      if (sim_wait - sim_wait_tgt > 25)
        --sim_wait;
    }
  }
}
