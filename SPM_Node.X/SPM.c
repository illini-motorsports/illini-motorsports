#include "SPM.h"

void main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_freq_meas();

  while(1){}
}

void init_freq_meas(void){
  FREQ_IN_0_TRIS = INPUT;
  FREQ_IN_2_TRIS = INPUT;
  FREQ_IN_3_TRIS = INPUT;
}
