#include "SPM.h"

void main(void){
	init_general();// Set general runtime configuration bits
	init_gpio_pins();// Set all I/O pins to low outputs
	init_oscillator();// Initialize oscillator configuration bits
	init_timer2();// Initialize timer2 (millis)
    
    while(1){}
}