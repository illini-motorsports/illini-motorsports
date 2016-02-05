/* 
 * File: Wheel.c
 * Author: Jake Leonard
 * Comments: Main code that controls the wheel module
 */
#include "Wheel.h"

int flag;
uint16_t readMsg;
uint16_t writeMsg = 0x0000;
uint16_t readData = 0x0000;
uint16_t readVals[100];
int readCount = 0;

void main(void){
    init_general(); // Set general runtime configuration bits
    init_gpio_pins(); // Set all I/O pins to low outputs
    //init_peripheral_modules(); // Disable unused peripheral modules
    init_oscillator(); // Initialize oscillator configuration bits
    init_timer1(); // Initialize timer1
    init_spi(); // Initialize SPI interface

    asm volatile("ei"); // Enable interrupts

    TRISBbits.TRISB6 = OUTPUT; // PicFn pin
    LATBbits.LATB6 = 1;
    TRISBbits.TRISB7 = OUTPUT; // MODE pin
    LATBbits.LATB8 = 0;
    TRISBbits.TRISB7 = OUTPUT; // CS pin
    LATBbits.LATB11 = 0;
    
    lcd_init();
    displayOn(1); // Turn Display On
    while(1){
    }
}


// Called every second
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
    LATBbits.LATB6 = LATBbits.LATB6 ? 0 : 1; // Invert LATB6 - Toggle the LED
    LATBbits.LATB7 = LATBbits.LATB7 ? 0 : 1; // Invert LATB6 - Toggle the LED
    IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}