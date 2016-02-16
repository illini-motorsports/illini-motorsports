/*
 * File: Wheel.c
 * Author: Jake Leonard
 * Comments: Main code that controls the wheel module
 */
#include "Wheel.h"

volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;
volatile uint32_t delay_millis = 0;
volatile uint8_t mode = 0;
uint8_t momentaries[4] = {0};
uint8_t toggles[4] = {0};
/*
 * Pin Assignments
 *
 * SCLK 	-		SCK1 	- 76
 * SD0 		- 	RB10 	- 34
 * SDI 		- 	RB9		-	33
 *	 LCD_CS -		RB8		-	32
 * LCD_RST 	-	RB11		-	35
 */

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer1();// Initialize Second interrupts
  init_timer2();// Initialize Millisecond interrupts
  init_spi();// Initialize SPI interface
  init_termination();// Initialize Programmable Termination
  init_can();// Initialize CAN
  STI();// Enable Interrupts

  // Output Tris
  PIC_FN_TRIS = OUTPUT;
  PIC_MODE_TRIS = OUTPUT;
  LCD_RST_TRIS = OUTPUT;
  LCD_BACKLITE_TRIS = OUTPUT;
  LCD_CS_TRIS = OUTPUT;

  // Input Tris
  SW_MOMENT1_TRIS = INPUT;
  SW_MOMENT2_TRIS = INPUT;
  SW_MOMENT3_TRIS = INPUT;
  SW_MOMENT4_TRIS = INPUT;
  SW_TOG1_TRIS = INPUT;
  SW_TOG2_TRIS = INPUT;
  SW_TOG3_TRIS = INPUT;
  SW_TOG4_TRIS = INPUT;

  // Output Set LAT
  PIC_FN_LAT = 1;
  PIC_MODE_LAT = 0;
  LCD_RST_LAT = 1;
  LCD_CS_LAT = 1;

  // Initialize LCD Routine
  LCD_Reset();// Reset
  LCD_Init();// Init
  Display_On(1);// Turn on
  while (1) {
    /*
    Display_On(1);
    delay(100);;
    PIC_FN_LAT = 1;
    PIC_MODE_LAT = 0;
    delay(100);
    PIC_FN_LAT = 0;
    PIC_MODE_LAT = 1;
     */
  }
}

// Update Switch values from port register
void update_sw_values() {
  momentaries[0] = SW_MOMENT1_PORT;
  momentaries[1] = SW_MOMENT2_PORT;
  momentaries[2] = SW_MOMENT3_PORT;
  momentaries[3] = SW_MOMENT4_PORT;
  toggles[0] = SW_TOG1_PORT;
  toggles[1] = SW_TOG2_PORT;
  toggles[2] = SW_TOG3_PORT;
  toggles[3] = SW_TOG4_PORT;
}

// Uses the millis counter to create a precise delay
void delay(uint32_t mil) {
  if (mil == 1) mil++;//Might be entering on the end of the millisecond
  CLI();//Turn off interrupts for any interaction with millis
  register uint32_t delay_stop = millis + mil;
  STI();// Turn it back on again
  Nop();
  CLI();
  while (delay_stop > millis) {
    STI();
    Nop();//Keep interrupts on and wait for stuff to happen
    Nop();
    Nop();
    CLI();
  }
  STI();
}

// Called every second
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
  seconds++;
  IFS0bits.T1IF = 0;// Clear TMR1 Interrupt Flag
}

// Called every millisecond
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL7SRS))) timer2_inthnd(void) {
  millis++;
  IFS0bits.T2IF = 0;// Clear TMR2 Interrupt Flag
}