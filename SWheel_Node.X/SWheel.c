/**
 * SWheel
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "SWheel.h"

// Count number of milliseconds since start of code execution
volatile uint32_t millis = 0;

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_spi();// Initialize SPI interface
  STI();// Enable interrupts

  LCD_CS_TRIS = OUTPUT;
  LCD_CS_LAT = 1;
  LCD_RST_TRIS = OUTPUT;
  TRISBbits.TRISB6 = OUTPUT;
  TRISBbits.TRISB7 = OUTPUT;
  LATBbits.LATB6 = 1;
  LATBbits.LATB7 = 0;

  reset();
  initialize();

  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX
  PWM1config(1, RA8875_PWM_CLK_DIV1024);// PWM output for backlight
  PWM1out(255);

  while(1){
	  double i = 0;
	  for(i = 0;i<100;i+=11.1){
  		fillScreen(RA8875_WHITE);
  		sevenSegmentDigit(190,20,100,RA8875_BLACK,i/10);
  		sevenSegmentDecimal(10,25,20,3,1,RA8875_BLACK,i);
  		sevenSegmentDecimal(10,100,20,3,1,RA8875_BLACK,i);
  		sevenSegmentDecimal(10,175,20,3,1,RA8875_BLACK,i);
  		sevenSegmentDecimal(400,25,20,3,1,RA8875_BLACK,i);
  		sevenSegmentDecimal(400,100,20,3,1,RA8875_BLACK,i);
  		sevenSegmentDecimal(400,175,20,3,1,RA8875_BLACK,i);
		delay(300);
	  }
  }

  while(1){
	  LATBbits.LATB6 = 0;
	  LATBbits.LATB7 = 1;
	  delay(100);
	  LATBbits.LATB6 = 1;
	  LATBbits.LATB7 = 0;
	  delay(100);
  }
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {

  millis++;// Increment millis count
  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

void delay(uint32_t num) {
  uint32_t start = millis;
  while (millis - start < num);
}