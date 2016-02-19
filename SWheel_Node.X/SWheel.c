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

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  init_oscillator(); // Initialize oscillator configuration bits
  init_timer2(); // Initialize timer2 (millis)
  init_spi(); // Initialize SPI interface

  LCD_CS_TRIS = OUTPUT;
  LCD_CS_LAT = 1;

  LCD_RST_TRIS = OUTPUT;
  LCD_RST_LAT = 0;

  LCD_RST_LAT = 0;
  delay(100);
  LCD_RST_LAT = 1;
  delay(100);

  STI(); // Enable interrupts

  initialize();

  displayOn(1);
  GPIOX(1); // Enable TFT - display enable tied to GPIOX
  PWM1config(1, RA8875_PWM_CLK_DIV1024); // PWM output for backlight
  PWM1out(255);

  fillScreen(RA8875_GREEN);
  fillCircle(360, 90, 50, RA8875_BLUE);
  fillCircle(360, 183, 50, RA8875_BLUE);
  fillRect(50, 99, 300, 75, RA8875_BLUE);

  // Main loop
  while (1);
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++; // Increment millis count
  IFS0CLR = _IFS0_T2IF_MASK; // Clear TMR2 Interrupt Flag
}

void delay(uint32_t num) {
  uint32_t start = millis;
  while(millis - start < num);
}