/**
 * FSAE Library Config Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_CONFIG_H
#define FSAE_CONFIG_H

#include <xc.h>
#include <sys/kmem.h>
#include <sys/types.h>

// Code control definitions
#define INTERNAL_CLK 0 // Determines whether the internal or external clock source is used
#define REFCLKO      0 // Determines whether SYSCLK / 10 is driven out on RF0
#define COMP         1 // Determines whether we are at competition
#define PBCLK2       100 // PBCLK is 100mhz, SYSCLK/2

// TRIS Settings
#define OUTPUT 0
#define INPUT 1

// ANSEL Settings
#define AN_INPUT  1
#define DIG_INPUT 0

// Definitions for interval override control
#define OVERRIDE    1
#define NO_OVERRIDE 0

// Programmable termination settings
#define TERMINATING 1
#define NOT_TERMINATING 0

// Pin definitions for programmable termination
#ifndef ECM064
  #define TERM_TRIS TRISAbits.TRISA2
  #define TERM_LAT  LATAbits.LATA2
#else
  #define TERM_TRIS TRISBbits.TRISB2
  #define TERM_LAT  LATBbits.LATB2
#endif

#define CLI() asm volatile("di; ehb;")
#define STI() asm volatile("ei;")

// Function definitions
void unlock_config(void);
void lock_config(void);
void init_general(void);
void init_peripheral_modules(void);
void init_gpio_pins(void);
void init_oscillator(uint8_t whl_refoclk4);
void init_timer2(void);
void init_timer4(uint16_t period1);
void init_timer6(uint16_t period2);
void init_timers_45();
void init_timer6_ecu();
void init_termination(uint8_t isTerm);

#endif /* FSAE_CONFIG_H */
