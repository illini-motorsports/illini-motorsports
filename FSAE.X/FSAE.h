/**
 * FSAE Library Header
 *
 * Processor:   PIC18F46K80
 * Compiler:    Microchip C18
 * Author:      George Schwieters
 * Created:     2014-2015
 */

#ifndef FSAE_H
#define FSAE_H

// Definitions for variable types
#define uint8_t UINT8
#define uint16_t UINT16
#define uint32_t UINT32
#define int8_t INT8
#define int16_t INT16
#define int32_t INT32

/**
 * Magic Numbers
 */

#define INPUT 1
#define OUTPUT 0
#define TMR0_RELOAD 0x82
#define TMR1_RELOAD 0x8000
#define TMR1H_RELOAD 0x80
#define TMR1L_RELOAD 0x00

// Definitions for interval override control
#define OVERRIDE    1
#define NO_OVERRIDE 0

/**
 * Function Definitions
 */

void CLI(void);
void STI(void);
void init_ADC(void);
void init_timer0(void);
void init_timer1(void);
void init_oscillator(void);
void init_unused_pins(void);

#endif
