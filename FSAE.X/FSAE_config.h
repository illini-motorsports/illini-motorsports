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

#ifdef BUILD_PDM
#include "../PDM_Node.X/PDM.h"
#elif BUILD_WHEEL
#include "../Wheel_Node.X/Wheel.h"
#elif BUILD_LOGGER
#include "../Logger_Node.X/Logger.h"
#endif

// Code control defines
//#define REFCLKO

// TRIS Settings
#define OUTPUT 0
#define INPUT 1

// ANSEL Settings
#define AN_INPUT  1
#define DIG_INPUT 0

// Definitions for interval override control
#define OVERRIDE    1
#define NO_OVERRIDE 0

// Pin definitions for programmable termination
#define TERM_TRIS TRISAbits.TRISA2
#define TERM_LAT  LATAbits.LATA2

#define CLI() asm volatile("di; ehb;")
#define STI() asm volatile("ei;")

// Function definitions
void unlock_config(void);
void lock_config(void);
void init_general(void);
void init_peripheral_modules(void);
void init_gpio_pins(void);
void init_oscillator(void);
void init_timer1(void);
void init_timer2(void);
void init_termination(uint8_t isTerm);

#endif /* FSAE_CONFIG_H */
