/**
 * FSAE Library 32bit Config Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_CONFIG_32_H
#define FSAE_CONFIG_32_H

#include <xc.h>
#include <sys/kmem.h>
#include <sys/types.h>

#ifdef BUILD_PDM
#include "../PDM_Node.X/PDM.h"
#endif

// Code control defines
//#define REFCLKO

// TRIS Settings
#define OUTPUT 0
#define INPUT 1

// Pin definitions for programmable termination
#define TERM_TRIS TRISAbits.TRISA2
#define TERM_LAT  LATAbits.LATA2

// Function definitions
void init_general(void);
void init_gpio_pins(void);
void init_peripheral_modules(void);
void init_oscillator(void);
void init_can(void);
void init_timer1(void);
void init_spi(void);
void init_adc(void);
void init_termination(void);
inline void CLI(void);
inline void STI(void);

#endif /* FSAE_CONFIG_32_H */
