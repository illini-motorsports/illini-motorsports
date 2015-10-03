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

#define OUTPUT 0
#define INPUT 1

void init_general(void);
void init_gpio_pins(void);
void init_peripheral_modules(void);
void init_oscillator(void);
void init_can(void);
void init_timer1(void);
void init_spi(void);

#endif /* FSAE_CONFIG_32_H */