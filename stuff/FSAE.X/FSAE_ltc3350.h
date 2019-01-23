/**
 * FSAE Library LTC3350 Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef FSAE_LTC3350_H
#define FSAE_LTC3350_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"

#define LTC3350_DEV_ADDR 0b0001001
#define PIC_DEV_ADDR     0b0011100

// Pin definitions
#define CHG_PFO_TRIS  TRISCbits.TRISC4
#define CHG_PFO_ANSEL ANSELCbits.ANSC4
#define CHG_PFO_PORT  PORTCbits.RC4
#define CHG_ALM_TRIS  TRISCbits.TRISC2
#define CHG_ALM_ANSEL ANSELCbits.ANSC2
#define CHG_ALM_PORT  PORTCbits.RC2

// Function definitions
void init_ltc3350(void);
uint16_t _ltc3350_read(uint8_t addr);
void _ltc3350_write(uint8_t addr, uint16_t data);
void _ltc3350_init_i2c(void);

#endif /* FSAE_LTC3350_H */
