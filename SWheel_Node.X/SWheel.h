/**
 * SWheel Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef SWHEEL_H
#define SWHEEL_H

#include <sys/types.h>
#include "../FSAE_32/FSAE_config_32.h"
#include "RA8875_driver.h"

// Enable programmable termination
#define TERMINATING 1

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 1

#define LCD_CS_TRIS TRISBbits.TRISB8
#define LCD_CS_LAT  LATBbits.LATB8
#define LCD_RST_TRIS TRISBbits.TRISB11
#define LCD_RST_LAT  LATBbits.LATB11

// Function definitions
void main(void);
void delay(uint32_t num);
void drawChevron(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t thick);

#endif /* SWHEEL_H */
