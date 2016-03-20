/**
 * Wheel Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef WHEEL_H
#define WHEEL_H

#include <sys/types.h>
#include "../FSAE_32/FSAE_config_32.h"
#include "../FSAE_32/FSAE_CAN_32.h"
#include "../FSAE.X/CAN.h"
#include "RA8875_driver.h"

// Enable programmable termination
#define TERMINATING 1

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 0

#define LCD_CS_TRIS TRISGbits.TRISG9
#define LCD_CS_LAT  LATGbits.LATG9
#define LCD_RST_TRIS TRISBbits.TRISB4
#define LCD_RST_LAT  LATBbits.LATB4

// Function definitions
void main(void);
void delay(uint32_t num);
void process_CAN_msg(CAN_message msg);

#endif /* WHEEL_H */
