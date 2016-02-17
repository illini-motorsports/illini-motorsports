/**
 * PaddleShifting Header
 *
 * Processor:    PIC18F46K80
 * Complier:     Microchip C18
 * Author:       Andrew Mass
 * Date:         2015-2016
 */

#ifndef PADDLESHIFTING_H
#define PADDLESHIFTING_H

#include "p18f46k80.h"
#include "GenericTypeDefs.h"
//#include <stdlib.h>
#include "../ECAN.X/ECAN.h"
#include "../FSAE.X/FSAE.h"
#include "../FSAE.X/CAN.h"

// Pin definitions for TERM signal
#define TERM_TRIS TRISCbits.TRISC6
#define TERM_LAT	LATCbits.LATC6

// Function definitions
void high_isr(void);
void low_isr (void);

#endif /* PADDLESHIFTING_H */
