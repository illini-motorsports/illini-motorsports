/**
 * FSAE Library CAN Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2016-2017
 */
#ifndef FSAE_SPI_H
#define FSAE_SPI_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"
#include <math.h>

#define PBCLK2 100

void init_spi1(int mhz, int size); // SDI: RB9, SDO: RB10
//void init_spi2(int mhz, int size); // SDI: RE5, SDO: RC1
//void init_spi3(int mhz, int size); // SDI: RB5, SDO: RB3
//void init_spi4(int mhz, int size); // Not Used
void init_spi5(int mhz, int size); // SDI: RF4, SDO: RA14
//void init_spi6(int mhz, int size); // SDI: RF2, SDO: RF8

#endif /* FSAE_SPI_H */
