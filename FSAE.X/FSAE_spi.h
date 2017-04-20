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

#include <sys/types.h>
#include "FSAE_config.h"
#include <math.h>

#define PBCLK2 100

void init_spi1(int mhz, int size); // SDI: RB9, SDO: RB10
void init_spi2(int mhz, int size); // SDI: RE5, SDO: RC1
void init_spi3(int mhz, int size); // SDI: RB5, SDO: RB3
//void init_spi4(int mhz, int size); // Not Used
void init_spi5(int mhz, int size); // SDI: RF4, SDO: RA14
void init_spi6(int mhz, int size); // SDI: RF2, SDO: RF8

uint32_t send_spi1(uint32_t value, uint32_t *cs_lat, int cs_num);
uint32_t send_spi2(uint32_t value, uint32_t *cs_lat, int cs_num);
uint32_t send_spi3(uint32_t value, uint32_t *cs_lat, int cs_num);
uint32_t send_spi5(uint32_t value, uint32_t *cs_lat, int cs_num);
uint32_t send_spi6(uint32_t value, uint32_t *cs_lat, int cs_num);

#endif /* FSAE_SPI_H */
