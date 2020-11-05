/**
 * FSAE Library AD7490 Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef FSAE_ad7490_H
#define FSAE_ad7490_H

#include "FSAE_config.h"
#include "FSAE_spi.h"
#include <sys/types.h>
#include <xc.h>

#define AD7490_NUM_CHN 16
#define AD7490_DEFAULT 0b0000001101000000

// Pin definitions for CS_AD7490
#define CS_AD7490_TRIS TRISAbits.TRISA1
#define CS_AD7490_LAT LATAbits.LATA1

// Struct representing the chip's control register
typedef union uAD7490ControlReg {
  struct {
    unsigned UNUSED : 4;
    unsigned CODING : 1;
    unsigned RANGE : 1;
    unsigned WEAK_TRI : 1;
    unsigned SHADOW : 1;
    unsigned PM : 2;
    unsigned ADDR : 4;
    unsigned SEQ : 1;
    unsigned WRITE : 1;
  };
  uint16_t reg;
} AD7490ControlReg;

// Function definitions
SPIConn *init_ad7490(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);
void ad7490_read_channels(uint16_t *channel_values, SPIConn *conn);

#endif /* FSAE_ad7490_H */
