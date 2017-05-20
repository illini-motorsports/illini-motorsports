/**
 * FSAE Library MAX31855 Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2016-2017
 */
#ifndef FSAE_MAX31855_H
#define FSAE_MAX31855_H

#include <sys/types.h>
#include "FSAE_config.h"
#include "FSAE_spi.h"

typedef struct {
  double thermocoupleTemp;
  double junctionTemp;
  uint8_t fault;
} max31855_data;

SPIConn* init_max31855(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);

double read_max31855_temp(SPIConn *conn);
max31855_data read_max31855_data(SPIConn *conn);

#endif /* FSAE_MAX31855_H */
