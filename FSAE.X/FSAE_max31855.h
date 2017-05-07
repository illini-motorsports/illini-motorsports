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

typedef struct {
  double tCoupleTemp;
  double junctionTemp;
  uint8_t fault;
} max31855_data;

void init_max31855(void (*init_spi)(double mhz, int size));

double read_max31855_temp(uint32_t (*send_value)(uint32_t));
max31855_data read_max31855_data(uint32_t (*send_value)(uint32_t));

#endif /* FSAE_MAX31855_H */
