/**
 * FSAE Library SPI Header
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

typedef uint32_t (*send_spi_fp) (uint32_t, uint32_t*, uint8_t);

typedef struct {
  send_spi_fp send_fp; // send_spi function pointer
  uint32_t *cs_lat; // pointer to CS lat bits
  uint8_t cs_num; // number of bit in lat register
} SPIConn;

void init_spi(uint8_t bus, double mhz, uint8_t size);

send_spi_fp get_send_spi(uint8_t bus);
uint32_t send_spi1(uint32_t value, uint32_t *cs_lat, uint8_t cs_num);
uint32_t send_spi2(uint32_t value, uint32_t *cs_lat, uint8_t cs_num);
uint32_t send_spi3(uint32_t value, uint32_t *cs_lat, uint8_t cs_num);
uint32_t send_spi5(uint32_t value, uint32_t *cs_lat, uint8_t cs_num);
uint32_t send_spi6(uint32_t value, uint32_t *cs_lat, uint8_t cs_num);
uint32_t send_spi(uint32_t value, SPIConn *conn);

#endif /* FSAE_SPI_H */
