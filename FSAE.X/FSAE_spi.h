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

#include "FSAE_config.h"
#include <math.h>
#include <sys/types.h>

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

typedef uint32_t (*send_spi_fp)(uint32_t);

/*
 *Struct used for all SPI communcation in the FSAE Library.
 *Contains a send_spi function pointer, and CS pin infomration.
 *send_spi functions will take SPIConn pointer as an argument
 *to communicate with a specific chip on a given SPI Bus.
 */
typedef struct {
  send_spi_fp send_fp; // send_spi function pointer
  uint32_t *cs_lat;    // pointer to CS lat bits
  uint8_t cs_num;      // number of bit in lat register
} SPIConn;

void init_spi(uint8_t bus, double mhz, uint8_t size, uint8_t mode);

send_spi_fp get_send_spi(uint8_t bus);
uint32_t send_spi1(uint32_t value);
uint32_t send_spi2(uint32_t value);
uint32_t send_spi3(uint32_t value);
uint32_t send_spi5(uint32_t value);
uint32_t send_spi6(uint32_t value);
uint32_t send_spi(uint32_t value, SPIConn *conn);
uint64_t send_spi_double(uint32_t value1, uint32_t value2, SPIConn *conn);
uint64_t send_spi_triple_16(uint64_t value, SPIConn *conn);
void send_spi_many_16(uint16_t value, SPIConn *conn);
uint64_t send_spi_triple(uint64_t value, SPIConn *conn);
void send_spi_CAN(uint64_t value[], uint64_t rec[], SPIConn *conn);

// Set !CS Low
inline void spi_select(SPIConn *conn) {
  *(conn->cs_lat) &= ~(1 << (conn->cs_num));
};

// Set !CS High
inline void spi_deselect(SPIConn *conn) {
  *(conn->cs_lat) |= 1 << conn->cs_num;
}

#endif /* FSAE_SPI_H */
