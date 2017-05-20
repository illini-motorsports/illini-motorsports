/**
 * FSAE Library Digital Rheostat Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2016-2017
 */
#ifndef FSAE_RHEO_H
#define FSAE_RHEO_H

#include <sys/types.h>
#include "FSAE_config.h"
#include "FSAE_spi.h"

SPIConn* init_rheo(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);

void set_rheo(uint8_t val, SPIConn *conn);
void send_all_rheo(uint16_t msg);

#endif /* FSAE_RHEO_H */
