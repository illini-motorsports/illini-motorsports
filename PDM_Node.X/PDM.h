/**
 * PDM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef PDM_H
#define PDM_H

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <sys/types.h>
#include "FSAE_config_32.h"

void main(void);
void send_rheo(uint16_t msg);

#endif /* PDM_H */
