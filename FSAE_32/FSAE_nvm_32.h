/**
 * FSAE Library 32bit NVM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_nvm_32_H
#define FSAE_nvm_32_H

#include <xc.h>
#include <sys/types.h>
#include <sys/kmem.h>
#include "FSAE_config_32.h"

/**
 * Use the last 8KB of program flash as NVM space.
 *
 * NOTE: The Pickit3 configuration must be set to preserve these last two pages
 * of program flash, otherwise the values will be erased every time the device
 * is programmed.
 */
#define USER_NVM_PHYS_ADDR 0x1D1FE000

// Function definitions
void read_nvm_data(void* buffer, uint32_t count);
void write_nvm_data(void* buffer, uint32_t count);

#endif /* FSAE_nvm_32_H */
