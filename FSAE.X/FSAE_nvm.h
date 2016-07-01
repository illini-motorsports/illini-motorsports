/**
 * FSAE Library NVM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_nvm_H
#define FSAE_nvm_H

#include <xc.h>
#include <sys/types.h>
#include <sys/kmem.h>
#include "FSAE_config.h"

/**
 * TODO: The Pickit3 configuration must be set to preserve this page of program
 * flash, otherwise the values will be erased every time the device is programmed.
 */
#define USER_NVM_PHYS_ADDR 0x1D00F000

// Function definitions
void read_nvm_data(void* buffer, uint32_t count);
void write_nvm_data(void* buffer, uint32_t count);
void nvm_erase_page(void);
void nvm_init_operation(void);

#endif /* FSAE_nvm_H */
