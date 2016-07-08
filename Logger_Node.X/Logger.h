/**
 * Logger Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_nvm.h"
#include "../FSAE.X/CAN.h"

// Disable programmable termination
#define TERMINATING 0

// Determines whether the internal or external clock source is used
#define INTERNAL_CLK 1

// Determines whether SYSCLK / 10 is driven out on RF0
#define REFCLKO 0

// Timing constants (ms)
#define DIAG_MSG_SEND      1000

// Function definitions
void main(void);
void process_CAN_msg(CAN_message msg);
void send_diag_can(void);

#endif /* LOGGER_H */