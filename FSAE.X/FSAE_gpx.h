/**
 * FSAE Library GPX Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef FSAE_gpx_H
#define FSAE_gpx_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"

// Pin definitions for CS_GPX
#define CS_GPX_TRIS  TRISEbits.TRISE7
#define CS_GPX_LAT   LATEbits.LATE7

// Pin definitions for GPX_INT
#define GPX_INT_TRIS   TRISEbits.TRISE6
#define GPX_INT_PORT   PORTEbits.RE6
#define GPX_INT_ANSEL  ANSELEbits.ANSE6

// Opcodes
#define GPX_OPCODE_READ  0b01001111
#define GPX_OPCODE_WRITE 0b01001110

// Register addresses
#define GPX_IOCON 0x0A
#define GPX_GPINTENA 0x04
#define GPX_GPINTENB 0x05
#define GPX_GPIOA 0x12
#define GPX_GPIOB 0x13

// Function definitions
void init_gpx(void);
uint16_t gpx_read_state(void);
uint8_t _gpx_send_mesg(uint8_t opcode, uint8_t addr, uint8_t data);
void _gpx_init_spi(void);
void _gpx_init_int(void);

#endif /* FSAE_gpx_H */
