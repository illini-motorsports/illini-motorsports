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
#include "FSAE_config.h"
#include "FSAE_spi.h"

// Microchip 25LC1024

// Default NVM definitions
#define _NVM_STD_BUS        6
#define _NVM_STD_CS_LATBITS ((uint32_t*) (&LATDbits))
#define _NVM_STD_CS_LATNUM  14

// Struct representing the NVM chip's status register
typedef union uNvmStatusReg {
  struct {
    unsigned WIP:1;
    unsigned WEL:1;
    unsigned BP0:1;
    unsigned BP1:1;
    unsigned RESV:3;
    unsigned WPEN:1;
  };
  uint8_t reg;
} _NvmStatusReg;

// NVM Chip Instruction Set
#define _NVM_IN_READ  0b00000011
#define _NVM_IN_WRITE 0b00000010
#define _NVM_IN_WREN  0b00000110
#define _NVM_IN_WRDI  0b00000100
#define _NVM_IN_RDSR  0b00000101
#define _NVM_IN_WRSR  0b00000001
#define _NVM_IN_PE    0b01000010
#define _NVM_IN_SE    0b11011000
#define _NVM_IN_CE    0b11000111
#define _NVM_IN_RDID  0b10101011
#define _NVM_IN_DPD   0b10111001

// Function definitions

// Public Interface
SPIConn* init_nvm_std(); // Use standard settings
SPIConn* init_nvm(uint8_t bus, uint32_t* cs_lat, uint8_t cs_num);
uint8_t nvm_alloc(SPIConn* conn, uint32_t size);
int8_t nvm_read(SPIConn* conn, uint8_t fd, uint8_t* buf);
int8_t nvm_write(SPIConn* conn, uint8_t fd, uint8_t* buf);

// Private Implementation
//TODO: Modify/implement private functions to support new public interface
void _nvm_write_enable(SPIConn* conn);
void _nvm_write_status_reg(_NvmStatusReg status, SPIConn* conn);
_NvmStatusReg _nvm_read_status_reg(SPIConn* conn);
void _nvm_read_data(uint32_t address, uint8_t* bytes, uint8_t numBytes, SPIConn* conn);
uint8_t _nvm_send_two(uint8_t one, uint8_t two, SPIConn* conn);

#endif /* FSAE_nvm_H */
