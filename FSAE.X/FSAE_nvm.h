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

// Struct representing the NVM chip's status register
typedef union uNvmStatusReg {
  struct {
    unsigned WIP:1;
    unsigned WEL:1;
    unsigned BP0:1;
    unsigned BP1:1;
    unsigned IGN:3;
    unsigned WPEN:1;
  };
  uint8_t reg;
} NvmStatusReg;

// NVM Chip Instruction Set
#define IN_READ  0b00000011
#define IN_WRITE 0b00000010
#define IN_WREN  0b00000110
#define IN_WRDI  0b00000100
#define IN_RDSR  0b00000101
#define IN_WRSR  0b00000001
#define IN_PE    0b01000010
#define IN_SE    0b11011000
#define IN_CE    0b11000111
#define IN_RDID  0b10101011
#define IN_DPD   0b10111001

// Pin definitions for CS_NVM
#define CS_NVM_TRIS  TRISDbits.TRISD14
#define CS_NVM_LAT LATDbits.LATD14

// Function definitions
void nvm_write_enable(SPIConn *conn);
void nvm_write_status_reg(NvmStatusReg status, SPIConn *conn);
NvmStatusReg nvm_read_status_reg(SPIConn *conn);
void nvm_read_data(uint32_t address, uint8_t *bytes, uint8_t numBytes, SPIConn *conn);
SPIConn* init_nvm(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);
uint8_t _nvm_send_two(uint8_t one, uint8_t two, SPIConn *conn);

#endif /* FSAE_nvm_H */
