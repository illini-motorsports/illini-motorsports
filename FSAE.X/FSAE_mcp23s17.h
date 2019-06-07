/**
 * FSAE Library MCP23S17  Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2016-2017
 */
#ifndef FSAE_MCP23S17_H
#define FSAE_MCP23S17_H

#include <xc.h>
#include <sys/types.h>
#include "FSAE_config.h"
#include "FSAE_spi.h"

#define MCP23S17_IODIR    0x00 //1=input
#define MCP23S17_IPOL     0x02 //1=inverted inputs
#define MCP23S17_GPINTEN  0x04 //enables interrupts on changes
#define MCP23S17_DEFVAL   0x06 //Interrupt occurs if port != default
#define MCP23S17_INTCON   0x08 //1=use default value, 0= use previous value
#define MCP23S17_IOCON    0x0A //See bitfield below
#define MCP23S17_GPPU     0x0C //Attach internal pullup resistor (100k)
#define MCP23S17_INTF     0x0E //Interrupt Flag
#define MCP23S17_INTCAP   0x10 //Interrupt captured value
#define MCP23S17_GPIO     0x12 //Reading reads PORT, Writing writes LAT
#define MCP23S17_OLAT     0x14 //Output Latch

#define MCP23S17_CONTROL  0b0100 // Control Nibble
#define MCP23S17_HW_ADDR  0b000  // Hardware byte address (ususally 0)
#define MCP23S17_READ     0b1    // Read Bit
#define MCP23S17_WRITE    0b0    // Write Bit

#define MCP23S17_INT_TRIS TRISFbits.TRISF5

// Struct representing the chip's control register
typedef union {
  struct {
    unsigned UNUSED:1;
    unsigned INTPOL:1; // Polarity of int pin
    unsigned ODR:1; // Int pin as open drain
    unsigned HAEN:1; // Hardware Address enable
    unsigned DISSLW:1; // Slew Rate
    unsigned SEQOP:1; // sequential operation mode
    unsigned MIRROR:1; // interrupt pin mirroring
    unsigned BANK:1; // Bank Addressing
  };
  uint8_t reg;
} MCP23S17IoconReg;

// Function definitions
SPIConn* init_mcp23s17(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num);
uint16_t mcp23s17_read_all(SPIConn *conn);
uint8_t mcp23s17_read_one(uint8_t pin, SPIConn *conn);
void mcp23s17_write_all(uint16_t value, SPIConn *conn);
void mcp23s17_write_one(uint8_t pin, uint8_t value, SPIConn *conn);
uint8_t mcp23s17_read_reg(uint8_t addr, SPIConn *conn);
void mcp23s17_write_reg(uint8_t addr, uint8_t value, SPIConn *conn);
void mcp23s17_write_reg_ctrl(uint8_t addr, uint8_t value, SPIConn *conn);

#endif /* FSAE_ad7490_H */
