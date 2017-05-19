#include "FSAE_mcp23s17.h"

SPIConn mcp23s17Connections[10];
uint8_t mcp23s17ConnIdx = 0;

SPIConn* init_mcp23s17(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  if(!mcp23s17ConnIdx) {
    init_spi(bus, 3, 32);
  }

  SPIConn *currConn = &mcp23s17Connections[mcp23s17ConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  mcp23s17ConnIdx++;

  // Declare and set configuration bits
  MCP23S17IoconReg defaultConfig;
  defaultConfig.INTPOL = 1;
  defaultConfig.ODR = 0;
  defaultConfig.HAEN = 0;
  defaultConfig.DISSLW = 0;
  defaultConfig.SEQOP = 0;
  defaultConfig.MIRROR = 1;
  defaultConfig.BANK = 0;

  // Set both configuration registers
  uint16_t send = (defaultConfig.reg << 8) | defaultConfig.reg;
  mcp23s17_write_reg(MCP23S17_IOCON, send, currConn);

  // initialize the rest of the registers to default values
  mcp23s17_write_reg(MCP23S17_IODIR, 0x0000, currConn); // All outputs
  mcp23s17_write_reg(MCP23S17_IPOL, 0x0000, currConn); // Do not reverse polarity
  mcp23s17_write_reg(MCP23S17_IPOL, 0x0000, currConn); // Do not reverse polarity
  mcp23s17_write_reg(MCP23S17_GPINTEN, 0x0000, currConn); // no interrupts
  mcp23s17_write_reg(MCP23S17_DEFVAL, 0x0000, currConn); // default value of 0
  mcp23s17_write_reg(MCP23S17_INTCON, 0x0000, currConn); // use prev value for interrupts
  mcp23s17_write_reg(MCP23S17_GPPU, 0x0000, currConn); // Do not use pullup
  mcp23s17_write_reg(MCP23S17_GPIO, 0x0000, currConn); // sets all pins to 0
}

uint16_t mcp23s17_read_all(SPIConn *conn) {
  return mcp23s17_read_reg(MCP23S17_GPIO, conn);
}

uint8_t mcp23s17_read_one(uint8_t pin, SPIConn *conn) {
  return mcp23s17_read_reg(MCP23S17_GPIO, conn);
}

void mcp23s17_write_all(uint16_t value, SPIConn *conn) {
  return mcp23s17_write_reg(MCP23S17_GPIO, value, conn);
}

void mcp23s17_write_one(uint8_t pin, uint8_t value, SPIConn *conn) {
  uint16_t current_values = mcp23s17_read_reg(MCP23S17_GPIO, conn);
  uint16_t new_values = (current_values & ~(1 << pin)) | (value << pin); // substitute in new value
  mcp23s17_write_reg(MCP23S17_GPIO, new_values, conn);
}

uint16_t mcp23s17_read_reg(uint8_t addr, SPIConn *conn) {
  uint32_t send = (MCP23S17_READ << 24) | (addr << 16) | 0x0000;
  return (uint16_t) send_spi(send, conn);
}

void mcp23s17_write_reg(uint8_t addr, uint16_t value, SPIConn *conn) {
  uint32_t send = (MCP23S17_WRITE << 24) | (addr << 16) | value;
  send_spi(send, conn);
}
