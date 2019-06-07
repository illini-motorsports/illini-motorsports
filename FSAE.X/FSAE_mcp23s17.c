#include "FSAE_mcp23s17.h"

SPIConn mcp23s17Connections[10];
uint8_t mcp23s17ConnIdx = 0;

SPIConn* init_mcp23s17(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  if(!mcp23s17ConnIdx) {
    // TODO: Check if mode is correct
    // TODO: Try different speeds
    init_spi(bus, 1, 32, 0);
  }

  SPIConn *currConn = &mcp23s17Connections[mcp23s17ConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  mcp23s17ConnIdx++;
  
  MCP23S17_INT_TRIS = INPUT;

  // Declare and set configuration bits
  MCP23S17IoconReg defaultConfig;
  defaultConfig.INTPOL = 1; // Active High INT
  defaultConfig.ODR = 0; // Do not set INT pin to open drain
  defaultConfig.HAEN = 0; // Set hardware address to 0
  defaultConfig.DISSLW = 0; // Do not use slew rate control
  defaultConfig.SEQOP = 0; // Do not use sequential operation mode
  defaultConfig.MIRROR = 1; // An interrupt on either bank will cause both INT pins to activate
  defaultConfig.BANK = 0; // Use Paired address mappings

  // Set both configuration registers
          
  uint16_t send = (defaultConfig.reg << 8) | defaultConfig.reg;
  mcp23s17_write_reg_ctrl(MCP23S17_IOCON, send, currConn);

  // initialize the rest of the registers to default values
  mcp23s17_write_reg_ctrl(MCP23S17_IODIR, 0xFF, currConn); // All inputs
  mcp23s17_write_reg_ctrl(MCP23S17_IPOL, 0x00, currConn); // Do not reverse polarity
  mcp23s17_write_reg_ctrl(MCP23S17_GPINTEN, 0x00, currConn); // no interrupts
  mcp23s17_write_reg_ctrl(MCP23S17_DEFVAL, 0x00, currConn); // default value of 0
  mcp23s17_write_reg_ctrl(MCP23S17_INTCON, 0x00, currConn); // use prev value for interrupts
  mcp23s17_write_reg_ctrl(MCP23S17_GPPU, 0x00, currConn); // Do not use pullup

  return currConn;
}

// Returns all pins of the IC, bank A LSB and bank B MSB
uint16_t mcp23s17_read_all(SPIConn *conn) {
    //send_spi(0xAAAA, conn);
    //return 0;
    uint8_t reg2 = mcp23s17_read_reg(MCP23S17_GPIO, conn);
    uint8_t reg1 = mcp23s17_read_reg(MCP23S17_GPIO + 1, conn);
    return ((reg1 << 8) | reg2);
}
/*
// Returns 1 if bit is set, 0 if not.
// Pins 0-7 go to bank a and pins 8-15 go to bank b
uint8_t mcp23s17_read_one(uint8_t pin, SPIConn *conn) {
  if(pin < 8) {
    return (mcp23s17_read_reg(MCP23S17_GPIO, conn) & (0x1 << pin)) != 0;
  }
  return (mcp23s17_read_reg(MCP23S17_GPIO + 1, conn) & (0x1 << (pin - 8))) != 0;
}
*/
// Writes to all pins of the IC, bank A is LSB, bank B is MSB
void mcp23s17_write_all(uint16_t value, SPIConn *conn) {
  mcp23s17_write_reg(MCP23S17_GPIO, value & 0xFF, conn);
  mcp23s17_write_reg(MCP23S17_GPIO + 1, (value & 0xFF00) >> 8, conn);
}

// Reads value of register at specified address
uint8_t mcp23s17_read_reg(uint8_t addr, SPIConn *conn) {
  uint32_t control_byte = (MCP23S17_CONTROL << 4) | (MCP23S17_HW_ADDR << 1) | MCP23S17_READ;
  uint32_t send = (control_byte << 24) | (addr << 16) | 0xFFFF;
  uint32_t response = send_spi(send,conn);
  response = (response & 0xFFFF) >> 9; //Fuck if I know but it works
  return (uint8_t) response;
}

// Writes value to specified address
void mcp23s17_write_reg(uint8_t addr, uint8_t value, SPIConn *conn) {
  uint8_t control_byte = (MCP23S17_CONTROL << 4) | (MCP23S17_HW_ADDR << 1) | MCP23S17_WRITE;
  uint16_t send = (control_byte << 24) | (addr << 16) | (value << 8);
  send_spi(send, conn);
}

// Writes value to specified address and address + 1
void mcp23s17_write_reg_ctrl(uint8_t addr, uint8_t value, SPIConn *conn) {
  uint8_t control_byte = (MCP23S17_CONTROL << 4) | (MCP23S17_HW_ADDR << 1) | MCP23S17_WRITE;
  uint16_t sendA = (control_byte << 24) | (addr << 16) | (value << 8);
  uint16_t sendB = (control_byte << 24) | ((addr+1) << 16) | (value << 8);
  send_spi(sendA, conn);
  send_spi(sendB, conn);
}