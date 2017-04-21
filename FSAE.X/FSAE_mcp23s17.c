#include "FSAE_mcp23s17.h"

void init_mcp23s17(uint32_t (*send_value)(uint32_t)) {
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
	mcp23s17_write_reg(MCP23S17_IOCON, send, send_value);

	// initialize the rest of the registers to default values
	mcp23s17_write_reg(MCP23S17_IODIR, 0x0000, send_value); // All outputs
	mcp23s17_write_reg(MCP23S17_IPOL, 0x0000, send_value); // Do not reverse polarity
	mcp23s17_write_reg(MCP23S17_IPOL, 0x0000, send_value); // Do not reverse polarity
	mcp23s17_write_reg(MCP23S17_GPINTEN, 0x0000, send_value); // no interrupts
	mcp23s17_write_reg(MCP23S17_DEFVAL, 0x0000, send_value); // default value of 0
	mcp23s17_write_reg(MCP23S17_INTCON, 0x0000, send_value); // use prev value for interrupts
	mcp23s17_write_reg(MCP23S17_GPPU, 0x0000, send_value); // Do not use pullup
	mcp23s17_write_reg(MCP23S17_GPIO, 0x0000, send_value); // sets all pins to 0
}

uint16_t mcp23s17_read_all(uint32_t (*send_value)(uint32_t)) {
	return mcp23s17_read_reg(MCP23S17_GPIO, send_value);
}

uint8_t mcp23s17_read_one(uint8_t pin, uint32_t (*send_value)(uint32_t)) {
	return mcp23s17_read_reg(MCP23S17_GPIO, send_value);
}

void mcp23s17_write_all(uint16_t value, uint32_t (*send_value)(uint32_t)) {
	return mcp23s17_write_reg(MCP23S17_GPIO, value, send_value);
}

void mcp23s17_write_one(uint8_t pin, uint8_t value, uint32_t (*send_value)(uint32_t)) {
	uint16_t current_values = mcp23s17_read_reg(MCP23S17_GPIO, send_value);
	uint16_t new_values = (current_values & ~(1 << pin)) | (value << pin); // substitute in new value
	mcp23s17_write_reg(MCP23S17_GPIO, new_values, send_value);
}

uint16_t mcp23s17_read_reg(uint8_t addr, uint32_t (*send_value)(uint32_t)) {
	uint32_t send = (MCP23S17_READ << 24) | (addr << 16) | 0x0000;
	return (uint16_t) send_value(send);
}

void mcp23s17_write_reg(uint8_t addr, uint16_t value, uint32_t (*send_value)(uint32_t)) {
	uint32_t send = (MCP23S17_WRITE << 24) | (addr << 16) | value;
	send_value(send);
}
