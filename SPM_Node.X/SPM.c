#include "SPM.h"

uint16_t analog_channels[32] = {0};

void main(void){
	init_general();// Set general runtime configuration bits
	init_gpio_pins();// Set all I/O pins to low outputs
	init_oscillator();// Initialize oscillator configuration bits
	init_timer2();// Initialize timer2 (millis)

	while(1){
		update_analog_channels();
	}
}

void update_analog_channels(void){
	uint16_t ad7490_values[32] = {0};
	ad7490_read_channels(ad7490_values, ad7490_0_send_spi);
	ad7490_read_channels(ad7490_values+16, ad7490_1_send_spi);
	int i;
	for(i = 0;i<32;i++){
		analog_channels[i] = ad7490_values[analogMappings[i]];
	}
}

void init_adcs(void) {
	AD7490_0_CS_LAT = 1;
	AD7490_1_CS_LAT = 1;
	AD7490_0_CS_TRIS = OUTPUT;
	AD7490_1_CS_TRIS = OUTPUT;

	init_spi1(1, 16);
	init_ad7490(ad7490_0_send_spi);
	init_ad7490(ad7490_1_send_spi);// SPI1 will initialize twice
}

void init_gpio_ext(void){
	// Initialize all CS pins
	MCP23S17_0_CS_LAT = 1;
	MCP23S17_1_CS_LAT = 1;
	MCP23S17_2_CS_LAT = 1;
	MCP23S17_0_CS_TRIS = OUTPUT;
	MCP23S17_1_CS_TRIS = OUTPUT;
	MCP23S17_2_CS_TRIS = OUTPUT;
	
	init_spi5(3, 32); // initialize to 3 mhz and 32 bit width
	
	// initialize all gpio chips
	init_mcp23s17(gpio_0_send_spi);
	init_mcp23s17(gpio_1_send_spi);
	init_mcp23s17(gpio_2_send_spi);

	mcp23s17_write_reg(MCP23S17_IODIR, 0xFFFF, gpio_2_send_spi); // digital inputs
}

uint32_t ad7490_0_send_spi(uint32_t value){
	return send_spi1(value, AD7490_0_CS_LATBITS, AD7490_0_CS_LATNUM);
}

uint32_t ad7490_1_send_spi(uint32_t value){
	return send_spi1(value, AD7490_1_CS_LATBITS, AD7490_1_CS_LATNUM);
}

uint32_t gpio_0_send_spi(uint32_t value){
	return send_spi5(value, MCP23S17_0_CS_LATBITS, MCP23S17_0_CS_LATNUM);
}

uint32_t gpio_1_send_spi(uint32_t value){
	return send_spi5(value, MCP23S17_1_CS_LATBITS, MCP23S17_1_CS_LATNUM);
}

uint32_t gpio_2_send_spi(uint32_t value){
	return send_spi5(value, MCP23S17_2_CS_LATBITS, MCP23S17_2_CS_LATNUM);
}
