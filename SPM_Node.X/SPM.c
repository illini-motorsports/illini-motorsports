#include "SPM.h"

double analog_channels[36] = {0};
double thermocouple_channels[6] = {0};
volatile uint32_t millis = 0;
uint32_t canAnalogMillis = 0;

void main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_adcs();// Initialize all of the ADC's
  init_tcouples(); // Initialize all max31855 thermocouple readers

  while(1){
    update_analog_channels();
    update_thermocouples();

    if(millis-canAnalogMillis > 25){
      CANAnalogChannels();
    }
  }
}

void CANAnalogChannels(void){
  CAN_data data = {0};
  int i;
  for(i = 0;i<9;i++){
    data.halfword0 = (uint16_t) (analog_channels[i*4]*ANALOG_CAN_SCL);
    data.halfword1 = (uint16_t) (analog_channels[(i*4)+1]*ANALOG_CAN_SCL);
    data.halfword2 = (uint16_t) (analog_channels[(i*4)+2]*ANALOG_CAN_SCL);
    data.halfword3 = (uint16_t) (analog_channels[(i*4)+3]*ANALOG_CAN_SCL);
    CAN_send_message(SPM_ID + i, 8, data);
  }
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count

  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

// reads from both ADC's at the same time, which kinda negates the point of the
// low speed vs high speed ADC's
void update_analog_channels(void){
  // Update ad7490 values
  uint16_t ad7490_values[32] = {0};
  ad7490_read_channels(ad7490_values, ad7490_0_send_spi);
  ad7490_read_channels(ad7490_values+16, ad7490_1_send_spi);
  int i;
  for(i = 0;i<32;i++){
    analog_channels[i] = 5*(ad7490_values[analogMappings[i]]/4095.0);
  }

  // manually read analog channels
  AD7680_0_CS_LAT = 0;
  analog_channels[32] = 5*(ad7680_read_spi()/65535.0);
  AD7680_0_CS_LAT = 1;

  AD7680_1_CS_LAT = 0;
  analog_channels[33] = 5*(ad7680_read_spi()/65535.0);
  AD7680_1_CS_LAT = 1;
  AD7680_2_CS_LAT = 0;
  analog_channels[34] = 5*(ad7680_read_spi()/65535.0);
  AD7680_2_CS_LAT = 1;

  AD7680_3_CS_LAT = 0;
  analog_channels[35] = 5*(ad7680_read_spi()/65535.0);
  AD7680_3_CS_LAT = 1;
}

void update_thermocouples(void) {
  thermocouple_channels[0] = read_max31855_temp(max31855_0_send_spi);
  thermocouple_channels[1] = read_max31855_temp(max31855_1_send_spi);
  thermocouple_channels[2] = read_max31855_temp(max31855_2_send_spi);
  thermocouple_channels[3] = read_max31855_temp(max31855_3_send_spi);
  thermocouple_channels[4] = read_max31855_temp(max31855_4_send_spi);
  thermocouple_channels[5] = read_max31855_temp(max31855_5_send_spi);
}

void set_pga(uint8_t chan, uint8_t level){
  if(level > 7 || chan > 3) {
    return; // invalid level or chan
  }

  // get current values
  uint16_t current_vals = mcp23s17_read_all(gpio_1_send_spi);

  // modify proper bits according to mappings
  uint16_t new_vals = set_bit_val(current_vals, pgaMappings[3*chan], level & 0x1);
  new_vals = set_bit_val(new_vals, pgaMappings[3*chan+1], (level>>1) & 0x1);
  new_vals = set_bit_val(new_vals, pgaMappings[3*chan+2], (level>>2) & 0x1);

  // set bits
  mcp23s17_write_all(new_vals, gpio_1_send_spi);
}

void init_adcs(void) {
  AD7490_0_CS_LAT = 1;
  AD7490_1_CS_LAT = 1;
  AD7490_0_CS_TRIS = OUTPUT;
  AD7490_1_CS_TRIS = OUTPUT;

  AD7680_0_CS_LAT = 1;
  AD7680_1_CS_LAT = 1;
  AD7680_2_CS_LAT = 1;
  AD7680_3_CS_LAT = 1;
  AD7680_0_CS_TRIS = OUTPUT;
  AD7680_1_CS_TRIS = OUTPUT;
  AD7680_2_CS_TRIS = OUTPUT;
  AD7680_3_CS_TRIS = OUTPUT;

  init_spi1(1, 16);
  init_ad7490(ad7490_0_send_spi);
  init_ad7490(ad7490_1_send_spi);// SPI1 will initialize twice

  init_spi2(2, 8); // 2mhz clk, 24 bits
}

void init_tcouples(void) {
  MAX31855_0_CS_LAT = 1;
  MAX31855_1_CS_LAT = 1;
  MAX31855_2_CS_LAT = 1;
  MAX31855_3_CS_LAT = 1;
  MAX31855_4_CS_LAT = 1;
  MAX31855_5_CS_LAT = 1;

  MAX31855_0_CS_TRIS = OUTPUT;
  MAX31855_1_CS_TRIS = OUTPUT;
  MAX31855_2_CS_TRIS = OUTPUT;
  MAX31855_3_CS_TRIS = OUTPUT;
  MAX31855_4_CS_TRIS = OUTPUT;
  MAX31855_5_CS_TRIS = OUTPUT;

  init_max31855(init_spi3);
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

uint32_t max31855_0_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_0_CS_LATBITS, MAX31855_0_CS_LATNUM);
}

uint32_t max31855_1_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_1_CS_LATBITS, MAX31855_1_CS_LATNUM);
}

uint32_t max31855_2_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_2_CS_LATBITS, MAX31855_2_CS_LATNUM);
}

uint32_t max31855_3_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_3_CS_LATBITS, MAX31855_3_CS_LATNUM);
}

uint32_t max31855_4_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_4_CS_LATBITS, MAX31855_4_CS_LATNUM);
}

uint32_t max31855_5_send_spi(uint32_t value){
  return send_spi3(value, MAX31855_5_CS_LATBITS, MAX31855_5_CS_LATNUM);
}

// build up result from 3 seperate spi reads
uint16_t ad7680_read_spi(){
  uint16_t result;

  // first 8 bits
  SPI2BUF = 0x00;
  while (!SPI2STATbits.SPIRBF);
  result = SPI2BUF << 12;
  // next 8 bits
  SPI2BUF = 0x00;
  while (!SPI2STATbits.SPIRBF);
  result |= (SPI2BUF << 4) & 0xFF0;
  // last 8 bits
  SPI2BUF = 0x00;
  while (!SPI2STATbits.SPIRBF);
  result |= (SPI2BUF >> 4) & 0xF;

  return result;
}

uint16_t set_bit_val(uint16_t current, uint8_t pos, uint8_t val) {
  return (current & ~(1 << pos)) | (val << pos);
}
