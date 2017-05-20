#include "SPM.h"

double analog_channels[36] = {0};
SPIConn *analog_connections[6] = {0};
double thermocouple_channels[6] = {0};
double thermocouple_junctions[6] = {0};
SPIConn *thermocouple_connections[6] = {0};
uint8_t thermocouple_fault = 0;
uint16_t digital_channels = 0;
SPIConn *digital_connections[3] = {0};
volatile uint32_t millis = 0;
uint32_t canAnalogMillis = 0;
uint32_t canThermocoupleMillis = 0;

void main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(0);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_adcs();// Initialize all of the ADC's
  init_tcouples(); // Initialize all max31855 thermocouple readers

  while(1){
    update_analog_channels();
    update_thermocouples();
    update_digital_channels();

    if(millis-canAnalogMillis > 50){
      CANAnalogChannels();
      CANThermocouples();
      canAnalogMillis = millis;
      canThermocoupleMillis = millis;
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
    CAN_send_message(SPM_ID + i + 1, 8, data);
  }
}

void CANThermocouples(void){
  CAN_data data = {};

  data.halfword0 = (int16_t) (thermocouple_channels[0]*THERMOCOUPLE_CAN_SCL);
  data.halfword1 = (int16_t) (thermocouple_channels[1]*THERMOCOUPLE_CAN_SCL);
  data.halfword2 = (int16_t) (thermocouple_channels[2]*THERMOCOUPLE_CAN_SCL);
  data.halfword3 = (int16_t) (thermocouple_channels[3]*THERMOCOUPLE_CAN_SCL);

  CAN_send_message(SPM_ID + 10, 8, data);

  data.halfword0 = (int16_t) (thermocouple_channels[0]*THERMOCOUPLE_CAN_SCL);
  data.halfword1 = (int16_t) (thermocouple_channels[1]*THERMOCOUPLE_CAN_SCL);

  double acc = 0;
  int i;
  for(i=0;i<6;i++){
    acc += thermocouple_junctions[i];
  }

  data.halfword2 = (int16_t) ((acc/6.0)*JUNCTION_CAN_SCL);
  data.halfword3 = thermocouple_fault;

  CAN_send_message(SPM_ID + 11, 8, data);
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
  ad7490_read_channels(ad7490_values, analog_connections[0]);
  ad7490_read_channels(ad7490_values+16, analog_connections[1]);
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
  max31855_data data;
  thermocouple_fault = 0;
  int i;

  for(i = 0;i < 6; i++){
    data = read_max31855_data(thermocouple_connections[i]);
    thermocouple_channels[i] = data.thermocoupleTemp;
    thermocouple_junctions[i] = data.junctionTemp;
    thermocouple_fault |= data.fault << i;   
  }
}

void set_pga(uint8_t chan, uint8_t level){
  if(level > 7 || chan > 3) {
    return; // invalid level or chan
  }

  // get current values
  uint16_t current_vals = mcp23s17_read_all(digital_connections[1]);

  // modify proper bits according to mappings
  uint16_t new_vals = set_bit_val(current_vals, pgaMappings[3*chan], level & 0x1);
  new_vals = set_bit_val(new_vals, pgaMappings[3*chan+1], (level>>1) & 0x1);
  new_vals = set_bit_val(new_vals, pgaMappings[3*chan+2], (level>>2) & 0x1);

  // set bits
  mcp23s17_write_all(new_vals, digital_connections[1]);
}

// divisor = 2^div
void set_freq_div(uint8_t chan, uint8_t div) {
  uint8_t byp, divBytes;
  uint16_t current_vals, new_vals;

  if(div <= 16) {
    byp = 1;
    divBytes = div - 1;
  }

  else {
    byp = 0;
    divBytes = 0x8 | ((div - 1) & 0x7);
  }

  switch(chan) {
    case 0:
      current_vals = mcp23s17_read_all(digital_connections[0]);
      new_vals = set_bit_val(current_vals, FREQ_BYP_0, byp);
      new_vals = set_bit_val(current_vals, FREQ_DIVA_0, divBytes & 0x1);
      new_vals = set_bit_val(current_vals, FREQ_DIVB_0, divBytes & 0x2);
      new_vals = set_bit_val(current_vals, FREQ_DIVC_0, divBytes & 0x4);
      new_vals = set_bit_val(current_vals, FREQ_DIVD_0, divBytes & 0x8);
      mcp23s17_write_all(new_vals, digital_connections[0]);
      break;

    case 1: // #2 constants are used due to bad naming standards on SPM
      current_vals = mcp23s17_read_all(digital_connections[0]);
      new_vals = set_bit_val(current_vals, FREQ_BYP_2, byp);
      new_vals = set_bit_val(current_vals, FREQ_DIVA_2, divBytes & 0x1);
      new_vals = set_bit_val(current_vals, FREQ_DIVB_2, divBytes & 0x2);
      new_vals = set_bit_val(current_vals, FREQ_DIVC_2, divBytes & 0x4);
      new_vals = set_bit_val(current_vals, FREQ_DIVD_2, divBytes & 0x8);
      mcp23s17_write_all(new_vals, digital_connections[0]);
      break;

    case 2: // This one has pins spread across two gpio chips
      mcp23s17_write_one(FREQ_DIVA_3, byp, digital_connections[0]);
      current_vals = mcp23s17_read_all(digital_connections[1]);
      new_vals = set_bit_val(current_vals, FREQ_DIVA_3, divBytes & 0x1);
      new_vals = set_bit_val(current_vals, FREQ_DIVB_3, divBytes & 0x2);
      new_vals = set_bit_val(current_vals, FREQ_DIVC_3, divBytes & 0x4);
      new_vals = set_bit_val(current_vals, FREQ_DIVD_3, divBytes & 0x8);
      mcp23s17_write_all(new_vals, digital_connections[1]);
      break;
  }
}

void update_digital_channels(void) {
  digital_channels = mcp23s17_read_all(digital_connections[2]);
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

  analog_connections[0] = init_ad7490(1, AD7490_0_CS_LATBITS, AD7490_0_CS_LATNUM);
  analog_connections[1] = init_ad7490(1, AD7490_1_CS_LATBITS, AD7490_1_CS_LATNUM);

  init_spi(1, 2, 8); // 2mhz clk, 24 bits
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

  thermocouple_connections[0] = init_max31855(3, MAX31855_0_CS_LATBITS, MAX31855_0_CS_LATNUM);
  thermocouple_connections[1] = init_max31855(3, MAX31855_1_CS_LATBITS, MAX31855_1_CS_LATNUM);
  thermocouple_connections[2] = init_max31855(3, MAX31855_2_CS_LATBITS, MAX31855_2_CS_LATNUM);
  thermocouple_connections[3] = init_max31855(3, MAX31855_3_CS_LATBITS, MAX31855_3_CS_LATNUM);
  thermocouple_connections[4] = init_max31855(3, MAX31855_4_CS_LATBITS, MAX31855_4_CS_LATNUM);
  thermocouple_connections[5] = init_max31855(3, MAX31855_5_CS_LATBITS, MAX31855_5_CS_LATNUM);
}

void init_gpio_ext(void){
  // Initialize all CS pins
  MCP23S17_0_CS_LAT = 1;
  MCP23S17_1_CS_LAT = 1;
  MCP23S17_2_CS_LAT = 1;
  MCP23S17_0_CS_TRIS = OUTPUT;
  MCP23S17_1_CS_TRIS = OUTPUT;
  MCP23S17_2_CS_TRIS = OUTPUT;

  // initialize all gpio chips
  digital_connections[0] = init_mcp23s17(5, MCP23S17_0_CS_LATBITS, MCP23S17_0_CS_LATNUM);
  digital_connections[1] = init_mcp23s17(5, MCP23S17_1_CS_LATBITS, MCP23S17_1_CS_LATNUM);
  digital_connections[2] = init_mcp23s17(5, MCP23S17_2_CS_LATBITS, MCP23S17_2_CS_LATNUM);

  mcp23s17_write_reg(MCP23S17_IODIR, 0xFFFF, digital_connections[2]); // digital inputs
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
