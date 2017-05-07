#include "FSAE_max31855.h"

void init_max31855(void (*init_spi)(double mhz, int size)) {
  init_spi(3, 32);
}

double read_max31855_temp(uint32_t (*send_value)(uint32_t)) {
  uint32_t data = send_value(0);
  int16_t thermocouple_temp;
  if(data & 0x80000000) { // deal with the number properly if it's negative
    thermocouple_temp = ((data & 0xFFFC0000) >> 18) | 0xC000;
  }
  else {
    int16_t thermocouple_temp = (data & 0xFFFC0000) >> 18;
  }
  return thermocouple_temp/4.0;
}

max31855_data read_max31855_data(uint32_t (*send_value)(uint32_t)) {
  uint32_t data = send_value(0);
  int16_t thermocouple_temp;
  int16_t junction_temp;

  // parse thermocouple temp
  if(data & 0x80000000) { // deal with the number properly if it's negative
    thermocouple_temp = ((data & 0xFFFC0000) >> 18) | 0xC000;
  }
  else {
    thermocouple_temp = (data & 0xFFFC0000) >> 18;
  }

  // parse junction temp
  if(data & 0x8000) { // handle negative numbers
    junction_temp = ((data & 0xFFF0) >> 4) | 0xF000;
  }
  else {
    junction_temp = (data & 0xFFF0) >> 4;
  }

  max31855_data output_data;
  output_data.thermocoupleTemp = thermocouple_temp/4.0;
  output_data.junctionTemp = junction_temp/16.0;
  output_data.fault = data & 0x07; // isolate fault data

  return output_data;
}
