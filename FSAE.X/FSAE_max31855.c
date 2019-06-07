#include "FSAE_max31855.h"

SPIConn max31855Connections[10];
uint8_t max31855ConnIdx = 0;

SPIConn* init_max31855(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  if(!max31855ConnIdx) {
    init_spi(bus, 0.5, 32, 0);
  }
  SPIConn *currConn = &max31855Connections[max31855ConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  max31855ConnIdx++;

  return currConn;
}

double read_max31855_temp(SPIConn *conn) {
  uint32_t data = send_spi(0, conn);
  int16_t thermocouple_temp;
  if(data & 0x80000000) { // deal with the number properly if it's negative
    thermocouple_temp = ((data & 0xFFFC0000) >> 18) | 0xC000;
  }
  else {
    int16_t thermocouple_temp = (data & 0xFFFC0000) >> 18;
  }
  return thermocouple_temp/4.0;
}

max31855_data read_max31855_data(SPIConn *conn) {
  uint32_t data = send_spi(0, conn);
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
  output_data.thermocoupleTemp = thermocouple_temp;
  output_data.junctionTemp = junction_temp/16.0;
  output_data.fault = data & 0x07; // isolate fault data

  return output_data;
}
