#include "FSAE_rheo.h"

SPIConn rheoConnections[10];
uint8_t rheoConnIdx = 0;

SPIConn* init_rheo(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  if(!rheoConnIdx) {
    init_spi(bus, 10, 16);
  }
  SPIConn *currConn = &rheoConnections[rheoConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  rheoConnIdx++;

  return currConn;
}

void set_rheo(uint8_t val, SPIConn *conn) {
  send_spi(val, conn);
}

void send_all_rheo(uint16_t msg) {
  int i;
  for(i=0;i<rheoConnIdx;i++) {
    send_spi(msg, &rheoConnections[i]);
  }
}
