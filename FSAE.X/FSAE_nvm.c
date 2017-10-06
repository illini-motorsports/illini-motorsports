/**
 * FSAE Library NVM
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#include "FSAE_nvm.h"

SPIConn nvmConnections[4];
uint8_t nvmConnIdx = 0;

//=============================== INTERFACE ====================================

/**
 * Initializes the 25LC1024 NVM module for the standard FSAE template.
 */
SPIConn* init_nvm_std() {
  init_nvm(_NVM_STD_BUS, _NVM_STD_CS_LATBITS, _NVM_STD_CS_LATNUM);
}

/**
 * Initializes the 25LC1024 NVM module with user-defined settings.
 */
SPIConn* init_nvm(uint8_t bus, uint32_t* cs_lat, uint8_t cs_num) {
  // Initialize SPI communciations to the NVM chip
  if(!nvmConnIdx) {
    init_spi(bus, 10.0, 8, SPI_MODE0);
  }

  SPIConn* currConn = &nvmConnections[nvmConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  nvmConnIdx++;

  return currConn;
}

//============================= IMPLEMENTATION =================================

/**
 * Sends a message to the NVM chip to set the Write Enable Latch
 */
void _nvm_write_enable(SPIConn* conn) {
  send_spi(_NVM_IN_WREN, conn);
}

/**
 * Sends a message to the NVM chip with new values for fields in the
 * status register.
 */
void _nvm_write_status_reg(_NvmStatusReg status, SPIConn* conn) {
  _nvm_send_two(_NVM_IN_WRSR, status.reg, conn);
}

/**
 * Reads the contents of the status register.
 */
_NvmStatusReg _nvm_read_status_reg(SPIConn* conn) {
  _NvmStatusReg status = { .reg = _nvm_send_two(_NVM_IN_RDSR, 0x00, conn) };
  return status;
}

/**
 * Reads data starting from the 24 bit address into the provided byte array.
 * It will read for numBytes bytes, using the provided conn struct.
 */
void _nvm_read_data(uint32_t address, uint8_t *bytes, uint8_t numBytes, SPIConn* conn){
  uint8_t i;
  spi_select(conn);
  conn->send_fp(_NVM_IN_READ);
  conn->send_fp((address >> 16) & 0xFF);
  conn->send_fp((address >> 8) & 0xFF);
  conn->send_fp(address & 0xFF);
  for(i = 0;i < numBytes; i++) {
    bytes[i] = conn->send_fp(0x00);
  }
  spi_deselect(conn);
}

/**
 * Sends two bytes via SPI and returns the response from the NVM
 * chip. The response is the 8 bits clocked in during the transmission
 * of the second byte.
 */
uint8_t _nvm_send_two(uint8_t one, uint8_t two, SPIConn* conn) {
  uint8_t resp = 0;
  spi_select(conn);
  conn->send_fp(one);
  resp = (uint8_t) conn->send_fp(two);
  spi_deselect(conn);
  return resp;
}
