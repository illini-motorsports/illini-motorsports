/**
 * FSAE Library NVM
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_nvm.h"

SPIConn nvmConnections[10];
uint8_t nvmConnIdx = 0;

/**
 * void nvm_write_enable(void)
 *
 * Sends a message to the NVM chip to set the Write Enable Latch
 */
void nvm_write_enable(SPIConn *conn) {
  spi_send(IN_WREN, conn);
}

/**
 * void nvm_write_status_reg(NvmStatusReg status)
 *
 * Sends a message to the NVM chip with new values for fields in the
 * status register.
 *
 * @param status- Struct used to set the NVM chip's status register
 */
void nvm_write_status_reg(NvmStatusReg status, SPIConn *conn) {
  _nvm_send_two(IN_WRSR, status.reg, conn);
}

/**
 * NvmStatusReg nvm_read_status_reg(void)
 *
 * Reads the contents of the status register.
 *
 * @return The current status register
 */
NvmStatusReg nvm_read_status_reg(SPIConn *conn) {
  NvmStatusReg status = { .reg = _nvm_send_two(IN_RDSR, 0x00, conn) };
  return status;
}

/**
 * void nvm_read_data(uint32_t address, uint8_t *bytes, uint8_t numBytes, SPIConn *conn)
 *
 * Reads data starting from the 24 bit address into the provided byte array.
 * It will read for numBytes bytes, using the provided conn struct.
 */
void nvm_read_data(uint32_t address, uint8_t *bytes, uint8_t numBytes, SPIConn *conn){
  uint8_t i;
  *(conn->cs_lat) &= ~(1 << (conn->cs_num)); // Set CS Low

  conn->send_fp(IN_READ);
  conn->send_fp((address >> 16) & 0xFF);
  conn->send_fp((address >> 8) & 0xFF);
  conn->send_fp(address & 0xFF);
  for(i = 0;i < numBytes; i++) {
    bytes[i] = conn->send_fp(0x00);
  }

  *(conn->cs_lat) |= 1 << conn->cs_num; // Set CS High
}

/**
 *
 * Initializes the 25LC1024 NVM module.
 */
SPIConn* init_nvm(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {
  // Initialize SPI communciations to the NVM chip
  if(!nvmConnIdx) {
    init_spi(bus, 7, 8, 0); // init to 7 mhz
  }

  SPIConn * currConn = &nvmConnections[nvmConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  nvmConnIdx++;

  return currConn;
}

/**
 * uint8_t _nvm_send_two(uint8_t one, uint8_t two)
 *
 * Sends two bytes via SPI and returns the response from the NVM
 * chip. The response is the 8 bits clocked in during the transmission
 * of the second byte.
 *
 * @param one- The first byte to send over SPI
 * @param two- The second byte to send over SPI
 * @return The response clocked in during the second transmission
 */
uint8_t _nvm_send_two(uint8_t one, uint8_t two, SPIConn *conn) {
  uint8_t resp = 0;

  *(conn->cs_lat) &= ~(1 << (conn->cs_num)); // Set CS Low

  conn->send_fp(one);
  resp = (uint8_t) conn->send_fp(two);

  *(conn->cs_lat) |= 1 << conn->cs_num; // Set CS High

  return resp;
}
