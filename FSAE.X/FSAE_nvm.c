/**
 * FSAE Library NVM
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_nvm.h"

/**
 * void nvm_write_enable(void)
 *
 * Sends a message to the NVM chip to set the Write Enable Latch
 */
void nvm_write_enable(void) {
  _nvm_send_one(IN_WREN);
}

/**
 * void nvm_write_status_reg(NvmStatusReg status)
 *
 * Sends a message to the NVM chip with new values for fields in the
 * status register.
 *
 * @param status- Struct used to set the NVM chip's status register
 */
void nvm_write_status_reg(NvmStatusReg status) {
  _nvm_send_two(IN_WRSR, status.reg);
}

/**
 * NvmStatusReg nvm_read_status_reg(void)
 *
 * Reads the contents of the status register.
 *
 * @return The current status register
 */
NvmStatusReg nvm_read_status_reg(void) {
  NvmStatusReg status = { .reg = _nvm_send_two(IN_RDSR, 0x00) };
  return status;
}

/**
 * void init_nvm(void)
 *
 * Initializes the 25LC1024 NVM module.
 */
void init_nvm(void) {
  // Initialize SPI communciations to the NVM chip
  _nvm_init_spi();

  //TODO: Set up registers, etc.
}

/**
 * uint8_t _nvm_send_one(uint8_t one)
 *
 * Sends one 8-bit SPI message to the NVM chip and returns the
 * response. This response will be clocked in at the same time
 * as the message being sent. If a response is sent after the
 * instruction is clocked in, the two-byte SPI function must
 * be used.
 *
 * @param one- The byte to send over SPI
 * @return Response clocked in concurrently with message
 */
uint8_t _nvm_send_one(uint8_t one) {
  uint8_t resp = 0;

  CS_NVM_LAT = 0;
  SPI6BUF = one;
  while (!SPI6STATbits.SPIRBF);
  resp = SPI6BUF;
  CS_NVM_LAT = 1;

  return resp;
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
uint8_t _nvm_send_two(uint8_t one, uint8_t two) {
  uint8_t resp = 0;

  CS_NVM_LAT = 0;

  SPI6BUF = one;
  while (!SPI6STATbits.SPIRBF);
  resp = SPI6BUF;

  SPI6BUF = two;
  while (!SPI6STATbits.SPIRBF);
  resp = SPI6BUF;

  CS_NVM_LAT = 1;

  return resp;
}

/**
 * void _nvm_init_spi(void)
 *
 * Initializes the SPI6 communication bus for use with the NVM chip.
 */
void _nvm_init_spi(void) {
  unlock_config();

  // Initialize SDI6/SDO6 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISFbits.TRISF2 = INPUT;
  SDI6Rbits.SDI6R = 0b1011; // RPF2
  TRISFbits.TRISF8 = OUTPUT;
  RPF8Rbits.RPF8R = 0b1010; // SDO6
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK6 and !CS_NVM pins
  TRISDbits.TRISD15 = OUTPUT; // SCK6
  CS_NVM_TRIS = OUTPUT; // !CS_NVM
  CS_NVM_LAT = 1;

  // Disable interrupts
  IEC5bits.SPI6IE = 0;
  IEC5bits.SPI6RXIE = 0;
  IEC5bits.SPI6TXIE = 0;

  // Disable SPI6 module
  SPI6CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI6BUF;

  // Use standard buffer mode
  SPI6CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI6BRG = 4;

  SPI6STATbits.SPIROV = 0;

  SPI6CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI6CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI6CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI6CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI6CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI6CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI6CONbits.DISSDI = 0;
  SPI6CONbits.DISSDO = 0;
  SPI6CONbits.SMP = 1;
  SPI6CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI6 module
  SPI6CONbits.ON = 1;

  lock_config();
}