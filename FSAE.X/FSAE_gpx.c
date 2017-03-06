/**
 * FSAE Library GPX
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_gpx.h"

/**
 * void init_gpx(void)
 *
 * Initializes the MCP23S17 GPIO Expander module.
 */
void init_gpx(void) {
  // Initialize SPI communciations to the GPX chip
  _gpx_init_spi();

  // Configure IOCON register
  uint8_t IOCON = 0x0;
  IOCON |= (0 << 7); // BANK (Same bank)
  IOCON |= (1 << 6); // MIRROR (INT pins connected)
  IOCON |= (1 << 5); // SEQOP (Sequential operation disabled)
  IOCON |= (0 << 3); // HAEN (Hardware address pins disabled)
  IOCON |= (0 << 1); // INTPOL (Interrupt pins active-low)
  _gpx_send_mesg(GPX_OPCODE_WRITE, GPX_IOCON, IOCON);

  // Enable interrupts for pins in use
  _gpx_send_mesg(GPX_OPCODE_WRITE, GPX_GPINTENA, 0xF0);
  _gpx_send_mesg(GPX_OPCODE_WRITE, GPX_GPINTENB, 0xFF);

  // Enable interrupt for GPX_INT pin
  _gpx_init_int();
}

/**
 * uint16_t gps_read_state(void)
 *
 * Gets the state of all inputs from the GPX module.
 *
 * @returns A bitmask of the two input port registers
 */
uint16_t gpx_read_state(void) {
  uint16_t port_a = _gpx_send_mesg(GPX_OPCODE_READ, GPX_GPIOA, 0x00);
  uint16_t port_b = _gpx_send_mesg(GPX_OPCODE_READ, GPX_GPIOB, 0x00);
  return (port_a << 8) | (port_b & 0xFF);
}

/**
 * uint8_t _gpx_send_mesg(uint8_t opcode, uint8_t addr, uint8_t data)
 *
 * Sends a message to the GPX module and returns the response.
 *
 * @param opcode- Identify the message as a write or a read
 * @param addr- The register address to write/read
 * @param data- If writing, the data to write to the register
 * @returns The data returned by the GPX module
 */
uint8_t _gpx_send_mesg(uint8_t opcode, uint8_t addr, uint8_t data) {
  uint8_t resp = 0;

  CS_GPX_LAT = 0;

  SPI2BUF = opcode;
  while (!SPI2STATbits.SPIRBF);
  resp = SPI2BUF;

  SPI2BUF = addr;
  while (!SPI2STATbits.SPIRBF);
  resp = SPI2BUF;

  SPI2BUF = data;
  while (!SPI2STATbits.SPIRBF);
  resp = SPI2BUF;

  CS_GPX_LAT = 1;

  return resp;
}

/**
 * void _gpx_init_spi(void)
 *
 * Initializes the SPI2 communication bus for use with the GPX chip.
 */
void _gpx_init_spi(void) {
  unlock_config();

  // Initialize SDI2/SDO2 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISEbits.TRISE5 = INPUT;
  SDI2Rbits.SDI2R = 0b0110; // RPE5
  TRISCbits.TRISC1 = OUTPUT;
  RPC1Rbits.RPC1R = 0b0110; // SDO2
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK2 and !CS_GPX pins
  TRISGbits.TRISG6 = OUTPUT; // SCK2
  CS_GPX_TRIS = OUTPUT; // !CS_AD7490
  CS_GPX_LAT = 1;

  // Disable interrupts
  IEC4bits.SPI2EIE = 0;
  IEC4bits.SPI2RXIE = 0;
  IEC4bits.SPI2TXIE = 0;

  // Disable SPI2 module
  SPI2CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI2BUF;

  // Use standard buffer mode
  SPI2CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI2BRG = 4;

  SPI2STATbits.SPIROV = 0;

  SPI2CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI2CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI2CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI2CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI2CONbits.DISSDI = 0;
  SPI2CONbits.DISSDO = 0;
  SPI2CONbits.SMP = 1;
  SPI2CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI2 module
  SPI2CONbits.ON = 1;

  lock_config();
}

/**
 * void _gpx_init_int(void)
 *
 * Initializes the GPX change notification interrupt
 */
void _gpx_init_int(void) {
  CLI();

  // Configure GPX_INT as a digital input
  GPX_INT_TRIS = INPUT;
  GPX_INT_ANSEL = DIG_INPUT;

  // CONCONE
  CNCONEbits.ON = 1;         // Change Notice (CN) Control ON (CN is enabled)
  CNCONEbits.SIDL = 0;       // Stop in Idle Control (CPU Idle does not affect CN operation)
  CNCONEbits.EDGEDETECT = 1; // Change Notification Style (Edge Style)

  // Enable interrupts for the RE6 pin
  CNENEbits.CNIEE6 = 1;

  // Clear mismatch condition
  uint32_t dummy = PORTE;

  // Configure change notification interrupt
  IFS3CLR = _IFS3_CNEIF_MASK; // CNE Interrupt Flag Status (No interrupt request has occurred)
  IPC30bits.CNEIP = 3;        // CNE Interrupt Priority (Interrupt priority is 3)
  IPC30bits.CNEIS = 3;        // CNE Interrupt Subpriority (Interrupt subpriority is 3)
  IEC3SET = _IEC3_CNAIE_MASK; // CNE Interrupt Enable Control (Interrupt is enabled)

  STI();
}
