/**
 * FSAE Library AD7490
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_ad7490.h"

// Array holding sampled channel values
uint16_t channel_values[AD7490_NUM_CHN];

/**
 * void init_ad7490(void)
 *
 * Initializes the AD7490 ADC module.
 */
void init_ad7490(void) {
  memset(&channel_values, 0x0, AD7490_NUM_CHN * sizeof(uint16_t));

  // Initialize SPI communciations to the AD7490 chip
  _ad7490_init_spi();
}

/**
 * uint16_t** read_channels(void)
 *
 * Iterates through all channels, instructs the ADC to sample the current
 * value, and returns all the sample values.
 */
uint16_t* ad7490_read_channels(void) {
  AD7490ControlReg control = {.reg = AD7490_DEFAULT};
  control.WRITE = 1;
  control.ADDR = 0;

  // Set up chip for first read in sequence
  _ad7490_send_one(control.reg);

  // Sample each channel and save the response
  uint8_t i;
  for (i = 0; i < AD7490_NUM_CHN; i++) {
    control.ADDR = i + 1; // Set up ADDR for next read in sequence
    uint16_t resp = _ad7490_send_one(control.reg);
    channel_values[i] = resp & 0x0FFF; // Only bottom 12 bits are value
  }

  return channel_values;
}

/**
 * uint16_t _ad7490_send_one(uint16_t one)
 *
 * Sends one 16-bit SPI message to the AD7490 chip and returns the response.
 * This response will be clocked in at the same time as the message being sent.
 *
 * @param one- The halfword to send over SPI
 * @return Response clocked in concurrently with message
 */
uint16_t _ad7490_send_one(uint16_t one) {
  uint8_t resp = 0;

  CS_AD7490_LAT = 0;
  SPI5BUF = one;
  while (!SPI5STATbits.SPIRBF);
  resp = SPI5BUF;
  CS_AD7490_LAT = 1;

  return resp;
}

/**
 * void _ad7490_init_spi(void)
 *
 * Initializes the SPI5 communication bus for use with the AD7490 chip.
 */
void _ad7490_init_spi(void) {
  unlock_config();

  // Initialize SDI5/SDO5 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISFbits.TRISF4 = INPUT;
  SDI5Rbits.SDI5R = 0b0010; // RPF4
  TRISAbits.TRISA14 = OUTPUT;
  RPA14Rbits.RPA14R = 0b1001; // SDO5
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK5 and !CS_ad7490 pins
  TRISFbits.TRISF13 = OUTPUT; // SCK5
  CS_AD7490_TRIS = OUTPUT; // !CS_AD7490
  CS_AD7490_LAT = 1;

  // Disable interrupts
  IEC5bits.SPI5EIE = 0;
  IEC5bits.SPI5RXIE = 0;
  IEC5bits.SPI5TXIE = 0;

  // Disable SPI5 module
  SPI5CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI5BUF;

  // Use standard buffer mode
  SPI5CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI5BRG = 4;

  SPI5STATbits.SPIROV = 0;

  SPI5CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI5CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI5CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI5CONbits.MODE16 = 1;  // 32/16-Bit Communication Select bits (16-bit)
  SPI5CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI5CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI5CONbits.DISSDI = 0;
  SPI5CONbits.DISSDO = 0;
  SPI5CONbits.SMP = 1;
  SPI5CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI5 module
  SPI5CONbits.ON = 1;

  lock_config();
}
