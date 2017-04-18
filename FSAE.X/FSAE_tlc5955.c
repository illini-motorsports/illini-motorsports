/**
 * FSAE Library TLC5955
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_tlc5955.h"

uint8_t pending_register[NUM_BYTES] = {0};
uint16_t bit_ctr = 0;

/**
 * Initializes the TLC5955 module.
 */
void init_tlc5955(void) {
  // GSCLK PWM Signal
  PWM_TLC5955_TRIS = OUTPUT;
  PWM_TLC5955_LAT = 1;
  
  // Latch - SHFLAT
  LAT_TLC5955_TRIS = OUTPUT;
  LAT_TLC5955_LAT = 0;

  _tlc5955_init_spi();

  _tlc5955_write_control();
  _tlc5955_write_gs();
}

/**
 * Initializes the SPI2 communication bus for use with the TLC5955 chip.
 */
void _tlc5955_init_spi(void) {
  unlock_config();

  // Initialize SDO2 PPS pin
  CFGCONbits.IOLOCK = 0;
  TRISGbits.TRISG7 = OUTPUT;
  RPG7Rbits.RPG7R = 0b0110; // SDO2
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK2
  TRISGbits.TRISG6 = OUTPUT; // SCK2

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
   * F_SCK = F_PBCLK2 / (2 * (SPI2BRG + 1))
   * F_SCK = 100Mhz / (2 * (49 + 1))
   * F_SCK = 1Mhz
   */

  // Set the baud rate (see above equation)
  SPI2BRG = 49;

  SPI2STATbits.SPIROV = 0;

  SPI2CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI2CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI2CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.DISSDI = 0;
  SPI2CONbits.DISSDO = 0;
  SPI2CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI2CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI2CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI2CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI2 module
  SPI2CONbits.ON = 1;

  lock_config();
}

/**
 * Writes default values to the TLC5955 control registers.
 */
void _tlc5955_write_control(void) {
  uint8_t i, j;

  _tlc5955_reg_append(1, 0b1); // MSB indicates control register write
  _tlc5955_reg_append(8, 0x96); // More indication for control register write

  // Write DC registers
  for (i = 0; i < 16; i++) {
    for (j = 0; j < 3; j++) {
      _tlc5955_reg_append(7, 0b1111111);
    }
  }

  // Write MC registers
  _tlc5955_reg_append(3, 0b100); // Red
  _tlc5955_reg_append(3, 0b100); // Green
  _tlc5955_reg_append(3, 0b100); // Blue

  // Write BC registers
  _tlc5955_reg_append(7, 0b1111111); // Red
  _tlc5955_reg_append(7, 0b1111111); // Green
  _tlc5955_reg_append(7, 0b1111111); // Blue

  // Write FC registers
  _tlc5955_reg_append(1, 0b1); // DSPRPT
  _tlc5955_reg_append(1, 0b0); // TMGRST
  _tlc5955_reg_append(1, 0b0); // RFRESH
  _tlc5955_reg_append(1, 0b1); // ESPWM
  _tlc5955_reg_append(1, 0b0); // LSDVLT

  _tlc5955_send_register();
}

/**
 * Writes default values to the TLC5955 GS (grayscale) registers.
 */
void _tlc5955_write_gs(void) {
  uint8_t i, j;

  _tlc5955_reg_append(1, 0b0); // MSB indicates GS write

  for (i = 0; i < 16; i++) {
    for (j = 0; j < 3; j++) {
      _tlc5955_reg_append(8, 0xFF); // GS HW
      _tlc5955_reg_append(8, 0xFF); // GW LW
    }
  }

  _tlc5955_send_register();
}

/**
 * Appends <num_bits> of <data> to the pending_register array in preparation
 * for sending a register to the chip.
 */
void _tlc5955_reg_append(uint8_t num_bits, uint8_t data) {
  uint8_t i;
  for (i = 0; i < num_bits; i++) {
    uint8_t bit = data & (1 << i);
    uint8_t idx = ((bit_ctr + i) / 8);
    uint8_t pos = ((bit_ctr + i) % 8);

    if (bit) {
      pending_register[idx] |= (1 << pos);
    } else {
      pending_register[idx] &= ~(1 << pos);
    }
  }

  bit_ctr += num_bits;
}

/**
 * Sends the pending_register array over the SPI bus as a continuous steam of
 * <NUM_BYTES> messages.
 */
void _tlc5955_send_register(void) {
  uint8_t i;
  uint8_t resp;

  // Send latch pulse
  PWM_TLC5955_LAT = 1;
  for (i = 0; i < 6; i++);
  PWM_TLC5955_LAT = 0;

  for (i = 0; i < NUM_BYTES; i++) {
    SPI2BUF = pending_register[i];
    while( !SPI2STATbits.SPIRBF);
    resp = SPI2BUF;
  }
}