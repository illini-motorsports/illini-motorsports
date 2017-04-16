#include "FSAE_spi.h"

void init_spi1(int mhz, int size) {
  unlock_config();

  // Initialize SDI1/SDO1 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB9 = INPUT;
  SDI1Rbits.SDI1R = 0b0101; // RPB9
  TRISBbits.TRISB10 = OUTPUT;
  RPB10Rbits.RPB10R = 0b0101; // SDO1
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK5 and !CS_ad7490 pins
  TRISDbits.TRISD1 = OUTPUT; // SCK5

  // Disable interrupts
  IEC3bits.SPI1EIE = 0;
  IEC3bits.SPI1RXIE = 0;
  IEC3bits.SPI1TXIE = 0;

  // Disable SPI1 module
  SPI1CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI1BUF;

  // Use standard buffer mode
  SPI1CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
	 * SPI1BRG = (F_PBCLK2/(2*F_SCK))-1
   */

  // Set the baud rate (see above equation, ceil will round to lower freq)
  SPI1BRG = ceil((PBCLK2/(2.0*mhz))-1);

  SPI1STATbits.SPIROV = 0;

  SPI1CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI1CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI1CONbits.MODE32 = size & 0x20;  // 32/16-Bit Communication Select bits (8-bit)
  SPI1CONbits.MODE16 = size & 0x10;  // 32/16-Bit Communication Select bits (16-bit)
  SPI1CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
	SPI1CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
  SPI1CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI1CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI1CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI1CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI1 module
  SPI1CONbits.ON = 1;

  lock_config();
}

void init_spi5(int mhz, int size) {
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
	 * SPI1BRG = (F_PBCLK2/(2*F_SCK))-1
   */

  // Set the baud rate (see above equation, ceil will round to lower freq)
  SPI5BRG = ceil((PBCLK2/(2.0*mhz))-1);

  SPI5STATbits.SPIROV = 0;

  SPI5CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI5CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI5CONbits.MODE32 = size & 0x20;  // 32/16-Bit Communication Select bits (8-bit)
  SPI5CONbits.MODE16 = size & 0x10;  // 32/16-Bit Communication Select bits (16-bit)
  SPI5CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
	SPI5CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
  SPI5CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI5CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI5CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI5CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI5 module
  SPI5CONbits.ON = 1;

  lock_config();
}
