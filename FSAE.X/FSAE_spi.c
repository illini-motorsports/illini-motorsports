#include "FSAE_spi.h"

void init_spi1(double mhz, int size) {
  unlock_config();

  // Initialize SDI1/SDO1 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB9 = INPUT;
  SDI1Rbits.SDI1R = 0b0101; // RPB9
  TRISBbits.TRISB10 = OUTPUT;
  RPB10Rbits.RPB10R = 0b0101; // SDO1
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK1
  TRISDbits.TRISD1 = OUTPUT;

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

void init_spi2(double mhz, int size) {
  unlock_config();

  // Initialize SDI2/SDO2 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISEbits.TRISE5 = INPUT;
  SDI2Rbits.SDI2R = 0b0110; // RPE5
  TRISCbits.TRISC1 = OUTPUT;
  RPC1Rbits.RPC1R = 0b0110; // SDO2
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK2
  TRISGbits.TRISG6 = OUTPUT;

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
   * SPI2BRG = (F_PBCLK2/(2*F_SCK))-1
   */

  // Set the baud rate (see above equation, ceil will round to lower freq)
  SPI2BRG = ceil((PBCLK2/(2.0*mhz))-1);

  SPI2STATbits.SPIROV = 0;

  SPI2CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI2CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI2CONbits.MODE32 = size & 0x20;  // 32/16-Bit Communication Select bits (8-bit)
  SPI2CONbits.MODE16 = size & 0x10;  // 32/16-Bit Communication Select bits (16-bit)
  SPI2CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
  SPI2CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
  SPI2CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI2CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI2CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI2CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI2 module
  SPI2CONbits.ON = 1;

  lock_config();
}

void init_spi3(double mhz, int size) {
  unlock_config();

  // Initialize SDI3/SDO3 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB5 = INPUT;
  SDI3Rbits.SDI3R = 0b1000; // RPB5
  TRISBbits.TRISB3 = OUTPUT;
  RPB3Rbits.RPB3R = 0b0111; // SDO3
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK3
  TRISBbits.TRISB14 = OUTPUT;

  // Disable interrupts
  IEC4bits.SPI3EIE = 0;
  IEC4bits.SPI3RXIE = 0;
  IEC4bits.SPI3TXIE = 0;

  // Disable SPI3 module
  SPI3CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI3BUF;

  // Use standard buffer mode
  SPI3CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI3BRG + 1))
   * SPI3BRG = (F_PBCLK2/(2*F_SCK))-1
   */

  // Set the baud rate (see above equation, ceil will round to lower freq)
  SPI3BRG = ceil((PBCLK2/(2.0*mhz))-1);

  SPI3STATbits.SPIROV = 0;

  SPI3CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI3CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI3CONbits.MODE32 = size & 0x20;  // 32/16-Bit Communication Select bits (8-bit)
  SPI3CONbits.MODE16 = size & 0x10;  // 32/16-Bit Communication Select bits (16-bit)
  SPI3CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
  SPI3CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
  SPI3CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI3CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI3CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI3CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI3 module
  SPI3CONbits.ON = 1;

  lock_config();
}

void init_spi5(double mhz, int size) {
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

void init_spi6(double mhz, int size) {
  unlock_config();

  // Initialize SDI6/SDO6 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISFbits.TRISF2 = INPUT;
  SDI6Rbits.SDI6R = 0b1011; // RPF4
  TRISFbits.TRISF8 = OUTPUT;
  RPF8Rbits.RPF8R = 0b1010; // SDO6
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK6
  TRISDbits.TRISD15 = OUTPUT;

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
   * SPI1BRG = (F_PBCLK2/(2*F_SCK))-1
   */

  // Set the baud rate (see above equation, ceil will round to lower freq)
  SPI6BRG = ceil((PBCLK2/(2.0*mhz))-1);

  SPI6STATbits.SPIROV = 0;

  SPI6CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI6CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI6CONbits.MODE32 = size & 0x20;  // 32/16-Bit Communication Select bits (8-bit)
  SPI6CONbits.MODE16 = size & 0x10;  // 32/16-Bit Communication Select bits (16-bit)
  SPI6CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
  SPI6CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
  SPI6CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI6CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI6CONbits.SMP = 0;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
  SPI6CONbits.CKP = 1;     // Clock Polarity Select (Idle state for clock is a high level)

  // Enable SPI6 module
  SPI6CONbits.ON = 1;

  lock_config();
}

uint32_t send_spi1(uint32_t value, uint32_t *cs_lat, int cs_num){
  uint32_t resp = 0;

  *cs_lat &= ~(1 << cs_num); // Set CS Low
  SPI1BUF = value;
  while (!SPI1STATbits.SPIRBF);
  resp = SPI1BUF;
  *cs_lat |= 1 << cs_num;

  return resp;
}

uint32_t send_spi2(uint32_t value, uint32_t *cs_lat, int cs_num){
  uint32_t resp = 0;

  *cs_lat &= ~(1 << cs_num); // Set CS Low
  SPI2BUF = value;
  while (!SPI2STATbits.SPIRBF);
  resp = SPI2BUF;
  *cs_lat |= 1 << cs_num;

  return resp;
}

uint32_t send_spi3(uint32_t value, uint32_t *cs_lat, int cs_num){
  uint32_t resp = 0;

  *cs_lat &= ~(1 << cs_num); // Set CS Low
  SPI3BUF = value;
  while (!SPI3STATbits.SPIRBF);
  resp = SPI3BUF;
  *cs_lat |= 1 << cs_num;

  return resp;
}

uint32_t send_spi5(uint32_t value, uint32_t *cs_lat, int cs_num){
  uint32_t resp = 0;

  *cs_lat &= ~(1 << cs_num); // Set CS Low
  SPI5BUF = value;
  while (!SPI5STATbits.SPIRBF);
  resp = SPI5BUF;
  *cs_lat |= 1 << cs_num;

  return resp;
}

uint32_t send_spi6(uint32_t value, uint32_t *cs_lat, int cs_num){
  uint32_t resp = 0;

  *cs_lat &= ~(1 << cs_num); // Set CS Low
  SPI6BUF = value;
  while (!SPI6STATbits.SPIRBF);
  resp = SPI6BUF;
  *cs_lat |= 1 << cs_num;

  return resp;
}
