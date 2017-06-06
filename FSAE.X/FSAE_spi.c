#include "FSAE_spi.h"

/*
 * Generic initializaiton function for all SPI busses
 * The SPI pins used for each bus are the defaults for the FSAE Library:
 *      |  SDI | SDO | SCK
 * SPI1 |  RB9 | RB10| RD1
 * SPI2 |  RE5 | RC1 | RG6
 * SPI3 |  RB5 | RB3 | RB14
 * SPI5 |  RF4 | RA14| RF13
 * SPI6 |  RF4 | RF8 | RD15
 *
 * size can have a value of 8, 16, or 32.  This determines the message
 * size in bits of the SPI communication.
 */
void init_spi(uint8_t bus, double mhz, uint8_t size, uint8_t mode) {
  unlock_config();

  uint32_t readVal;
  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * SPI1BRG = (F_PBCLK2/(2*F_SCK))-1
   */
  uint16_t brg = ceil((PBCLK2/(2.0*mhz))-1);
  if(brg > 8191) { // ensure brg fits in 13 bit register
    brg = 8191;
  }

  uint8_t mode16 = (size & 0x10) >> 4;
  uint8_t mode32 = (size & 0x20) >> 5;
  uint8_t ckp, cke, smp;
  switch(mode) {
    case 0:
      ckp = 0;
      smp = 0;
      cke = 1;
      break;
    case 1:
      ckp = 0;
      smp = 1;
      cke = 0;
      break;
    case 2:
      ckp = 1;
      smp = 0;
      cke = 1;
      break;
    case 3:
      ckp = 1;
      smp = 1;
      cke = 0;
      break;
    default:
      ckp = 0;
      smp = 0;
      cke = 1;
      break;
  }

  // Initialize correct bus
  switch(bus) {
    case 1:    
      // Initialize SDI1/SDO1 PPS pins
      CFGCONbits.IOLOCK = 0;
      TRISBbits.TRISB9 = INPUT;
      SDI1Rbits.SDI1R = 0b0101; // RPB9
      TRISBbits.TRISB10 = OUTPUT;
      RPB10Rbits.RPB10R = 0b0101; // SDO1
      CFGCONbits.IOLOCK = 1;

      TRISDbits.TRISD1 = OUTPUT; // Initialize SCK1
      // Disable interrupts
      IEC3bits.SPI1EIE = 0;
      IEC3bits.SPI1RXIE = 0;
      IEC3bits.SPI1TXIE = 0;

      SPI1CONbits.ON = 0; // Disable SPI1 module
      readVal = SPI1BUF; // Clear receive buffer
      SPI1CONbits.ENHBUF = 0; // Use standard buffer mode
      SPI1BRG = brg; // Set the baud rate
      SPI1STATbits.SPIROV = 0;
      SPI1CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
      SPI1CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
      SPI1CONbits.MODE32 = mode32;  // 32/16-Bit Communication Select bits (32-bit)
      SPI1CONbits.MODE16 = mode16;  // 32/16-Bit Communication Select bits (16-bit)
      SPI1CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
      SPI1CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
      SPI1CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
      SPI1CONbits.CKE = cke;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
      SPI1CONbits.SMP = smp;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
      SPI1CONbits.CKP = ckp;     // Clock Polarity Select (Idle state for clock is a high level)
      SPI1CONbits.ON = 1; // Enable SPI1 module
      break;

    case 2:    
      // Initialize SDI2/SDO2 PPS pins
      CFGCONbits.IOLOCK = 0;
      TRISEbits.TRISE5 = INPUT;
      SDI2Rbits.SDI2R = 0b0110; // RPE5
      TRISCbits.TRISC1 = OUTPUT;
      RPC1Rbits.RPC1R = 0b0110; // SDO2
      CFGCONbits.IOLOCK = 1;

      TRISGbits.TRISG6 = OUTPUT; // Initialize SCK2
      // Disable interrupts
      IEC4bits.SPI2EIE = 0;
      IEC4bits.SPI2RXIE = 0;
      IEC4bits.SPI2TXIE = 0;

      SPI2CONbits.ON = 0; // Disable SPI2 module
      readVal = SPI2BUF; // Clear receive buffer
      SPI2CONbits.ENHBUF = 0; // Use standard buffer mode
      SPI2BRG = brg; // Set the baud rate
      SPI2STATbits.SPIROV = 0;
      SPI2CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
      SPI2CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
      SPI2CONbits.MODE32 = mode32;  // 32/16-Bit Communication Select bits (32-bit)
      SPI2CONbits.MODE16 = mode16;  // 32/16-Bit Communication Select bits (16-bit)
      SPI2CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
      SPI2CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
      SPI2CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
      SPI2CONbits.CKE = cke;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
      SPI2CONbits.SMP = smp;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
      SPI2CONbits.CKP = ckp;     // Clock Polarity Select (Idle state for clock is a high level)
      SPI2CONbits.ON = 1; // Enable SPI2 module
      break;

    case 3:    
      // Initialize SDI3/SDO3 PPS pins
      CFGCONbits.IOLOCK = 0;
      TRISBbits.TRISB5 = INPUT;
      SDI3Rbits.SDI3R = 0b1000; // RPB5
      TRISBbits.TRISB3 = OUTPUT;
      RPB3Rbits.RPB3R = 0b0111; // SDO3
      CFGCONbits.IOLOCK = 1;

      TRISBbits.TRISB14 = OUTPUT; // Initialize SCK3
      // Disable interrupts
      IEC4bits.SPI3EIE = 0;
      IEC4bits.SPI3RXIE = 0;
      IEC4bits.SPI3TXIE = 0;

      SPI3CONbits.ON = 0; // Disable SPI3 module
      readVal = SPI3BUF; // Clear receive buffer
      SPI3CONbits.ENHBUF = 0; // Use standard buffer mode
      SPI3BRG = brg; // Set the baud rate
      SPI3STATbits.SPIROV = 0;
      SPI3CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
      SPI3CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
      SPI3CONbits.MODE32 = mode32;  // 32/16-Bit Communication Select bits (32-bit)
      SPI3CONbits.MODE16 = mode16;  // 32/16-Bit Communication Select bits (16-bit)
      SPI3CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
      SPI3CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
      SPI3CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
      SPI3CONbits.CKE = cke;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
      SPI3CONbits.SMP = smp;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
      SPI3CONbits.CKP = ckp;     // Clock Polarity Select (Idle state for clock is a high level)
      SPI3CONbits.ON = 1; // Enable SPI3 module
      break;

    case 4:    
      // Not used currently
      break;
    case 5:    
      // Initialize SDI5/SDO5 PPS pins
      CFGCONbits.IOLOCK = 0;
      TRISFbits.TRISF4 = INPUT;
      SDI5Rbits.SDI5R = 0b0010; // RPF4
      TRISAbits.TRISA14 = OUTPUT;
      RPA14Rbits.RPA14R = 0b1001; // SDO5
      CFGCONbits.IOLOCK = 1;

      TRISFbits.TRISF13 = OUTPUT; // SCK5
      // Disable interrupts
      IEC5bits.SPI5EIE = 0;
      IEC5bits.SPI5RXIE = 0;
      IEC5bits.SPI5TXIE = 0;

      SPI5CONbits.ON = 0; // Disable SPI5 module
      readVal = SPI5BUF; // Clear receive buffer
      SPI5CONbits.ENHBUF = 0; // Use standard buffer mode
      SPI5BRG = brg; // Set the baud rate
      SPI5STATbits.SPIROV = 0;
      SPI5CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
      SPI5CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
      SPI5CONbits.MODE32 = mode32;  // 32/16-Bit Communication Select bits (32-bit)
      SPI5CONbits.MODE16 = mode16;  // 32/16-Bit Communication Select bits (16-bit)
      SPI5CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
      SPI5CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
      SPI5CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
      SPI5CONbits.CKE = cke;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
      SPI5CONbits.SMP = smp;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
      SPI5CONbits.CKP = ckp;     // Clock Polarity Select (Idle state for clock is a high level)
      SPI5CONbits.ON = 1; // Enable SPI5 module
      break;

    case 6:    
      // Initialize SDI6/SDO6 PPS pins
      CFGCONbits.IOLOCK = 0;
      TRISFbits.TRISF2 = INPUT;
      SDI6Rbits.SDI6R = 0b1011; // RPF4
      TRISFbits.TRISF8 = OUTPUT;
      RPF8Rbits.RPF8R = 0b1010; // SDO6
      CFGCONbits.IOLOCK = 1;

      TRISDbits.TRISD15 = OUTPUT; // Initialize SCK6
      // Disable interrupts
      IEC5bits.SPI6IE = 0;
      IEC5bits.SPI6RXIE = 0;
      IEC5bits.SPI6TXIE = 0;

      SPI6CONbits.ON = 0; // Disable SPI6 module
      readVal = SPI6BUF; // Clear receive buffer
      SPI6CONbits.ENHBUF = 0; // Use standard buffer mode
      SPI6BRG = brg; // Set the baud rate
      SPI6STATbits.SPIROV = 0;
      SPI6CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
      SPI6CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
      SPI6CONbits.MODE32 = mode32;  // 32/16-Bit Communication Select bits (32-bit)
      SPI6CONbits.MODE16 = mode16;  // 32/16-Bit Communication Select bits (16-bit)
      SPI6CONbits.DISSDI = 0; // receiver to ignore the unused bit slots
      SPI6CONbits.DISSDO = 0; // transmit unused bit slots with logic level 0
      SPI6CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
      SPI6CONbits.CKE = cke;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
      SPI6CONbits.SMP = smp;     // SPI Data Input Sample Phase (Input data sampled at middle of output time)
      SPI6CONbits.CKP = ckp;     // Clock Polarity Select (Idle state for clock is a high level)
      SPI6CONbits.ON = 1; // Enable SPI6 module
      break;
  }

  lock_config();
}

/*
 *Returns a function pointer to a generic send_spi function
 *based on a bus number.  Returns NULL if invalid bus.
 *Function pointer is of type send_spi_fp
 */
send_spi_fp get_send_spi(uint8_t bus){
  switch(bus) {
    case 1:
      return send_spi1;
    case 2:
      return send_spi2;
    case 3:
      return send_spi3;
    case 5:
      return send_spi5;
    case 6:
      return send_spi6;
    default:
      return NULL;
  }
}

uint32_t send_spi1(uint32_t value){
  SPI1BUF = value;
  while (!SPI1STATbits.SPIRBF);
  return SPI1BUF;
}

uint32_t send_spi2(uint32_t value){
  SPI2BUF = value;
  while (!SPI2STATbits.SPIRBF);
  return SPI2BUF;
}

uint32_t send_spi3(uint32_t value){
  SPI3BUF = value;
  while (!SPI3STATbits.SPIRBF);
  return SPI3BUF;
}

uint32_t send_spi5(uint32_t value){
  SPI5BUF = value;
  while (!SPI5STATbits.SPIRBF);
  return SPI5BUF;
}

uint32_t send_spi6(uint32_t value){
  SPI6BUF = value;
  while (!SPI6STATbits.SPIRBF);
  return SPI6BUF;
}

/*
 *Generic send_spi function, which takes a value and a connection pointer.
 *Will set CS low, send the value using the connection's send_spi
 *function pointer, return the buffer, and set CS back to high.
 */
uint32_t send_spi(uint32_t value, SPIConn *conn){
  *(conn->cs_lat) &= ~(1 << (conn->cs_num)); // Set CS Low
  uint32_t buff = conn->send_fp(value);
  *(conn->cs_lat) |= 1 << conn->cs_num; // Set CS High
  return buff;
}
