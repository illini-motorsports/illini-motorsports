/**
 * FSAE Library LTC3350
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_ltc3350.h"

/**
 */
void init_ltc3350(void) {
  // Initialize pins
  CHG_PFO_TRIS = INPUT;
  CHG_PFO_ANSEL = DIG_INPUT;
  CHG_ALM_TRIS = INPUT;
  CHG_ALM_ANSEL = DIG_INPUT;

  // Initialize I2C bus
  _ltc3350_init_i2c();

  uint8_t count = 0;
  count = _ltc3350_read(0x1A);
  count = 1;
}

/**
 * Wait for idle bus state
 */
inline void _ltc3350_wait_idle(void) {
  while (I2C4CON & 0x1F || 
         I2C4STATbits.TRSTAT || 
         I2C4STATbits.S || 
         I2C4STATbits.P);
}

/**
 * 
 * S, Slave addr/wr, command, RS, Slave addr/rd, data low, data high, P
 */
uint16_t _ltc3350_read(uint8_t addr) {
  uint8_t retry_cnt = 0;
  uint8_t data_low, data_high;

  while (retry_cnt <= 10) {
    retry_cnt++;

    // Generate start bus event
    I2C4STATCLR = _I2C4STAT_BCL_MASK;
    _ltc3350_wait_idle();
    I2C4CONbits.SEN = 1;
    while (I2C4CONbits.SEN);
    //I2C4STATCLR = _I2C4STAT_BCL_MASK;

    if (I2C4STATbits.BCL) {
      _ltc3350_wait_idle();
      I2C4STATCLR = _I2C4STAT_BCL_MASK;
      IFS5CLR = _IFS5_I2C4BIF_MASK;
      continue;
    }
    
    while (I2C4STATbits.BCL);
    
    // Send device address byte (write)
    I2C4TRN = (LTC3350_DEV_ADDR << 1) & 0xFE;
    while (I2C4STATbits.TBF); // Wait for empty buffer
    while (I2C4STATbits.ACKSTAT); // Wait for ACK
    
    // Send register address byte
    I2C4TRN = addr;
    while (I2C4STATbits.TBF); // Wait for empty buffer
    while (I2C4STATbits.ACKSTAT); // Wait for ACK
    
    // Repeated start
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.RSEN = 1; // Generate repeated start event
    while (I2C4CONbits.RSEN);
    
    // Send device address byte (read)
    I2C4TRN = (LTC3350_DEV_ADDR << 1) | 0b1;
    while (I2C4STATbits.TBF); // Wait for empty buffer
    while (I2C4STATbits.ACKSTAT); // Wait for ACK
    
    // Enable reception
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.RCEN = 1; // Begin receive sequence
    while (I2C4CONbits.RCEN);
    while (!I2C4STATbits.RBF); // Wait for buffer to fill
    data_low = I2C4RCV;
    
    // Generate ACK/NACK
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.ACKDT = 0; // ACK
    I2C4CONbits.ACKEN = 1; // Generate ACK event
    while (I2C4CONbits.ACKEN);
    
    // Enable reception
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.RCEN = 1; // Begin receive sequence
    while (I2C4CONbits.RCEN);
    while (!I2C4STATbits.RBF); // Wait for buffer to fill
    data_high = I2C4RCV;
    
    // Generate ACK/NACK
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.ACKDT = 0; // ACK
    I2C4CONbits.ACKEN = 1; // Generate ACK event
    while (I2C4CONbits.ACKEN);
    
    // Stop
    while (I2C4CON & 0x1F); // Wait for master logic inactive
    I2C4CONbits.PEN = 1; // Generate stop event
    while (I2C4CONbits.PEN);

    break;
  }

  if (retry_cnt == 11) {
    return 0x0;
  }

  return (uint16_t) ((data_high << 8) | data_low);
}

/**
 * 
 * S, Slave addr/wr, command, data low, data high, P 
 */
void _ltc3350_write(uint8_t addr, uint16_t data) {

}

/**
 */
void _ltc3350_init_i2c(void) {
  unlock_config();

  // Disable I2C4 Module
  I2C4CONbits.ON = 0;

  // SDA4
  TRISGbits.TRISG7 = OUTPUT;
  LATGbits.LATG7 = 0;
  TRISGbits.TRISG7 = INPUT;

  TRISGbits.TRISG8 = OUTPUT;
  LATGbits.LATG8 = 0;
  TRISGbits.TRISG8 = INPUT;

  //ANSELGbits.ANSG7 = DIG_INPUT;
  CNPUGbits.CNPUG7 = 1;

  // SCL4
  //TRISGbits.TRISG8 = OUTPUT;
  CNPUGbits.CNPUG8 = 1;

  // Disable interrupts
  IEC5CLR = _IEC5_I2C4MIE_MASK;
  IEC5CLR = _IEC5_I2C4SIE_MASK;
  IEC5CLR = _IEC5_I2C4BIE_MASK;
  IFS5CLR = _IFS5_I2C4MIF_MASK;
  IFS5CLR = _IFS5_I2C4SIF_MASK;
  IFS5CLR = _IFS5_I2C4BIF_MASK;

  /**
   * Baud rate (400kHz desired)
   * 
   * I2CxBRG = (F_FBCLK2 * ((1 / (2 * F_SCL)) - T_PGD)) - 2
   * I2CxBRG = (100Mhz * ((1.25 uS - 104nS)) - 2
   * I2CxBRG = 112
   */
  //I2C4BRG = 0x70; 400
  //I2C4BRG = 0x1E8; 100
  I2C4BRG = 0x9B8; // 20

  I2C4ADD = PIC_DEV_ADDR;

  // I2C4CON
  I2C4CONbits.A10M = 0; // 7-bit Slave Address
  I2C4CONbits.SMEN = 1; // Compliant with SMBus Spec

  // Enable I2C4 Module
  I2C4CONbits.ON = 1;

  lock_config();
}
