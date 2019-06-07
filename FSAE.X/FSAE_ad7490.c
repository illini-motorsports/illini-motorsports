/**
 * FSAE Library AD7490
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_ad7490.h"

SPIConn ad7490connections[10];
uint8_t ad7490ConnIdx = 0;

/**
 * Initializes the AD7490 ADC module.
 */
SPIConn* init_ad7490(uint8_t bus, uint32_t *cs_lat, uint8_t cs_num) {

    init_spi(bus, 1.0, 16, 2);

  SPIConn * currConn = &ad7490connections[ad7490ConnIdx];
  currConn->send_fp = get_send_spi(bus);
  currConn->cs_lat = cs_lat;
  currConn->cs_num = cs_num;

  ad7490ConnIdx++;

  // Send two dummy cycles to reset the chip
  AD7490ControlReg dummy = {.reg = 0xFFFF};
  send_spi(dummy.reg, currConn);
  send_spi(dummy.reg, currConn);

  // Send initial configuration message
  AD7490ControlReg config = {.reg = 0x0};
  config.WRITE = 0b1;
  config.ADDR = 0b0;
  config.PM = 0b11;
  config.WEAK_TRI = 0b1;
  send_spi(config.reg, currConn);

  return currConn;
}

/**
 * Iterates through all channels, instructs the ADC to sample the current
 * value, and returns all the sample values.
 */
void ad7490_read_channels(uint16_t* channel_values, SPIConn *conn) {
  AD7490ControlReg control = {.reg = 0x0};
  control.WRITE = 0b1;
  control.ADDR = 0b0;
  control.PM = 0b11;
  control.WEAK_TRI = 0b1;
  control.CODING = 0b1;

  // Set up chip for first read in sequence
  send_spi(control.reg, conn);

  // Sample each channel and save the response
  uint8_t i;
  for (i = 0; i < AD7490_NUM_CHN; i++) {
    control.ADDR = i + 1; // Set up ADDR for next read in sequence
    uint16_t resp = (uint16_t) send_spi(control.reg, conn);
    channel_values[i] = resp & 0x0FFF; // Only bottom 12 bits are value
  }
}
