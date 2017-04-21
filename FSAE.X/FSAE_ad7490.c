/**
 * FSAE Library AD7490
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2016-2017
 */
#include "FSAE_ad7490.h"

/**
 * void init_ad7490(void)
 *
 * Initializes the AD7490 ADC module.
 */
void init_ad7490(uint32_t (*send_value)(uint32_t)) {
  // Send two dummy cycles to reset the chip
  AD7490ControlReg dummy = {.reg = 0xFFFF};
  send_value(dummy.reg);
  send_value(dummy.reg);

  // Send initial configuration message
  AD7490ControlReg config = {.reg = 0x0};
  config.WRITE = 0b1;
  config.ADDR = 0b0;
  config.PM = 0b11;
  config.WEAK_TRI = 0b1;
  send_value(config.reg);
}

/**
 * uint16_t** read_channels(Uint16_t, send_value)
 *
 * Iterates through all channels, instructs the ADC to sample the current
 * value, and returns all the sample values.
 */
void ad7490_read_channels(uint16_t* channel_values, uint32_t (*send_value)(uint32_t)) {
  AD7490ControlReg control = {.reg = 0x0};
  control.WRITE = 0b1;
  control.ADDR = 0b0;
  control.PM = 0b11;
  control.WEAK_TRI = 0b1;
  control.CODING = 0b1;

  // Set up chip for first read in sequence
  send_value(control.reg);

  // Sample each channel and save the response
  uint8_t i;
  for (i = 0; i < AD7490_NUM_CHN; i++) {
    control.ADDR = i + 1; // Set up ADDR for next read in sequence
    uint16_t resp = (uint16_t) send_value(control.reg);
    channel_values[i] = resp & 0x0FFF; // Only bottom 12 bits are value
  }
}
