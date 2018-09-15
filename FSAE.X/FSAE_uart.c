/**
 * FSAE Library UART
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#include "FSAE_uart.h"

/**
 * Generic initialization function for all UART busses
 */
void init_uart(uint8_t bus, double baud) {
  /**
   * BRG = (F_PB / (4 * BAUD)) - 1
   * (BRGH = 0)
   */
  //TODO: Switch from 38400
  uint16_t brg = ((uint16_t) ((PBCLK2 * 1000000) / (16.0 * 38400.0))) - 1;

  // Initialize correct bus
  //TODO-AM: Initialize other busses
  switch(bus) {
    case 1:
      //TODO-AM: Disable/enable interrupts

      unlock_config();
      TRISDbits.TRISD11 = OUTPUT;
      RPD11Rbits.RPD11R = 0b0001; // U1TX

      TRISDbits.TRISD10 = INPUT;
      U1RXRbits.U1RXR = 0b0011; // D10
      lock_config();

      U1STA = 0;
      U1MODE = 0;

      U1BRG = brg;

      IEC3bits.U1TXIE = 0;
      IFS3bits.U1TXIF = 0;

      U1MODEbits.ON = 1;
      U1MODEbits.UEN = 0b00;

      U1STAbits.URXEN = 1;
      U1STAbits.UTXEN = 1;

      break;

    case 2:
      break;

    case 3:
      break;

    case 4:
      break;

    case 5:
      break;

    case 6:
      break;
  }
}

/**
 * Return the correct function pointer associated with the given bus
 */
uart_send_fp uart_get_send(uint8_t bus) {
  switch(bus) {
    case 1:
      return _uart_send1;
    case 2:
    case 3:
    case 5:
    case 6:
    default:
      return NULL;
  }
}

uint8_t _uart_send1(uint8_t val) {
  //TODO-AM: Send data and return response
  return val;
}
