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
   * BRG = (F_PB / (16 * BAUD)) - 1
   * (BRGH = 0)
   */
  uint16_t brg = ((uint16_t) ((PBCLK2 * 1000000) / (16.0 * baud))) - 1;

  //TODO: Initialize other busses

  // Initialize correct bus
  switch(bus) {
    case 1:
      unlock_config();
      CFGCONbits.IOLOCK = 0;
      RPD11Rbits.RPD11R = 0b0001; // U1TX
      U1RXRbits.U1RXR = 0b0011; // D10
      CFGCONbits.IOLOCK = 1;
      lock_config();

      U1MODE = 0;
      U1STA = 0;

      // Set up transmit interrupt (leave disabled until transmitting)
      IPC28bits.U1TXIP = 2;
      IPC28bits.U1TXIS = 3;
      IEC3CLR = _IEC3_U1TXIE_MASK;
      IFS3CLR = _IFS3_U1TXIF_MASK;
      U1STAbits.UTXISEL = 0b00; // Interrupt fires when at least one space available

      //TODO: Set up receive interrupt

      U1BRG = brg;

      U1MODEbits.UEN = 0b00; // Only using U1TX and U1RX
      U1MODEbits.BRGH = 0; // Standard (16x) baud rate mode
      U1MODEbits.PDSEL = 0b00; // 8 bit, no parity
      U1MODEbits.STSEL = 0b0; // 1 stop bit
      U1MODEbits.ON = 1;

      U1STAbits.URXEN = 1;
      U1STAbits.UTXEN = 1;

      IFS3CLR = _IFS3_U1TXIF_MASK;

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
  //TODO: Send data and return response
  return val;
}
