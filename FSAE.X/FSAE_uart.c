/**
 * FSAE Library UART
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#include "FSAE_uart.h"

#include <string.h>

#define TX_BUF_SIZE 4096

UARTBus busses[6] = {0};
uint8_t txBuffers[6][TX_BUF_SIZE];

/**
 * Initializes a UART bus with the given parameters.
 *
 * Note: Unlike SPI, only one client is allowed to use a single UART bus at a
 * time. UART has no concept of chip select. Attempting to init a UART bus which
 * has already been initialized will return NULL.
 */
UARTBus* init_uart(uint8_t busNum, double baud) {
  if (busNum < 1 || busNum > 6) { return NULL; }

  UARTBus* bus = &busses[busNum - 1];
  if (bus->init) { return NULL; }

  _uart_init(busNum, baud);

  bus->bus = busNum;
  bus->write_byte_fp = _uart_get_write_byte(busNum);
  bus->init = 1;

  return bus;
}

/**
 * Write a single byte to the given UART bus.
 *
 * Returns UART_SUCCESS if the byte was successfully loaded into the TX buffer
 * Returns UART_ERR_TXBF if the TX buffer is full
 * Returns UART_ERR_INIT if the bus was not yet initialized
 */
uint8_t uart_write_byte(UARTBus* bus, uint8_t val) {
  if (!bus->init) { return UART_ERR_INIT; }
  return bus->write_byte_fp(val);
}

/**
 * Initiate a write of a large buffer to the given UART bus.
 *
 * This function allows writing buffers of up to size TX_BUF_SIZE. The data
 * is first copied into an internal buffer. Then, the TX interrupts are enabled
 * and data is copied from the internal buffer into the UART module TX FIFO as
 * fast as possible. When all bytes have been transfered into the TX FIFO, the
 * WIP bit is cleared, and the write finished callback is called.
 *
 * Returns UART_SUCCESS if the buffer transfas was successfully started
 * Returns UART_ERR_INIT if the bus was not yet initialized
 * Returns UART_ERR_TX_SIZE if the buffer to transfer is too large
 * Returns UART_ERR_TX_WIP if a buffer transfer is already in progress
 */
uint8_t uart_write_buffer(UARTBus* bus, uint8_t* buf, uint32_t len) {
  if (!bus->init) { return UART_ERR_INIT; }
  if (len > TX_BUF_SIZE) { return UART_ERR_TX_SIZE; }
  if (bus->wip) { return UART_ERR_TX_WIP; }
  bus->wip = 1;
  bus->iter = txBuffers[bus->bus];
  bus->end = bus->iter + len;
  memcpy(bus->iter, buf, len);

  // Enable TX interrupt to transfer from internal buffer to UART FIFO
  switch (bus->bus) {
    case 1: IEC3SET = _IEC3_U1TXIE_MASK; break;
    case 2: break;
    case 3: break;
    case 4: break;
    case 5: break;
    case 6: break;
  }

  return UART_SUCCESS;
}

/**
 * Generic initialization function for all UART busses
 */
void _uart_init(uint8_t bus, double baud) {
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

uart_write_byte_fp _uart_get_write_byte(uint8_t bus)
{
  switch (bus) {
    case 1:
      return _uart_write_byte_1;
    case 2:
      return _uart_write_byte_2;
    case 3:
      return _uart_write_byte_3;
    case 4:
      return _uart_write_byte_4;
    case 5:
      return _uart_write_byte_5;
    case 6:
      return _uart_write_byte_6;
    default:
      return NULL;
  }
}

uint8_t _uart_write_byte_1(uint8_t val) {
  if (U1STAbits.UTXBF)
    return UART_ERR_TXBF;
  U1TXREG = val;
  return UART_SUCCESS;
}

uint8_t _uart_write_byte_2(uint8_t val) {
  if (U2STAbits.UTXBF)
    return UART_ERR_TXBF;
  U2TXREG = val;
  return UART_SUCCESS;
}

uint8_t _uart_write_byte_3(uint8_t val) {
  if (U3STAbits.UTXBF)
    return UART_ERR_TXBF;
  U3TXREG = val;
  return UART_SUCCESS;
}

uint8_t _uart_write_byte_4(uint8_t val) {
  if (U4STAbits.UTXBF)
    return UART_ERR_TXBF;
  U4TXREG = val;
  return UART_SUCCESS;
}

uint8_t _uart_write_byte_5(uint8_t val) {
  if (U5STAbits.UTXBF)
    return UART_ERR_TXBF;
  U5TXREG = val;
  return UART_SUCCESS;
}

uint8_t _uart_write_byte_6(uint8_t val) {
  if (U6STAbits.UTXBF)
    return UART_ERR_TXBF;
  U6TXREG = val;
  return UART_SUCCESS;
}

void
__attribute__((vector(_UART1_TX_VECTOR), interrupt(IPL2SRS)))
_uart_tx1_inthnd(void)
{
  UARTBus* bus = &(busses[0]);
  while (!U1STAbits.UTXBF && bus->iter != bus->end)
    U1TXREG = *(bus->iter)++;
  if (bus->iter == bus->end) {
    bus->wip = 0;
    IEC3CLR = _IEC3_U1TXIE_MASK;
    //TODO: Call finished callback
  }
  IFS3CLR = _IFS3_U1TXIF_MASK;
}
