/**
 * FSAE Library USB
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#include "FSAE_usb.h"

#define NUM_CONN 10
UARTConn connections[NUM_CONN];
uint8_t connIdx = 0;

//=============================== INTERFACE ====================================

/**
 * Initializes the FTDI FT230X USB module for the standard FSAE template.
 */
UARTConn* init_usb_std() {
  return init_usb(_USB_STD_BUS);
}

/**
 * Initializes the FTDI FT230X USB module with user-defined settings.
 */
UARTConn* init_usb(uint8_t bus) {
  if (connIdx == NUM_CONN) { return NULL; }

  // Initialize UART communications to the NVM chip
  if (!connIdx) {
    init_uart(bus, _USB_BAUD);
  }

  UARTConn* currConn = &connections[connIdx];
  currConn->send_fp = uart_get_send(bus);
  connIdx++;

  // Set up VUSB pin and change interrupt
  _USB_VUSB_TRIS = INPUT;
  _USB_VUSB_ANSEL = DIG_INPUT;
  CNENBSET = _CNENB_CNIEB12_MASK;
  CNNEBSET = _CNNEB_CNNEB12_MASK;
  CNCONBSET = _CNCONB_ON_MASK;
  CNCONBCLR = _CNCONB_EDGEDETECT_MASK;
  IFS3CLR = _IFS3_CNBIF_MASK;
  IEC3SET = _IEC3_CNBIE_MASK;
  IPC29bits.CNBIP = 3;
  IPC29bits.CNBIS = 3;

  usb_connected = _USB_VUSB_PORT;

  return currConn;
}

//============================= IMPLEMENTATION =================================

/**
 * Handle USB connection
 */
void _usb_connected(void) {
  //TODO
}

/**
 * Handle USB disconnection
 */
void _usb_disconnected(void) {
  //TODO
}

/**
 * Interrupt handler for port B change notification (VUSB pin)
 */
void __attribute__((vector(_CHANGE_NOTICE_B_VECTOR), interrupt(IPL3SRS))) cn_inthnd(void) {
  uint16_t read = PORTB; // Clear pin state mismatches
  uint8_t prev = usb_connected;

  if (IFS3bits.CNBIF) {
    usb_connected = _USB_VUSB_PORT;
  }
  IFS3CLR = _IFS3_CNBIF_MASK;

  if (!prev && usb_connected) {
    _usb_connected();
  } else if (prev && !usb_connected) {
    _usb_disconnected();
  }
}
