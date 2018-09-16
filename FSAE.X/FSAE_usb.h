/**
 * FSAE Library USB Header
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2017-2018
 */
#ifndef FSAE_usb_H
#define FSAE_usb_H

#include <string.h>
#include "FSAE_config.h"
#include "FSAE_uart.h"

// FTDI FT230X

#define _USB_STD_BUS 1
#define _USB_BAUD 38400 //TODO: Set this value

#define _USB_VUSB_TRIS  TRISBbits.TRISB12
#define _USB_VUSB_ANSEL ANSELBbits.ANSB12
#define _USB_VUSB_PORT  PORTBbits.RB12

static uint8_t usb_connected = 0;

// Function definitions

// Public Interface
//UARTConn* init_usb_std(); // Use standard settings
//UARTConn* init_usb(uint8_t bus);

// Private Implementation
void _usb_connected(void);
void _usb_disconnected(void);

#endif /* FSAE_usb_H */
