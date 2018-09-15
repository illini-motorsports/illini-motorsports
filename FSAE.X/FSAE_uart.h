/**
 * FSAE Library UART Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2017-2018
 */
#ifndef FSAE_uart_H
#define FSAE_uart_H

#include "FSAE_config.h"

typedef uint8_t (*uart_send_fp) (uint8_t);

/**
 * Struct used for all UART communication in the FSAE library. UART functions
 * will take a UARTConn pointer as an argument to communicate with a specific
 * chip on a given UART bus.
 */
typedef struct {
  uart_send_fp send_fp;
} UARTConn;

void init_uart(uint8_t bus, double baud);
uart_send_fp uart_get_send(uint8_t bus);

uint8_t _uart_send1(uint8_t val);

#endif /* FSAE_uart_H */
