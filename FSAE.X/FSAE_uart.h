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

// UART Errors
#define UART_SUCCESS     0 // No error
#define UART_ERR_TXBF    1 // TX FIFO full
#define UART_ERR_INIT    2 // UART bus is not initialized
#define UART_ERR_TX_SIZE 3 // TX buffer write too large
#define UART_ERR_TX_WIP  4 // TX buffer write already in progress

typedef uint8_t (*uart_write_byte_fp) (uint8_t);

/**
 * Struct used for all UART communication in the FSAE library. UART functions
 * will take a UARTBus pointer as an argument to communicate with a specific
 * UART bus.
 */
typedef struct {
  uint8_t init;
  uint8_t bus;
  uart_write_byte_fp write_byte_fp;
  uint8_t wip;   // Buffer write in progress
  uint8_t* iter; // Current position in the buffer write
  uint8_t* end;  // End position for the buffer write
} UARTBus;

// Public Interface
UARTBus* init_uart(uint8_t busNum, double baud);
uint8_t uart_write_byte(UARTBus* conn, uint8_t val);
uint8_t uart_write_buffer(UARTBus* conn, uint8_t* buf, uint32_t len);

// Private Implementation
void _uart_init(uint8_t bus, double baud);
uart_write_byte_fp _uart_get_write_byte(uint8_t bus);
uint8_t _uart_write_byte_1(uint8_t val);
uint8_t _uart_write_byte_2(uint8_t val);
uint8_t _uart_write_byte_3(uint8_t val);
uint8_t _uart_write_byte_4(uint8_t val);
uint8_t _uart_write_byte_5(uint8_t val);
uint8_t _uart_write_byte_6(uint8_t val);
void _uart_tx1_inthnd(void);

#endif /* FSAE_uart_H */
