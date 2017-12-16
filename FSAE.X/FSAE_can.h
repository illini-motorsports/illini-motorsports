/**
 * FSAE Library CAN Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_CAN_H
#define FSAE_CAN_H

#include <xc.h>
#include <sys/kmem.h>
#include <sys/types.h>

/**
 * A friendly struct for storing CAN message data.
 */
typedef struct {
    uint32_t id;
    uint32_t dlc;
    uint8_t data[8];
} CAN_message;

/**
 * Enum making it easier to define endinaness
 */
typedef enum endian {BIG, LITTLE} Endian;

/**
 * Data type representing a CAN data buffer
 */
typedef union uCAN_data {
  struct {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
    uint8_t byte4;
    uint8_t byte5;
    uint8_t byte6;
    uint8_t byte7;
  };
  struct {
    uint16_t halfword0;
    uint16_t halfword1;
    uint16_t halfword2;
    uint16_t halfword3;
  };
  struct {
    uint32_t word0;
    uint32_t word1;
  };
  uint64_t doubleword;
} CAN_data;

// Must include this after the struct to avoid compilation errors (I'm probably doing something wrong)
#include "FSAE_config.h"

/**
 * CanRxMessageBuffer struct
 */

// CMSGSID
typedef struct {
  unsigned SID:11;
  unsigned FILHIT:5;
  unsigned CMSGTS:16;
} rxcmsgsid;

// CMSGEID
typedef struct {
  unsigned DLC:4;
  unsigned RB0:1;
  unsigned :3;
  unsigned RB1:1;
  unsigned RTR:1;
  unsigned EID:18;
  unsigned IDE:1;
  unsigned SRR:1;
  unsigned :2;
} rxcmsgeid;

// CMSGDATA0
typedef struct {
  unsigned Byte0:8;
  unsigned Byte1:8;
  unsigned Byte2:8;
  unsigned Byte3:8;
} rxcmsgdata0;

// CMSGDATA1
typedef struct {
  unsigned Byte4:8;
  unsigned Byte5:8;
  unsigned Byte6:8;
  unsigned Byte7:8;
} rxcmsgdata1;

// CanRxMessageBuffer
typedef union uCanRxMessageBuffer {
  struct {
    rxcmsgsid CMSGSID;
    rxcmsgeid CMSGEID;
    rxcmsgdata0 CMSGDATA0;
    rxcmsgdata1 CMSGDATA1;
  };
  int messageWord[4];
} CanRxMessageBuffer;

/**
 * CanTxMessageBuffer struct
 */

// CMSGSID
typedef struct {
  unsigned SID:11;
  unsigned :21;
} txcmsgsid;

// CMSGEID
typedef struct {
  unsigned DLC:4;
  unsigned RB0:1;
  unsigned :3;
  unsigned RB1:1;
  unsigned RTR:1;
  unsigned EID:18;
  unsigned IDE:1;
  unsigned SRR:1;
  unsigned :2;
} txcmsgeid;

// CMSGDATA0
typedef struct {
  unsigned Byte0:8;
  unsigned Byte1:8;
  unsigned Byte2:8;
  unsigned Byte3:8;
} txcmsgdata0;

// CMSGDATA1
typedef struct {
  unsigned Byte4:8;
  unsigned Byte5:8;
  unsigned Byte6:8;
  unsigned Byte7:8;
} txcmsgdata1;

// CanTxMessageBuffer
typedef union uCanTxMessageBuffer {
  struct {
    txcmsgsid CMSGSID;
    txcmsgeid CMSGEID;
    txcmsgdata0 CMSGDATA0;
    txcmsgdata1 CMSGDATA1;
  };
  int messageWord[4];
} CanTxMessageBuffer;

// Defined in FSAE_CAN.c
extern volatile uint32_t CAN_rx_ovf;
extern volatile uint32_t CAN_tx_ovf;

// Function definitions
void CAN_send_message(uint32_t id, uint32_t dlc, CAN_data data);
void CAN_recv_messages(void (*handler)(CAN_message msg));
void init_can(void);
double CAN_extract_numeric(uint8_t * data, uint8_t position, uint8_t length, 
		Endian endianness, uint8_t sgn, double scl, double off);
double CAN_extract_bit(uint8_t * data, uint8_t bytePos, uint8_t length, 
		Endian endianness, uint8_t bitPos);
uint32_t CAN_parse_bytes(uint8_t * data, uint8_t pos, uint8_t length, Endian endianness);


#endif /* FSAE_CAN_H */
