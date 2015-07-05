/**
 * Logger Header
 *
 * Processor:   PIC32MZ2048ECM064
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "Logger_config.h"

/**
 * Create CanRxMessageBuffer struct
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
    rxcmsgdata0 CMSGDATA1;
  };
  int messageWord[4];
} CanRxMessageBuffer;

#endif /* LOGGER_H */
