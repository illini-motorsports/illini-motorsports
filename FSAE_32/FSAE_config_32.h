/**
 * FSAE Library 32bit Config Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#ifndef FSAE_CONFIG_32_H
#define FSAE_CONFIG_32_H

#include <xc.h>
#include <sys/kmem.h>

#ifdef BUILD_PDM
#include "../PDM_Node.X/PDM.h"
#endif

#define OUTPUT 0
#define INPUT 1

// Pin definitions for programmable termination
#define TERM_TRIS TRISAbits.TRISA2
#define TERM_LAT  LATAbits.LATA2

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
    rxcmsgdata1 CMSGDATA1;
  };
  int messageWord[4];
} CanRxMessageBuffer;

/**
 * Create CanTxMessageBuffer struct
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

// Function definitions
void init_general(void);
void init_gpio_pins(void);
void init_peripheral_modules(void);
void init_oscillator(void);
void init_can(void);
void init_timer1(void);
void init_spi(void);
void init_adc(void);
void init_termination(void);
void init_can(void);

#endif /* FSAE_CONFIG_32_H */
