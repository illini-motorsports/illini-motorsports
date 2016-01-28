/**
 * FSAE Library 32bit CAN
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_CAN_32.h"

static CanRxMessageBuffer* receive;
static CanTxMessageBuffer* transmit;

int32_t CAN_send_message(uint16_t id, uint8_t dlc, uint8_t* data) {
  // Only allow even DLCs that are less than or equal to 8
  if(!(dlc == 0 || dlc == 2 || dlc == 4 || dlc == 6 || dlc == 8)) {
    return -1;
  }

  transmit = (CanTxMessageBuffer*) (PA_TO_KVA1(C1FIFOUA1));
  transmit->messageWord[0] = 0;
  transmit->messageWord[1] = 0;
  transmit->messageWord[2] = 0;
  transmit->messageWord[3] = 0;

  transmit->CMSGSID.SID = id;
  transmit->CMSGEID.DLC = dlc;
  transmit->messageWord[2] = ((uint32_t*) data)[1]; // CMSGDATA0
  transmit->messageWord[3] = ((uint32_t*) data)[1]; // CMSGDATA1

  C1FIFOCON1bits.UINC = 1;
  C1FIFOCON1bits.TXREQ = 1;

  return 0;
}

/*
 * Receive:
    // Keep polling until the FIFO isn't empty
    while(C1FIFOINT0bits.RXNEMPTYIF == 1) {
      receive = (CanRxMessageBuffer*) (PA_TO_KVA1(C1FIFOUA0));

      if(receive->CMSGSID.SID == 0x200) {
          receive = NULL;
      }

      // Signal to the CAN module that we've processed a message
      C1FIFOCON0bits.UINC = 1;
    }
 */