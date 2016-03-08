/**
 * FSAE Library 32bit CAN
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_CAN_32.h"

/**
 * We have to allocate memory in RAM for the CAN1 module's FIFOs. There can be
 * up to 32 FIFOs, and each FIFO can contain up to 32 message buffers for a
 * total of 1024 message buffers. Each message buffer takes up 4 words
 * (16 bytes) 2 for the CAN message data and 2 for a timestamp. Note that on a
 * PIC32 an int is 1 word (4 bytes).
 *
 * Here, we allocate 256 words, enough for 32 message buffers in two FIFOs.
 */
static volatile uint32_t CAN_FIFO_Buffers[256];

/**
 * Construct and send a CAN message based on data provided by the caller.
 *
 * @param id The message ID of the CAN message we are sending
 * @param dlc The number of data bytes in the message
 * @param data A Pointer to the data bytes
 * @return -1 on failure, 0 on success
 */
void CAN_send_message(uint32_t id, uint32_t dlc, CAN_data data) {
  // Get pointer to correct location in transmit FIFO
  CanTxMessageBuffer* transmit = (CanTxMessageBuffer*) (PA_TO_KVA1(C1FIFOUA1));

  // Clear location in transmit FIFO
  transmit->messageWord[0] = 0;
  transmit->messageWord[1] = 0;
  transmit->messageWord[2] = 0;
  transmit->messageWord[3] = 0;

  // Copy message to send into transmit FIFO location
  transmit->CMSGSID.SID = id;
  transmit->CMSGEID.DLC = dlc;

  // CMSGDATA0
  if (dlc >= 2) {
    ((uint16_t*) &(transmit->messageWord[2]))[0] = data.halfword0;
  }
  if (dlc >= 4) {
    ((uint16_t*) &(transmit->messageWord[2]))[1] = data.halfword1;
  }

  // CMSGDATA1
  if (dlc >= 6) {
    ((uint16_t*) &(transmit->messageWord[3]))[0] = data.halfword2;
  }

  if (dlc >= 8) {
    ((uint16_t*) &(transmit->messageWord[3]))[1] = data.halfword3;
  }

  // Signal to the CAN module that we have finished queuing a message
  C1FIFOCON1bits.UINC = 1;

  // Request transmission of the message
  C1FIFOCON1bits.TXREQ = 1;
}

/**
 * Receive and handle any available CAN messages.
 *
 * This function calls the provided handler function once for each received
 * message. There can be anywhere from 0 to 32 available messages, so the
 * handler function can be called up to 32 times before this function will return.
 *
 * @param handler The handler function to call with each received message
 */
void CAN_recv_messages(void (*handler)(CAN_message msg)) {
    CanRxMessageBuffer* receive = NULL;

    // Keep polling until the FIFO isn't empty
    while(C1FIFOINT0bits.RXNEMPTYIF == 1) {

      // Get pointer to the next available message
      receive = (CanRxMessageBuffer*) (PA_TO_KVA1(C1FIFOUA0));

      // Copy data from the receive FIFO into a CAN_message struct
      CAN_message msg;
      msg.id = receive->CMSGSID.SID;
      msg.dlc = receive->CMSGEID.DLC;
      ((uint32_t*) msg.data)[0] = receive->messageWord[2];
      ((uint32_t*) msg.data)[1] = receive->messageWord[3];

      // Signal to the CAN module that we've processed a message
      C1FIFOCON0bits.UINC = 1;

      // Call the provided handler function
      handler(msg);
    }
}

/**
 * void init_can(void)
 *
 * Sets up everything relating to CAN. This function assumes that C1RX is wired
 * to RD3 and C1TX is wired to RD2. All PIC32 microcontrollers using this
 * library should use that configuration.
 */
void init_can(void) {
  unlock_config();
  CFGCONbits.IOLOCK = 0; // Peripheral Pin Select Lock (Not locked)

  // Set PPS pins for C1TX/C1RX
  RPD2R = 0b1111; // RPD2 Peripheral Pin Select (C1TX)
  C1RXR = 0b0000; // C1RX Peripheral Pin Select (RPD3)

  // Set port direction for C1TX/C1RX
  TRISDbits.TRISD2 = OUTPUT;
  TRISDbits.TRISD3 = INPUT;

  // Set to configuration mode
  C1CONbits.REQOP = 0b100; // Request Operation Mode (Set Configuration mode)
  while(C1CONbits.OPMOD != 0b100); // Wait for the module to finish

  C1CONbits.ON = 1; // CAN On (Enabled)

  // C1CON
  C1CONbits.CANCAP = 1; // CAN Message Receive Timestamp Timer Capture Enable (Enabled)
  C1CONbits.SIDL = 0;   // CAN Stop in Idle (CAN continues operation when system enters idle mode)

  /**
   * Configure CAN1 to run at 1Mpbs baud rate
   *
   * Ntq = 10 (1 + 4 + 3 + 2)
   * Ftq = Ntq * Fbaud = 10 * 1Mbps = 10Mhz
   * BRP = (Fsys / (2 * Ftq)) - 1 = (200Mhz / 20Mhz) - 1 = 9
   *
   * Note: A BRP value of 9 will cause the bus to run at a rate of 500kbps. Setting BRP to 4 gives
   * the correct rate of 1Mpbs. I'm not sure how the math on this checks out, but it works.
   */

  // C1CFG
  C1CFGbits.BRP = 4;       // Baud Rate Prescaler (See above equation)
  C1CFGbits.SEG2PHTS = 1; // Phase Segment 2 Time Select (Freely programmable)
  C1CFGbits.SEG2PH = 2;   // Phase Buffer Segment 2 (Length is 3 * Tq)
  C1CFGbits.SEG1PH = 2;   // Phase Buffer Segment 1 (Length is 3 * Tq)
  C1CFGbits.PRSEG = 2;    // Propagation Time Segment (Length is 3 * Tq)
  C1CFGbits.SAM = 1;      // Sample of the CAN Bus Line (Bus line is sampled three times at the sample point)
  C1CFGbits.SJW = 2;      // Synchronization Jump Width (Length is 3 * Tq)
  C1CFGbits.WAKFIL = 0;   // CAN Bus Line Filter Enable (CAN bus line filter is not used for wake-up)

  /**
   * Set the base address of the CAN1 FIFOs as physical address of the memory
   * that we previously allocated.
   */
  C1FIFOBA = KVA_TO_PA(CAN_FIFO_Buffers);

  // CAN1 FIFO 0
  C1FIFOCON0bits.TXEN = 0;        // TX/RX Buffer Selection (Receive FIFO)
  C1FIFOCON0bits.FSIZE = 0b11111; // FIFO Size bits (32 messages deep)
  C1FIFOCON0bits.DONLY = 0;       // Store Message Data Only (Full message is stored, including identifier)
  C1FIFOINT0bits.RXHALFIE = 1;    // FIFO Half Full Interrupt Enable (Enabled)
  C1FIFOINT0bits.RXFULLIE = 0;    // FIFO Full Interrupt Enable (Disabled)
  C1FIFOINT0bits.RXNEMPTYIE = 0;  // FIFO Not Empty Interrupt Enable (Disabled)
  C1FIFOINT0bits.RXOVFLIE = 1;    // FIFO Overflow Interrupt Enable (Enabled)

  // CAN1 FIFO 1
  C1FIFOCON1bits.TXEN = 1;        // TX/RX Buffer Selection (Transmit FIFO)
  C1FIFOCON1bits.FSIZE = 0b11111; // FIFO Size bits (32 messages deep)
  C1FIFOINT1bits.TXEMPTYIE = 0;  // FIFO TX Empty Interrupt Enable (Disabled)
  C1FIFOINT1bits.TXHALFIE = 0;   // FIFO TX Half Full Interrupt Enable (Disabled)
  C1FIFOINT1bits.TXNFULLIE = 0;  // FIFO TX Not Full Interrupt Enable (Disabled)

  /**
   * Here, we set up the first CAN acceptance filter to allow any messages with standard
   * identifiers to be sent to the receive buffer (FIFO 0). All messages with extended
   * identifiers are ignored.
   */

  // CAN1 Acceptance Filter 0
  C1RXF0bits.SID = 0;  // Standard Identifier bits (Must be all zeros (minus mask) to match)
  C1RXF0bits.EXID = 0; // Extended Identifier Enable bits (Match only standard identifier)
  C1RXF0bits.EID = 0;  // Extended Identifier bits (Must be all zeros (minus mask) to match)

  // CAN1 Acceptance Filter Mask 0
  C1RXM0bits.SID = 0;       // Standard Identifier bits (No bits included in filter comparison)
  C1RXM0bits.MIDE = 1;      // Identifier Receive Mode bit (Match only message types that correspond to EXID bit in filter)
  C1RXM0bits.EID = 0x3FFFF; // Extended Identifier bits (All bits included in filter comparison)

  // CAN1 Filter 0 - Filter must be disabled before modification
  C1FLTCON0bits.FLTEN0 = 0;       // Filter 0 Enable bit (Filter is disabled)
  C1FLTCON0bits.MSEL0 = 0b00;     // Filter 0 Mask Select bits (Acceptance Mask 0 selected)
  C1FLTCON0bits.FSEL0 = 0b00000;  // FIFO Selection bits (Message matching filter is stored in FIFO 0)
  C1FLTCON0bits.FLTEN0 = 1;       // Filter 0 Enable bit (Filter is enabled)

  // Set up CAN1 Interrupt
  IFS4bits.CAN1IF = 0;  // CAN1 Interrupt Flag Status (No interrupt request has occurred)
  IPC37bits.CAN1IP = 4; // CAN1 Interrupt Priority (Interrupt priority is 4)
  IPC37bits.CAN1IS = 3; // CAN1 Interrupt Subpriority (Interrupt subpriority is 3)
  IEC4bits.CAN1IE = 1;  // CAN1 Interrupt Enable Control (Interrupt is disabled)

  C1INTbits.RBIF = 0; // Receive Buffer Interrupt Flag (Not pending)
  C1INTbits.RBIE = 1; // Receive Buffer Interrupt Enable (Enabled)
  C1INTbits.RBOVIF = 0; // Receive Buffer Overflow Interrupt Flag (Not pending)
  C1INTbits.RBOVIE = 1; // Receive Buffer Overflow Interrupt Enable (Enabled)

  C1CONbits.REQOP = 0b000; // Request Operation Mode (Set Normal Operation mode)
  while(C1CONbits.OPMOD != 0b000); // Wait for the module to finish

  CFGCONbits.IOLOCK = 1; // Peripheral Pin Select Lock (Locked)
  lock_config();
}