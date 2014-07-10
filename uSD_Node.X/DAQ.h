/******************************************************************************
 *
 *					MDD File I/O Main Code Header
 *
 ******************************************************************************
 *
 * FileName:        DAQ.h
 * Dependencies:    GenericTypeDefs.h
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18

*******************************************************************************
	USER REVISON HISTORY
//
// 03/04/13 - created file and added function prototypes and defines and structures
//          code sections.
// 03/13/13 - added RPM defines for control over when to start logging
//

*******************************************************************************/

#ifndef DAQ_H
#define DAQ_H

#include "GenericTypeDefs.h"

/***********************************************/
/*  User Structures & Defines                  */
/***********************************************/

//#define INTERNAL
//#define DEBUGGING
#define DEBUG_LEN 1000

#define BUFF_SIZE MEDIA_SECTOR_SIZE
#define MSGS_READ 4
#define MSG_ID_LEN 2
#define TIMESTAMP_LEN 4
#define ID_PLUS_TIME 6

#define BEACON_ID 0xE5L

#define RPM_ID 0x200L
#define RPM_BYTE 0
#define RPM_COMP 600	// check for an RPM above this value to know whether the engine is on or not

#define TERM_LAT	LATCbits.LATC6

// main function flags and other information
typedef struct {
	unsigned BufferAFull:1;
	unsigned Swap:1;
	unsigned Written:1;
	unsigned NumRead:3;
	unsigned MsgNum:2;
	unsigned int BufferALen;
	unsigned int BufferBLen;
} MAIN;

typedef struct {
	BYTE * BufferA;
	BYTE * BufferB;
} BUFF_HOLDER;


/***********************************************/
/*  User Function Prototypes                   */
/***********************************************/

void funct_error(void);
void init_unused_pins(void);
void service_FIFO(void);
void append_write_buffer(static const BYTE * temp, static BYTE applen);
void buff_cat(static BYTE *WriteBuffer, static const BYTE *writeData, static unsigned int *bufflen, static const BYTE applen, static const BYTE offset);
void high_isr(void);
void low_isr (void);
void swap_buff(void);
void swap_len(void);

#endif
