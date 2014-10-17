/*
 * MDD File I/O Main File Header
 *
 * File Name:       DAQ.h
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 * Created:         2012-2013
 */

#ifndef DAQ_H
#define DAQ_H

#include "GenericTypeDefs.h"
#include "FSconfig.h"

/*
 * Code Control
 */

//#define INTERNAL
//#define DEBUGGING
#define DEBUG_LEN 1000

/*
 * Magic Numbers
 */

#define BUFF_SIZE MEDIA_SECTOR_SIZE
#define MSGS_READ 4
#define MSG_ID_LEN 2
#define TIMESTAMP_LEN 4
#define ID_PLUS_TIME 6
#define RPM_COMP 600    // check for an RPM above this value to know whether the engine is on or not

/*
 * Pin Defintions
 */

#define TERM_LAT    LATCbits.LATC6

/*
 * Typedefs
 */

// main function flags and other information

typedef struct {
    unsigned BufferAFull : 1;
    unsigned Swap : 1;
    unsigned Written : 1;
    unsigned NumRead : 3;
    unsigned MsgNum : 2;
    unsigned EngTO : 1;
    unsigned int BufferALen;
    unsigned int BufferBLen;
} MAIN;

typedef struct {
    BYTE * BufferA;
    BYTE * BufferB;
} BUFF_HOLDER;

void funct_error(void);
void init_unused_pins(void);
void service_FIFO(void);
void append_write_buffer(static const unsigned char * temp, static unsigned char applen);
void buff_cat(static unsigned char *WriteBuffer, static const unsigned char *writeData,
        static unsigned int *bufflen, static const unsigned char applen,
        static const unsigned char offset);
void high_isr(void);
void low_isr(void);
void swap_buff(void);
void swap_len(void);

#endif
