/*
 * Data Node Main File Header
 *
 * File Name:       DAQ.h
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 * Author:          Andrew Mass
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

#define BUFFER_SIZE MEDIA_SECTOR_SIZE
#define MSGS_READ 4
#define RPM_THRESH 600 // RPM threshold for engine to be considered on

/*
 * Pin Defintions
 */

#define TERM_LAT    LATCbits.LATC6

/*
 * Typedefs
 */

// Main function flags and other information

typedef struct {
    unsigned NumRead : 3;
    unsigned MsgNum : 2;
} MAIN;

typedef struct {
    BYTE * BufferA;
    BYTE * BufferB;
} BUFF_HOLDER;

void low_isr(void);
void high_isr(void);
void abort(void);
void read_CAN_buffers(void);

void append_write_buffer(static const unsigned char * temp, static unsigned char applen);
void buff_cat(static unsigned char *WriteBuffer, static const unsigned char *writeData,
        static unsigned int *bufflen, static const unsigned char applen,
        static const unsigned char offset);
void swap_len(void);
void swap_buff(void);

#endif
