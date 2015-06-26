/**
 * Logger Header
 *
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

/**
 * Code Control
 */

//#define INTERNAL

/**
 * Magic Numbers
 */

#define BUFFER_SIZE MEDIA_SECTOR_SIZE
#define MSGS_READ 4
#define RPM_THRESH 600 // RPM threshold for engine to be considered on
#define CAN_PERIOD 250 // Send log filename every 250ms
#define RPM_WAIT 1000 // Wait 1000ms before closing file

/**
 * Pin Defintions
 */

#define TERM_LAT    LATCbits.LATC6

/**
 * Typedefs
 */

typedef struct {
    unsigned char* left;
    unsigned char* right;
} BUFFER_TUPLE;

void low_isr(void);
void high_isr(void);
void abort(void);
void read_CAN_buffers(void);

void append_write_buffer(const unsigned char * temp, unsigned char applen);
void buff_cat(unsigned char *WriteBuffer, const unsigned char *writeData,
        unsigned int *bufflen, const unsigned char applen,
        const unsigned char offset);
void swap_len(void);
void swap_buff(void);

#endif
