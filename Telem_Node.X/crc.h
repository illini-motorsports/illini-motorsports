/**********************************************************************
 *
 * Filename:    crc.h
 * 
 * Description: A header file with prototypes
 *
 * Notes:       
 *
 *
 **********************************************************************/

#ifndef _crc_h
#define _crc_h

#define INITIAL_REMAINDER 0xFFFFFFFF

typedef unsigned long  crc;

crc   crcFast(unsigned char const message[], int nBytes);

#endif /* _crc_h */