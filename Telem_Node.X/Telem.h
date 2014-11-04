/******************************************************************************
 *
 *                  Telemetry Node C Main Code Header
 *
 ******************************************************************************
 * FileName:        ShiftLights.h
 * Dependencies:    none
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 ******************************************************************************
 * Revision History
 *
 *
 *******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "GenericTypeDefs.h"

#define INPUT   1
#define OUTPUT  0

// location of channel data in message array
#define OIL_T       0
#define ENGINE_T    2
#define VOLTAGE     4
#define OIL_P       6
#define SPEED       8
#define RPM         10
// channel data identifiers (addr)
#define OT      0x11
#define ET      0x22
#define VOLT    0x33
#define OP      0x44
#define SPD     0x55
#define TACH    0x66

#define NUM_MSG     6

#define DATA        0x01
#define ADDR        0x02
#define ERROR       0x03

#define SIGNAL      0
#define CAN         1

#define DATA_PER    10
#define ADDR_PER    100
#define ERROR_PER   100

#define ECU_ID_0    0x200
#define ECU_ID_1    0x201
#define ECU_ID_2    0x202
#define ECU_ID_3    0x203
#define ECU_ID_4    0x204

#define GDN_SPD_BYTE    0
#define OIL_T_BYTE      6
#define ENGINE_T_BYTE   0
#define VOLTAGE_BYTE    6
#define OIL_P_BYTE      4
#define RPM_BYTE        0

#define TERM_LAT    LATCbits.LATC6

/*********************
 * Method Headers    *
 *********************/

void high_isr(void);
void write(BYTE data);
void send_msg(BYTE type,unsigned int errorType);
void send(BYTE num);
void bufferData(void);

#endif
