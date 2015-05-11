/******************************************************************************
 * Telemetry Node C Main Code Header
 ******************************************************************************
 * FileName:        ShiftLights.h
 * Dependencies:    none
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 *******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

// Location of channel data in message array
#define OIL_T       0
#define ENGINE_T    2
#define VOLTAGE     4
#define OIL_P       6
#define SPEED       8
#define RPM         10

#define DATA_PER    10

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
void write(unsigned char data);
void send_msg(void);
void send(unsigned char num);
void bufferData(void);

#endif
