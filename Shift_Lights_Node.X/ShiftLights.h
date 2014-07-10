/******************************************************************************
 *
 *					Shift Lights Node C Main Code Header
 *
 ******************************************************************************
 * FileName:        ShiftLights.h
 * Dependencies:    none
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          Andrew Mass, George Schwieters
 ******************************************************************************
 * Revision History
 *
 * 9-12-2013: AMass - Cleaned the file and converted tabs to spaces.
*******************************************************************************/

#ifndef MAIN_H
#define MAIN_H

#include "GenericTypeDefs.h"

#define INPUT   1
#define OUTPUT  0

#define BLINK_TIME 150
#define ET_ID 0x201L
#define OT_ID 0x200L
#define OP_ID 0x200L
#define RPM_ID 0x200L

#define OP_BYTE 4
#define OT_BYTE 6
#define ET_BYTE 0
#define RPM_BYTE 0

/*********************
 * Address Bytes     *
 *********************/

#define NO_OP       0x00
#define DIG0        0x01
#define DIG1        0x02
#define DIG2        0x03
#define DIG3        0x04
#define DIG4        0x05
#define DIG5        0x06
#define DIG6        0x07
#define DIG7        0x08
#define DECODE      0x09
#define INTENSITY   0x0A
#define SCAN        0x0B
#define SHUTDOWN    0x0C
#define DISP_MODE   0x0F

// Chip Select
#define CS          LATDbits.LATD3

#define TERM_LAT	LATCbits.LATC6


/*********************
 * Data Bytes        *
 *********************/

// Config
#define SHUTDOWN_ON         0b00000000
#define SHUTDOWN_OFF        0b00000001
#define NO_DECODE           0b00000000
#define FULL_SCAN           0b00000111

// Intensity
#define INTENSITY_MIN       0b00000001
#define INTENSITY_MAX       0b00001111

// Display mode
#define TEST                0b00000001
#define NORMAL              0b00000000

// Colors
#define NONE                0b00000000
#define RED                 0b01001000
#define GREEN               0b00100100
#define BLUE                0b00010010
#define RGB                 0b01111110
#define RED_GREEN           0b01101100
#define RED_BLUE            0b01011010
#define GREEN_BLUE          0b00110110

// Rev range
#define REV_RANGE_1         7500
#define REV_RANGE_2         8500
#define REV_RANGE_3         9500
#define REV_RANGE_4         10500
#define REV_RANGE_5         11000
#define REV_RANGE_6         11500
#define REV_RANGE_7         12000
#define REV_RANGE_8         12250
#define REV_RANGE_LIMIT     12500

// Custom color choices
#define REV_COLOR           GREEN_BLUE
#define REV_LIMIT_COLOR     GREEN


/*********************
 * Method Headers    *
 *********************/

void high_isr(void);
void low_isr (void);
void init_unused_pins(void);

#endif
