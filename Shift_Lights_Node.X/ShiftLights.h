/*
 * @file: ShiftLights.h
 *
 * @processor: PIC18F46K80
 * @compiler: Microchip C18
 * @author: Andrew Mass
 * @author: George Schwieters
 */
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

/*****************
 * Address Bytes *
 *****************/

#define TERM_LAT	LATCbits.LATC6

/**************
 * Data Bytes *
 **************/

// Colors
#define NONE       0b00000000
#define RED        0b01001000
#define GREEN      0b00100100
#define BLUE       0b00010010
#define RGB        0b01111110
#define RED_GREEN  0b01101100
#define RED_BLUE   0b01011010
#define GREEN_BLUE 0b00110110

// Rev range
#define REV_RANGE_1     7500
#define REV_RANGE_2     8500
#define REV_RANGE_3     9500
#define REV_RANGE_4     10500
#define REV_RANGE_5     11000
#define REV_RANGE_6     11500
#define REV_RANGE_7     12000
#define REV_RANGE_8     12250
#define REV_RANGE_LIMIT 12500

// Custom color choices
#define REV_COLOR       GREEN_BLUE
#define REV_LIMIT_COLOR GREEN

/******************
 * Method Headers *
 ******************/

void high_isr(void);
void low_isr(void);
void init_unused_pins(void);

#endif
