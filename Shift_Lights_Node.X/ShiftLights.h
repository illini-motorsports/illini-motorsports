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

#define INPUT  1
#define OUTPUT 0

#define BLINK_TIME 150
#define ET_ID      0x201L
#define OT_ID      0x200L
#define OP_ID      0x200L
#define RPM_ID     0x200L

#define OP_BYTE  4
#define OT_BYTE  6
#define ET_BYTE  0
#define RPM_BYTE 0

/*****************
 * Address Bytes *
 *****************/

#define TERM_LAT	 LATCbits.LATC6

#define RED0_LAT   LATAbits.LATA2
#define GREEN0_LAT LATAbits.LATA1
#define BLUE0_LAT  LATAbits.LATA0

#define RED1_LAT   LATDbits.LATD0
#define GREEN1_LAT LATCbits.LATC3
#define BLUE1_LAT  LATCbits.LATC2

#define RED2_LAT   LATDbits.LATD3
#define GREEN2_LAT LATDbits.LATD2
#define BLUE2_LAT  LATDbits.LATD1

#define RED3_LAT   LATCbits.LATC7
#define GREEN3_LAT LATCbits.LATC5
#define BLUE3_LAT  LATCbits.LATC4

#define RED4_LAT   LATDbits.LATD6
#define GREEN4_LAT LATDbits.LATD5
#define BLUE4_LAT  LATDbits.LATD4

#define RED0_TRIS   TRISAbits.TRISA2
#define GREEN0_TRIS TRISAbits.TRISA1
#define BLUE0_TRIS  TRISAbits.TRISA0

#define RED1_TRIS   TRISDbits.TRISD0
#define GREEN1_TRIS TRISCbits.TRISC3
#define BLUE1_TRIS  TRISCbits.TRISC2

#define RED2_TRIS   TRISDbits.TRISD3
#define GREEN2_TRIS TRISDbits.TRISD2
#define BLUE2_TRIS  TRISDbits.TRISD1

#define RED3_TRIS   TRISCbits.TRISC7
#define GREEN3_TRIS TRISCbits.TRISC5
#define BLUE3_TRIS  TRISCbits.TRISC4

#define RED4_TRIS   TRISDbits.TRISD6
#define GREEN4_TRIS TRISDbits.TRISD5
#define BLUE4_TRIS  TRISDbits.TRISD4

/**************
 * Data Bytes *
 **************/

// Colors
#define NONE       0b000
#define RED        0b001
#define GREEN      0b010
#define BLUE       0b100
#define RED_GREEN  RED | GREEN
#define RED_BLUE   RED | BLUE
#define GREEN_BLUE GREEN | BLUE
#define RGB        RED | GREEN | BLUE

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
void setLedToColor(unsigned char led, unsigned char color);
void set_all(unsigned char color);
void set_lights(unsigned char max);
void init_unused_pins();

#endif
