/******************************************************************************
 *
 *                  Thermocouple Reader C Main Code Header
 *
 ******************************************************************************
 * FileName:        Thermo.h
 * Dependencies:    none
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18

 *******************************************************************************
    USER REVISON HISTORY
//
//
//

 *******************************************************************************/

#ifndef THERMO_H
#define THERMO_H


/***********************************************/
/*  User Structures & Defines                  */
/***********************************************/

//#define INTERNAL
//#define DEBUGGING
#define INPUT   1
#define OUTPUT  0

// emission period
// must be multiple of 2 for consistent sampling
#define SAMPLE 512

// define IDs and DLCs
#define DLC_0 8
#define DLC_1 2
#define ID_0 0x250L
#define ID_1 0x251L

// setup chip selects for maxim thermocouple chips
#define CS0 LATAbits.LATA0
#define CS1 LATAbits.LATA1
#define CS2 LATAbits.LATA2
#define CS3 LATAbits.LATA3
#define CS4 LATAbits.LATA5


/***********************************************/
/*  User Function Prototypes                   */
/***********************************************/

void init_unused_pins(void);
void high_isr(void);
void sample(BYTE *data, const BYTE byte);

#endif
