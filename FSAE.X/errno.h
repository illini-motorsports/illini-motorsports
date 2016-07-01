/**
 * Error Number Header
 *
 * Processor:   PIC32MZ2048EFM
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */

#ifndef ERRNO_H
#define ERRNO_H

#include "FSAE.h"
#include "CAN.h"
#include "../ECAN.X/ECAN.h"

#define NUM_ERR 33

/**
 * Error Codes
 */

#define ERR_NONE               0   // No error

#define ERR_PDL_LOWVOLT        20  // Low voltage neutral shift prevented
#define ERR_PDL_LOWERVOLT      21  // Lower voltage shift prevented
#define ERR_PDL_KILLSW         22  // Prevented shifting when kill switch pressed
#define ERR_PDL_UNDERREV       23  // Prevented shifting that would stall engine
#define ERR_PDL_OVERREV        24  // Prevented shifting that would overrev engine
#define ERR_PDL_SHIFTPAST      25  // Prevented shifting past 1st or 6th gear
#define ERR_PDL_BADNEUT        26  // Prevented neutral shift from not 1st/2nd
#define ERR_PDL_MAXRETRY       27  // Exceeded maximum number of retry shifts
#define ERR_PDL_NOCAN          28  // CAN state variables are out-of-date
#define ERR_PDL_LOCKOUT        29  // Shift queue prevented due to lockout timer
#define ERR_PDL_MAXDUR         30  // Shift ended due to maximum duration timer
#define ERR_PDL_MISSNEUT       31  // Neutral shift missed neutral gear
#define ERR_PDL_GEARFAIL       32  // Gear sensor reading invalid

// Defined in errno.c
//TODO: Modify this for xc32 compiler?
extern const rom char* errno_msg[NUM_ERR];

// Function definitions
void send_errno_CAN_msg(uint16_t origin_id, uint16_t errno);

#endif /* ERRNO_H */
