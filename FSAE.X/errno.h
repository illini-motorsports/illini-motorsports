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

#include <sys/types.h>
#include "CAN.h"
#include "FSAE_can.h"

#define NUM_ERR 33

/**
 * Error Codes
 */

#define ERR_NONE               0   // No error

#define ERR_GCM_LOWVOLT        20  // Low voltage neutral shift prevented
#define ERR_GCM_LOWERVOLT      21  // Lower voltage shift prevented
#define ERR_GCM_KILLSW         22  // Prevented shifting when kill switch pressed
#define ERR_GCM_UNDERREV       23  // Prevented shifting that would stall engine
#define ERR_GCM_OVERREV        24  // Prevented shifting that would overrev engine
#define ERR_GCM_SHIFTPAST      25  // Prevented shifting past 1st or 6th gear
#define ERR_GCM_BADNEUT        26  // Prevented neutral shift from not 1st/2nd
#define ERR_GCM_MAXRETRY       27  // Exceeded maximum number of retry shifts
#define ERR_GCM_NOCAN          28  // CAN state variables are out-of-date
#define ERR_GCM_LOCKOUT        29  // Shift queue prevented due to lockout timer
#define ERR_GCM_MAXDUR         30  // Shift ended due to maximum duration timer
#define ERR_GCM_MISSNEUT       31  // Neutral shift missed neutral gear
#define ERR_GCM_GEARFAIL       32  // Gear sensor reading invalid

// Defined in errno.c
extern const char* errno_msg[NUM_ERR];

// Function definitions
void send_errno_CAN_msg(uint16_t origin_id, uint16_t errno);

#endif /* ERRNO_H */
