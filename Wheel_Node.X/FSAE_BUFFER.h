/**
 * FSAE BUFFER Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Nathan Cueto
 * Created:     2019-2020
 */

#ifndef _FSAE_BUFFER_H    
#define _FSAE_BUFFER_H

#include <xc.h>
#include <sys/types.h>
#include "RA8875_driver.h"

#define RINGSIZE 128

/*
 * Struct that will define type of buf. This allows for better flexibility
 * Buffer can be priority (=1) or not (=0). Buffer will have data, a read, and a write ptr.
 * 
 */
typedef struct packed {
  uint8_t data[SIZE];
  uint8_t read_ptr;
  uint8_t write_ptr;
  uint8_t priority;
} buffer;

/*
 * Defines the command struct that will be used in and around the ring buffer
 * msg_type - define what kind of message: e.g. drawLine, etc.
 * x -    X coordinate
 * y -    Y coordinate
 * size -   Size of item
 */
typedef struct packed {
  msg_type_enum msg_type;
  union packed {
    cmd1_struct cmd1;
    cmd2_struct cmd2;
  } msg;
  uint16_t x;
  uint16_t y;
  uint16_t size;
} cmd_struct;

/* Defines types of commands for use in the above struct
 *  Please add commands here and assign them to number values
 *  Also, depending on number of commands, change to uint16_t
 *
 */
typedef enum uint8_t {
  redrawDigit           = 0;
  redrawGearPos         = 1;
  redrawFanSw           = 2;
  redrawFUELPumpSw      = 3;
  redrawWTRPumpSw       = 4;
  redrawGCMMode         = 5;
  redrawTireTemp        = 6;
  redrawSPBar           = 7;
  redrawBrakeBar        = 8;
  redrawRotary          = 9;
  redrawShiftLightsRPM  = 10;
  redrawKILLCluster     = 11;
  redrawGforceGraph     = 12;
} msg_type_enum;

/*  Defines the specific command structs for use in the cmd_struct.
 *  This allows for varying data types from different functions(msg_type)
 *  
 *
 *  Padding might be implicit in C, so maybe its not needed. Double check that if the union fails
 */
typedef struct packed {
  // logic [38:0]             padding;
  // logic [7:0]              stuff;
  // logic [7:0]              things;
} cmd1_struct;



#endif /* _FSAE_BUFFER_H */