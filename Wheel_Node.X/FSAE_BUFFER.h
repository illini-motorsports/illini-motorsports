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
//This is max size for dataitem, which is from PDM dataitem.
#define MAXDATASIZE 93

/* Defines types of commands for use in the above struct
 *  Please add commands here and assign them to number values
 *  Also, depending on number of commands, change to uint16_t
 *
 */
typedef enum msg_type_enum {
  dredrawDigit           ,
  dredrawGearPos         ,
  dredrawFanSw           ,
  dredrawFUELPumpSw      ,
  dredrawWTRPumpSw       ,
  dredrawGCMMode         ,
  dredrawTireTemp        ,
  dredrawSPBar           ,
  dredrawBrakeBar        ,
  dredrawRotary          ,
  dredrawShiftLightsRPM  ,
  dredrawKILLCluster     ,
  dredrawGforceGraph     
} msg_type_enum;

/*  Defines the specific command structs for use in the cmd_struct.
 *  This allows for varying data types from different functions(msg_type)
 *  
 *
 *  Padding might be implicit in C, so maybe its not needed. Double check that if the union fails
 */
typedef struct __attribute__((packed)) cmd1_struct{
  // logic [38:0]             padding;
  // logic [7:0]              stuff;
  // logic [7:0]              things;
} cmd1_struct;

typedef struct __attribute__((packed)) cmd2_struct{
  // logic [38:0]             padding;
  // logic [7:0]              stuff;
  // logic [7:0]              things;
} cmd2_struct;


/*
 * Defines the command struct that will be used in and around the ring buffer
 * msg_type - define what kind of message: e.g. drawLine, etc.
 * x -    X coordinate
 * y -    Y coordinate
 * size -   Size of item
 * using an anonymous union with no name, which allows access as if they are normal struct members
 * Union allows different types of data to be store here, since each redraw funciton 
 * is different aside from the basic x,y,size
 */
typedef struct __attribute__((packed)) cmd_struct {
  msg_type_enum msg_type;
  union __attribute__((packed)) {
    cmd1_struct cmd1;
    cmd2_struct cmd2;
  };
  uint16_t x;
  uint16_t y;
  uint16_t size;
} cmd_struct;

/*
 * Struct that will define type of buf. This allows for better flexibility
 * Buffer can be priority (=1) or not (=0). Buffer will have data, a read, and a write ptr.
 * 
 */
typedef struct __attribute__((packed)) buffer {
  cmd_struct * data;
  uint8_t read_ptr;
  uint8_t write_ptr;
  uint8_t priority;
} buffer;

#endif /* _FSAE_BUFFER_H */