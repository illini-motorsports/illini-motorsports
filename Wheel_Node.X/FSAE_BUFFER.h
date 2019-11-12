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

/*  Defines the specific command structs for use in the cmd_struct union.
 *  This allows for varying data types from different functions( aka. msg_type)
 *  Each redraw function will utilize a cmd struct.
 *  Populate each of the following command structs with the important data that each redraw function uses.
 *  x, y, and size are in the cmd_struct, but not a part of the union.
 *  Padding might be implicit in C, so maybe its not needed. Double check that if the union fails
 */
typedef struct __attribute__((packed)) cmd_struct_digit{
  //packed[];
  uint16_t fillW;         //size of rectangle, height is 1.75xWidth
  uint16_t colorFill;
  uint16_t colorNumber;   //color values, TODO map these out sometime
  uint16_t numActual;    //Value to be printed, if Diablo can do that. will make the following two vars reduntant
  uint16_t numWhole;    //Whole digits before decimal point
  uint16_t numDecimal;  //digits after decimal point
  uint8_t drawNumber;   //decides if number should be redrawn (=1 is true)
} cmd_struct_digit;

/* For this instruciton, be sure to check the value of gear for Neutral or Error
*/
typedef struct __attribute__((packed)) cmd_struct_gear{
  uint16_t colorBG; //Diablo can draw the digit as not the background color
  uint8_t gear;
} cmd_struct_gear;

typedef struct __attribute__((packed)) cmd_struct_fan{
  uint16_t colorC;
} cmd_struct_fan;

typedef struct __attribute__((packed)) cmd_struct_fuel{
  uint16_t colorC;
} cmd_struct_fuel;

typedef struct __attribute__((packed)) cmd_struct_wtr{
  uint16_t colorC;
} cmd_struct_wtr;

typedef struct __attribute__((packed)) cmd_struct_gcm{

} cmd_struct_gcm;

typedef struct __attribute__((packed)) cmd_struct_tire{
  uint16_t color0;
  uint16_t color1;
  uint16_t color2;
  uint16_t color3;
  uint16_t width;
  uint16_t height;
} cmd_struct_tire;

typedef struct __attribute__((packed)) cmd_struct_sp{
  uint16_t height;
  uint16_t colorC;
  uint16_t colorBG;
} cmd_struct_sp;

typedef struct __attribute__((packed)) cmd_struct_brake{
  uint16_t brake_press; //this is a height
  uint16_t colorC;
  uint16_t colorBG; //background colors may be redundant but just in case, including them anyway
} cmd_struct_brake;

typedef struct __attribute__((packed)) cmd_struct_rotary{
  uint16_t val;
  uint16_t colorC;
} cmd_struct_rotary;

typedef struct __attribute__((packed)) cmd_struct_shift{
  uint16_t latsnap;
  uint16_t longsnap;
  uint16_t maxG;
  uint16_t maxRadius;
  uint16_t radii[4];
  uint16_t colorBG;
  uint16_t colorFG;
  uint16_t colorFG2; //foreground, and foreground 2 color, followed by error color
  uint16_t colorE;
} cmd_struct_shift;

typedef struct __attribute__((packed)) cmd_struct_kill{

} cmd_struct_kill;

typedef struct __attribute__((packed)) cmd_struct_gforce{

} cmd_struct_gforce;

/*
 * Defines the command struct that will be used in and around the ring buffer
 * msg_type - define what kind of message: e.g. drawLine, etc.
 * x -    X coordinate
 * y -    Y coordinate
 * size -   Size of item
 * using an anonymous union with no name, which allows access as if they are normal struct members
 * Union allows different types of data to be store here, since each redraw funciton 
 * is different aside from the basic x,y,size. However, we can only use one item from the union at a time.
 */
typedef struct __attribute__((packed)) cmd_struct {
  msg_type_enum msg_type;
  union __attribute__((packed)) {
    cmd_struct_digit  cmd_digit;
    cmd_struct_gear   cmd_gear;
    cmd_struct_fan    cmd_fan;
    cmd_struct_fuel   cmd_fuel;
    cmd_struct_wtr    cmd_wtr;
    cmd_struct_gcm    cmd_gcm;
    cmd_struct_tire   cmd_tire;
    cmd_struct_sp     cmd_sp;
    cmd_struct_brake  cmd_brake;
    cmd_struct_rotary cmd_rotary;
    cmd_struct_shift  cmd_shift;
    cmd_struct_kill   cmd_kill;
    cmd_struct_gforce cmd_gforce;
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