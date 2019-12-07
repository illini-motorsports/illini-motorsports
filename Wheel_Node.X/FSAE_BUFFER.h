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

// #include <xc.h>
#include <sys/types.h>
// #include "RA8875_driver.h"

//This is max size for dataitem, which is from PDM dataitem.
#define MAXDATASIZE 93


/* Defines types of commands for use in the above struct
 *  Please add commands here and assign them to number values
 *  Also, depending on number of commands, change to int
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
  dredrawGforceGraph     , 
  dstuckEmpty             //Diablo side will handle stuck
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
  int fillW;         //size of rectangle, height is 1.75xWidth
  int colorFill;
  int colorNumber;   //color values, TODO map these out sometime
  int numActual;    //Value to be printed, if Diablo can do that. will make the following two vars reduntant
  int numWhole;    //Whole digits before decimal point
  int numDecimal;  //digits after decimal point
  int drawNumber;   //decides if number should be redrawn (=1 is true)
} cmd_struct_digit;

/* For this instruciton, be sure to check the value of gear for Neutral or Error
*/
typedef struct __attribute__((packed)) cmd_struct_gear{
  int colorBG; //Diablo can draw the digit as not the background color
  int gear;
} cmd_struct_gear;

typedef struct __attribute__((packed)) cmd_struct_fan{
  int colorC;
} cmd_struct_fan;

typedef struct __attribute__((packed)) cmd_struct_fuel{
  int colorC;
} cmd_struct_fuel;

typedef struct __attribute__((packed)) cmd_struct_wtr{
  int colorC;
} cmd_struct_wtr;

typedef struct __attribute__((packed)) cmd_struct_gcm{
  int colorC;
} cmd_struct_gcm;

typedef struct __attribute__((packed)) cmd_struct_tire{
  int color0;
  int color1;
  int color2;
  int color3;
  int width;
  int height;
} cmd_struct_tire;

typedef struct __attribute__((packed)) cmd_struct_sp{
  int height;
  int colorC;
  int colorBG;
} cmd_struct_sp;

typedef struct __attribute__((packed)) cmd_struct_brake{
  int brake_press; //this is a height
  int height;
  int colorC;
  int colorBG; //background colors may be redundant but just in case, including them anyway
} cmd_struct_brake;

typedef struct __attribute__((packed)) cmd_struct_rotary{
  int val;
  int colorC;
} cmd_struct_rotary;

typedef struct __attribute__((packed)) cmd_struct_shift{

} cmd_struct_shift;

typedef struct __attribute__((packed)) cmd_struct_kill{

} cmd_struct_kill;

typedef struct __attribute__((packed)) cmd_struct_gforce{
  int latsnap;
  int longsnap;
  int maxG;
  int maxRadius;
  int radii[4];
  int colorBG;
  int colorFG;
  int colorFG2; //foreground, and foreground 2 color, followed by error color
  int colorE;
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
  int x;
  int y;
  int size;
} cmd_struct;

/*
 * Struct that will define type of buf. This allows for better flexibility
 * Buffer can be priority (=1) or not (=0). Buffer will have data, a read, and a write ptr.
 * 
 */
typedef struct __attribute__((packed)) buffer {
  cmd_struct * data;
  int read_ptr;
  int write_ptr;
  int priority;
} buffer;

#endif /* _FSAE_BUFFER_H */