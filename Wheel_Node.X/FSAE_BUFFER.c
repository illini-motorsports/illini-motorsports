/**
 * FSAE Ring Buffer
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Nathan Cueto
 * Created:     2019-2020
 */
// #include "RA8875_driver.h"
// #include "Wheel.h"
#include "FSAE_BUFFER.h"
#include <stdlib.h>
#include "Wheel.h"
// #define RINGSIZE 128
//This is max size for dataitem, which is from PDM dataitem.
#define RINGSIZE 7 //DO NOT move this to the .h file
#define MAXDATASIZE 93
#define MAXWAIT 100  //in cycles,TODO: fine tune this later
//Lower level functions

/* init
 * initialize the buffer. allocate memory as necessary
 * Buffers can be priority, but its up to Diablo side to grab from priority buffer first
 * so the priority value is kind of redundant for the time being...
 */
void init_buffer(buffer * initBuf, uint16_t pri) {
// 	if(initBuf == NULL) {
// //		printf("NULL in init_buffer"); //print error msg
// 		return;
// 	}
	initBuf->data = malloc(RINGSIZE * sizeof(cmd_struct)); //verify the sizeof
  initBuf->priority = pri;
  initBuf->read_ptr = 0;
  initBuf->write_ptr = 0;
            
}

/* free
 * free the buffer. might not ever get used since car is shutdown often
 * 
 */
void free_buffer(buffer * killBuf) {
	// free(killBuf->data);
  // free(killBuf);
}
  
/* push
 * will push some cmd_struct into the Ring buffer
 * and then adjust write_ptr
 */
void push(cmd_struct command, buffer * buf_ptr)
{
  buf_ptr->data[buf_ptr->write_ptr] = command; //might need to individually set fields equal
  buf_ptr->write_ptr = (buf_ptr->write_ptr + 1) % RINGSIZE;
}
  
/* pop
 * will pop some cmd_struct from the Ring buffer
 * then adjust read_ptr,
 * and also return whatever was popped
 */
cmd_struct pop(buffer * buf_ptr)
{
  cmd_struct ret_command;
  ret_command = buf_ptr->data[buf_ptr->read_ptr];
  buf_ptr->read_ptr = (buf_ptr->read_ptr+1) % RINGSIZE; //TODO was previously MAXDATASIZE
  if(ret_command.msg_type == dredrawFUELPumpSw)
  {
            //     CAN_data data = {0};
            // data.halfword0 = 69;
            // data.halfword1 = 69;
            // data.halfword2 = 69;
            // CAN_send_message(0x400 + 0, 6, data);
  }
  return ret_command;
}
  
/* empty
 * check if the ring buffer is empty.
 * return bool
 */
uint16_t empty(buffer * buf_ptr)
{
	if(buf_ptr->read_ptr == buf_ptr->write_ptr)
        return 1;
    else
        return 0;
}

/* full
 * check if the ring buffer is full.
 * return bool
 */
uint16_t full(buffer * buf_ptr)
{
	//i think this is incorrect, say we push 1 thing into an empty buffer. then full will be true.
	//should probably be switched. if write is right behind read.
	if(buf_ptr->write_ptr == (buf_ptr->read_ptr - 1)){ // might be off by 1, needs to account for rollover
        

            return 1;
    }
    else
        return 0;
}

//Mid level functions 
//TODO implement priority handling

/* blocking push
 * Will attempt to push a cmd_struct
 * Will not push if it is full. If full, then wait until empty. This is a blocking funciton.
 * Function should be blocking, so don't use fork
 * TODO maybe future implement checking if a function belongs in priority or not
 * return: 0 if success, 1 on failure
 */
uint16_t blocking_push(cmd_struct command, buffer * buf_ptr)
{
  uint16_t cycles = 0;
  while(cycles < MAXWAIT)  { 
  	if(!full(buf_ptr)) {
    	push(command, buf_ptr);
          // CAN_data data = {0};
          //   data.halfword0 = 69;
          //   data.halfword1 = 69;
          //   data.halfword2 = 69;
//            CAN_send_message(0x400 + 0, 6, data);
    	return 0;
    }
    cycles++;
    
  }
  return 1;
}
  
/* blocking pop
 * Will attempt to pop a cmd_struct
 * Will not pop if it is empty. If empty, then wait until somthing put in. returns popped item, also blocking
 * Function should be blocking, so don't use fork
 * IMPORTANT: when handling popped structs on either PIC or Diablo, check msg_type first!!!
 * TODO consider having an actual timeout
 */
cmd_struct blocking_pop(buffer * buf_ptr)
{
  uint16_t cycles = 0;
  while(cycles < MAXWAIT)  {
  	if(!empty(buf_ptr)) {
    	return pop(buf_ptr);
	 }
   cycles++;
  }
  //return empty struct in this case
  cmd_struct cmd_data; //type fuel, doesnt really matter though
  cmd_data.msg_type = dstuckEmpty;
  return cmd_data;
}

