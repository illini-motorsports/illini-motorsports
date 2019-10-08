/**
 * FSAE Ring Buffer
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Nathan Cueto
 * Created:     2019-2020
 */
#include "RA8875_driver.h"
#include "Wheel.h"
#include "FSAE_BUFFER.h"

// Macro definitions
#define RINGSIZE = 128

//global variables
uint8_t BUFF[SIZE];

uint8_t read_ptr;
uint8_t write_ptr;

//Lower level functions
  
/* push
 * will push some cmd_struct into the Ring buffer
 * and then adjust write_ptr
 */
void push(cmd_struct command)
{
  BUFF[write_ptr] = command;
  write_ptr = (write_ptr + 1) % RINGSIZE;
}
  
/* pop
 * will pop some cmd_struct from the Ring buffer
 * then adjust read_ptr,
 * and also return whatever was popped
 */
cmd_struct pop()
{
  cmd_struct ret_command = BUFF[read_ptr];
  read_ptr = (read_ptr+1) % SIZE;
  return ret_command;
}
  
/* empty
 * check if the ring buffer is empty.
 * return bool
 */
bool empty()
{
	return (read_ptr == write_ptr);
}

/* full
 * check if the ring buffer is full.
 * return bool
 */
bool full()
{
	//i think this is incorrect, say we push 1 thing into an empty buffer. then full will be true.
	return (read_ptr == (write_ptr - 1)); // might be off by 1, needs to account for rollover
}

//Mid level functions

/* blocking push
 * Will attempt to push a cmd_struct
 * Will not push if it is full. If full, then wait until empty. This is a blocking funciton.
 */
void blocking_push(cmd_struct command)
{
  while(1)  {
  	if(!full()) {
    	push(command);
    	return;
    }
  }
}
  
/* blocking pop
 * Will attempt to pop a cmd_struct
 * Will not pop if it is empty. If empty, then wait until somthing put in. returns popped item, also blocking
 */
uint8_t blocking_pop()
{
  while(1)  {
  	if(!empty()) {
    	return pop();
	}
  }
}

