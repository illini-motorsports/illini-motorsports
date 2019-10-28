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

//Lower level functions

/* init
 * initialize the buffer. allocate memory as necessary
 * For now, input is only priority. consider using custom buffer sizes
 */
void init_buffer(buffer * initBuf, uint8_t pri) {
	if(initBuf == NULL) {
		printf("NULL in init_buffer");
		return;
	}
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
	free(killBuf->data);
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
  cmd_struct * ret_command;
  ret_command = buf_ptr->data[buf_ptr->read_ptr];
  buf_ptr->read_ptr = (buf_ptr->read_ptr+1) % MAXDATASIZE;
  return ret_command;
}
  
/* empty
 * check if the ring buffer is empty.
 * return bool
 */
uint8_t empty(buffer * buf_ptr)
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
uint8_t full(buffer * buf_ptr)
{
	//i think this is incorrect, say we push 1 thing into an empty buffer. then full will be true.
	//should probably be switched. if write is right behind read.
	if(buf_ptr->write_ptr == (buf_ptr->read_ptr - 1)) // might be off by 1, needs to account for rollover
        return 1;
    else
        return 0;
}

//Mid level functions

/* blocking push
 * Will attempt to push a cmd_struct
 * Will not push if it is full. If full, then wait until empty. This is a blocking funciton.
 */
void blocking_push(cmd_struct command, buffer * buf_ptr)
{
  while(1)  {
  	if(!full(buf_ptr)) {
    	push(command, buf_ptr);
    	return;
    }
  }
}
  
/* blocking pop
 * Will attempt to pop a cmd_struct
 * Will not pop if it is empty. If empty, then wait until somthing put in. returns popped item, also blocking
 */
cmd_struct * blocking_pop(buffer * buf_ptr)
{
  while(1)  {
  	if(!empty(buf_ptr)) {
    	return pop(buf_ptr);
	 }
  }
}

