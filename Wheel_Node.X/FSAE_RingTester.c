/**
 * RING BUFFER TESTING 
 * Author:      Nathan Cueto
 * Created:     2019-2020
 */

#include "FSAE_BUFFER.h"
#include "FSAE_BUFFER.c"
// extern cmd_struct blocking_pop(buffer * buf_ptr);
// extern void blocking_push(cmd_struct command, buffer * buf_ptr);

/* Test function space
 * circumsize the function, only testing buffer stuff
 *
 */
int redrawFUELPumpSw(int value, buffer * buf_ptr){
  // Override
  cmd_struct_fuel cmd_data;
  if(value==0){
    cmd_data.colorC = 1;
  }
  // FUEL On, Switch Not toggled
  else if(value==1){
    cmd_data.colorC = 2;
  }
  // Load off, switch off
  else{
    cmd_data.colorC = 3;
  }

  //Create cmd struct using function inputs
  cmd_struct rFUEL;
  rFUEL.msg_type = dredrawFUELPumpSw;
  rFUEL.cmd_fuel = cmd_data;
  rFUEL.x = 10;
  rFUEL.y = 20;
  rFUEL.size = 30;

  //Pass this struct into buffer, choose priority or not
  blocking_push(rFUEL, buf_ptr);
  return value;
}


int main(void) {
  /* 
   * TEST 1: Simple Push and Pop Test
   */
    // Initilize a buffer
  buffer * testBuf;
  init_buffer(testBuf, 0);
  /* Call redraw function, which will push a cmd struct onto buffer
   * These redraw calls will determine priority by calling a redraw with a priority or nonpriority buffer
   */
  redrawFUELPumpSw(0, testBuf);
  //pop, make sure its same data and is correct
  cmd_struct poppedStruct;
  poppedStruct = blocking_pop(testBuf);
  if(poppedStruct.msg_type == dredrawFUELPumpSw)
    if(poppedStruct.cmd_fuel.colorC == 1)
      return 0; //to enable further tests, replace this with break;
    else
      return 1;
  else 
    return 1; 

  /* 
    TEST 2: Multi Push, Multi Pop Test
  */
  //  init_buffer();

  //various redraw calls

  //pop the same amount of times we pushed, assure in order and correct

  /* 
    TEST 3: Full and empty Test
  */
  // init_buffer();
  //some redraw calls

  //pop more than pushed

  //On empty buffer, fill it to the max

  //try to push when buffer is full

  /*
        TEST 4: Priority Test
  */
  //Initialize priority and non-priority buffers
  //  init_buffer();
  //  init_buffer();

  //redraw calls for both buffers, try threading this??

  //pop from either buffer, try threading this??

  //print out when a buffer gets or returns, make sure priority is done first
  return 0;
}

