/**
 * RING BUFFER TESTER
 * Author:      Nathan Cueto
 * Created:     2019-2020
 * Note: Not to be used on PIC! For standalone use on unix only along with Buffer.c and h
 */

#include "FSAE_BUFFER.h"
#include "FSAE_BUFFER.c"
// extern cmd_struct blocking_pop(buffer * buf_ptr);
// extern void blocking_push(cmd_struct command, buffer * buf_ptr);

/*TODO: Create FSAE_RingTester.h and create an enum for error types
 * Error codes listed here, can be viewed when running suite on gdb:
 * 1 : push/pop operation failure
 * 2 : cmd_struct correctness failure
 * 3 : buffer empty failure
 * 4 : buffer full failure
 */

/* Test function space
 * circumsize the function, only testing buffer stuff
 * testing redrrawFUEL because its simple, and gforce which is most complex
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
  //also return error code if any
  return blocking_push(rFUEL, buf_ptr);;
}

double redrawGforceGraph(double currentValue, buffer * buf_ptr)
{
  int i = 0;
  double dataArray[2] = {1.0, 2.0};
          
  //constants for various markers representing different g values on the graph
  int maxRadius, radii[4], maxG;
    
  //take a "snapshot" of the gForce CAN message
  double lateralSnap, longitSnap;
  lateralSnap = dataArray[0];
  longitSnap = dataArray[1];
  //initialize constants
  maxRadius = 50;
  maxG = (-1 * -4.17464);

  cmd_struct_gforce cmd_data;
    
  //determine where to erase the previous dot by mapping currentValue to coordinates
  double lateral, longit;
  lateral = (int)currentValue % 10000;
  if(lateral > 999)
  {
    lateral -= 1000;
    lateral = lateral * -1;
  }
  lateral = lateral / 100;

  longit = (int)currentValue / 10000;
  if(longit > 999)
  {
    longit-=1000;
    longit = longit * -1;
  }
  longit = longit / 100;    
  //derive radii and draw axis lines
  for(i = 2; i >= 0; i--)
  {
      radii[i] = ((i + 1) * maxRadius) / maxG ;
      cmd_data.radii[i] = radii[i];
  }
  cmd_data.maxG = maxG;
  cmd_data.maxRadius = maxRadius;
  cmd_data.colorBG = 5;
  cmd_data.colorFG = 6;
  cmd_data.colorFG2 = 7;

    //map the horizontal and lateral G snapshots to a double
  lateralSnap = lateralSnap * 100;
  lateralSnap = (int) lateralSnap;
  if(lateralSnap < 0)
  {
    lateralSnap = lateralSnap * -1;
    lateralSnap += 1000;
  }
  longitSnap = longitSnap * 100;
  longitSnap = (int) longitSnap;
  longitSnap = longitSnap * 10000;
  if(longitSnap < 0)
  {
    longitSnap = longitSnap * -1;
    longitSnap += 10000000;
  }
  cmd_data.latsnap = lateralSnap;
  cmd_data.longsnap = longitSnap;

  cmd_struct rGforce;
  rGforce.msg_type = dredrawGforceGraph;
  rGforce.cmd_gforce = cmd_data;
  rGforce.x = 5;
  rGforce.y = 10;
  rGforce.size = 50;

  blocking_push(rGforce, buf_ptr);
  return longitSnap + lateralSnap;
}

int endTesting(int err, buffer * buf_ptr)
{
  free_buffer(buf_ptr);
  return err;
}

int main(void) {
  /* 
   * TEST 1: Simple Push and Pop Test
   */
    // Initilize a buffer
  buffer * testBuf = malloc(sizeof(testBuf));
  init_buffer(testBuf, 0);
  /* Call redraw function, which will push a cmd struct onto buffer
   * These redraw calls will determine priority by calling a redraw with a priority or nonpriority buffer
   */
  if (redrawFUELPumpSw(0, testBuf) != 0) //rets 0 if passed push, buf should be empty rn
    endTesting(3, testBuf);
  //pop, make sure its same data and is correct
  cmd_struct poppedStruct;
  poppedStruct = blocking_pop(testBuf);
  if(poppedStruct.msg_type != dredrawFUELPumpSw)
    endTesting(1, testBuf);
  if(poppedStruct.cmd_fuel.colorC != 1)
    endTesting(2, testBuf);

  /* 
    TEST 2: Multi Push, Multi Pop Test
  */
  //  init_buffer();

  //various redraw calls
  redrawFUELPumpSw(0, testBuf);
  redrawFUELPumpSw(1, testBuf);
  redrawGforceGraph(50, testBuf);
  redrawGforceGraph(50, testBuf);
  redrawFUELPumpSw(2, testBuf);
  //pop the same amount of times we pushed, assure in order and correct
  cmd_struct popm0;
  cmd_struct popm1;
  cmd_struct popm2;
  cmd_struct popm3;
  cmd_struct popm4;
  popm0 = blocking_pop(testBuf); //gets FUEL 0 
  popm1 = blocking_pop(testBuf); //gets FUEL 1
  popm2 = blocking_pop(testBuf); //gets Gforce 0
  popm3 = blocking_pop(testBuf); //gets Gforce 1
  popm4 = blocking_pop(testBuf); //gets FUEL 2
  if(popm0.msg_type != dredrawFUELPumpSw ||  popm1.msg_type != dredrawFUELPumpSw || popm4.msg_type != dredrawFUELPumpSw)
  {
    endTesting(1, testBuf);
  }
  if(popm2.msg_type != dredrawGforceGraph || popm3.msg_type != dredrawGforceGraph)
  {
    endTesting(1, testBuf);
  }

  /* 
    TEST 3: Full and empty Test
  */
  //buffer SHOULD be empty, try to pop
  cmd_struct poppedEmpty;
  poppedEmpty = blocking_pop(testBuf);
  if(poppedEmpty.msg_type != dstuckEmpty)
    endTesting(3, testBuf);
  //On empty buffer, fill it to the max (temp ring max is 6, so push 6 things)
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  if (redrawFUELPumpSw(0, testBuf) != 0)
    endTesting(4, testBuf);
  //try to push when buffer is full
  if (redrawFUELPumpSw(0, testBuf) != 1) //should fail push
    endTesting(4, testBuf);

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

