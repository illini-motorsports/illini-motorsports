#include "DRS.h"

volatile uint32_t CAN_recv_tmr = 0;

int main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(0);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_timer6(0x30D4);
  init_pwm(0x30D4,7); //Initialize pwm signal with given period
  init_pwm(0x30D4,8); //Initialize pwm signal with given period
  init_pwm(0x30D4,9); //Initialize pwm signal with given period
  init_can();
  
  millis = 0;
  periods = 0;
  enable = 0;
  
  while(1);
  
  return 0;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count
  //if (!(millis%250)){
  //    PIC_LED_LAT = !PIC_LED_LAT;
  //}
  
  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

void __attribute__((vector(_TIMER_4_VECTOR), interrupt(IPL7SRS))) timer4_inthnd(void) {
  periods++;// Increment periods count
  
  if (enable == 1){
      pwm_set(0x03E8,1);
      pwm_set(0x03E8,2);
      pwm_set(0x03E8,3);
  } else if (enable == 0){
      pwm_set(0x07D0,1);
      pwm_set(0x07D0,2);
      pwm_set(0x07D0,3);
  }
  
  /*
  if (periods == 100){
      pwm_set(0x03E8,1);
      pwm_set(0x03E8,2);
      pwm_set(0x03E8,3);
  } else if (periods == 200){
      pwm_set(0x07D0,1);
      pwm_set(0x07D0,2);
      pwm_set(0x07D0,3);
      periods = 0;
  }
   */
  
  IFS0CLR = _IFS0_T4IF_MASK;// Clear TMR4 Interrupt Flag
}

void __attribute__((vector(_TIMER_6_VECTOR), interrupt(IPL7SRS))) timer6_inthnd(void) {
  periods++;// Increment periods count
  
  
  if (periods == 100){
      pwm_set(0x186A,7);
      pwm_set(0x186A,8);
      pwm_set(0x186A,9);
  }
      /*
   else if (periods == 200){
      pwm_set(0x1562,7);
      pwm_set(0x1562,8);
      pwm_set(0x0,9);
      periods = 0;
  }
        */
  
  IFS0CLR = _IFS0_T6IF_MASK;// Clear TMR4 Interrupt Flag
}

void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL4SRS))) can_inthnd(void) {
  if (C1INTbits.RBIF) {
    CAN_recv_messages(process_CAN_msg); // Process all available CAN messages
  }

  if (C1INTbits.RBOVIF) {
    CAN_rx_ovf++;
  }

  IFS4CLR = _IFS4_CAN1IF_MASK; // Clear CAN1 Interrupt Flag
}

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  // Declare local variables
  uint8_t load_idx, peak_mode, switch_bitmap = 0;
  double cutoff = 0;

  CAN_recv_tmr = millis; // Record time of latest received CAN message

  switch (msg.id) {
    

    case WHEEL_ID + 0x1:
        switch_bitmap = msg.data[0];
        enable = (uint8_t) ((switch_bitmap & 0x04) >> 2);;
      break;
      
  }
}