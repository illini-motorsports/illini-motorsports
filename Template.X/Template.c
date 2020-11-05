#include "Template.h"

int main(void) {
  init_general();     // Set general runtime configuration bits
  init_gpio_pins();   // Set all I/O pins to low outputs
  init_oscillator(0); // Initialize oscillator configuration bits
  init_timer2();      // Initialize timer2 (millis)

  int i;
  while (1) {
    for (i = 0; i < 1000000; i++)
      ;
    PIC_LED_LAT = 1;
    for (i = 0; i < 1000000; i++)
      ;
    PIC_LED_LAT = 0;
  }
  return 0;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS)))
timer2_inthnd(void) {
  IFS0CLR = _IFS0_T2IF_MASK; // Clear TMR2 Interrupt Flag
}

/**
 * CAN1 Interrupt Handler
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL4SRS)))
can_inthnd(void) {
  if (C1INTbits.RBIF) {
    CAN_recv_messages(process_CAN_msg); // Process all available CAN messages
  }

  if (C1INTbits.RBOVIF) {
    CAN_rx_ovf++;
  }

  IFS4CLR = _IFS4_CAN1IF_MASK; // Clear CAN1 Interrupt Flag
}

void process_CAN_msg(CAN_message msg) {
  // CAN Parsing goes here
}
