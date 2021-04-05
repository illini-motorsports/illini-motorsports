//#include <proc/p32mz2048efm100.h>

#include "FSAE_pwm.h"

void pwm_set(uint16_t duty_cycle, uint8_t OC) {

  switch (OC) {
  case 1:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC1R = duty_cycle;  // becomes read only during pwm mode operation
    OC1RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 2:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC2R = duty_cycle;  // becomes read only during pwm mode operation
    OC2RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 3:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC3R = duty_cycle;  // becomes read only during pwm mode operation
    OC3RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 4:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC4R = duty_cycle;  // becomes read only during pwm mode operation
    OC4RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 5:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC5R = duty_cycle;  // becomes read only during pwm mode operation
    OC5RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 6:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC6R = duty_cycle;  // becomes read only during pwm mode operation
    OC6RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 7:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC7R = duty_cycle;  // becomes read only during pwm mode operation
    OC7RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 8:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC8R = duty_cycle;  // becomes read only during pwm mode operation
    OC8RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 9:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    // CFGCONbits.OCACLK = 1;

    OC9R = duty_cycle;  // becomes read only during pwm mode operation
    OC9RS = duty_cycle; // sets to 5% duty cycle

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;
  }
}

// Initializes pwm signal with input period, and 10% duty cycle
void init_pwm(uint16_t period, uint8_t OC) {

  switch (OC) {
  case 1:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC1
    PMD3bits.OC1MD = 0;

    T4CONbits.ON = 0;  // disables Timer4
    OC1CONbits.ON = 0; // disables OC1

    T4CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T4CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T4CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB2Rbits.RPB2R = 0b1100; // OC1

    OC1CONbits.OCM = 0b0000; // set to zero to start
    PR4 = period;            // sets signal period
    OC1R = 0.1 * period;     // becomes read only during pwm mode operation
    OC1RS = 0.1 * period;    // sets to 10% duty cycle
    OC1CONbits.OCM = 0b110;  // set to PWM without fault

    T4CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC1CONbits.ON = 1;        // enables OC1
    T4CONbits.ON = 1;         // enables Timer4

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;
  case 2:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC2
    PMD3bits.OC2MD = 0;

    T4CONbits.ON = 0;  // disables Timer4
    OC2CONbits.ON = 0; // disables OC2

    T4CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T4CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T4CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB6Rbits.RPB6R = 0b1011; // OC2

    OC2CONbits.OCM = 0b0000; // set to zero to start
    PR4 = period;            // sets signal period
    OC2R = 0.1 * period;     // becomes read only during pwm mode operation
    OC2RS = 0.1 * period;    // sets to 10% duty cycle
    OC2CONbits.OCM = 0b110;  // set to PWM without fault

    T4CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC2CONbits.ON = 1;        // enables OC2
    T4CONbits.ON = 1;         // enables Timer4

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 3:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC3
    PMD3bits.OC3MD = 0;

    T4CONbits.ON = 0;  // disables Timer4
    OC3CONbits.ON = 0; // disables OC3

    T4CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T4CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T4CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB5Rbits.RPB5R = 0b1011; // OC3

    OC3CONbits.OCM = 0b0000; // set to zero to start
    PR4 = period;            // sets signal period
    OC3R = 0.1 * period;     // becomes read only during pwm mode operation
    OC3RS = 0.1 * period;    // sets to 10% duty cycle
    OC3CONbits.OCM = 0b110;  // set to PWM without fault

    T4CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC3CONbits.ON = 1;        // enables OC3
    T4CONbits.ON = 1;         // enables Timer4

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 4:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC4
    PMD3bits.OC3MD = 0;

    T2CONbits.ON = 0;  // disables Timer2
    OC4CONbits.ON = 0; // disables OC4

    T2CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T2CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T2CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB1Rbits.RPB1R = 0b1011; // OC4

    OC4CONbits.OCM = 0b0000; // set to zero to start
    PR2 = 0x3E8;             // main clock period (DO NOT CHANGE)
    period = 0x3E8;
    OC4R = 0.1 * period;    // becomes read only during pwm mode operation
    OC4RS = 0.1 * period;   // sets to 10% duty cycle
    OC4CONbits.OCM = 0b110; // set to PWM without fault

    T2CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC4CONbits.ON = 1;        // enables OC4
    T2CONbits.ON = 1;         // enables Timer2

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 5:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC5
    PMD3bits.OC3MD = 0;

    T2CONbits.ON = 0;  // disables Timer2
    OC5CONbits.ON = 0; // disables OC5

    T2CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T2CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T2CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB8Rbits.RPB8R = 0b1011; // OC5

    OC5CONbits.OCM = 0b0000; // set to zero to start
    PR2 = 0x3E8;             // main clock period (DO NOT CHANGE)
    period = 0x3E8;
    OC5R = 0.1 * period;    // becomes read only during pwm mode operation
    OC5RS = 0.1 * period;   // sets to 10% duty cycle
    OC5CONbits.OCM = 0b110; // set to PWM without fault

    T2CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC5CONbits.ON = 1;        // enables OC5
    T2CONbits.ON = 1;         // enables Timer2

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 6:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC6
    PMD3bits.OC3MD = 0;

    T2CONbits.ON = 0;  // disables Timer2
    OC6CONbits.ON = 0; // disables OC6

    T2CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T2CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T2CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB9Rbits.RPB9R = 0b1100; // OC6

    OC6CONbits.OCM = 0b0000; // set to zero to start
    PR2 = 0x3E8;             // main clock period (DO NOT CHANGE)
    period = 0x3E8;
    OC6R = 0.1 * period;    // becomes read only during pwm mode operation
    OC6RS = 0.1 * period;   // sets to 10% duty cycle
    OC6CONbits.OCM = 0b110; // set to PWM without fault

    T2CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC6CONbits.ON = 1;        // enables OC6
    T2CONbits.ON = 1;         // enables Timer2

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 7:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC7
    PMD3bits.OC7MD = 0;

    T6CONbits.ON = 0;  // disables Timer6
    OC7CONbits.ON = 0; // disables OC7

    T6CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T6CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T6CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB3Rbits.RPB3R = 0b1100; // OC7

    OC7CONbits.OCM = 0b0000; // set to zero to start
    PR6 = period;            // sets signal period
    OC7R = 0.1 * period;     // becomes read only during pwm mode operation
    OC7RS = 0.1 * period;    // sets to 10% duty cycle
    OC7CONbits.OCM = 0b110;  // set to PWM without fault

    T6CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC7CONbits.ON = 1;        // enables OC7
    T6CONbits.ON = 1;         // enables Timer6

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 8:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC8
    PMD3bits.OC8MD = 0;

    T6CONbits.ON = 0;  // disables Timer6
    OC8CONbits.ON = 0; // disables OC8

    T6CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T6CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T6CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPB7Rbits.RPB7R = 0b1100; // OC8

    OC8CONbits.OCM = 0b0000; // set to zero to start
    PR6 = period;            // sets signal period
    OC8R = 0.1 * period;     // becomes read only during pwm mode operation
    OC8RS = 0.1 * period;    // sets to 10% duty cycle
    OC8CONbits.OCM = 0b110;  // set to PWM without fault

    T6CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC8CONbits.ON = 1;        // enables OC8
    T6CONbits.ON = 1;         // enables Timer6

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;

  case 9:
    unlock_config();

    CFGCONbits.IOLOCK = 0;
    CFGCONbits.OCACLK = 1; // alternate clock for OC9
    PMD3bits.OC8MD = 0;

    T6CONbits.ON = 0;  // disables Timer6
    OC9CONbits.ON = 0; // disables OC9

    T6CONbits.TCS = 0; // Timer Clock Source Select (Internal peripheral clock)
    T6CONbits.SIDL =
        0; // Stop in Idle Mode (Continue operation even in Idle mode)
    T6CONbits.TGATE = 0; // Timer Gated Time Accumulation Enable (Gated time
                         // accumulation is disabled)

    RPD5Rbits.RPD5R = 0b1101; // OC9

    OC8CONbits.OCM = 0b0000; // set to zero to start
    PR6 = period;            // sets signal period
    OC9R = 0.1 * period;     // becomes read only during pwm mode operation
    OC9RS = 0.1 * period;    // sets to 10% duty cycle
    OC9CONbits.OCM = 0b110;  // set to PWM without fault

    T6CONbits.TCKPS = 0b0010; // timer scaling by 4
    OC9CONbits.ON = 1;        // enables OC9
    T6CONbits.ON = 1;         // enables Timer6

    CFGCONbits.IOLOCK = 1;

    lock_config();
    break;
  }
}