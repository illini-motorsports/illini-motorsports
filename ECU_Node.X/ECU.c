/**
 * ECU
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#include "ECU.h"

// Count number of seconds and milliseconds since start of code execution
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

// State/status variables determined by various sources
volatile double eng_rpm = 0; // From ECU
int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

// Timing interval variables
volatile uint32_t CAN_recv_tmr = 0;
uint32_t diag_send_tmr = 0;
uint32_t temp_samp_tmr = 0;

volatile uint32_t tsampctr = 0;
volatile uint32_t tsamps[100] = {0};
volatile uint32_t tdeltas[100] = {0};

// Crank wheel has 22 teeth (24 - 2). We count rising and falling edges.
// So we should see 44 edges per turn of the crank
volatile uint8_t edge = CRANK_EDGES - 1;
volatile uint32_t total_edges = 0;

volatile uint32_t edge_samp[CRANK_EDGES] = {0};
volatile uint32_t edge_delta[CRANK_EDGES] = {0};

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(0); // Initialize oscillator configuration bits
  init_timer1(); // Initialize timer1 (seconds)
  init_timer2(); // Initialize timer2 (millis)
  init_adc(init_adc_ecu); // Initialize ADC module
  init_termination(TERMINATING); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  // Initialize pins
  unlock_config();
  CFGCONbits.IOLOCK = 0;
  VR1_TRIS = INPUT;
  VR1_ANSEL = DIG_INPUT;
  IC1R = 0b1100;
  VR2_TRIS = INPUT;
  VR2_ANSEL = DIG_INPUT;
  IC2R = 0b1100;
  CFGCONbits.IOLOCK = 1;
  lock_config();

  // Initialize IC1 for VR1 TODO-AM: Move this
  IC1CONbits.ON = 0;
  init_timers_45();
  IC1CONbits.C32 = 1;     // 32 bit timer
  IC1CONbits.ICM = 0b001; // Edge Detect mode (rising and falling edges)
  IFS0bits.IC1IF = 0;
  IPC1bits.IC1IP = 7;
  IPC1bits.IC1IS = 3;
  IEC0bits.IC1IE = 1;
  IC1CONbits.ON = 1;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  TRISCbits.TRISC4 = OUTPUT;
  #define WAIT 1500

  // Main loop
  while (1) {

    // Simulate (24-2) teeth
    int i,j;
    for (i = 0; i < 22; i++) {
      for(j = 0; j < WAIT; j++);
      LATCbits.LATC4 = 1;
      for(j = 0; j < WAIT; j++);
      LATCbits.LATC4 = 0;
    }
    // Missing tooth #1
    for(j = 0; j < WAIT; j++);
    for(j = 0; j < WAIT; j++);
    // Missing tooth #2
    for(j = 0; j < WAIT; j++);
    for(j = 0; j < WAIT; j++);

    //STI(); // Enable interrupts (in case anything disabled without re-enabling)

    /**
     * Call helper functions
     */

    // Separate logic functions

    // Analog sampling functions
    //sample_temp();

    // CAN message sending functions
    //send_diag_can();
  }
}

//=============================== INTERRUPTS ===================================

/**
 * IC1 Interrupt Handler
 */
void __attribute__((vector(_INPUT_CAPTURE_1_VECTOR), interrupt(IPL7SRS))) ic1_inthnd(void) {
  while (!IC1CONbits.ICBNE); // Error!

  uint32_t val = IC1BUF;

  ++total_edges;

  uint8_t prev = edge;
  ++edge;
  if (edge == CRANK_EDGES) edge = 0;

  edge_samp[edge] = val;
  edge_delta[edge] = (val - edge_samp[prev]);

  //TODO: Use timer delta to calculate pulse width / frequency

  while (IC1CONbits.ICBNE); // Error!

  IFS0CLR = _IFS0_IC1IF_MASK; // Clear IC1 Interrupt Flag
}

/**
 * CAN1 Interrupt Handler
 */
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
 * TMR1 Interrupt Handler
 *
 * Fires once every second.
 */
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL5SRS))) timer1_inthnd(void) {
  seconds++; // Increment seconds count
  IFS0CLR = _IFS0_T1IF_MASK; // Clear TMR1 Interrupt Flag
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++; // Increment millis count

  //TODO: Move this?
  if (ADCCON2bits.EOSRDY) {
    ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion
  }

  IFS0CLR = _IFS0_T2IF_MASK; // Clear TMR2 Interrupt Flag
}

/**
 * NMI Handler
 *
 * This interrupt handler will reset the device when a clock failure occurs.
 */
void _nmi_handler(void) {
  // Perform a software reset
  unlock_config();
  RSWRSTSET = 1;
  uint16_t dummy = RSWRST;
  while (1);
  asm volatile("eret;"); // Should never be called
}

//============================ LOGIC FUNCTIONS =================================

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  CAN_recv_tmr = millis; // Record time of latest received CAN message

  switch (msg.id) {
    case MOTEC_ID + 0:
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) |
          msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      break;
  }
}

//============================= ADC FUNCTIONS ==================================

/**
 * void sample_temp(void)
 *
 * Samples the PCB temp sensor and internal die temp sensor, then updates
 * variables if the interval has passed.
 */
void sample_temp(void) {
  if(millis - temp_samp_tmr >= TEMP_SAMP_INTV) {

    /**
     * PCB Temp [C] = (Sample [V] - 0.75 [V]) / 10 [mV/C]
     * PCB Temp [C] = ((3.3 * (pcb_temp_samp / 4095)) [V] - 0.75 [V]) / 0.01 [V/C]
     * PCB Temp [C] = (3.3 * (pcb_temp_samp / 40.95)) - 75) [C]
     * PCB Temp [C] = (pcb_temp_samp * 0.080586080586) - 75 [C]
     * PCB Temp [C / 0.005] = 200 * ((pcb_temp_samp * 0.080586080586) - 75) [C / 0.005]
     * PCB Temp [C / 0.005] = (temp_samp * 16.1172161172) - 15000 [C / 0.005]
     */
    uint32_t pcb_temp_samp = read_adc_chn(ADC_PTEMP_CHN);
    pcb_temp = (((double) pcb_temp_samp) * 16.1172161172) - 15000.0;

    /**
     * Junc Temp [C] = 200 [C/V] * (1 [V] - Sample [V])
     * Junc Temp [C] = 200 [C/V] * (1 - (3.3 * (junc_temp_samp / 4095))) [V]
     * Junc Temp [C] = 200 [C/V] * (1 - (junc_temp_samp / 1240.9090909)) [V]
     * Junc Temp [C] = 200 - (junc_temp_samp * 0.161172161172) [C]
     * Junc Temp [C / 0.005] = 40000 - (junc_temp_samp * 32.234432234432) [C / 0.005]
     */

    uint32_t junc_temp_samp = read_adc_chn(ADC_JTEMP_CHN);
    junc_temp = (int16_t) (40000.0 - (((double) junc_temp_samp) * 32.234432234432));

    temp_samp_tmr = millis;
  }
}

//============================= CAN FUNCTIONS ==================================

/**
 * void send_diag_can(void)
 *
 * Sends the diagnostic CAN message if the interval has passed.
 */
void send_diag_can(void) {
  if (millis - diag_send_tmr >= DIAG_SEND) {
    CAN_data data = {0};
    data.halfword0 = (uint16_t) seconds;
    data.halfword1 = pcb_temp;
    data.halfword2 = junc_temp;

    CAN_send_message(ECU_ID + 0, 6, data);
    diag_send_tmr = millis;
  }
}

//========================== UTILITY FUNCTIONS =================================

/**
 * void init_adc_ecu(void)
 *
 * Initializes ADC1 & ADC2 modules so that AN46 & AN46 will work, along with
 * all other pins.
 */
void init_adc_ecu(void) {

}
