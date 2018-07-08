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
uint32_t adj_samp_tmr = 0;

// Crank wheel has 22 teeth (24 - 2). We count rising and falling edges.
// So we should see 44 edges & periods per turn of the crank
volatile uint8_t edge = CRANK_PERIODS - 1;
volatile uint32_t total_edges = 0;

volatile uint32_t crank_samp[CRANK_PERIODS] = {0};
volatile uint32_t crank_delta[CRANK_PERIODS] = {0};

volatile uint16_t deg, udeg = 0; // Engine cycle angular position. From 0.0 to 720.0 degrees
const uint16_t CRIP = 400; // Crank Reference Index Position. Offset from ref tooth to TDC Comp C1
volatile uint8_t sync = 0; // Whether or not the timing logic has acquired sync
volatile uint8_t medGap, longGap = 0;
volatile uint8_t refEdge, medEdge = 0;
volatile uint32_t eventMask[720 * 2] = {0}; // One eventMask per half degree of engine cycle
volatile uint8_t udeg_rem = 0;
volatile uint32_t udeg_period = 0;
volatile int32_t inj_pulse_width = -1;

// Events
// (4) Enable INJ (C1-C4)
// (4) Disable INJ (C1-C4)
// (4) Enable IGN (C1-C4)
// (4) Disable IGN (C1-C4)
// ...

////////////////////////////////////////////////////////////////////////////////
/// Engine Cycle
///
/// TC: TDC Compression Stroke
/// BP: BDC Power Stroke
/// TE: TDC Exhaust Stroke
/// BI: BDC Intake Stroke
///
///   Deg | C1 | C2 | C3 | C4 |
/// -----------------------------
///   000 | TC | BI | TE | BP |
///   180 | BP | TC | BI | TE |
///   360 | TE | BP | TC | BI |
///   540 | BI | TE | BP | TC |
///
/// End of Injection: 100 to 300 degrees before TDC compression, increasing with RPM
/// Start of Injection: EoI - Pulse Width
/// - INJ_PULSE_WIDTH 100 (for now)
///
/// For ignition, at some point before desired spark, we bring high to begin
///   charging the coil. At the moment of desired spark, we bring low to spark.
/// End of Ignition: ~30 deg before TDC compression
/// Start of Ignition: EoI - Dwell Time (~3ms, depends on battery voltage)


////////////////////////////////////////////////////////////////////////////////
/// Crank / CAM
///
/// Reference tooth is first edge after missing teeth (med gap then long gap)

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
  init_timer6_ecu();

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

  INJ1_TRIS = OUTPUT;
  INJ2_TRIS = OUTPUT;
  INJ3_TRIS = OUTPUT;
  INJ4_TRIS = OUTPUT;
  IGN1_TRIS = OUTPUT;
  IGN2_TRIS = OUTPUT;
  IGN3_TRIS = OUTPUT;
  IGN4_TRIS = OUTPUT;
  INJ1_CLR();
  INJ2_CLR();
  INJ3_CLR();
  INJ4_CLR();
  IGN1_CLR();
  IGN2_CLR();
  IGN3_CLR();
  IGN4_CLR();

  UDEG_SIG_TRIS = OUTPUT;
  UDEG_SIG_CLR();

  // Set some test angles for injection and ignition
  eventMask[520*2] |= INJ1_DS_MASK;
  eventMask[700*2] |= INJ2_DS_MASK;
  eventMask[160*2] |= INJ3_DS_MASK;
  eventMask[340*2] |= INJ4_DS_MASK;

  eventMask[690*2] |= IGN1_DS_MASK;
  eventMask[150*2] |= IGN2_DS_MASK;
  eventMask[330*2] |= IGN3_DS_MASK;
  eventMask[510*2] |= IGN4_DS_MASK;
  eventMask[(690-75)*2] |= IGN1_EN_MASK;
  eventMask[(150-75)*2] |= IGN2_EN_MASK;
  eventMask[(330-75)*2] |= IGN3_EN_MASK;
  eventMask[(510-75)*2] |= IGN4_EN_MASK;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  init_ic1(); // Initialize IC1 for VR1 crank signal

  STI(); // Enable interrupts

  while (inj_pulse_width == -1)
    sample_adj();

  // Main loop
  while (1) {
    STI(); // Enable interrupts (in case anything disabled without re-enabling)

    /**
     * Call helper functions
     */

    // Separate logic functions

    // Analog sampling functions
    sample_temp();
    sample_adj();

    // CAN message sending functions
    send_diag_can();
  }
}

//=============================== INTERRUPTS ===================================

/**
 * IC1 Interrupt Handler
 */
void __attribute__((vector(_INPUT_CAPTURE_1_VECTOR), interrupt(IPL6SRS))) ic1_inthnd(void) {
  // Disable further udeg interrupts
  T6CONCLR = _T6CON_ON_MASK;
  IFS0CLR = _IFS0_T6IF_MASK;

  if (!IC1CONbits.ICBNE)
    kill_engine(7 /*TODO*/); // Error

  uint32_t val = IC1BUF;

  ++total_edges;

  uint8_t prev = edge;
  ++edge;
  if (edge == CRANK_PERIODS) edge = 0;

  crank_samp[edge] = val;
  crank_delta[edge] = (val - crank_samp[prev]);

  /**
   * Not synced: Detect med/ref edges
   */
  if (!sync) {
    if (total_edges > CRANK_PERIODS) {
      double gap = (double) crank_delta[edge];
      double prevGap = (double) crank_delta[prev];

      if (!medGap &&
          (gap > (1.5 * prevGap)) && (gap < (2.5 * prevGap)))
      {
        // Found medium gap
        medGap = 1;
        medEdge = edge;
      }
      else if (medGap) {
        if ((gap > (1.75 * prevGap)) && (gap < (2.25 * prevGap))) {
          // Found long gap, we are SYNC'd!
          sync = 1;
          deg = CRIP;
          refEdge = edge;
        } else {
          // Error, didn't find long gap after med gap
          kill_engine(0 /*TODO*/);
        }
      }
    }
  }

  /**
   * Synced: Verify gaps and set udeg interrupts
   */
  else {
    // Gap verification
    //TODO: Need to verify that med gap is 2x regular gap, and long gap is 4x regular
    uint32_t delta = crank_delta[edge];
    double gap = (double) delta;
    double prevGap = (double) crank_delta[prev];
    if (edge == medEdge) {
      if (!((gap > (1.75 * prevGap)) && (gap < (2.25 * prevGap))))
        kill_engine(1 /*TODO*/); // Error, invalid gap
    } else if (edge == refEdge) {
      if (!((gap > (1.75 * prevGap)) && (gap < (2.25 * prevGap))))
        kill_engine(2 /*TODO*/); // Error, invalid gap
    } else if (prev == refEdge) {
      if (!((prevGap > (3.5 * gap)) && (prevGap < (4.5 * gap))))
        kill_engine(3 /*TODO*/); // Error, invalid gap
    } else {
      if (!((gap > (0.9 * prevGap)) && (gap < (1.1 * prevGap))))
        kill_engine(4 /*TODO*/); // Error, invalid gap
    }

    // Catch up if we skipped a udeg interrupt
    while (udeg_rem != 0) {
      if (udeg_rem > 0) //TODO: Make greater than 0 when we move to a real/variable engine
        kill_engine(5 /*TODO*/); // Error
      --udeg_rem;
      ADD_DEG(udeg, 1); // 0.5deg per interrupt (7.5deg per edge, 15 int/edge)
      check_event_mask();
    }

    // Calculate udeg interrupts (one for each half degree). We should fire all
    // 15 before another IC1 interrupt fires. If not, the engine may have
    // accelerated or we could have done shit wrong. In this case, we can do some
    // catchup logic. But this should be limited to just a few missed interrupts

    // Set angular position & calculate udeg interrupts
    if (edge == refEdge) {
      if (deg != (CRIP - 60) && deg != (CRIP + 720 - 60))
        kill_engine(6 /*TODO*/); // Error
      ADD_DEG(deg, 60)
    } else if (edge == medEdge) {
      ADD_DEG(deg, 30);
    } else {
      ADD_DEG(deg, 15); // 7.5 deg per edge
    }

    if (edge == refEdge) {
      // Expecting normal gap after long gap
      udeg_rem = 14;
      udeg_period = delta / 60;
    } else if (edge == medEdge) {
      // Expecting long gap after med gap
      udeg_rem = 59;
      udeg_period = delta / 30;
    } else if (edge == medEdge - 1) {
      // Expecting med gap after normal gap
      udeg_rem = 29;
      udeg_period = delta / 15;
    } else {
      // Expecting normal gap after normal gap
      udeg_rem = 14;
      udeg_period = delta / 15;
    }

    udeg = deg;
    check_event_mask();

    uint32_t startDelay = TMR4 - val;
    while (startDelay > udeg_period) {
      --udeg_rem;
      ADD_DEG(udeg, 1); // 0.5deg per interrupt (7.5deg per edge, 15 int/edge)
      check_event_mask();
      startDelay -= udeg_period;
    }

    PR6 = udeg_period;
    TMR6 = startDelay; // 1st udeg period accounts for this long interrupt
    T6CONSET = _T6CON_ON_MASK;

    //TODO: Use timer delta to calculate pulse width / frequency
    //TODO: Account for int flooring in udeg_period calculation
  }

  if (IC1CONbits.ICBNE)
    kill_engine(8 /*TODO*/); // Error

  IFS0CLR = _IFS0_IC1IF_MASK; // Clear IC1 Interrupt Flag
}

/**
 * Timer6 Interrupt Handler
 *
 * This interrupt is used for each fast 0.5deg interval. All it does is update
 * 'udeg' and check the event mask for actions.
 *
 * Note: We must not call any functions from this handler! Calling any function
 * causes much more context switching code to be inserted, as the compiler freaks
 * out and doesn't know which registers need to be saved. Macro functions are OK.
 *
 * Optimized w/o check_event_mask: 44 instructions
 * Optimized w/ check_event_mask:  154 instructions
 * Unoptimized w/ check_event_mask: 273 instructions!
 *
 * check_event_mask not optimized: 138 instructions
 * check_event_mask optimized: 110 instructions
 */
void __attribute__((vector(_TIMER_6_VECTOR), interrupt(IPL7SRS))) timer6_inthnd(void) {
  UDEG_SIG_INV();

  if (--udeg_rem == 0)
    T6CONCLR = _T6CON_ON_MASK; // Disable further udeg interrupts

  ADD_DEG(udeg, 1); // 0.5deg per interrupt (7.5deg per edge, 15 int/edge)
  check_event_mask();

  IFS0CLR = _IFS0_T6IF_MASK; // Clear TMR6 Interrupt Flag
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
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS))) timer2_inthnd(void) {
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

/**
 * Samples the adjustment potentiometers. Currently used to adjust injector pulse width
 */
void sample_adj() {
  if(millis - adj_samp_tmr >= ADJ_SAMP_INTV) {
    uint32_t adj1_samp = read_adc_chn(ADC_ADJ1_CHN);
    uint32_t adj2_samp = read_adc_chn(ADC_ADJ2_CHN);

    int32_t old_pulse_width = inj_pulse_width;
    inj_pulse_width = adj2_samp / 20;

    uint32_t old1 = deg_mod(520, -old_pulse_width) * 2;
    uint32_t old2 = deg_mod(700, -old_pulse_width) * 2;
    uint32_t old3 = deg_mod(160, -old_pulse_width) * 2;
    uint32_t old4 = deg_mod(340, -old_pulse_width) * 2;

    uint32_t new1 = deg_mod(520, -inj_pulse_width) * 2;
    uint32_t new2 = deg_mod(700, -inj_pulse_width) * 2;
    uint32_t new3 = deg_mod(160, -inj_pulse_width) * 2;
    uint32_t new4 = deg_mod(340, -inj_pulse_width) * 2;

    CLI(); // Begin critical section

    eventMask[old1] &= ~INJ1_EN_MASK;
    eventMask[old2] &= ~INJ2_EN_MASK;
    eventMask[old3] &= ~INJ3_EN_MASK;
    eventMask[old4] &= ~INJ4_EN_MASK;
    eventMask[new1] |= INJ1_EN_MASK;
    eventMask[new2] |= INJ2_EN_MASK;
    eventMask[new3] |= INJ3_EN_MASK;
    eventMask[new4] |= INJ4_EN_MASK;

    /**
     * We need to enable INJ signals if we "skipped" them.
     *
     * - New pulse shorter than old pulse
     *      1   2    3      4
     *        |-----------|
     * old ___|           |___
     *            |-------|
     * new _______|       |___
     *
     * 2,3: Pulse too long this cycle.

     * - New pulse longer than old pulse
     *      1   2    3      4
     *            |-------|
     * old _______|       |___
     *        |-----------|
     * new ___|           |___
     *
     * 2: Skipped enable, so enable now. Pulse too short this cycle.
     * 3: Pulse too short this cycle.
     */

    if (inj_pulse_width > old_pulse_width) {
      if (deg_between(udeg, new1, old1))
        INJ1_SET();
      if (deg_between(udeg, new2, old2))
        INJ2_SET();
      if (deg_between(udeg, new3, old3))
        INJ3_SET();
      if (deg_between(udeg, new4, old4))
        INJ4_SET();
    }

    adj_samp_tmr = millis;

    STI();
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
  // Initialize pins
  ADC_ADJ1_TRIS = INPUT;
  ADC_ADJ1_ANSEL = AN_INPUT;
  ADC_ADJ1_CSS = 1;

  ADC_ADJ2_TRIS = INPUT;
  ADC_ADJ2_ANSEL = AN_INPUT;
  ADC_ADJ2_CSS = 1;
}

/**
 *
 */
void init_ic1() {
  IC1CONbits.ON = 0;

  init_timers_45();

  IC1CONbits.C32 = 1;     // 32 bit timer
  IC1CONbits.ICM = 0b001; // Edge Detect mode (rising and falling edges)

  IFS0bits.IC1IF = 0;
  IPC1bits.IC1IP = 6;
  IPC1bits.IC1IS = 2;
  IEC0bits.IC1IE = 1;

  IC1CONbits.ON = 1;
}

/**
 * Returns (start + offset), shifted around 720 degrees.
 *
 * Note: This only supports values of (start + offset) from
 * -720 degrees to 1439 degrees.
 */
uint32_t deg_mod(int32_t start, int32_t offset)
{
  int32_t res = start + offset;
  if (res < 0)
    return (res + 720);
  if (res >= 720)
    return (res - 720);
  return res;
}

/**
 * Returns whether t is in the range of [a,b] in the
 * 1440 degree domain.
 *
 * Note: t, a, and b must be in the range of [0,1440). a must
 * also be "before" b in the range.
 */
uint8_t deg_between(uint32_t t, uint32_t a, uint32_t b)
{
  if (a < b) // Not shifted around 0/1440
    return (t >= a) && (t <= b);
  else if (a > b) // Shifted around 0/1440
    return (t >= a) || (t <= b);
  else
    return (t == a);
}
