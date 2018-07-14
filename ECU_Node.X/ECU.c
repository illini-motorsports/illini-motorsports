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

volatile uint16_t deg = 0; // Engine cycle angular position. From 0.0 to 720.0 degrees [deg/2]
const uint16_t CRIP = 463; // Crank Reference Index Position. Offset from ref tooth to TDC Comp C1 [deg]
volatile uint8_t sync = 0; // Whether or not the timing logic has acquired sync
volatile uint8_t medGap = 0;
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
  init_timer2(); // Initialize timer2 (millis)
  init_adc(init_adc_ecu); // Initialize ADC module
  init_termination(TERMINATING); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  // Initialize pins

  // VR1, VR2
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

  // INJ[1-4], IGN[1-4]
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

  init_timer6_ecu(); // Initialize TMR6 for udeg interrupts
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
void
__attribute__((vector(_INPUT_CAPTURE_1_VECTOR), interrupt(IPL6SRS), no_fpu))
ic1_inthnd(void)
{
  // Disable further udeg interrupts
  T6CONCLR = _T6CON_ON_MASK;
  IFS0CLR = _IFS0_T6IF_MASK;

  register uint32_t val = IC1BUF;

  ++total_edges;

  register uint8_t prev = edge;
  ++edge;
  if (edge == CRANK_PERIODS) edge = 0;

  crank_samp[edge] = val;
  crank_delta[edge] = (val - crank_samp[prev]);
  register uint32_t delta = crank_delta[edge];

  // For gap verification, we use integer division with a max divisor of 8. We are
  // only interested in the ratio of the new gap to previous, so we can multiply
  // each by 8 to ensure we don't lose any resolution from int division truncating.
  register uint32_t gap = delta * 8;
  register uint32_t prevGap = crank_delta[prev] * 8;

  /**
   * Not synced: Detect med/ref edges
   */
  if (!sync) {
    if (total_edges > CRANK_PERIODS) {
      if (!medGap &&
          (gap > (prevGap * 3 / 2)) && (gap < (prevGap * 5 / 2)))
      {
        // Found medium gap
        medGap = 1;
        medEdge = edge;
      }
      else if (medGap) {
        if ((gap > (prevGap * 7 / 4)) && (gap < (prevGap * 9 / 4))) {
          // Found long gap, we are SYNC'd!
          sync = 1;
          refEdge = edge;
          deg = (CRIP * 2);

          // We expect a normal gap (7.5 degrees) after a the long ref gap. We
          // aren't going to generate udeg interrupts until the next edge, so we
          // need to fake 14 udeg interrupts worth of deg advancement. In reality,
          // deg is currently equal to CRIP here.
          ADD_DEG(deg, 14);

          //TODO: Should only sync directly after a SYNC pulse
        } else {
          kill_engine(0); // Error, didn't find long gap after med gap
        }
      }
    }
  }

  /**
   * Synced: Verify gaps and set udeg interrupts
   */
  else {
    // Gap verification
    if (edge == medEdge || edge == refEdge) {
      if (!((gap > (prevGap * 7 / 4)) && (gap < (prevGap * 9 / 4))))
        kill_engine(1); // Error, invalid gap
    } else if (prev == refEdge) {
      if (!((prevGap > (gap * 7 / 2)) && (prevGap < (gap * 9 / 2))))
        kill_engine(2); // Error, invalid gap
    } else {
      if (!((gap > (prevGap * 7 / 8)) && (gap < (prevGap * 9 / 8))))
        kill_engine(3); // Error, invalid gap
    }

    // Catch up if we skipped udeg interrupts
    while (udeg_rem != 0) {
      if (udeg_rem > 0) //TODO: Make greater than 0 when we move to a real/variable engine
        kill_engine(4);
      --udeg_rem;
      ADD_DEG(deg, 1);
      check_event_mask();
    }

    // This tooth edge also counts as one half degree (we set N-1 udeg interrupts)
    ADD_DEG(deg, 1);
    check_event_mask();

    // Check to make sure everything is still sync'd properly
    // TODO: If refEdge == 1, then this check always fails. deg short by 15. WTF?
    if (edge == refEdge && !
        (deg == (CRIP * 2) || deg == (deg_mod(CRIP, 360) * 2)))
      kill_engine(6);

    // Calculate udeg interrupts (one for each half degree). We should fire all
    // before another IC1 interrupt fires. If not, the engine may have accelerated.
    // In this case, we can do some catchup logic. But this should be limited
    // to just a few missed interrupts.

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

    // Skip a few udeg interrupts if this interrupt took a long time
    uint32_t startDelay = TMR4 - val;
    while (startDelay > udeg_period) {
      --udeg_rem;
      ADD_DEG(deg, 1); // 0.5deg per interrupt (7.5deg per edge, 15 int/edge)
      check_event_mask();
      startDelay -= udeg_period;
    }

    PR6 = udeg_period;
    TMR6 = startDelay; // 1st udeg period accounts for this long interrupt
    T6CONSET = _T6CON_ON_MASK;

    //TODO: Need to verify that med gap is 2x regular gap, and long gap is 4x regular
    //TODO: Use timer delta to calculate pulse width / frequency
    //TODO: Account for int flooring in udeg_period calculation
  }

  IFS0CLR = _IFS0_IC1IF_MASK; // Clear IC1 Interrupt Flag
}

/**
 * Timer6 Interrupt Handler
 *
 * This interrupt is used for each fast 0.5deg interval. All it does is update
 * 'udeg' and check the event mask for actions.
 *
 * Note: We must not call any non-inlined functions from this handler! Calling
 * any non-inlined function causes much more context switching code to be
 * inserted, as the compiler freaks out and doesn't know which registers need
 * to be saved. Macro and/or inlined functions are OK.
 */
void
__attribute__((vector(_TIMER_6_VECTOR), interrupt(IPL7SRS), no_fpu))
timer6_inthnd(void)
{
  if (--udeg_rem == 0)
    T6CONCLR = _T6CON_ON_MASK; // Disable further udeg interrupts

  ADD_DEG(deg, 1); // 0.5deg per interrupt (7.5deg per edge, 15 int/edge)
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
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS))) timer2_inthnd(void) {
  ++millis;
  if (millis % 1000 == 0)
    ++seconds;

  if (ADCCON2bits.EOSRDY)
    ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion

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
      if (deg_between(deg, new1, old1))
        INJ1_SET();
      if (deg_between(deg, new2, old2))
        INJ2_SET();
      if (deg_between(deg, new3, old3))
        INJ3_SET();
      if (deg_between(deg, new4, old4))
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

void
__attribute__((noreturn, always_inline))
kill_engine(uint16_t errno)
{
  CLI();
  register uint16_t for_debugger = errno;
  INJ1_CLR(); INJ2_CLR(); INJ3_CLR(); INJ4_CLR();
  IGN1_CLR(); IGN2_CLR(); IGN3_CLR(); IGN4_CLR();
  while(1); // Do nothing until reset
}

void
__attribute__((always_inline))
check_event_mask()
{
  register uint32_t mask = eventMask[deg];
  if (mask != 0) {
    // Toggle INJ outputs
    if (mask & INJ1_EN_MASK)
      INJ1_SET();
    if (mask & INJ1_DS_MASK)
      INJ1_CLR();
    if (mask & INJ2_EN_MASK)
      INJ2_SET();
    if (mask & INJ2_DS_MASK)
      INJ2_CLR();
    if (mask & INJ3_EN_MASK)
      INJ3_SET();
    if (mask & INJ3_DS_MASK)
      INJ3_CLR();
    if (mask & INJ4_EN_MASK)
      INJ4_SET();
    if (mask & INJ4_DS_MASK)
      INJ4_CLR();

    // Toggle IGN outputs
    if (mask & IGN1_EN_MASK)
      IGN1_SET();
    if (mask & IGN1_DS_MASK)
      IGN1_CLR();
    if (mask & IGN2_EN_MASK)
      IGN2_SET();
    if (mask & IGN2_DS_MASK)
      IGN2_CLR();
    if (mask & IGN3_EN_MASK)
      IGN3_SET();
    if (mask & IGN3_DS_MASK)
      IGN3_CLR();
    if (mask & IGN4_EN_MASK)
      IGN4_SET();
    if (mask & IGN4_DS_MASK)
      IGN4_CLR();
  }
}
