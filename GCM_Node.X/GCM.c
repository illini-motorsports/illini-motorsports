/**
 * GCM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "GCM.h"

//TODO: Add more checks/handling for stale CAN data

// Count number of seconds and milliseconds since start of code execution
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

volatile uint8_t gear = GEAR_FAIL; // Current gear #CHECK
double shift_force = 0.0;          // Shift force sensor reading

volatile uint8_t queue_up = 0; // Number of queued upshifts
volatile uint8_t queue_dn = 0; // Number of queued downshifts
volatile uint8_t queue_nt = 0; // Number of queued neutral shifts

uint8_t retry_up = 0; // Number of retried upshifts
uint8_t retry_dn = 0; // Number of retried downshifts
uint8_t retry_nt = 0; // Number of retried neutral shifts

uint8_t switch_debounced = 0; // Debounced (safe) switch state
uint8_t switch_prev = 0;      // Previous sampled value of switches

volatile uint8_t prev_switch_up = 0; // Previous switch state of SHIFT_UP
volatile uint8_t prev_switch_dn = 0; // Previous switch state of SHIFT_DN

volatile double eng_rpm = 0;       // Engine RPM (from ECU)
volatile double bat_volt = 0;      // Battery voltage (from ECU)
volatile double throttle_pos = 0;  // Throttle position (from ECU)
volatile uint16_t shift_force_ecu = 0; // Spoofed shift force (from ECU)
volatile uint8_t kill_sw = 0;      // Holds state of KILL_SW (from PDM)

int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

uint16_t shift_force_spoof = 0; // Holds desired value of gear shift force

volatile uint8_t gear_fail_nt_shift = SHIFT_ENUM_NT; // Stores direction of neutral shift while in gear failure mode

// Timing interval variables
volatile uint32_t CAN_recv_tmr = 0;
uint32_t temp_samp_tmr, sensor_samp_tmr = 0;
uint32_t diag_send_tmr, state_send_tmr = 0;
uint32_t switch_debounce_tmr = 0;
volatile uint32_t lockout_tmr = 0; // Holds millis value of last lockout set
uint32_t act_tmr = 0;              // Records the time the actuator was fired
uint32_t pwr_cut_tmr = 0;          // Records when the power cut was initiated
uint32_t pwr_cut_retry_tmr = 0;    // Records when the ADL power cut message was last sent

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
  init_adc(NULL); // Initialize ADC module
  init_termination(NOT_TERMINATING); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  //TODO: USB
  //TODO: NVM

  // Initialize pins
  SHIFT_UP_TRIS = INPUT;
  SHIFT_UP_ANSEL = DIG_INPUT;
  SHIFT_DN_TRIS = INPUT;
  SHIFT_DN_ANSEL = DIG_INPUT;
  SHIFT_NT_TRIS = INPUT;
  SHIFT_NT_ANSEL = DIG_INPUT;
  ACT_UP_TRIS = OUTPUT;
  ACT_UP_LAT = ACT_OFF;
  ACT_DN_TRIS = OUTPUT;
  ACT_DN_LAT = ACT_OFF;
  ADC_GEAR_TRIS = INPUT;
  ADC_GEAR_ANSEL = AN_INPUT;
  ADC_GEAR_CSS = 1;
  ADC_FORCE_TRIS = INPUT;
  ADC_FORCE_ANSEL = AN_INPUT;
  ADC_FORCE_CSS = 1;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  // Main loop
  while (1) {
    STI(); // Enable interrupts (in case anything disabled without re-enabling)

    // Misc helper functions
    debounce_switches();

    // Analog sampling functions
    sample_temp();
    sample_sensors(NOT_SHIFTING);

    // Do upshift sequence if queued
    if(queue_up > 0) {
      do_shift(SHIFT_ENUM_UP);
    }

    // Do downshift sequence if queued
    if(queue_dn > 0) {
      do_shift(SHIFT_ENUM_DN);
    }

    // Do neutral shift sequence if queued
    if(queue_nt > 0) {
      do_shift(SHIFT_ENUM_NT);
    }

    // Send power cut message again if MoTeC hasn't received it
    if ((shift_force_spoof != shift_force_ecu) && (millis - pwr_cut_retry_tmr >= CUT_RETRY_WAIT)) {
        send_power_cut(CUT_RESEND);
    }

    // CAN message sending functions
    send_diag_can();
    send_state_can(NO_OVERRIDE);
  }
}

//=============================== INTERRUPTS ===================================

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

  // Check RPM to call upshifting logic, add upshifting logic to its own function in logic functions section #CHECK

  if (SHIFT_UP_SW != prev_switch_up ||
      SHIFT_DN_SW != prev_switch_dn) {
    send_state_can(OVERRIDE);
  }

  // Check for a new shift_up switch press
  if(SHIFT_UP_SW && !prev_switch_up) {
    process_upshift_press();
  }
  prev_switch_up = SHIFT_UP_SW;

  // Check for a new shift_dn switch press
  if(SHIFT_DN_SW && !prev_switch_dn) {
    process_downshift_press();
  }
  prev_switch_dn = SHIFT_DN_SW;

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

//============================= ADC FUNCTIONS ==================================

/**
 * void sample_temp(void)
 *
 * Samples the PCB temp sensor and internal die temp sensor, then updates
 * variables if the interval has passed.
 */
void sample_temp(void) {
  if (millis - temp_samp_tmr >= TEMP_SAMP_INTV) {

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
 * void sample_sensors(uint8_t is_shifting)
 *
 * Samples the gear position sensor and shift force sensor and updates
 * variables if the interval has passed.
 *
 * @param is_shifting- Whether or not the car is currently mid-shift
 */
void sample_sensors(uint8_t is_shifting) {
  if (millis - sensor_samp_tmr >= SENSOR_SAMP_INTV) {
    uint32_t gear_samp = read_adc_chn(ADC_GEAR_CHN);
    double gear_voltage =  ((((double) gear_samp) / 4095.0) * 5.0);
    uint16_t gear_voltage_can = (uint16_t) (gear_voltage * 10000.0);

    if (abs(gear_voltage - 0.595) <= GEAR_VOLT_RIPPLE) {
      gear = 1;
    } else if (abs(gear_voltage - 1.05) <= GEAR_VOLT_RIPPLE) {
      gear = GEAR_NEUT;
    } else if (abs(gear_voltage - 1.58) <= GEAR_VOLT_RIPPLE) {
      gear = 2;
    } else if (abs(gear_voltage - 2.32) <= GEAR_VOLT_RIPPLE) {
      gear = 3;
    } else if (abs(gear_voltage - 3.16) <= GEAR_VOLT_RIPPLE) {
      gear = 4;
    } else if (abs(gear_voltage - 4.12) <= GEAR_VOLT_RIPPLE) {
      gear = 5;
    } else if (abs(gear_voltage - 4.81) <= GEAR_VOLT_RIPPLE) {
      gear = 6;
    } else {
      gear = GEAR_FAIL;
    }

    // Only send gear failure error message if we aren't shifting
    if (!is_shifting && gear == GEAR_FAIL) {
      send_errno_CAN_msg(GCM_ID, ERR_GCM_GEARFAIL);
    }

    // Sample shift force sensor
    uint32_t force_samp = read_adc_chn(ADC_FORCE_CHN);
    double force_voltage =  ((((double) force_samp) / 4095.0) * 5.0);
    shift_force = (428.571428571 * force_voltage) + 1071.42857143;
    int16_t shift_force_can = (int16_t) (shift_force / FORCE_SCL);

    CAN_data data = {0};
    data.byte0 = gear;
    data.halfword1 = gear_voltage_can;
    data.halfword2 = shift_force_can;
    CAN_send_message(GCM_ID + 0x1, 6, data);

    sensor_samp_tmr = millis;
  }
}

//============================= CAN FUNCTIONS ==================================

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  uint8_t switch_bitmap;

  switch (msg.id) {
    case MOTEC_ID + 0x0:
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) |
          msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      throttle_pos = ((double) ((msg.data[THROTTLE_POS_BYTE] << 8) |
          msg.data[THROTTLE_POS_BYTE + 1])) * THROTTLE_POS_SCL;
      bat_volt = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) |
          msg.data[VOLT_ECU_BYTE + 1])) * VOLT_ECU_SCL;
      CAN_recv_tmr = millis;
      break;

    case MOTEC_ID + 0x7:
      shift_force_ecu = (uint16_t) ((msg.data[SHIFT_FORCE_BYTE] << 8) |
          msg.data[SHIFT_FORCE_BYTE + 1]);
      break;

    case PDM_ID + 0x1:
      switch_bitmap = msg.data[PDM_SWITCH_BYTE];
      kill_sw = switch_bitmap & KILL_PDM_SW_MASK;
      CAN_recv_tmr = millis;
      break;
  }
}

/**
 * void send_diag_can(void)
 *
 * Sends the diagnostic CAN message if the interval has passed.
 */
void send_diag_can(void) {
  if (millis - diag_send_tmr >= DIAG_MSG_SEND) {
    CAN_data data = {0};
    data.halfword0 = (uint16_t) seconds;
    data.halfword1 = pcb_temp;
    data.halfword2 = junc_temp;

    CAN_send_message(GCM_ID + 0, 6, data);
    diag_send_tmr = millis;
  }
}

/**
 * void send_state_can(uint8_t override)
 *
 * Sends the switch/queue state CAN message if the interval has passed.
 *
 * @param override - Whether to override the interval
 */
void send_state_can(uint8_t override) {
  if ((millis - state_send_tmr >= STATE_MSG_SEND) || override) {
    CAN_data data = {0};
    data.byte0 = 0x0 |
      (SHIFT_NT_SW << 2) |
      (SHIFT_DN_SW << 1) |
      SHIFT_UP_SW;
    data.byte1 = queue_up;
    data.byte2 = queue_dn;
    data.byte3 = queue_nt;

    CAN_send_message(GCM_ID + 2, 4, data);
    state_send_tmr = millis;
  }
}

//=========================== LOGIC FUNCTIONS ==================================

/**
 * void process_upshift_press(void)
 *
 * After registering an upshift press, determine if we should increment the queue
 */
void process_upshift_press(void) {
  if (queue_nt == 1) {
    return;
  }

  if (millis - lockout_tmr < LOCKOUT_DUR) {
    send_errno_CAN_msg(GCM_ID, ERR_GCM_LOCKOUT);
    return;
  }

  if (SHIFT_NT_SW) { // Pressed while holding the neutral button
    if (gear == GEAR_NEUT) {
      queue_dn = 1;
      queue_nt = queue_up = 0;
    } else if (gear == 1 || gear == 2) {
      queue_nt = 1;
      queue_up = queue_dn = 0;
    } else if (gear == GEAR_FAIL) {
      queue_nt = 1;
      queue_up = queue_dn = 0;
      gear_fail_nt_shift = SHIFT_ENUM_UP;
    } else {
      queue_dn = 1;
    }
    lockout_tmr = millis;
    send_state_can(OVERRIDE); // Send new queue values on CAN
  } else { // Pressed while not holding the neutral button
    if (gear == GEAR_NEUT) {
      queue_up = 1;
      queue_dn = queue_nt = 0;
      lockout_tmr = millis;
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    if (queue_dn > 0 || queue_nt > 0) {
      queue_dn = queue_up = queue_nt = 0;
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    if (gear == GEAR_FAIL || queue_up < 6 - gear) {
      queue_up++;
      lockout_tmr = millis;
      send_state_can(OVERRIDE); // Send new queue values on CAN
    }
  }
}

/**
 * void process_downshift_press(void)
 *
 * After registering an downshift press, determine if we should increment the queue
 */
void process_downshift_press(void) {
  if (queue_nt == 1) {
    return;
  }

  if (millis - lockout_tmr < LOCKOUT_DUR) {
    send_errno_CAN_msg(GCM_ID, ERR_GCM_LOCKOUT);
    return;
  }

  if (SHIFT_NT_SW) { // Pressed while holding the neutral button
    if (gear == GEAR_NEUT) {
      queue_nt = 0;
    } else if (gear == 1 || gear == 2) {
      queue_nt = 1;
      queue_up = queue_dn = 0;
    } else if (gear == GEAR_FAIL) {
      queue_nt = 1;
      queue_up = queue_dn = 0;
      gear_fail_nt_shift = SHIFT_ENUM_DN;
    } else {
      queue_dn = 1;
      queue_up = queue_nt = 0;
    }
    lockout_tmr = millis;
    send_state_can(OVERRIDE); // Send new queue values on CAN
  } else { // Pressed while not holding the neutral button
    if (queue_up > 0 || queue_nt > 0) {
      queue_dn = queue_up = queue_nt = 0;
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    if (gear == GEAR_FAIL || gear == GEAR_NEUT || queue_dn + 1 < gear) {
      queue_dn++;
      lockout_tmr = millis;
      send_state_can(OVERRIDE); // Send new queue values on CAN
    }
  }
}

/**
 * uint8_t check_shift_conditions(uint8_t shift_enum)
 *
 * Checks various conditions and determines if the requested shift is allowable.
 *
 * @param shift_enum - The type of shift being requested
 * @return 0 if restricted, 1 if allowable
 */
uint8_t check_shift_conditions(uint8_t shift_enum) {
  // Prevent shifting past 1st gear
  if (gear == 1 && shift_enum == SHIFT_ENUM_DN) {
    send_errno_CAN_msg(GCM_ID, ERR_GCM_SHIFTPAST);
    return 0;
  }

  // Prevent shifting past 6th gear
  if (gear == 6 && shift_enum == SHIFT_ENUM_UP) {
    send_errno_CAN_msg(GCM_ID, ERR_GCM_SHIFTPAST);
    return 0;
  }

  // Prevent shifting into neutral from anything other than 1st or 2nd
  if (shift_enum == SHIFT_ENUM_NT &&
      !(gear == 1 || gear == 2 || gear == GEAR_FAIL)) {
    send_errno_CAN_msg(GCM_ID, ERR_GCM_BADNEUT);
    return 0;
  }

  // Only perform these checks if CAN data isn't stale
  uint8_t stale_CAN = (millis - CAN_recv_tmr >= CAN_STATE_WAIT);
  if (!stale_CAN) {
    // Prevent shifting while the kill switch is pressed
    if (kill_sw) {
      send_errno_CAN_msg(GCM_ID, ERR_GCM_KILLSW);
      return 0;
    }

    // Prevent shifting from neutral into gear at high rpm
    if (gear == GEAR_NEUT && eng_rpm >= RPM_NEUT_LOCK) {
      send_errno_CAN_msg(GCM_ID, ERR_GCM_NEUTLOCK);
      return 0;
    }

    // Check for over/under rev
    if (shift_enum != SHIFT_ENUM_NT && ENG_ON &&
        gear != GEAR_NEUT && gear != GEAR_FAIL) {
      double output_speed = eng_rpm / gear_ratio[gear];

      if (shift_enum == SHIFT_ENUM_UP && gear != 6) {
        double new_rpm = output_speed * gear_ratio[gear + 1];
        if (new_rpm <= RPM_UNDERREV) {
          send_errno_CAN_msg(GCM_ID, ERR_GCM_UNDERREV);
          return 0;
        }
      } else if (shift_enum == SHIFT_ENUM_DN && gear != 1) {
        double new_rpm = output_speed * gear_ratio[gear - 1];
        if (new_rpm >= RPM_OVERREV) {
          send_errno_CAN_msg(GCM_ID, ERR_GCM_OVERREV);
          return 0;
        }
      }
    }
  } else if (!COMP) {
    // Prevent shifting if CAN state variables aren't updated
    send_errno_CAN_msg(GCM_ID, ERR_GCM_NOCAN);
    return 0;
  }

  // Only perform these checks if we aren't at competition and have good CAN data
  if (!COMP && !stale_CAN) {
    // Prevent shifting up past neutral on low voltage
    if (gear == 1 && shift_enum == SHIFT_ENUM_UP) {
      if (bat_volt != 0.0 && bat_volt < LOW_VOLT) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
        return 0;
      }
    }

    // Prevent shifting down past neutral on low voltage
    if (gear == 2 && shift_enum == SHIFT_ENUM_DN) {
      if (bat_volt != 0 && bat_volt < LOW_VOLT) {
        send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWVOLT);
        return 0;
      }
    }

    // Prevent any shifting on very low voltage
    if (bat_volt != 0 && bat_volt < LOWER_VOLT) {
      send_errno_CAN_msg(GCM_ID, ERR_GCM_LOWERVOLT);
      return 0;
    }
  }

  return 1;
}

/**
 * void do_shift(uint8_t shift_enum)
 *
 * Performs the requested shift sequence.
 *
 * @param shift_enum The type of requested shift
 */
void do_shift(uint8_t shift_enum) {
  const uint8_t SHIFT_UP = shift_enum == SHIFT_ENUM_UP;
  const uint8_t SHIFT_DN = shift_enum == SHIFT_ENUM_DN;
  const uint8_t SHIFT_NT = shift_enum == SHIFT_ENUM_NT;

  uint8_t orig_gear = gear;
  uint8_t gear_target;

  // Fall back to gear sensor failure mode if necessary
  if (gear == GEAR_FAIL) {
    do_shift_gear_fail(shift_enum);
    return;
  }

  // Store the target gear
  if (SHIFT_UP) {
    gear_target = (gear == GEAR_NEUT) ? 2 : gear + 1;
  } else if (SHIFT_DN) {
    gear_target = (gear == GEAR_NEUT) ? 1 : gear - 1;
  } else if (SHIFT_NT) {
    gear_target = GEAR_NEUT;
  }

  if (SHIFT_UP || SHIFT_DN) {
    /**
     * Perform an upshift/downshift
     */

    if (!check_shift_conditions(shift_enum)) {
      switch (shift_enum) {
        case SHIFT_ENUM_UP:
          queue_up = 0;
          retry_up = 0;
          break;
        case SHIFT_ENUM_DN:
          queue_dn = 0;
          retry_dn = 0;
          break;
      }
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    // Fire actuator
    if (SHIFT_UP) {
      ACT_UP_LAT = ACT_ON;
    } else if (SHIFT_DN) {
      ACT_DN_LAT = ACT_ON;
    }
    act_tmr = millis;

    // Wait a bit and signal the ECU to cut power
    while (millis - act_tmr < PWR_CUT_WAIT) {
      main_loop_misc();
    }
    send_power_cut(CUT_START);

    while (1) {
      sample_sensors(SHIFTING);

      if (gear == gear_target) {
        // Relax actuator
        if (SHIFT_UP) {
          ACT_UP_LAT = ACT_OFF;
          retry_up = 0;
        } else if (SHIFT_DN) {
          ACT_DN_LAT = ACT_OFF;
          retry_dn = 0;
        }

        send_power_cut(CUT_END);

        relax_wait();

        // Decrement queued shifts value
        if (SHIFT_UP) {
          queue_up--;
        } else if (SHIFT_DN){
          queue_dn--;
        }
        send_state_can(OVERRIDE); // Send new queue values on CAN

        return;
      }

      if (millis - act_tmr >= MAX_SHIFT_DUR) {
        // Relax actuator
        if (SHIFT_UP) {
          ACT_UP_LAT = ACT_OFF;
          retry_up++;
        } else if (SHIFT_DN){
          ACT_DN_LAT = ACT_OFF;
          retry_dn++;
        }

        send_power_cut(CUT_END);

        relax_wait();

        send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXDUR);

        if (SHIFT_UP && retry_up > MAX_RETRY) {
          queue_up = 0;
          retry_up = 0;
          send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
          send_state_can(OVERRIDE); // Send new queue values on CAN
        } else if (SHIFT_DN && retry_dn > MAX_RETRY) {
          queue_dn = 0;
          retry_dn = 0;
          send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
          send_state_can(OVERRIDE); // Send new queue values on CAN
        }

        return;
      }

      // Also do main loop functions during "busy" loop
      main_loop_misc();
    }
  } else if (SHIFT_NT) {
    /**
     * Perform a neutral shift
     */

    if (!check_shift_conditions(shift_enum)) {
      queue_nt = 0;
      retry_nt = 0;
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    // Fire actuator
    if (orig_gear == 1) {
      ACT_UP_LAT = ACT_ON;
    } else if (orig_gear == 2) {
      ACT_DN_LAT = ACT_ON;
    }
    act_tmr = millis;

    while (1) {
      sample_sensors(SHIFTING);

      if (gear == GEAR_NEUT) {
        // Relax actuator
        ACT_DN_LAT = ACT_OFF;
        ACT_UP_LAT = ACT_OFF;
        relax_wait();

        retry_nt = 0;
        queue_nt = 0;
        send_state_can(OVERRIDE); // Send new queue values on CAN
        return;
      } else if (gear == GEAR_FAIL || gear == orig_gear) {
        if (millis - act_tmr >= MAX_SHIFT_DUR) {
          // Relax actuator
          ACT_DN_LAT = ACT_OFF;
          ACT_UP_LAT = ACT_OFF;
          relax_wait();

          send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXDUR);

          retry_nt++;
          break;
        }
      } else {
        // Relax actuator
        ACT_DN_LAT = ACT_OFF;
        ACT_UP_LAT = ACT_OFF;
        relax_wait();

        send_errno_CAN_msg(GCM_ID, ERR_GCM_MISSNEUT);

        retry_nt++;
        break;
      }

      // Do main loop functions while waiting
      main_loop_misc();
    }

    if (retry_nt > MAX_RETRY) {
      queue_nt = 0;
      retry_nt = 0;
      send_errno_CAN_msg(GCM_ID, ERR_GCM_MAXRETRY);
      send_state_can(OVERRIDE); // Send new queue values on CAN
    }
  }
}

/**
 * void do_shift_gear_fail(uint8_t shift_enum)
 *
 * Performs the requested shift sequence in gear sensor
 * failure mode.
 *
 * TODO: Combine this with normal do_shift function?
 *
 * @param shift_enum The type of requested shift
 */
void do_shift_gear_fail(uint8_t shift_enum) {
  const uint8_t SHIFT_UP = shift_enum == SHIFT_ENUM_UP;
  const uint8_t SHIFT_DN = shift_enum == SHIFT_ENUM_DN;
  const uint8_t SHIFT_NT = shift_enum == SHIFT_ENUM_NT;

  if (SHIFT_UP || SHIFT_DN) {
    /**
     * Perform an upshift/downshift
     */

    if (!check_shift_conditions(shift_enum)) {
      switch (shift_enum) {
        case SHIFT_ENUM_UP:
          queue_up = 0;
          retry_up = 0;
          break;
        case SHIFT_ENUM_DN:
          queue_dn = 0;
          retry_dn = 0;
          break;
      }
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    // Fire actuator
    if (SHIFT_UP) {
      ACT_UP_LAT = ACT_ON;
    } else if (SHIFT_DN) {
      ACT_DN_LAT = ACT_ON;
    }
    act_tmr = millis;

    // Wait a bit and signal the ECU to cut power
    while (millis - act_tmr < PWR_CUT_WAIT) {
      main_loop_misc();
    }
    send_power_cut(CUT_START);

    while (1) {
      if ((SHIFT_UP && millis - act_tmr >= UP_SHIFT_DUR) ||
          (SHIFT_DN && millis - act_tmr >= DN_SHIFT_DUR)) {
        // Relax actuator
        if (SHIFT_UP) {
          ACT_UP_LAT = ACT_OFF;
          retry_up = 0;
        } else if (SHIFT_DN){
          ACT_DN_LAT = ACT_OFF;
          retry_dn = 0;
        }

        send_power_cut(CUT_END);

        relax_wait();

        // Decrement queued shifts value
        if (SHIFT_UP) {
          queue_up--;
        } else if (SHIFT_DN){
          queue_dn--;
        }
        send_state_can(OVERRIDE); // Send new queue values on CAN

        return;
      }

      // Also do main loop functions during "busy" loop
      main_loop_misc();
    }
  } else if (SHIFT_NT) {
    /**
     * Perform a neutral shift
     */

    if (!check_shift_conditions(shift_enum)) {
      queue_nt = 0;
      retry_nt = 0;
      send_state_can(OVERRIDE); // Send new queue values on CAN
      return;
    }

    // Fire actuator
    if (gear_fail_nt_shift == SHIFT_ENUM_UP) {
      ACT_UP_LAT = ACT_ON;
    } else if (gear_fail_nt_shift == SHIFT_ENUM_DN) {
      ACT_DN_LAT = ACT_ON;
    }
    act_tmr = millis;

    while (1) {
      if (millis - act_tmr >= NT_SHIFT_DUR) {
        // Relax actuator
        ACT_DN_LAT = ACT_OFF;
        ACT_UP_LAT = ACT_OFF;
        relax_wait();

        retry_nt = 0;
        queue_nt = 0;
        send_state_can(OVERRIDE); // Send new queue values on CAN
        return;
      }

      // Do main loop functions while waiting
      main_loop_misc();
    }
  }
}

//========================== UTILITY FUNCTIONS =================================

/**
 * void relax_wait(void)
 *
 * Waits for the actuator to return to the relaxed position
 */
void relax_wait(void) {
  // Wait for RELAX_WAIT millis and do main loop functions in the process
  uint32_t relax_wait_tmr = millis;
  while (millis - relax_wait_tmr < RELAX_WAIT) {
    main_loop_misc();
  }
}

/**
 * void main_loop_misc(void)
 *
 * Do non-shift logic main-loop functions. Typically run
 * while waiting in a blocking section of the shifting logic.
 */
void main_loop_misc(void) {
  debounce_switches();

  sample_temp();
  sample_sensors(SHIFTING);

  // Send power cut message again if MoTeC hasn't received it
  if ((shift_force_spoof != shift_force_ecu) && (millis - pwr_cut_retry_tmr >= CUT_RETRY_WAIT)) {
    send_power_cut(CUT_RESEND);
  }

  send_diag_can();
  send_state_can(NO_OVERRIDE);
}

/**
 * void debounce_switches(void)
 *
 * Keeps track of switch state in order to debounce switch inputs. The inputs
 * are debounced collectively, rather than debouncing each individual switch
 * input separately.
 */
void debounce_switches(void) {
  uint8_t switch_raw = 0x0 |
    SHIFT_UP_RAW << 2 |
    SHIFT_DN_RAW << 1 |
    SHIFT_NT_RAW << 0;

  if (switch_raw != switch_prev) {
    switch_debounce_tmr = millis;
  } else if (millis - switch_debounce_tmr >= DEBOUNCE_WAIT) {
    switch_debounced = switch_raw;
  }

  switch_prev = switch_raw;
}

/**
 * void send_power_cut(uint8_t is_start)
 *
 * Sends the ECU a message to perform a power cut, allowing us to upshift
 * without lifting or clutching.
 *
 * @param is_start Used to start a cut, end it, or resend the message
 */
void send_power_cut(uint8_t is_start) {
  if (is_start == CUT_START) {
    shift_force_spoof = PWR_CUT_SPOOF;
  } else if (is_start == CUT_END) {
    shift_force_spoof = 0;
  }

  CAN_data data = {0};
  data.halfword0 = ADL_IDX_10_12;
  data.halfword1 = shift_force_spoof;
  CAN_send_message(ADL_ID, 4, data);
  pwr_cut_retry_tmr = millis;
}
