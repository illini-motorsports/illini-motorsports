/**
 * PDM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "PDM.h"

// Count number of seconds and milliseconds since start of code execution
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

// State/status variables determined by various sources
volatile double eng_rpm, oil_pres, oil_temp, eng_temp, bat_volt_ecu, brk_press = 0; // From ECU
uint16_t total_current_draw = 0;
uint8_t fuel_prime_flag, over_temp_flag = 0;
uint8_t kill_engine_flag, kill_car_flag = 0;
uint8_t crit_volt_pending, crit_oilpres_pending, crit_oiltemp_pending,
        crit_engtemp_pending = 0;
uint8_t wtr_override_sw, fan_override_sw, fuel_override_sw, ack_btn = 0;
uint8_t wtr_override, fan_override, fuel_override, abs_override = 0;
uint16_t switch_debounced = 0; // Debounced (safe) switch state
uint16_t switch_prev = 0;      // Previous sampled value of switches
int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]
uint16_t rail_vbat, rail_12v, rail_5v, rail_3v3 = 0; // Sampled rail voltages
uint8_t load_state_changed = 0;
uint16_t ad7490_samples[AD7490_NUM_CHN] = {0}; // Sample data from ad7490
SPIConn* ad7490_connection = {0}; // ADC Connection pointer
SPIConn* gpio_connection = {0};   // GPIO Connection pointer

// Load-specific state/status variables
SPIConn *rheo_connections[NUM_CTL] = {0}; // Rheostat SPI Connections
uint8_t wiper_values[NUM_CTL] = {0};      // Rheostat wiper values
uint8_t peak_wiper_values[NUM_CTL] = {0}; // Rheostat peak-mode wiper values
uint8_t peak_state[NUM_LOADS] = {0};      // Stores whether each load is currently in peak mode
double fb_resistances[NUM_LOADS] = {0.0};   // Stores the calculated FB pin resistance for each load
uint16_t load_current[NUM_LOADS] = {0};   // Stores the sampled current draw values for each load
uint8_t overcurrent_flag[NUM_LOADS] = {NO_OVERCRT}; // Overcurrent state
uint8_t overcurrent_count[NUM_LOADS] = {0};
uint32_t overcurrent_tmr[NUM_LOADS] = {0};
uint32_t load_tmr[NUM_LOADS] = {0};       // Millis timestamp of when load was last enabled

// Timing interval variables
volatile uint32_t CAN_recv_tmr, motec0_recv_tmr, motec1_recv_tmr,
         motec2_recv_tmr, override_sw_tmr = 0;
uint32_t fuel_prime_tmr = 0;
uint32_t diag_send_tmr, diag_state_send_tmr, rail_volt_send_tmr,
         load_current_send_tmr, cutoff_send_tmr, overcrt_count_send_tmr = 0;
uint32_t temp_samp_tmr, ext_adc_samp_tmr = 0;
uint32_t switch_debounce_tmr, overcrt_chk_tmr = 0;
uint32_t crit_volt_tmr, crit_oilpres_tmr, crit_oiltemp_tmr, crit_engtemp_tmr, crit_idle_tmr = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(0); // Initialize oscillator configuration bits
  init_timer2(); // Initialize timer2 (millis)
  init_adc(NULL); // Initialize ADC module
  init_termination(TERMINATING); // Initialize programmable CAN termination
  init_can(); // Initialize CAN
  init_rheostats(); // Initialize SPI interface for digital rheostats

  // Initialize AD7490 external ADC chip
  CS_AD7490_LAT = 1;
  CS_AD7490_TRIS = OUTPUT;
  ad7490_connection = init_ad7490(5, CS_AD7490_LATBITS, CS_AD7490_LATNUM);
  gpio_connection = init_mcp23s17(2, CS_GPIO_LATBITS, CS_GPIO_LATNUM);

  //TODO: USB
  //TODO: NVM

  // Set EN pins to outputs
  EN_FUEL_TRIS = OUTPUT;
  EN_IGN_TRIS = OUTPUT;
  EN_INJ_TRIS = OUTPUT;
  EN_ABS_TRIS = OUTPUT;
  EN_PDLU_TRIS = OUTPUT;
  EN_PDLD_TRIS = OUTPUT;
  EN_FAN_TRIS = OUTPUT;
  EN_WTR_TRIS = OUTPUT;
  EN_ECU_TRIS = OUTPUT;
  EN_AUX_TRIS = OUTPUT;
  EN_BVBAT_TRIS = OUTPUT;
  EN_STR_TRIS = OUTPUT;

  // Set some EN pins to digital (necessary to read from PORT)
  EN_FUEL_ANSEL = DIG_INPUT;
  EN_ABS_ANSEL = DIG_INPUT;
  EN_WTR_ANSEL = DIG_INPUT;
  EN_ECU_ANSEL = DIG_INPUT;
  EN_AUX_ANSEL = DIG_INPUT;

  // Turn off all loads
  EN_FUEL_LAT = PWR_OFF;
  EN_IGN_LAT = PWR_OFF;
  EN_INJ_LAT = PWR_OFF;
  EN_ABS_LAT = PWR_OFF;
  EN_PDLU_LAT = PWR_OFF;
  EN_PDLD_LAT = PWR_OFF;
  EN_FAN_LAT = PWR_OFF;
  EN_WTR_LAT = PWR_OFF;
  EN_ECU_LAT = PWR_OFF;
  EN_AUX_LAT = PWR_OFF;
  EN_BVBAT_LAT = PWR_OFF;
  EN_STR_LAT = PWR_OFF;

  // Set !SW pins to inputs
  SW1_TRIS = INPUT;
  SW2_TRIS = INPUT;
  SW3_TRIS = INPUT;
  SW4_TRIS = INPUT;
  SW5_TRIS = INPUT;
  SW6_TRIS = INPUT;
  SW7_TRIS = INPUT;
  SW8_TRIS = INPUT;
  SW9_TRIS = INPUT;
  KILL_TRIS = INPUT;

  // Set !SW pins to digital inputs
  SW1_ANSEL = DIG_INPUT;
  SW2_ANSEL = DIG_INPUT;
  SW3_ANSEL = DIG_INPUT;
  SW4_ANSEL = DIG_INPUT;
  SW5_ANSEL = DIG_INPUT;
  SW6_ANSEL = DIG_INPUT;
  SW7_ANSEL = DIG_INPUT;
  SW8_ANSEL = DIG_INPUT;
  SW9_ANSEL = DIG_INPUT;
  KILL_ANSEL = DIG_INPUT;

  // Set TRIS registers - !CS
  CS_FUEL_TRIS = OUTPUT;
  CS_IGN_TRIS = OUTPUT;
  CS_INJ_TRIS = OUTPUT;
  CS_ABS_TRIS = OUTPUT;
  CS_PDLU_TRIS = OUTPUT;
  CS_PDLD_TRIS = OUTPUT;
  CS_FAN_TRIS = OUTPUT;
  CS_WTR_TRIS = OUTPUT;
  CS_ECU_TRIS = OUTPUT;
  CS_AUX_TRIS = OUTPUT;
  CS_BVBAT_TRIS = OUTPUT;

  // Set all !CS signals high
  CS_FUEL_LAT = 1;
  CS_IGN_LAT = 1;
  CS_INJ_LAT = 1;
  CS_ABS_LAT = 1;
  CS_PDLU_LAT = 1;
  CS_PDLD_LAT = 1;
  CS_FAN_LAT = 1;
  CS_WTR_LAT = 1;
  CS_ECU_LAT = 1;
  CS_AUX_LAT = 1;
  CS_BVBAT_LAT = 1;

  // Disconnect terminal A from resistor network for all rheostats
  send_all_rheo(0b0100000011111011); // TCON0bits.R0A = 0

  // Initialize normal and peak wiper values to general settings
  uint32_t i;
  for (i = 0; i < NUM_CTL; i++) {
    double fb_resistance = (load_cutoff[i] == 0.0) ? 5000.0
      : (load_current_ratios[i] * VFB_CUTOFF) / load_cutoff[i];
    wiper_values[i] = res_to_wpr(fb_resistance);
    fb_resistance = (load_peak_cutoff[i] == 0.0) ? 5000.0
      : (load_current_ratios[i] * VFB_CUTOFF) / load_peak_cutoff[i];
    peak_wiper_values[i] = res_to_wpr(fb_resistance);
  }

  // Set each rheostat to the correct value
  for (i = 0; i < NUM_CTL; i++) {
    set_rheo(wiper_values[i], rheo_connections[i]);
    peak_state[i] = 0;
    fb_resistances[i] = wpr_to_res(wiper_values[i]);
  }
  fb_resistances[STR_IDX] = 500.0; // The STR load uses a 500 Ohm resistor

  // Turn on state-independent loads
//  enable_load(AUX_IDX);
  enable_load(BVBAT_IDX);
  enable_load(ECU_IDX);

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  // Main loop
  while (1) {
    load_state_changed = 0;
    prevent_engine_blowup();
    STI(); // Enable interrupts (in case anything disabled without re-enabling)

    /**
     * Determine if the fuel pump should be priming
     */
    if (!ON_SW) {
      fuel_prime_flag = 1;
    } else if (millis - fuel_prime_tmr > FUEL_PRIME_DUR && FUEL_EN) {
      fuel_prime_flag = 0;
    }

    /**
     * Determine if the car is experiencing an over-temperature condition
     */
    if (over_temp_flag) {
      if (eng_temp < FAN_THRESHOLD_L) {
        over_temp_flag = 0;
      }
    } else {
      if (eng_temp > FAN_THRESHOLD_H) {
        over_temp_flag = 1;
      }
    }
    
    /**
     * Reset override switches if the CAN variables are stale
     */
    if (millis - override_sw_tmr >= OVERRIDE_SW_WAIT) {
      fan_override_sw = 0;
      wtr_override_sw = 0;
      fuel_override_sw = 0;
    }

    /**
     * Load override logic
     */

    wtr_override = !WATER_SW || wtr_override_sw;
    fan_override = !FAN_SW || fan_override_sw;
    abs_override = ABS_SW;
    fuel_override = !FUEL_SW || fuel_override_sw;

    /**
     * Reset idle tmr if engine is on
     */
    if (eng_rpm > 0) {
      crit_idle_tmr = millis;
    }

    /**
     * Respond to critical errors
     */
    if (kill_car_flag || kill_engine_flag) {
      if (kill_car_flag) {
        uint8_t i;
        for (i = 0; i < NUM_LOADS; i++) {
          disable_load(i);
        }
      } else if (kill_engine_flag) {
        disable_load(FUEL_IDX);
        disable_load(IGN_IDX);
        disable_load(INJ_IDX);
        disable_load(STR_IDX);
      }
    } else {

      /**
       * Toggle state-dependent loads
       */
      if (millis - CAN_recv_tmr > BASIC_CONTROL_WAIT ||
          millis - motec0_recv_tmr > BASIC_CONTROL_WAIT ||
          millis - motec1_recv_tmr > BASIC_CONTROL_WAIT ||
          millis - motec2_recv_tmr > BASIC_CONTROL_WAIT) {

        /**
         * Perform basic load control
         *
         * IGN, INJ, FUEL, WTR, and FAN will turn on when the ON_SW is in the on
         * position and turn off when the ON_SW is in the off position. Other
         * loads will still be controlled normally as they do not depend on CAN.
         * The kill switch will disable the aforementioned loads regardless of the
         * position of the ON_SW.
         */

        set_load(IGN_IDX, ON_SW && !KILL_SW);
        set_load(INJ_IDX, ON_SW && !KILL_SW);
        set_load(FUEL_IDX, (ON_SW && !KILL_SW) || fuel_override);
        set_load(WTR_IDX, (ON_SW && !KILL_SW && !STR_EN) || wtr_override);
        set_load(FAN_IDX, (ON_SW && !KILL_SW && !STR_EN) || fan_override);

        // STR
        if (STR_SW && (millis - load_tmr[STR_IDX] < STR_MAX_DUR)) {
          enable_load(STR_IDX);
        } else {
          if (!STR_SW) { load_tmr[STR_IDX] = millis; }
          disable_load(STR_IDX);
        }
      } else {

        /**
         * Perform regular load control
         */

        set_load(IGN_IDX, ON_SW && !KILL_SW);
        set_load(INJ_IDX, ON_SW && !KILL_SW);
        set_load(FUEL_IDX, ON_SW && !KILL_SW &&
            (ENG_ON || fuel_prime_flag || fuel_override || STR_EN));
        set_load(WTR_IDX, !STR_EN &&
            (ENG_ON || over_temp_flag || wtr_override_sw || wtr_override));
        set_load(FAN_IDX, !STR_EN && (over_temp_flag || fan_override));
        //set_load(AUX_IDX, ack_btn || brk_press > BRK_SWITCH);

        // STR
        if (STR_SW && (millis - load_tmr[STR_IDX] < STR_MAX_DUR)) {
          enable_load(STR_IDX);
        } else {
          if (!STR_SW) { load_tmr[STR_IDX] = millis; }
          disable_load(STR_IDX);
        }
      }

      /**
       * Toggle loads that do not depend on CAN variables
       */

      // PDLU
      if (ACT_UP_SW && !ACT_DN_SW && !KILL_SW &&
          (millis - load_tmr[PDLU_IDX] < PDL_MAX_DUR)) {
        enable_load(PDLU_IDX);
      } else {
        if (!ACT_UP_SW) { load_tmr[PDLU_IDX] = millis; }
        disable_load(PDLU_IDX);
      }

      // PDLD
      if (ACT_DN_SW && !ACT_UP_SW && !KILL_SW &&
          (millis - load_tmr[PDLD_IDX] < PDL_MAX_DUR)) {
        enable_load(PDLD_IDX);
      } else {
        if (!ACT_DN_SW) { load_tmr[PDLD_IDX] = millis; }
        disable_load(PDLD_IDX);
      }

      // ABS
      //set_load(ABS_IDX, abs_override && !KILL_SW);
      enable_load(ABS_IDX);
    }

    /**
     * Call helper functions
     */

    // Separate logic functions
    debounce_switches();
    check_peak_timer();
    check_load_overcurrent();

    // Analog sampling functions
    sample_temp();
    sample_ext_adc();

    // CAN message sending functions
    send_diag_can();
    send_diag_state_can(load_state_changed ? OVERRIDE : NO_OVERRIDE);
    send_rail_volt_can();
    send_load_current_can();
    send_cutoff_values_can(NO_OVERRIDE);
    send_overcrt_count_can(NO_OVERRIDE);
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
  // Declare local variables
  uint8_t load_idx, peak_mode, switch_bitmap = 0;
  double cutoff = 0;

  CAN_recv_tmr = millis; // Record time of latest received CAN message

  switch (msg.id) {
    case MOTEC_ID + 0:
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) |
            msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      bat_volt_ecu = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) |
            msg.data[VOLT_ECU_BYTE + 1])) * VOLT_ECU_SCL;

      motec0_recv_tmr = millis;
      break;
    case MOTEC_ID + 1:
      eng_temp = ((double) ((msg.data[ENG_TEMP_BYTE] << 8) |
            msg.data[ENG_TEMP_BYTE + 1])) * ENG_TEMP_SCL;
      oil_temp = ((double) ((msg.data[OIL_TEMP_BYTE] << 8) |
            msg.data[OIL_TEMP_BYTE + 1])) * OIL_TEMP_SCL;

      motec1_recv_tmr = millis;
      break;
    case MOTEC_ID + 2:
      oil_pres = ((double) ((msg.data[OIL_PRES_BYTE] << 8) |
            msg.data[OIL_PRES_BYTE + 1])) * OIL_PRES_SCL;

      motec2_recv_tmr = millis;
      break;
    case MOTEC_ID + 6:
      brk_press = ( ((msg.data[6] << 8) |
            msg.data[7]));
      break;

    case WHEEL_ID + 0x1:
      switch_bitmap = msg.data[SWITCH_BITS_BYTE];
      //fan_override_sw = switch_bitmap & (1 << (FAN_OVR_BITPOS - 1));
      wtr_override_sw = switch_bitmap & (1 << (WTR_OVR_BITPOS - 1));
      fuel_override_sw = switch_bitmap & (1 << (FUEL_OVR_BITPOS - 1));
      ack_btn = switch_bitmap & (1 << ACK_BTN_BITPOS);
      override_sw_tmr = millis;
      break;
  }
}
/**
 * void debounce_switches(void)
 *
 * Keeps track of switch state in order to debounce switch inputs. The inputs
 * are debounced collectively, rather than debouncing each individual switch
 * input separately.
 */
void debounce_switches(void) {
  uint16_t switch_raw = 0x0 |
    STR_SW_RAW << 9 |
    ON_SW_RAW << 8 |
    ACT_UP_SW_RAW << 7 |
    ACT_DN_SW_RAW << 6 |
    KILL_SW_RAW << 5 |
    ABS_SW_RAW << 4 |
    FUEL_SW_RAW << 3 |
    WATER_SW_RAW << 2 |
    FAN_SW_RAW << 1|
    AUX_SW_RAW << 0;

  if (switch_raw != switch_prev) {
    switch_debounce_tmr = millis;
  } else if (millis - switch_debounce_tmr >= DEBOUNCE_WAIT) {
    switch_debounced = switch_raw;
  }

  switch_prev = switch_raw;
}

/**
 * void check_peak_timer(void)
 *
 * For each load, check to see if it is currently in peak mode and if it has
 * exceeded the peak mode duration. If so, set the rheostats back to normal mode.
 */
void check_peak_timer(void) {
  uint8_t idx;
  for (idx = 0; idx < NUM_CTL; idx++) {
    if (load_enabled(idx) && peak_state[idx] &&
        (millis - load_tmr[idx] > load_peak_duration[idx])) {
      uint16_t wpr_val = wiper_values[idx];
      set_rheo(wpr_val, rheo_connections[idx]);
      fb_resistances[idx] = wpr_to_res(wpr_val);
      peak_state[idx] = 0;
      load_state_changed = 1;
    }
  }
}

/**
 * void check_load_overcurrent(void)
 *
 * If a load is enabled but drawing zero (or close to zero) current, we can
 * reasonably assume that it has overcurrented. If this is the case, toggle
 * the enable pin of the relevant MOSFET to reset the load.
 */
void check_load_overcurrent(void) {
  if (millis - overcrt_chk_tmr >= OVERCRT_CHK_INTV) {
    uint8_t load_idx;
    for (load_idx = 0; load_idx < NUM_CTL; load_idx++) {
      switch (overcurrent_flag[load_idx]) {

        case NO_OVERCRT:
          if (load_enabled(load_idx) &&
              load_current[load_idx] < OVERCRT_DETECT) {
            overcurrent_flag[load_idx] = OVERCRT;
            overcurrent_tmr[load_idx] = millis;
          }
          break;

        case OVERCRT:
          if (load_enabled(load_idx) &&
              load_current[load_idx] < OVERCRT_DETECT) {
            if (millis - overcurrent_tmr[load_idx] >= OVERCRT_RESET_WAIT) {
              disable_load(load_idx);
              overcurrent_flag[load_idx] = OVERCRT_RESET;
              overcurrent_tmr[load_idx] = millis;
              overcurrent_count[load_idx]++;
              send_overcrt_count_can(OVERRIDE);
              send_diag_state_can(OVERRIDE);
            }
          } else {
            overcurrent_flag[load_idx] = NO_OVERCRT;
          }
          break;

        case OVERCRT_RESET:
          if (millis != overcurrent_tmr[load_idx]) {
            enable_load(load_idx);
            overcurrent_flag[load_idx] = NO_OVERCRT;
            send_diag_state_can(OVERRIDE);
          }
          break;
      }
    }
  }
}

/**
 * void prevent_engine_blowup(void);
 *
 * Checks various state variables and ensures they are within an acceptable
 * range. If not, kill the entire car or just the engine as needed.
 */
void prevent_engine_blowup(void) {
  if (COMP) {
    kill_engine_flag = 0;
    kill_car_flag = 0;
    return; // Godspeed
  }

  // Kill car if battery voltage too low
  if (rail_vbat <= CRIT_VOLTAGE) {
    if (!crit_volt_pending) {
      crit_volt_tmr = millis;
    }
    crit_volt_pending = 1;

    if (millis - crit_volt_tmr >= CRIT_VOLT_WAIT) {
      send_errno_CAN_msg(PDM_ID, ERR_PDM_CRITVOLT);
      kill_car_flag = 1;
      return;
    }
  } else {
    crit_volt_pending = 0;
  }

  // Kill car if it has been idling for too long
  // Someone has mistakenly left it on
  if (millis - crit_idle_tmr >= MAX_IDLE_TIME) {
    kill_car_flag = 1;
    return;
  }

  if (millis - motec0_recv_tmr < BASIC_CONTROL_WAIT &&
      millis - motec1_recv_tmr < BASIC_CONTROL_WAIT &&
      millis - motec2_recv_tmr < BASIC_CONTROL_WAIT &&
      eng_rpm >= RPM_CRIT_CHECK) {

    // Kill engine if oil pressure too low
    if (oil_pres <= CRIT_OILPRES) {
      if (!crit_oilpres_pending) {
        crit_oilpres_tmr = millis;
      }
      crit_oilpres_pending = 1;

      if (millis - crit_oilpres_tmr >= CRIT_OILPRES_WAIT) {
        send_errno_CAN_msg(PDM_ID, ERR_PDM_CRITOILPRES);
        kill_engine_flag = 1;
        return;
      }
    } else {
      crit_oilpres_pending = 0;
    }

    // Kill engine if oil temp too high
    if (oil_temp >= CRIT_OILTEMP) {
      if (!crit_oiltemp_pending) {
        crit_oiltemp_tmr = millis;
      }
      crit_oiltemp_pending = 1;

      if (millis - crit_oiltemp_tmr >= CRIT_OILTEMP_WAIT) {
        send_errno_CAN_msg(PDM_ID, ERR_PDM_CRITOILTEMP);
        kill_engine_flag = 1;
        return;
      }
    } else {
      crit_oiltemp_pending = 0;
    }

    // Kill engine if engine temp too high
    if (eng_temp >= CRIT_ENGTEMP) {
      if (!crit_engtemp_pending) {
        crit_engtemp_tmr = millis;
      }
      crit_engtemp_pending = 1;

      if (millis - crit_engtemp_tmr >= CRIT_ENGTEMP_WAIT) {
        send_errno_CAN_msg(PDM_ID, ERR_PDM_CRITENGTEMP);
        kill_engine_flag = 1;
        return;
      }
    } else {
      crit_engtemp_pending = 0;
    }
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
 * void sample_ext_adc(void)
 *
 * Reads each channel of the external ADC module to determine the current
 * draw of each load and the voltage of each rail, if the interval has passed.
 */
void sample_ext_adc(void) {
  if (millis - ext_adc_samp_tmr >= EXT_ADC_SAMP_INTV) {
    ad7490_read_channels(ad7490_samples, ad7490_connection);
    total_current_draw = 0;

    uint8_t i;
    for (i = 0; i < NUM_CTL; i++) {
      load_current[i] = (uint16_t) ((adc_to_volt(ad7490_samples[ADC_CHN[i]]) * SCL_INV
            * load_current_ratios[i]) / fb_resistances[i]);
      total_current_draw += ((uint16_t) ((double) load_current[i]) * (SCL_INV_LRG / SCL_INV));
    }

    load_current[STR_IDX] = (uint16_t) ((adc_to_volt(ad7490_samples[ADC_CHN[STR_IDX]])
          * SCL_INV_LRG * load_current_ratios[STR_IDX])
          / fb_resistances[STR_IDX]);
    total_current_draw += load_current[STR_IDX];

    rail_vbat = (uint16_t) (adc_to_volt(ad7490_samples[12]) * 4.0 * SCL_INV);
    rail_12v  = (uint16_t) (adc_to_volt(ad7490_samples[13]) * 4.0 * SCL_INV);
    rail_5v   = (uint16_t) (adc_to_volt(ad7490_samples[14]) * 2.0 * SCL_INV);
    rail_3v3  = (uint16_t) (adc_to_volt(ad7490_samples[15]) * 1.0 * SCL_INV);

    ext_adc_samp_tmr = millis;
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

    CAN_send_message(PDM_ID + 0, 6, data);
    diag_send_tmr = millis;
  }
}

/**
 * void send_diag_state_can(void)
 *
 * If the interval has passed or the caller overrode the check, send out the
 * enablity and peak mode state of all loads on CAN. Also send total current
 * draw, switch state, and flag state.
 *
 * TODO: Restrict to a certain rate, even with override
 *
 * @param override - Whether to override the interval
 */
void send_diag_state_can(uint8_t override) {
  if ((millis - diag_state_send_tmr >= DIAG_STATE_SEND) || override) {
    CAN_data data = {0};

    // Create load enablity bitmap
    data.halfword0 = 0x0 |
      FUEL_EN << (15 - FUEL_IDX) |
      IGN_EN << (15 - IGN_IDX) |
      INJ_EN << (15 - INJ_IDX) |
      ABS_EN << (15 - ABS_IDX) |
      PDLU_EN << (15 - PDLU_IDX) |
      PDLD_EN << (15 - PDLD_IDX) |
      FAN_EN << (15 - FAN_IDX) |
      WTR_EN << (15 - WTR_IDX) |
      ECU_EN << (15 - ECU_IDX) |
      AUX_EN << (15 - AUX_IDX) |
      BVBAT_EN << (15 - BVBAT_IDX) |
      STR_EN << (15 - STR_IDX);

    // Create load peak mode bitmap
    data.halfword1 = 0x0 |
      peak_state[FUEL_IDX] << (15 - FUEL_IDX) |
      peak_state[IGN_IDX] << (15 - IGN_IDX) |
      peak_state[INJ_IDX] << (15 - INJ_IDX) |
      peak_state[ABS_IDX] << (15 - ABS_IDX) |
      peak_state[PDLU_IDX] << (15 - PDLU_IDX) |
      peak_state[PDLD_IDX] << (15 - PDLD_IDX) |
      peak_state[FAN_IDX] << (15 - FAN_IDX) |
      peak_state[WTR_IDX] << (15 - WTR_IDX) |
      peak_state[ECU_IDX] << (15 - ECU_IDX) |
      peak_state[AUX_IDX] << (15 - AUX_IDX) |
      peak_state[BVBAT_IDX] << (15 - BVBAT_IDX) |
      peak_state[STR_IDX] << (15 - STR_IDX);

    data.halfword2 = total_current_draw;

    // Create switch state bitmap
    data.byte6 = 0x0 |
      STR_SW << 7 |
      ON_SW << 6 |
      ACT_UP_SW << 5 |
      ACT_DN_SW << 4 |
      KILL_SW << 3 |
      ABS_SW << 2 |
      AUX_SW << 1 |
      FAN_SW << 0;

    // Create flag bitmap
    data.byte7 = 0x0 |
      fuel_prime_flag << 7 |
      over_temp_flag << 6 |
      kill_car_flag << 5 |
      kill_engine_flag << 4;

    CAN_send_message(PDM_ID + 0x1, 8, data);
    diag_state_send_tmr = millis;
  }
}

/**
 * void send_rail_volt_can(void)
 *
 * If the interval has passed, samples rail voltages and sends related CAN
 * messages.
 */
void send_rail_volt_can(void) {
  if(millis - rail_volt_send_tmr >= RAIL_VOLT_SEND) {
    CAN_data rail_voltage_data = {0};
    rail_voltage_data.halfword0 = rail_vbat;
    rail_voltage_data.halfword1 = rail_12v;
    rail_voltage_data.halfword2 = rail_5v;
    rail_voltage_data.halfword3 = rail_3v3;
    CAN_send_message(PDM_ID + 2, 8, rail_voltage_data);

    rail_volt_send_tmr = millis;
  }
}

/**
 * void send_load_current_can(void)
 *
 * If the interval has passed, samples current draw for each load and sends
 * related CAN messages.
 */
void send_load_current_can(void) {
  if(millis - load_current_send_tmr >= LOAD_CUR_SEND) {
    CAN_data load_current_data = {0};

    load_current_data.halfword0 = load_current[FUEL_IDX];
    load_current_data.halfword1 = load_current[IGN_IDX];
    load_current_data.halfword2 = load_current[INJ_IDX];
    load_current_data.halfword3 = load_current[ABS_IDX];
    CAN_send_message(PDM_ID + 3, 8, load_current_data);

    load_current_data.doubleword = 0;
    load_current_data.halfword0 = load_current[PDLU_IDX];
    load_current_data.halfword1 = load_current[PDLD_IDX];
    load_current_data.halfword2 = load_current[FAN_IDX];
    load_current_data.halfword3 = load_current[WTR_IDX];
    CAN_send_message(PDM_ID + 4, 8, load_current_data);

    load_current_data.doubleword = 0;
    load_current_data.halfword0 = load_current[ECU_IDX];
    load_current_data.halfword1 = load_current[AUX_IDX];
    load_current_data.halfword2 = load_current[BVBAT_IDX];
    load_current_data.halfword3 = load_current[STR_IDX];
    CAN_send_message(PDM_ID + 5, 8, load_current_data);

    load_current_send_tmr = millis;
  }
}

/**
 * void send_cutoff_values_can(void)
 *
 * If the interval has passed, send current values of peak and normal mode
 *  current cutoff.
 *
 * @param override - Whether to override the interval
 */
void send_cutoff_values_can(uint8_t override) {
  if ((millis - cutoff_send_tmr >= CUTOFF_VAL_SEND) || override) {
    CAN_data cutoff_value_data = {0};
    int i;

    // FUEL, IGN, INJ, ABS
    double cutoffs[NUM_CTL] = {0};
    uint16_t cutoffs_scl[NUM_CTL] = {0};
    for(i = 0; i < NUM_CTL; i++) {
      cutoffs[i] = (4.7 / wpr_to_res(wiper_values[i])) * load_current_ratios[i];
      cutoffs_scl[i] = (uint16_t) (cutoffs[i] * SCL_INV_CUT);
    }

    cutoff_value_data.doubleword = 0;
    cutoff_value_data.halfword0 = cutoffs_scl[0]; // FUEL
    cutoff_value_data.halfword1 = cutoffs_scl[1]; // IGN
    cutoff_value_data.halfword2 = cutoffs_scl[2]; // INJ
    cutoff_value_data.halfword3 = cutoffs_scl[3]; // ABS
    CAN_send_message(PDM_ID + 0x6, 8, cutoff_value_data);

    cutoff_value_data.doubleword = 0;
    cutoff_value_data.halfword0 = cutoffs_scl[4]; // PDLU
    cutoff_value_data.halfword1 = cutoffs_scl[5]; // PDLD
    cutoff_value_data.halfword2 = cutoffs_scl[6]; // FAN
    cutoff_value_data.halfword3 = cutoffs_scl[7]; // WTR
    CAN_send_message(PDM_ID + 0x7, 8, cutoff_value_data);

    cutoff_value_data.doubleword = 0;
    cutoff_value_data.halfword0 = cutoffs_scl[8]; // ECU
    cutoff_value_data.halfword1 = cutoffs_scl[9]; // AUX
    cutoff_value_data.halfword2 = cutoffs_scl[10];// BVBAT
    CAN_send_message(PDM_ID + 0x8, 6, cutoff_value_data);

    double fuel_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[FUEL_IDX]))
      * load_current_ratios[FUEL_IDX];
    uint16_t fuel_peak_cutoff_scl = (uint16_t) (fuel_peak_cutoff * SCL_INV_CUT);
    double fan_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[FAN_IDX]))
      * load_current_ratios[FAN_IDX];
    uint16_t fan_peak_cutoff_scl = (uint16_t) (fan_peak_cutoff * SCL_INV_CUT);
    double wtr_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[WTR_IDX]))
      * load_current_ratios[WTR_IDX];
    uint16_t wtr_peak_cutoff_scl = (uint16_t) (wtr_peak_cutoff * SCL_INV_CUT);
    double ecu_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[ECU_IDX]))
      * load_current_ratios[ECU_IDX];
    uint16_t ecu_peak_cutoff_scl = (uint16_t) (ecu_peak_cutoff * SCL_INV_CUT);

    cutoff_value_data.doubleword = 0;
    cutoff_value_data.halfword0 = fuel_peak_cutoff_scl;
    cutoff_value_data.halfword1 = fan_peak_cutoff_scl;
    cutoff_value_data.halfword2 = wtr_peak_cutoff_scl;
    cutoff_value_data.halfword3 = ecu_peak_cutoff_scl;
    CAN_send_message(PDM_ID + 0x9, 8, cutoff_value_data);

    cutoff_send_tmr = millis;
  }
}

/**
 * void send_overcrt_count_can(void)
 *
 * If the interval has passed or the caller overrode the check, send out the
 * overcurrent count of all loads on CAN.
 *
 * @param override - Whether to override the interval
 */
void send_overcrt_count_can(uint8_t override) {
  if ((millis - overcrt_count_send_tmr >= OVERCRT_COUNT_SEND) || override) {
    CAN_data data = {0};

    data.byte0 = overcurrent_count[FUEL_IDX];
    data.byte1 = overcurrent_count[IGN_IDX];
    data.byte2 = overcurrent_count[INJ_IDX];
    data.byte3 = overcurrent_count[ABS_IDX];
    data.byte4 = overcurrent_count[PDLU_IDX];
    data.byte5 = overcurrent_count[PDLD_IDX];
    data.byte6 = overcurrent_count[FAN_IDX];
    data.byte7 = overcurrent_count[WTR_IDX];
    CAN_send_message(PDM_ID + 0xA, 8, data);

    data.byte0 = overcurrent_count[ECU_IDX];
    data.byte1 = overcurrent_count[AUX_IDX];
    data.byte2 = overcurrent_count[BVBAT_IDX];
    data.byte3 = overcurrent_count[STR_IDX];
    CAN_send_message(PDM_ID + 0xB, 4, data);

    overcrt_count_send_tmr = millis;
  }
}

//========================== UTILITY FUNCTIONS =================================

/**
 * void enable_load(uint8_t load_idx)
 *
 * Check if the specified load is currently disabled, and if so, enable it.
 *
 * When enabling an inductive load, set the peak current limit and set the
 * peak timer. The peak current limit will be disabled at a set time later.
 */
void enable_load(uint8_t load_idx) {
  if (load_peak_cutoff[load_idx] == 0.0) {
    if (!load_enabled(load_idx) && overcurrent_flag[load_idx] != OVERCRT_RESET) {
      set_en_load(load_idx, PWR_ON);
      load_tmr[load_idx] = millis;
      load_state_changed = 1;
    }
  } else  {
    if (!load_enabled(load_idx) && overcurrent_flag[load_idx] != OVERCRT_RESET) {
      uint16_t peak_wpr_val = peak_wiper_values[load_idx];
      set_rheo(peak_wpr_val, rheo_connections[load_idx]);
      fb_resistances[load_idx] = wpr_to_res(peak_wpr_val);
      peak_state[load_idx] = 1;

      set_en_load(load_idx, PWR_ON);
      load_tmr[load_idx] = millis;
      load_state_changed = 1;

      if (load_idx == FUEL_IDX) { fuel_prime_tmr = millis; }
    }
  }
}

/**
 * void disable_load(uint8_t load_idx)
 *
 * Check if the specified load is currently enabled, and if so, disable it.
 */
void disable_load(uint8_t load_idx) {
  if (load_enabled(load_idx)) {
    set_en_load(load_idx, PWR_OFF);
    load_state_changed = 1;
    peak_state[load_idx] = 0;
  }
}

/**
 * void set_load(uint8_t load_idx, uint8_t condition)
 *
 * Enable or disable a load based on some condition.
 */
void set_load(uint8_t load_idx, uint8_t condition) {
  if (condition) {
    enable_load(load_idx);
  } else {
    disable_load(load_idx);
  }
}

/**
 * Functions to convert a wiper value to the expected resistance of the rheostat
 * and vice versa.
 *
 * Luckily, the conversion is mostly linear, so a linear regression approximates
 * the value well. These values were determined experimentally and should give a
 * good general idea of the FB pin resistance, but there will be some error. The
 * error between measured and calculated current cut-off due to this error is
 * limited to ~0.5A worst case, and this is at the high end of the cut-off range.
 * So, the error should not be a problem. Refer to the "Digital Rheostat V+
 * Selection" tab of the "PCB Info" document for more info.
 */

/**
 * Converts a wiper value to the expected resistance of the rheostat.
 *
 * @param wpr The wiper value to convert
 * @return The expected resistance
 */
double wpr_to_res(uint8_t wpr) {
  return (19.11639223 * wpr) + 256.6676635;
}

/**
 * Converts a rheostat resistance to the wiper value.
 *
 * @param res The resistance to convert
 * @return The wiper value of the rheostat
 */
uint8_t res_to_wpr(double res) {
  double wiper = 0.0523111 * (res - 256.6676635);
  wiper = (wiper < 0) ? 0 : wiper;
  wiper = (wiper > 255) ? 255 : wiper;
  return ((uint8_t) wiper);
}

/**
 * Converts raw ADC output to a voltage.
 *
 * @param adc_val The raw ADC output
 * @return The corresponding voltage
 */
double adc_to_volt(uint16_t adc_val) {
  return (adc_val/EXT_ADC_NUM_STEPS)*EXT_ADC_VOLT_RANGE;
}

/**
 * uint8_t load_enabled(uint8_t load_idx)
 *
 * Returns whether or not the specified load is enabled.
 *
 * @param load_idx- Index of the load to get the enablity for
 * @return Whether or not the specified load is enabled
 */
uint8_t load_enabled(uint8_t load_idx) {
  switch (load_idx) {
    case FUEL_IDX: return FUEL_EN;
    case IGN_IDX: return IGN_EN;
    case INJ_IDX: return INJ_EN;
    case ABS_IDX: return ABS_EN;
    case PDLU_IDX: return PDLU_EN;
    case PDLD_IDX: return PDLD_EN;
    case FAN_IDX: return FAN_EN;
    case WTR_IDX: return WTR_EN;
    case ECU_IDX: return ECU_EN;
    case AUX_IDX: return AUX_EN;
    case BVBAT_IDX: return BVBAT_EN;
    case STR_IDX: return STR_EN;
    default: return 0;
  }
}

/**
 * void set_en_load(uint8_t load_idx, uint8_t load_state)
 *
 * Sets the specified load to the specified enablity state
 *
 * @param load_idx- The load to set
 * @param load_state- The new state
 */
void set_en_load(uint8_t load_idx, uint8_t load_state) {
  switch (load_idx) {
    case FUEL_IDX: EN_FUEL_LAT = load_state; break;
    case IGN_IDX: EN_IGN_LAT = load_state; break;
    case INJ_IDX: EN_INJ_LAT = load_state; break;
    case ABS_IDX: EN_ABS_LAT = load_state; break;
    case PDLU_IDX: EN_PDLU_LAT = load_state; break;
    case PDLD_IDX: EN_PDLD_LAT = load_state; break;
    case FAN_IDX: EN_FAN_LAT = load_state; break;
    case WTR_IDX: EN_WTR_LAT = load_state; break;
    case ECU_IDX: EN_ECU_LAT = load_state; break;
    case AUX_IDX: EN_AUX_LAT = load_state; break;
    case BVBAT_IDX: EN_BVBAT_LAT = load_state; break;
    case STR_IDX: EN_STR_LAT = load_state; break;
    default: return;
  }
}

void init_rheostats(void) {
  rheo_connections[0] = init_rheo(1, CS_FUEL_LATBITS, CS_FUEL_LATNUM);
  rheo_connections[1] = init_rheo(1, CS_IGN_LATBITS, CS_IGN_LATNUM);
  rheo_connections[2] = init_rheo(1, CS_INJ_LATBITS, CS_INJ_LATNUM);
  rheo_connections[3] = init_rheo(1, CS_ABS_LATBITS, CS_ABS_LATNUM);
  rheo_connections[4] = init_rheo(1, CS_PDLU_LATBITS, CS_PDLU_LATNUM);
  rheo_connections[5] = init_rheo(1, CS_PDLD_LATBITS, CS_PDLD_LATNUM);
  rheo_connections[6] = init_rheo(1, CS_FAN_LATBITS, CS_FAN_LATNUM);
  rheo_connections[7] = init_rheo(1, CS_WTR_LATBITS, CS_WTR_LATNUM);
  rheo_connections[8] = init_rheo(1, CS_ECU_LATBITS, CS_ECU_LATNUM);
  rheo_connections[9] = init_rheo(1, CS_AUX_LATBITS, CS_AUX_LATNUM);
  rheo_connections[10] = init_rheo(1, CS_BVBAT_LATBITS, CS_BVBAT_LATNUM);
}