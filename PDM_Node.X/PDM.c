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

// Car status variables reported over can from the ECU
volatile double eng_rpm, oil_pres, oil_temp, eng_temp, bat_volt_ecu = 0;

// Stores wiper values for each load
uint8_t wiper_values[NUM_LOADS] = {0};
uint8_t peak_wiper_values[NUM_LOADS] = {0};

// Stores whether each load is currently in peak mode
uint8_t peak_state[NUM_LOADS] = {0};

// Stores the calculated FB pin resistance for each load
double fb_resistances[NUM_LOADS] = {0.0};

// Stores the sampled current draw values for each load
uint16_t load_current[NUM_LOADS] = {0};

// Recorded values from temperature sensors
int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

// State variables determined by various sources
uint8_t fuel_prime_flag = 0;
uint8_t over_temp_flag = 0;
uint16_t total_current_draw = 0;
uint8_t wtr_override_sw, fan_override_sw, fuel_override_sw = 0;

// Timing interval variables
volatile uint32_t CAN_recv_tmr, motec0_recv_tmr, motec1_recv_tmr, motec2_recv_tmr = 0;
uint32_t fuel_prime_tmr = 0;
uint32_t str_en_tmr = 0;
uint32_t diag_send_tmr, rail_volt_send_tmr, load_current_send_tmr,
    cutoff_send_tmr, load_status_send_tmr = 0;
uint32_t fuel_peak_tmr, wtr_peak_tmr, fan_peak_tmr, ecu_peak_tmr = 0;
uint32_t pdlu_tmr, pdld_tmr = 0;
uint32_t temp_samp_tmr, current_samp_tmr = 0;

/**
 * For whatever reason, reading from EN_IGN_PORT does not work. Instead, we'll
 * keep track of the value with a global variable.
 */
uint8_t ign_enabled = 0;

// Initial overcurrent thresholds to use for all the loads
const double load_cutoff[NUM_LOADS] = {20.0, 5.0, 10.0, 10.0, 15.0, 20.0, 2.0,
                                       60.0, 60.0, 2.0, 2.0, 5.0, 2.0, 2.0};
const double load_peak_cutoff[NUM_LOADS] = {0.0, 0.0, 40.0, 60.0, 40.0, 100.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(); // Initialize oscillator configuration bits
  init_timer1(); // Initialize timer1 (seconds)
  init_timer2(); // Initialize timer2 (millis)
  init_spi(); // Initialize SPI interface
  init_adc(init_adc_pdm); // Initialize ADC module
  init_termination(); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  // Set EN pins to outputs
  EN_IGN_TRIS = OUTPUT;
  EN_INJ_TRIS = OUTPUT;
  EN_FUEL_TRIS = OUTPUT;
  EN_ECU_TRIS = OUTPUT;
  EN_WTR_TRIS = OUTPUT;
  EN_FAN_TRIS = OUTPUT;
  EN_AUX_TRIS = OUTPUT;
  EN_PDLU_TRIS = OUTPUT;
  EN_PDLD_TRIS = OUTPUT;
  EN_B5V5_TRIS = OUTPUT;
  EN_BVBAT_TRIS = OUTPUT;
  EN_STR_TRIS = OUTPUT;

  // Set unused EN_FUEL pin to input so it doesn't interfere
  EN_FUEL_UNUSED_TRIS = INPUT;

  // Turn off all loads
  EN_IGN_LAT = PWR_OFF;
  EN_INJ_LAT = PWR_OFF;
  EN_FUEL_LAT = PWR_OFF;
  EN_ECU_LAT = PWR_OFF;
  EN_WTR_LAT = PWR_OFF;
  EN_FAN_LAT = PWR_OFF;
  EN_AUX_LAT = PWR_OFF;
  EN_PDLU_LAT = PWR_OFF;
  EN_PDLD_LAT = PWR_OFF;
  EN_B5V5_LAT = PWR_OFF;
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
  //SW8_TRIS = INPUT;
  SW9_TRIS = INPUT;
  KILL_TRIS = INPUT;

  // Set TRIS registers - !CS
  CS_IGN_TRIS = OUTPUT;
  CS_INJ_TRIS = OUTPUT;
  CS_FUEL_TRIS = OUTPUT;
  CS_ECU_TRIS = OUTPUT;
  CS_WTR_TRIS = OUTPUT;
  CS_FAN_TRIS = OUTPUT;
  CS_AUX_TRIS = OUTPUT;
  CS_PDLU_TRIS = OUTPUT;
  CS_PDLD_TRIS = OUTPUT;
  CS_B5V5_TRIS = OUTPUT;
  CS_BVBAT_TRIS = OUTPUT;
  CS_STR0_TRIS = OUTPUT;
  CS_STR1_TRIS = OUTPUT;
  CS_STR2_TRIS = OUTPUT;

  // Set all !CS signals high
  CS_IGN_LAT = 1;
  CS_INJ_LAT = 1;
  CS_FUEL_LAT = 1;
  CS_ECU_LAT = 1;
  CS_WTR_LAT = 1;
  CS_FAN_LAT = 1;
  CS_AUX_LAT = 1;
  CS_PDLU_LAT = 1;
  CS_PDLD_LAT = 1;
  CS_B5V5_LAT = 1;
  CS_BVBAT_LAT = 1;
  CS_STR0_LAT = 1;
  CS_STR1_LAT = 1;
  CS_STR2_LAT = 1;

  // Disconnect terminal A from resistor network for all rheostats
  send_all_rheo(0b0100000011111011); // TCON0bits.R0A = 0

  // Get wiper data struct from NVM
  Wiper_nvm_data data = {0};
  read_nvm_data(&data, sizeof(Wiper_nvm_data));

  // Check to see if the wiper data in NVM has been initialized
  if(data.key != NVM_WPR_CONSTANT) {
    // Initialize normal and peak wiper values to general settings
    uint32_t i;
    for (i = 0; i < NUM_LOADS; i++) {
      double ratio = 0;

      switch (i) {
        case IGN_IDX: ratio = IGN_RATIO; break;
        case INJ_IDX: ratio = INJ_RATIO; break;
        case FUEL_IDX: ratio = FUEL_RATIO; break;
        case ECU_IDX: ratio = ECU_RATIO; break;
        case WTR_IDX: ratio = WTR_RATIO; break;
        case FAN_IDX: ratio = FAN_RATIO; break;
        case AUX_IDX: ratio = AUX_RATIO; break;
        case PDLU_IDX: ratio = PDLU_RATIO; break;
        case PDLD_IDX: ratio = PDLD_RATIO; break;
        case B5V5_IDX: ratio = B5V5_RATIO; break;
        case BVBAT_IDX: ratio = BVBAT_RATIO; break;
        case STR0_IDX: ratio = STR0_RATIO; break;
        case STR1_IDX: ratio = STR1_RATIO; break;
        case STR2_IDX: ratio = STR2_RATIO; break;
      }

      double fb_resistance = (load_cutoff[i] == 0.0) ? 5000.0 : (ratio * 4.7) / load_cutoff[i];
      wiper_values[i] = res_to_wpr(fb_resistance);
      fb_resistance = (load_peak_cutoff[i] == 0.0) ? 5000.0 : (ratio * 4.7) / load_peak_cutoff[i];
      peak_wiper_values[i] = res_to_wpr(fb_resistance);
    }

    // Store default wiper data struct in NVM
    data.key = NVM_WPR_CONSTANT;
    data.ign_wpr_val = wiper_values[IGN_IDX];
    data.inj_wpr_val = wiper_values[INJ_IDX];
    data.fuel_wpr_val = wiper_values[FUEL_IDX];
    data.ecu_wpr_val = wiper_values[ECU_IDX];
    data.wtr_wpr_val = wiper_values[WTR_IDX];
    data.fan_wpr_val = wiper_values[FAN_IDX];
    data.aux_wpr_val = wiper_values[AUX_IDX];
    data.pdlu_wpr_val = wiper_values[PDLU_IDX];
    data.pdld_wpr_val = wiper_values[PDLD_IDX];
    data.b5v5_wpr_val = wiper_values[B5V5_IDX];
    data.bvbat_wpr_val = wiper_values[BVBAT_IDX];
    data.str0_wpr_val = wiper_values[STR0_IDX];
    data.str1_wpr_val = wiper_values[STR1_IDX];
    data.str2_wpr_val = wiper_values[STR2_IDX];
    data.fuel_peak_wpr_val = peak_wiper_values[FUEL_IDX];
    data.wtr_peak_wpr_val = peak_wiper_values[WTR_IDX];
    data.fan_peak_wpr_val = peak_wiper_values[FAN_IDX];
    write_nvm_data(&data, sizeof(Wiper_nvm_data));
  } else {
    // Copy peak and normal wiper values from NVM
    wiper_values[IGN_IDX] = data.ign_wpr_val;
    wiper_values[INJ_IDX] = data.inj_wpr_val;
    wiper_values[FUEL_IDX] = data.fuel_wpr_val;
    wiper_values[ECU_IDX] = data.ecu_wpr_val;
    wiper_values[WTR_IDX] = data.wtr_wpr_val;
    wiper_values[FAN_IDX] = data.fan_wpr_val;
    wiper_values[AUX_IDX] = data.aux_wpr_val;
    wiper_values[PDLU_IDX] = data.pdlu_wpr_val;
    wiper_values[PDLD_IDX] = data.pdld_wpr_val;
    wiper_values[B5V5_IDX] = data.b5v5_wpr_val;
    wiper_values[BVBAT_IDX] = data.bvbat_wpr_val;
    wiper_values[STR0_IDX] = data.str0_wpr_val;
    wiper_values[STR1_IDX] = data.str1_wpr_val;
    wiper_values[STR2_IDX] = data.str2_wpr_val;
    peak_wiper_values[FUEL_IDX] = data.fuel_peak_wpr_val;
    peak_wiper_values[WTR_IDX] = data.wtr_peak_wpr_val;
    peak_wiper_values[FAN_IDX] = data.fan_peak_wpr_val;
  }

  // Set each rheostat to the value loaded from NVM
  uint32_t i;
  for (i = 0; i < NUM_LOADS; i++) {
    set_rheo(i, wiper_values[i]);
    peak_state[i] = 0;
    fb_resistances[i] = wpr_to_res(wiper_values[i]);
  }

  // Turn on state-independent loads
  EN_AUX_LAT = PWR_ON;
  EN_B5V5_LAT = PWR_ON;
  EN_BVBAT_LAT = PWR_ON;

  // Turn on ECU (state-independent)
  set_rheo(ECU_IDX, peak_wiper_values[ECU_IDX]);
  fb_resistances[ECU_IDX] = wpr_to_res(peak_wiper_values[ECU_IDX]);
  peak_state[ECU_IDX] = 1;
  EN_ECU_LAT = PWR_ON;
  ecu_peak_tmr = millis;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  // Main loop
  while (1) {
    uint8_t load_state_changed = 0;
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
     * Control PDLU/PDLD loads based on the ACT_UP/ACT_DN signals
     */

    // PDLU
    if (ACT_UP_SW && !ACT_DN_SW && (millis - pdlu_tmr < MAX_PDL_DUR)) {
      // Enable PDLU
      if (!PDLU_EN) {
        EN_PDLU_LAT = PWR_ON;
        pdlu_tmr = millis;
        load_state_changed = 1;
      }
    } else {
      // Reset PDLU timer if the ACT_UP signal has been disabled
      if (!ACT_UP_SW) {
        pdlu_tmr = millis;
      }

      if (PDLU_EN) {
        EN_PDLU_LAT = PWR_OFF;
        load_state_changed = 1;
      }
    }

    // PDLD
    if (ACT_DN_SW && !ACT_UP_SW && (millis - pdld_tmr < MAX_PDL_DUR)) {
      // Enable PDLD
      if (!PDLD_EN) {
        EN_PDLD_LAT = PWR_ON;
        pdld_tmr = millis;
        load_state_changed = 1;
      }
    } else {
      // Reset PDLD timer if the ACT_DN signal has been disabled
      if (!ACT_DN_SW) {
        pdld_tmr = millis;
      }

      if (PDLD_EN) {
        EN_PDLD_LAT = PWR_OFF;
        load_state_changed = 1;
      }
    }

    /**
     * Toggle state-dependent loads
     *
     * When enabling an inductive load, set the peak current limit and set the
     * peak timer. The peak current limit will be disabled at a set time later.
     *
     * Only power on or power off a load if it is not already on or off, even if
     * the conditions match.
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
       * The kill switch will disable the forementioned loads regardless of the
       * position of the ON_SW.
       */

      if (ON_SW && !KILL_SW) {
        // Enable IGN
        if (!IGN_EN) {
          EN_IGN_LAT = PWR_ON;
          load_state_changed = 1;
          ign_enabled = 1;
        }

        // Enable INJ
        if (!INJ_EN) {
          EN_INJ_LAT = PWR_ON;
          load_state_changed = 1;
        }

        // Enable FUEL
        if (!FUEL_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FUEL_IDX];
          set_rheo(FUEL_IDX, peak_wpr_val);
          fb_resistances[FUEL_IDX] = wpr_to_res(peak_wpr_val);
          peak_state[FUEL_IDX] = 1;
          EN_FUEL_LAT = PWR_ON;
          fuel_peak_tmr = millis;
          fuel_prime_tmr = millis;
          load_state_changed = 1;
        }

        // If STR load is on, disable WATER and FAN. Otherwise, enable them.
        if (STR_EN) {
          // Disable WTR
          if (WTR_EN) {
            EN_WTR_LAT = PWR_OFF;
            load_state_changed = 1;
            peak_state[WTR_IDX] = 0;
          }

          // Disable FAN
          if (FAN_EN) {
            EN_FAN_LAT = PWR_OFF;
            load_state_changed = 1;
            peak_state[FAN_IDX] = 0;
          }
        } else {
          // Enable WTR
          if (!WTR_EN) {
            uint16_t peak_wpr_val = peak_wiper_values[WTR_IDX];
            set_rheo(WTR_IDX, peak_wpr_val);
            fb_resistances[WTR_IDX] = wpr_to_res(peak_wpr_val);
            peak_state[WTR_IDX] = 1;
            EN_WTR_LAT = PWR_ON;
            wtr_peak_tmr = millis;
            load_state_changed = 1;
          }

          // Enable FAN
          if (!FAN_EN) {
            uint16_t peak_wpr_val = peak_wiper_values[FAN_IDX];
            set_rheo(FAN_IDX, peak_wpr_val);
            fb_resistances[FAN_IDX] = wpr_to_res(peak_wpr_val);
            peak_state[FAN_IDX] = 1;
            EN_FAN_LAT = PWR_ON;
            fan_peak_tmr = millis;
            load_state_changed = 1;
          }
        }
      } else {
        // Disable IGN
        if (IGN_EN) {
          EN_IGN_LAT = PWR_OFF;
          load_state_changed = 1;
          ign_enabled = 0;
        }

        // Disable INJ
        if (INJ_EN) {
          EN_INJ_LAT = PWR_OFF;
          load_state_changed = 1;
        }

        // Disable FUEL
        if (FUEL_EN) {
          EN_FUEL_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[FUEL_IDX] = 0;
        }

        // Disable WTR
        if (WTR_EN) {
          EN_WTR_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[WTR_IDX] = 0;
        }

        // Disable FAN
        if (FAN_EN) {
          EN_FAN_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[FAN_IDX] = 0;
        }
      }
    } else {

      /**
       * Perform regular load control
       */

      // IGN, INJ
      //TODO: Determine less dangerous way of keeping these loads on than ENG_ON?
      if(ON_SW && !KILL_SW && (ENG_ON || STR_EN)) {
        // Enable IGN if not already enabled
        if (!IGN_EN) {
          EN_IGN_LAT = PWR_ON;
          load_state_changed = 1;
          ign_enabled = 1;
        }

        // Enable INJ if not already enabled
        if (!INJ_EN) {
          EN_INJ_LAT = PWR_ON;
          load_state_changed = 1;
        }
      } else {
        // Disable IGN
        if (IGN_EN) {
          EN_IGN_LAT = PWR_OFF;
          load_state_changed = 1;
          ign_enabled = 0;
        }

        // Disable INJ
        if (INJ_EN) {
          EN_INJ_LAT = PWR_OFF;
          load_state_changed = 1;
        }
      }

      // FUEL
      if(ON_SW && !KILL_SW && (ENG_ON || fuel_prime_flag || STR_EN || fuel_override_sw)) {
        // Enable FUEL if not already enabled
        if (!FUEL_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FUEL_IDX];
          set_rheo(FUEL_IDX, peak_wpr_val);
          fb_resistances[FUEL_IDX] = wpr_to_res(peak_wpr_val);
          peak_state[FUEL_IDX] = 1;
          EN_FUEL_LAT = PWR_ON;
          fuel_peak_tmr = millis;
          fuel_prime_tmr = millis;
          load_state_changed = 1;
        }
      } else {
        // Disable FUEL
        if (FUEL_EN) {
          EN_FUEL_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[FUEL_IDX] = 0;
        }
      }

      // WTR
      if((ENG_ON || over_temp_flag || wtr_override_sw || fan_override_sw) && !STR_EN) {
        // Enable WTR if not already enabled
        if (!WTR_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[WTR_IDX];
          set_rheo(WTR_IDX, peak_wpr_val);
          fb_resistances[WTR_IDX] = wpr_to_res(peak_wpr_val);
          peak_state[WTR_IDX] = 1;
          EN_WTR_LAT = PWR_ON;
          wtr_peak_tmr = millis;
          load_state_changed = 1;
        }
      } else {
        // Disable WTR
        if (WTR_EN) {
          EN_WTR_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[WTR_IDX] = 0;
        }
      }

      // FAN
      if((over_temp_flag || fan_override_sw) && !STR_EN) {
        // Enable FAN if not already enabled
        if (!FAN_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FAN_IDX];
          set_rheo(FAN_IDX, peak_wpr_val);
          fb_resistances[FAN_IDX] = wpr_to_res(peak_wpr_val);
          peak_state[FAN_IDX] = 1;
          EN_FAN_LAT = PWR_ON;
          fan_peak_tmr = millis;
          load_state_changed = 1;
        }
      } else {
        // Disable FAN
        if (FAN_EN) {
          EN_FAN_LAT = PWR_OFF;
          load_state_changed = 1;
          peak_state[FAN_IDX] = 0;
        }
      }
    }

    // STR
    if (STR_SW && (millis - str_en_tmr < STR_MAX_DUR)) {
      if (!STR_EN) {
        EN_STR_LAT = PWR_ON;
        str_en_tmr = millis;
        load_state_changed = 1;
      }
    } else {
      if (!STR_SW) {
        // Reset str_en_tmr if the start switch is in the off position
        str_en_tmr = millis;
      }

      if (STR_EN) {
        EN_STR_LAT = PWR_OFF;
        load_state_changed = 1;
      }
    }

    /**
     * Check peak timers and reset to normal mode if enough time has passed
     */

    //FUEL
    if (FUEL_EN && peak_state[FUEL_IDX] && (millis - fuel_peak_tmr > FUEL_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[FUEL_IDX];
      set_rheo(FUEL_IDX, wpr_val);
      fb_resistances[FUEL_IDX] = wpr_to_res(wpr_val);
      peak_state[FUEL_IDX] = 0;
      load_state_changed = 1;
    }

    //WTR
    if (WTR_EN && peak_state[WTR_IDX] && (millis - wtr_peak_tmr > WTR_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[WTR_IDX];
      set_rheo(WTR_IDX, wpr_val);
      fb_resistances[WTR_IDX] = wpr_to_res(wpr_val);
      peak_state[WTR_IDX] = 0;
      load_state_changed = 1;
    }

    //FAN
    if (FAN_EN && peak_state[FAN_IDX] && (millis - fan_peak_tmr > FAN_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[FAN_IDX];
      set_rheo(FAN_IDX, wpr_val);
      fb_resistances[FAN_IDX] = wpr_to_res(wpr_val);
      peak_state[FAN_IDX] = 0;
      load_state_changed = 1;
    }

    //ECU
    if (ECU_EN && peak_state[ECU_IDX] && (millis - ecu_peak_tmr > ECU_PEAK_DUR)) {
      set_rheo(ECU_IDX, wiper_values[ECU_IDX]);
      fb_resistances[ECU_IDX] = wpr_to_res(wiper_values[ECU_IDX]);
      peak_state[ECU_IDX] = 0;
      load_state_changed = 1;
    }

    /**
     * Sample load current data and check if any of the loads have overcurrented.
     * If they have, reset the load and increment the overcurrent counter.
     */
    if (millis - current_samp_tmr >= CURRENT_SAMP_INTV) {
      sample_load_current();
      check_load_overcurrent();
      current_samp_tmr = millis;
    }

    /**
     * Sample temperature sensors
     */
    sample_temp();

    /**
     * Send diagnostic CAN messages
     */
    send_diag_can();

    /**
     * Send enablity state and peak mode state bitmaps on CAN
     */
    if (load_state_changed) {
      send_load_status_can(OVERRIDE);
    } else {
      send_load_status_can(NO_OVERRIDE);
    }

    /**
     * Sample voltage rail data and send results on CAN
     */
    send_rail_volt_can();

    /**
     * Send load current on CAN
     */
    send_load_current_can();

    /**
     * Send current values of peak and normal mode current cutoffs
     */
    send_cutoff_values_can(NO_OVERRIDE);
  }
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

    case PDM_CONFIG_ID:
      load_idx = msg.data[LOAD_IDX_BYTE];
      peak_mode = msg.data[PEAK_MODE_BYTE];
      cutoff = ((double) ((msg.data[CUTOFF_SETTING_BYTE + 1] << 8) |
          msg.data[CUTOFF_SETTING_BYTE])) / CUT_SCLINV;
      set_current_cutoff(load_idx, peak_mode, cutoff);
      break;

    case WHEEL_ID + 0x1:
      switch_bitmap = msg.data[SWITCH_BITS_BYTE];
      fan_override_sw = switch_bitmap & FAN_OVER_MASK;
      wtr_override_sw = switch_bitmap & WTR_OVER_MASK;
      fuel_override_sw = switch_bitmap & FUEL_OVER_MASK;
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
    data.halfword3 = total_current_draw;

    CAN_send_message(PDM_ID + 0, 8, data);
    diag_send_tmr = millis;
  }
}

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
 * void sample_load_current(void)
 *
 * Reads each necessary channel of the ADC module to determine the current
 * draw of each load.
 */
void sample_load_current(void) {
  load_current[IGN_IDX] = (((((double) read_adc_chn(ADC_IGN_CHN)) / 4095.0)
      * 3.3 * 1.5) * IGN_SCLINV * IGN_RATIO) / fb_resistances[IGN_IDX];

  load_current[INJ_IDX] = (((((double) read_adc_chn(ADC_INJ_CHN)) / 4095.0)
      * 3.3 * 1.5) * INJ_SCLINV * INJ_RATIO) / fb_resistances[INJ_IDX];

  load_current[FUEL_IDX] = (((((double) read_adc_chn(ADC_FUEL_CHN)) / 4095.0)
      * 3.3 * 1.5) * FUEL_SCLINV * FUEL_RATIO) / fb_resistances[FUEL_IDX];

  load_current[ECU_IDX] = (((((double) read_adc_chn(ADC_ECU_CHN)) / 4095.0)
      * 3.3 * 1.5) * ECU_SCLINV * ECU_RATIO) / fb_resistances[ECU_IDX];

  load_current[WTR_IDX] = (((((double) read_adc_chn(ADC_WTR_CHN)) / 4095.0)
      * 3.3 * 1.5) * WTR_SCLINV * WTR_RATIO) / fb_resistances[WTR_IDX];

  load_current[FAN_IDX] = (((((double) read_adc_chn(ADC_FAN_CHN)) / 4095.0)
      * 3.3 * 1.5) * FAN_SCLINV * FAN_RATIO) / fb_resistances[FAN_IDX];

  load_current[AUX_IDX] = (((((double) read_adc_chn(ADC_AUX_CHN)) / 4095.0)
      * 3.3 * 1.5) * AUX_SCLINV * AUX_RATIO) / fb_resistances[AUX_IDX];

  load_current[PDLU_IDX] = (((((double) read_adc_chn(ADC_PDLU_CHN)) / 4095.0)
      * 3.3 * 1.5) * PDLU_SCLINV * PDLU_RATIO) / fb_resistances[PDLU_IDX];

  load_current[PDLD_IDX] = (((((double) read_adc_chn(ADC_PDLD_CHN)) / 4095.0)
      * 3.3 * 1.5) * PDLD_SCLINV * PDLD_RATIO) / fb_resistances[PDLD_IDX];

  load_current[B5V5_IDX] = (((((double) read_adc_chn(ADC_B5V5_CHN)) / 4095.0)
      * 3.3 * 1.5) * B5V5_SCLINV * B5V5_RATIO) / fb_resistances[B5V5_IDX];

  load_current[BVBAT_IDX] = (((((double) read_adc_chn(ADC_BVBAT_CHN)) / 4095.0)
      * 3.3 * 1.5) * BVBAT_SCLINV * BVBAT_RATIO) / fb_resistances[BVBAT_IDX];

  load_current[STR0_IDX] = (((((double) read_adc_chn(ADC_STR0_CHN)) / 4095.0)
      * 3.3 * 1.5) * STR0_SCLINV * STR0_RATIO) / fb_resistances[STR0_IDX];

  /*
  load_current[STR1_IDX] = (((((double) read_adc_chn(ADC_STR1_CHN)) / 4095.0)
      * 3.3 * 1.5) * STR1_SCLINV * STR1_RATIO) / fb_resistances[STR1_IDX];

  load_current[STR2_IDX] = (((((double) read_adc_chn(ADC_STR2_CHN)) / 4095.0)
      * 3.3 * 1.5) * STR2_SCLINV * STR2_RATIO) / fb_resistances[STR2_IDX];
   */

  // Calculate total current consumption
  double current_total = ((double) load_current[IGN_IDX]) / (IGN_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[INJ_IDX]) / (INJ_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[FUEL_IDX]) / (FUEL_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[ECU_IDX]) / (ECU_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[WTR_IDX]) / (WTR_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[FAN_IDX]) / (FAN_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[AUX_IDX]) / (AUX_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[PDLU_IDX]) / (PDLU_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[PDLD_IDX]) / (PDLD_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[B5V5_IDX]) / (B5V5_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[BVBAT_IDX]) / (BVBAT_SCLINV / TOTAL_SCLINV);
  current_total += ((double) load_current[STR0_IDX]) / (STR0_SCLINV / TOTAL_SCLINV);
  //current_total += ((double) load_current[STR1_IDX]) / (STR1_SCLINV / TOTAL_SCLINV);
  //current_total += ((double) load_current[STR2_IDX]) / (STR2_SCLINV / TOTAL_SCLINV);
  total_current_draw = current_total;
}

/**
 * void check_load_overcurrent(void)
 *
 * If a load is enabled but drawing zero (or close to zero) current, we
 * can reasonably assume that it has overcurrented. If this is the case, toggle
 * the enable pin of the relevant MOSFET to reset the load.
 */
void check_load_overcurrent(void) {
  //TODO: Do overcurrent check for all loads
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
    uint16_t current_str_total = load_current[STR0_IDX];// + load_current[STR1_IDX]
    //    + load_current[STR2_IDX];

    load_current_data.halfword0 = load_current[IGN_IDX];
    load_current_data.halfword1 = load_current[INJ_IDX];
    load_current_data.halfword2 = load_current[FUEL_IDX];
    load_current_data.halfword3 = load_current[ECU_IDX];
    CAN_send_message(PDM_ID + 4, 8, load_current_data);

    load_current_data.doubleword = 0;
    load_current_data.halfword0 = load_current[WTR_IDX];
    load_current_data.halfword1 = load_current[FAN_IDX];
    load_current_data.halfword2 = load_current[AUX_IDX];
    load_current_data.halfword3 = load_current[PDLU_IDX];
    CAN_send_message(PDM_ID + 5, 8, load_current_data);

    load_current_data.doubleword = 0;
    load_current_data.halfword0 = load_current[PDLD_IDX];
    load_current_data.halfword1 = load_current[B5V5_IDX];
    load_current_data.halfword2 = load_current[BVBAT_IDX];
    CAN_send_message(PDM_ID + 6, 6, load_current_data);

    load_current_data.doubleword = 0;
    load_current_data.halfword0 = load_current[STR0_IDX];
    load_current_data.halfword1 = 0; //load_current[STR1_IDX];
    load_current_data.halfword2 = 0; //load_current[STR2_IDX];
    load_current_data.halfword3 = current_str_total;
    CAN_send_message(PDM_ID + 7, 8, load_current_data);

    load_current_send_tmr = millis;
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
    uint32_t rail_vbat = read_adc_chn(ADC_VBAT_CHN);
    uint16_t rail_vbat_conv = ((((double) rail_vbat) / 4095.0) * 3.3 * 5) * 1000.0;

    uint32_t rail_12v = read_adc_chn(ADC_12V_CHN);
    uint16_t rail_12v_conv = ((((double) rail_12v) / 4095.0) * 3.3 * 4) * 1000.0;

    uint32_t rail_5v5 = read_adc_chn(ADC_5V5_CHN);
    uint16_t rail_5v5_conv = ((((double) rail_5v5) / 4095.0) * 3.3 * 2) * 1000.0;

    uint32_t rail_5v = read_adc_chn(ADC_5V_CHN);
    uint16_t rail_5v_conv = ((((double) rail_5v) / 4095.0) * 3.3 * 2) * 1000.0;

    uint32_t rail_3v3 = read_adc_chn(ADC_3V3_CHN);
    uint16_t rail_3v3_conv = ((((double) rail_3v3) / 4095.0) * 3.3 * 2) * 1000.0;

    CAN_data rail_voltage_data = {0};
    rail_voltage_data.halfword0 = rail_vbat_conv;
    rail_voltage_data.halfword1 = rail_12v_conv;
    rail_voltage_data.halfword2 = rail_5v5_conv;
    rail_voltage_data.halfword3 = rail_5v_conv;
    CAN_send_message(PDM_ID + 2, 8, rail_voltage_data);

    rail_voltage_data.doubleword = 0;
    rail_voltage_data.halfword0 = rail_3v3_conv;
    CAN_send_message(PDM_ID + 3, 2, rail_voltage_data);

    rail_volt_send_tmr = millis;
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
    double ign_cutoff = (4.7 / wpr_to_res(wiper_values[IGN_IDX])) * IGN_RATIO;
    uint16_t ign_cutoff_scl = (uint16_t) (ign_cutoff * CUT_SCLINV);

    double inj_cutoff = (4.7 / wpr_to_res(wiper_values[INJ_IDX])) * INJ_RATIO;
    uint16_t inj_cutoff_scl = (uint16_t) (inj_cutoff * CUT_SCLINV);

    double aux_cutoff = (4.7 / wpr_to_res(wiper_values[AUX_IDX])) * AUX_RATIO;
    uint16_t aux_cutoff_scl = (uint16_t) (aux_cutoff * CUT_SCLINV);

    double pdlu_cutoff = (4.7 / wpr_to_res(wiper_values[PDLU_IDX])) * PDLU_RATIO;
    uint16_t pdlu_cutoff_scl = (uint16_t) (pdlu_cutoff * CUT_SCLINV);

    CAN_data cutoff_value_data = {0};
    cutoff_value_data.halfword0 = ign_cutoff_scl;
    cutoff_value_data.halfword1 = inj_cutoff_scl;
    cutoff_value_data.halfword2 = aux_cutoff_scl;
    cutoff_value_data.halfword3 = pdlu_cutoff_scl;
    CAN_send_message(PDM_ID + 0x8, 8, cutoff_value_data);

    double pdld_cutoff = (4.7 / wpr_to_res(wiper_values[PDLD_IDX])) * PDLD_RATIO;
    uint16_t pdld_cutoff_scl = (uint16_t) (pdld_cutoff * CUT_SCLINV);

    double b5v5_cutoff = (4.7 / wpr_to_res(wiper_values[B5V5_IDX])) * B5V5_RATIO;
    uint16_t b5v5_cutoff_scl = (uint16_t) (b5v5_cutoff * CUT_SCLINV);

    double bvbat_cutoff = (4.7 / wpr_to_res(wiper_values[BVBAT_IDX])) * BVBAT_RATIO;
    uint16_t bvbat_cutoff_scl = (uint16_t) (bvbat_cutoff * CUT_SCLINV);

    cutoff_value_data.halfword0 = pdld_cutoff_scl;
    cutoff_value_data.halfword1 = b5v5_cutoff_scl;
    cutoff_value_data.halfword2 = bvbat_cutoff_scl;
    CAN_send_message(PDM_ID + 0x9, 6, cutoff_value_data);

    double str0_cutoff = (4.7 / wpr_to_res(wiper_values[STR0_IDX])) * STR0_RATIO;
    uint16_t str0_cutoff_scl = (uint16_t) (str0_cutoff * CUT_SCLINV);

    double str1_cutoff = (4.7 / wpr_to_res(wiper_values[STR1_IDX])) * STR1_RATIO;
    uint16_t str1_cutoff_scl = (uint16_t) (str1_cutoff * CUT_SCLINV);

    double str2_cutoff = (4.7 / wpr_to_res(wiper_values[STR2_IDX])) * STR2_RATIO;
    uint16_t str2_cutoff_scl = (uint16_t) (str2_cutoff * CUT_SCLINV);

    cutoff_value_data.halfword0 = str0_cutoff_scl;
    cutoff_value_data.halfword1 = str1_cutoff_scl;
    cutoff_value_data.halfword2 = str2_cutoff_scl;
    CAN_send_message(PDM_ID + 0xA, 6, cutoff_value_data);

    double fuel_cutoff = (4.7 / wpr_to_res(wiper_values[FUEL_IDX])) * FUEL_RATIO;
    uint16_t fuel_cutoff_scl = (uint16_t) (fuel_cutoff * CUT_SCLINV);

    double wtr_cutoff = (4.7 / wpr_to_res(wiper_values[WTR_IDX])) * WTR_RATIO;
    uint16_t wtr_cutoff_scl = (uint16_t) (wtr_cutoff * CUT_SCLINV);

    double fan_cutoff = (4.7 / wpr_to_res(wiper_values[FAN_IDX])) * FAN_RATIO;
    uint16_t fan_cutoff_scl = (uint16_t) (fan_cutoff * CUT_SCLINV);

    double ecu_cutoff = (4.7 / wpr_to_res(wiper_values[ECU_IDX])) * ECU_RATIO;
    uint16_t ecu_cutoff_scl = (uint16_t) (ecu_cutoff * CUT_SCLINV);

    cutoff_value_data.halfword0 = fuel_cutoff_scl;
    cutoff_value_data.halfword1 = wtr_cutoff_scl;
    cutoff_value_data.halfword2 = fan_cutoff_scl;
    cutoff_value_data.halfword3 = ecu_cutoff_scl;
    CAN_send_message(PDM_ID + 0xB, 8, cutoff_value_data);

    double fuel_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[FUEL_IDX])) * FUEL_RATIO;
    uint16_t fuel_peak_cutoff_scl = (uint16_t) (fuel_peak_cutoff * CUT_SCLINV);

    double wtr_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[WTR_IDX])) * WTR_RATIO;
    uint16_t wtr_peak_cutoff_scl = (uint16_t) (wtr_peak_cutoff * CUT_SCLINV);

    double fan_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[FAN_IDX])) * FAN_RATIO;
    uint16_t fan_peak_cutoff_scl = (uint16_t) (fan_peak_cutoff * CUT_SCLINV);

    double ecu_peak_cutoff = (4.7 / wpr_to_res(peak_wiper_values[ECU_IDX])) * ECU_RATIO;
    uint16_t ecu_peak_cutoff_scl = (uint16_t) (ecu_peak_cutoff * CUT_SCLINV);

    cutoff_value_data.halfword0 = fuel_peak_cutoff_scl;
    cutoff_value_data.halfword1 = wtr_peak_cutoff_scl;
    cutoff_value_data.halfword2 = fan_peak_cutoff_scl;
    cutoff_value_data.halfword3 = ecu_peak_cutoff_scl;
    CAN_send_message(PDM_ID + 0xC, 8, cutoff_value_data);

    cutoff_send_tmr = millis;
  }
}

/**
 * void send_load_status_can(void)
 *
 * If the interval has passed or the caller overrode the check, send out the
 * enablity status and peak mode status of all loads on CAN.
 *
 * @param override - Whether to override the interval
 */
void send_load_status_can(uint8_t override) {
  if ((millis - load_status_send_tmr >= LOAD_STATUS_SEND) || override) {
    CAN_data data = {0};

    // Create load enablity bitmap
    data.halfword0 = 0x0 |
        IGN_EN << (15 - IGN_IDX) |
        INJ_EN << (15 - INJ_IDX) |
        FUEL_EN << (15 - FUEL_IDX) |
        ECU_EN << (15 - ECU_IDX) |
        WTR_EN << (15 - WTR_IDX) |
        FAN_EN << (15 - FAN_IDX) |
        AUX_EN << (15 - AUX_IDX) |
        PDLU_EN << (15 - PDLU_IDX) |
        PDLD_EN << (15 - PDLD_IDX) |
        B5V5_EN << (15 - B5V5_IDX) |
        BVBAT_EN << (15 - BVBAT_IDX) |
        STR_EN << (15 - STR_IDX);

    // Create load peak mode bitmap
    data.halfword1 = 0x0 |
        peak_state[IGN_IDX] << (15 - IGN_IDX) |
        peak_state[INJ_IDX] << (15 - INJ_IDX) |
        peak_state[FUEL_IDX] << (15 - FUEL_IDX) |
        peak_state[ECU_IDX] << (15 - ECU_IDX) |
        peak_state[WTR_IDX] << (15 - WTR_IDX) |
        peak_state[FAN_IDX] << (15 - FAN_IDX) |
        peak_state[AUX_IDX] << (15 - AUX_IDX) |
        peak_state[PDLU_IDX] << (15 - PDLU_IDX) |
        peak_state[PDLD_IDX] << (15 - PDLD_IDX) |
        peak_state[B5V5_IDX] << (15 - B5V5_IDX) |
        peak_state[BVBAT_IDX] << (15 - BVBAT_IDX) |
        peak_state[STR_IDX] << (15 - STR_IDX);

    // Create switch state bitmap
    data.byte4 = 0x0 |
        STR_SW << 7 |
        ON_SW << 6 |
        ACT_UP_SW << 5 |
        ACT_DN_SW << 4 |
        KILL_SW << 3;

    CAN_send_message(PDM_ID + 0x1, 6, data);
    load_status_send_tmr = millis;
  }
}

/**
 * void set_rheo(uint8_t load_idx, uint8_t val)
 *
 * Sets the specified load's rheostat to the specified value
 *
 * @param load_idx The index of the load for which the rheostat value will be changed
 * @param val The value to set the rheostat to
 */
void set_rheo(uint8_t load_idx, uint8_t val) {
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Select specific !CS signal
  switch (load_idx) {
    case IGN_IDX: CS_IGN_LAT = 0; break;
    case INJ_IDX: CS_INJ_LAT = 0; break;
    case FUEL_IDX: CS_FUEL_LAT = 0; break;
    case ECU_IDX: CS_ECU_LAT = 0; break;
    case WTR_IDX: CS_WTR_LAT = 0; break;
    case FAN_IDX: CS_FAN_LAT = 0; break;
    case AUX_IDX: CS_AUX_LAT = 0; break;
    case PDLU_IDX: CS_PDLU_LAT = 0; break;
    case PDLD_IDX: CS_PDLD_LAT = 0; break;
    case B5V5_IDX: CS_B5V5_LAT = 0; break;
    case BVBAT_IDX: CS_BVBAT_LAT = 0; break;
    case STR0_IDX: CS_STR0_LAT = 0; break;
    //case STR1_IDX: CS_STR1_LAT = 0; break;
    //case STR2_IDX: CS_STR2_LAT = 0; break;
    default: return;
  }

  SPI1BUF = ((uint16_t) val);
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Deselect specific !CS signal
  switch (load_idx) {
    case IGN_IDX: CS_IGN_LAT = 1; break;
    case INJ_IDX: CS_INJ_LAT = 1; break;
    case FUEL_IDX: CS_FUEL_LAT = 1; break;
    case ECU_IDX: CS_ECU_LAT = 1; break;
    case WTR_IDX: CS_WTR_LAT = 1; break;
    case FAN_IDX: CS_FAN_LAT = 1; break;
    case AUX_IDX: CS_AUX_LAT = 1; break;
    case PDLU_IDX: CS_PDLU_LAT = 1; break;
    case PDLD_IDX: CS_PDLD_LAT = 1; break;
    case B5V5_IDX: CS_B5V5_LAT = 1; break;
    case BVBAT_IDX: CS_BVBAT_LAT = 1; break;
    case STR0_IDX: CS_STR0_LAT = 1; break;
    //case STR1_IDX: CS_STR1_LAT = 1; break;
    //case STR2_IDX: CS_STR2_LAT = 1; break;
  }
}

/**
 * void set_all_rheo(uint16_t msg)
 *
 * Sends all rheostats a specific message.
 *
 * @param msg The 16-bit message to send to all rheostats
 */
void send_all_rheo(uint16_t msg) {
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Select all !CS signals
  CS_IGN_LAT = 0;
  CS_INJ_LAT = 0;
  CS_FUEL_LAT = 0;
  CS_ECU_LAT = 0;
  CS_WTR_LAT = 0;
  CS_FAN_LAT = 0;
  CS_AUX_LAT = 0;
  CS_PDLU_LAT = 0;
  CS_PDLD_LAT = 0;
  CS_B5V5_LAT = 0;
  CS_BVBAT_LAT = 0;
  CS_STR0_LAT = 0;
  //CS_STR1_LAT = 0;
  //CS_STR2_LAT = 0;

  SPI1BUF = msg; // Send msg on SPI bus
  while (SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Deselect all !CS signals
  CS_IGN_LAT = 1;
  CS_INJ_LAT = 1;
  CS_FUEL_LAT = 1;
  CS_ECU_LAT = 1;
  CS_WTR_LAT = 1;
  CS_FAN_LAT = 1;
  CS_AUX_LAT = 1;
  CS_PDLU_LAT = 1;
  CS_PDLD_LAT = 1;
  CS_B5V5_LAT = 1;
  CS_BVBAT_LAT = 1;
  CS_STR0_LAT = 1;
  //CS_STR1_LAT = 1;
  //CS_STR2_LAT = 1;
}

/**
 * void init_adc_pdm(void)
 *
 * Perform additional ADC configuration specific to the PDM.
 */
void init_adc_pdm(void) {
  // Configure pins as inputs
  ADC_IGN_TRIS = INPUT;
  ADC_INJ_TRIS = INPUT;
  ADC_FUEL_TRIS = INPUT;
  ADC_ECU_TRIS = INPUT;
  ADC_WTR_TRIS = INPUT;
  ADC_FAN_TRIS = INPUT;
  ADC_AUX_TRIS = INPUT;
  ADC_PDLU_TRIS = INPUT;
  ADC_PDLD_TRIS = INPUT;
  ADC_B5V5_TRIS = INPUT;
  ADC_BVBAT_TRIS = INPUT;
  ADC_STR0_TRIS = INPUT;
  ADC_STR1_TRIS = INPUT;
  ADC_STR2_TRIS = INPUT;
  ADC_3V3_TRIS = INPUT;
  ADC_5V_TRIS = INPUT;
  ADC_5V5_TRIS = INPUT;
  ADC_12V_TRIS = INPUT;
  ADC_VBAT_TRIS = INPUT;
  ADC_PTEMP_TRIS = INPUT;

  // Configure pins as analog inputs
  ADC_IGN_ANSEL = AN_INPUT;
  ADC_INJ_ANSEL = AN_INPUT;
  ADC_FUEL_ANSEL = AN_INPUT;
  ADC_ECU_ANSEL = AN_INPUT;
  ADC_WTR_ANSEL = AN_INPUT;
  ADC_FAN_ANSEL = AN_INPUT;
  ADC_AUX_ANSEL = AN_INPUT;
  ADC_PDLU_ANSEL = AN_INPUT;
  ADC_PDLD_ANSEL = AN_INPUT;
  ADC_B5V5_ANSEL = AN_INPUT;
  ADC_BVBAT_ANSEL = AN_INPUT;
  ADC_STR0_ANSEL = AN_INPUT;
  ADC_STR1_ANSEL = AN_INPUT;
  ADC_STR2_ANSEL = AN_INPUT;
  ADC_3V3_ANSEL = AN_INPUT;
  ADC_5V_ANSEL = AN_INPUT;
  ADC_5V5_ANSEL = AN_INPUT;
  ADC_12V_ANSEL = AN_INPUT;
  ADC_VBAT_ANSEL = AN_INPUT;
  ADC_PTEMP_ANSEL = AN_INPUT;

  /**
   * Select scan trigger as trigger source for class 2 inputs
   *
   * Note: INJ is the only class 3 input, and it automatically selects the scan
   * trigger as its trigger source.
   *
   * TODO: Registers missing, can't set the trigger source for most channels.
   */
  ADC_5V_TRG = SCAN_TRIGGER;
  ADC_5V5_TRG = SCAN_TRIGGER;
  ADC_12V_TRG = SCAN_TRIGGER;
  ADC_VBAT_TRG = SCAN_TRIGGER;
  ADC_PTEMP_TRG = SCAN_TRIGGER;

  // Include all channels as part of scan list
  ADC_IGN_CSS = 1;
  ADC_INJ_CSS = 1;
  ADC_FUEL_CSS = 1;
  ADC_ECU_CSS = 1;
  ADC_WTR_CSS = 1;
  ADC_FAN_CSS = 1;
  ADC_AUX_CSS = 1;
  ADC_PDLU_CSS = 1;
  ADC_PDLD_CSS = 1;
  ADC_B5V5_CSS = 1;
  ADC_BVBAT_CSS = 1;
  ADC_STR0_CSS = 1;
  //ADC_STR1_CSS = 1;
  //ADC_STR2_CSS = 1;
  ADC_3V3_CSS = 1;
  ADC_5V_CSS = 1;
  ADC_5V5_CSS = 1;
  ADC_12V_CSS = 1;
  ADC_VBAT_CSS = 1;
  ADC_PTEMP_CSS = 1;
  ADC_JTEMP_CSS = 1;
}

/**
 * void set_current_cutoff(uint8_t load_idx, uint8_t peak_mode, double cutoff)
 *
 * Sets the current cutoff of the indicated load to the requested value and
 * saves the setting in non-volatile memory.
 *
 * @param load_idx - Index of the load to change
 * @param peak_mode - Whether to set the peak mode cutoff or the normal mode cutoff
 * @param cutoff - The cutoff current (in amps) to set
 */
void set_current_cutoff(uint8_t load_idx, uint8_t peak_mode, double cutoff) {
  double ratio = 0;

  switch (load_idx) {
    case IGN_IDX: ratio = IGN_RATIO; break;
    case INJ_IDX: ratio = INJ_RATIO; break;
    case FUEL_IDX: ratio = FUEL_RATIO; break;
    case ECU_IDX: ratio = ECU_RATIO; break;
    case WTR_IDX: ratio = WTR_RATIO; break;
    case FAN_IDX: ratio = FAN_RATIO; break;
    case AUX_IDX: ratio = AUX_RATIO; break;
    case PDLU_IDX: ratio = PDLU_RATIO; break;
    case PDLD_IDX: ratio = PDLD_RATIO; break;
    case B5V5_IDX: ratio = B5V5_RATIO; break;
    case BVBAT_IDX: ratio = BVBAT_RATIO; break;
    case STR0_IDX: ratio = STR0_RATIO; break;
    case STR1_IDX: ratio = STR1_RATIO; break;
    case STR2_IDX: ratio = STR2_RATIO; break;
  }

  // Calculate FB pin resistance needed for the requested current cutoff
  double fb_resistance = (cutoff == 0.0) ? 5000.0 : (ratio * 4.7) / cutoff;

  // Store the desired peak or normal mode wiper setting
  if (peak_mode) {
    peak_wiper_values[load_idx] = res_to_wpr(fb_resistance);
  } else {
    wiper_values[load_idx] = res_to_wpr(fb_resistance);
  }

  // Set the rheostat wiper setting to the new value
  if (peak_state[load_idx]) {
    set_rheo(load_idx, peak_wiper_values[load_idx]);
    fb_resistances[load_idx] = wpr_to_res(peak_wiper_values[load_idx]);
  } else {
    set_rheo(load_idx, wiper_values[load_idx]);
    fb_resistances[load_idx] = wpr_to_res(wiper_values[load_idx]);
  }

  send_cutoff_values_can(OVERRIDE);

  // Save peak and normal mode wiper values in NVM
  Wiper_nvm_data data = {0};
  data.key = NVM_WPR_CONSTANT;
  data.ign_wpr_val = wiper_values[IGN_IDX];
  data.inj_wpr_val = wiper_values[INJ_IDX];
  data.fuel_wpr_val = wiper_values[FUEL_IDX];
  data.ecu_wpr_val = wiper_values[ECU_IDX];
  data.wtr_wpr_val = wiper_values[WTR_IDX];
  data.fan_wpr_val = wiper_values[FAN_IDX];
  data.aux_wpr_val = wiper_values[AUX_IDX];
  data.pdlu_wpr_val = wiper_values[PDLU_IDX];
  data.pdld_wpr_val = wiper_values[PDLD_IDX];
  data.b5v5_wpr_val = wiper_values[B5V5_IDX];
  data.bvbat_wpr_val = wiper_values[BVBAT_IDX];
  data.str0_wpr_val = wiper_values[STR0_IDX];
  data.str1_wpr_val = wiper_values[STR1_IDX];
  data.str2_wpr_val = wiper_values[STR2_IDX];
  data.fuel_peak_wpr_val = peak_wiper_values[FUEL_IDX];
  data.wtr_peak_wpr_val = peak_wiper_values[WTR_IDX];
  data.fan_peak_wpr_val = peak_wiper_values[FAN_IDX];
  write_nvm_data(&data, sizeof(Wiper_nvm_data));
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
