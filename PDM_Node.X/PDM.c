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

// State variables determined by various sources
uint8_t fuel_prime_flag = 0;
uint8_t over_temp_flag = 0;
uint16_t total_current_draw = 0;
uint8_t wtr_override_sw = 0;
uint8_t fan_override_sw = 0;

// Timing interval variables
volatile uint32_t CAN_recv_tmr, motec0_recv_tmr, motec1_recv_tmr = 0;
uint32_t fuel_prime_tmr = 0;
uint32_t str_en_tmr = 0;
uint32_t diag_send_tmr, rail_volt_send_tmr, load_current_send_tmr = 0;
uint32_t fuel_peak_tmr, wtr_peak_tmr, fan_peak_tmr = 0;
uint32_t pdlu_tmr, pdld_tmr = 0;

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
    // Initialize wiper values to 3kOhm and peak wiper values to 4kOhm
    uint32_t i;
    for (i = 0; i < NUM_LOADS; i++) {
      wiper_values[i] = RES_TO_WPR(3000);
      peak_wiper_values[i] = RES_TO_WPR(4000);
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
    fb_resistances[i] = WPR_TO_RES(wiper_values[i]);
  }

  // Turn on state-independent loads
  EN_ECU_LAT = PWR_ON;
  EN_AUX_LAT = PWR_ON;
  EN_B5V5_LAT = PWR_ON;
  EN_BVBAT_LAT = PWR_ON;

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  // Main loop
  while (1) {

    /**
     * Determine if the fuel pump should be priming
     */
    if (ON_SW) {
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
      if(!PDLU_EN) {
        EN_PDLU_LAT = PWR_ON;
        pdlu_tmr = millis;
      }
    } else {
      // Reset PDLU timer if the ACT_UP signal has been disabled 
      if(!ACT_UP_SW) {
        pdlu_tmr = millis;
      }

      EN_PDLU_LAT = PWR_OFF;
    }

    // PDLD
    if (ACT_DN_SW && !ACT_UP_SW && (millis - pdld_tmr < MAX_PDL_DUR)) {
      // Enable PDLD
      if(!PDLD_EN) {
        EN_PDLD_LAT = PWR_ON;
        pdld_tmr = millis;
      }
    } else {
      // Reset PDLD timer if the ACT_DN signal has been disabled 
      if(!ACT_DN_SW) {
        pdld_tmr = millis;
      }

      EN_PDLD_LAT = PWR_OFF;
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
        millis - motec1_recv_tmr > BASIC_CONTROL_WAIT) {

      /**
       * Perform basic load control
       *
       * IGN, INJ, FUEL, WTR, and FAN will turn on when the ON_SW is in the on
       * position and turn off when the ON_SW is in the off position. Other
       * loads will still be controlled normally as they do not depend on CAN.
       */

      if (ON_SW) {
        // Enable IGN
        if (!IGN_EN) {
          EN_IGN_LAT = PWR_ON;
        }

        // Enable INJ
        if (!INJ_EN) {
          EN_INJ_LAT = PWR_ON;
        }

        // Enable FUEL
        if (!FUEL_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FUEL_IDX];
          set_rheo(FUEL_IDX, peak_wpr_val);
          fb_resistances[FUEL_IDX] = WPR_TO_RES(peak_wpr_val);
          peak_state[FUEL_IDX] = 1;
          EN_FUEL_LAT = PWR_ON;
          fuel_peak_tmr = millis;
        }

        // If STR load is on, disable WATER and FAN. Otherwise, enable them.
        if (STR_EN) {
          // Disable WTR
          EN_WTR_LAT = PWR_OFF;

          // Disable FAN
          EN_FAN_LAT = PWR_OFF;
        } else {
          // Enable WTR
          if (!WTR_EN) {
            uint16_t peak_wpr_val = peak_wiper_values[WTR_IDX];
            set_rheo(WTR_IDX, peak_wpr_val);
            fb_resistances[WTR_IDX] = WPR_TO_RES(peak_wpr_val);
            peak_state[WTR_IDX] = 1;
            EN_WTR_LAT = PWR_ON;
            wtr_peak_tmr = millis;
          }

          // Enable FAN
          if (!FAN_EN) {
            uint16_t peak_wpr_val = peak_wiper_values[FAN_IDX];
            set_rheo(FAN_IDX, peak_wpr_val);
            fb_resistances[FAN_IDX] = WPR_TO_RES(peak_wpr_val);
            peak_state[FAN_IDX] = 1;
            EN_FAN_LAT = PWR_ON;
            fan_peak_tmr = millis;
          }
        }
      } else {
        // Disable IGN
        EN_IGN_LAT = PWR_OFF;

        // Disable INJ
        EN_INJ_LAT = PWR_OFF;

        // Disable FUEL
        EN_FUEL_LAT = PWR_OFF;

        // Disable WTR
        EN_WTR_LAT = PWR_OFF;

        // Disable FAN
        EN_FAN_LAT = PWR_OFF;
      }
    } else {

      /**
       * Perform regular load control
       */

      // IGN, INJ, FUEL
      //TODO: Determine less dangerous way of keeping these loads on than ENG_ON?
      if(ON_SW && (ENG_ON || fuel_prime_flag || STR_EN)) {
        // Enable IGN if not already enabled
        if (!IGN_EN) {
          EN_IGN_LAT = PWR_ON;
        }

        // Enable INJ if not already enabled
        if (!INJ_EN) {
          EN_INJ_LAT = PWR_ON;
        }

        // Enable FUEL if not already enabled
        if (!FUEL_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FUEL_IDX];
          set_rheo(FUEL_IDX, peak_wpr_val);
          fb_resistances[FUEL_IDX] = WPR_TO_RES(peak_wpr_val);
          peak_state[FUEL_IDX] = 1;
          EN_FUEL_LAT = PWR_ON;
          fuel_peak_tmr = millis;
        }
      } else {
        // Disable IGN
        EN_IGN_LAT = PWR_OFF;

        // Disable INJ
        EN_INJ_LAT = PWR_OFF;

        // Disable FUEL
        EN_FUEL_LAT = PWR_OFF;
      }

      // WTR
      if((ENG_ON || over_temp_flag || wtr_override_sw || fan_override_sw) && !STR_EN) {
        // Enable WTR if not already enabled
        if (!WTR_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[WTR_IDX];
          set_rheo(WTR_IDX, peak_wpr_val);
          fb_resistances[WTR_IDX] = WPR_TO_RES(peak_wpr_val);
          peak_state[WTR_IDX] = 1;
          EN_WTR_LAT = PWR_ON;
          wtr_peak_tmr = millis;
        }
      } else {
        // Disable WTR
        EN_WTR_LAT = PWR_OFF;
      }

      // FAN
      if((over_temp_flag || fan_override_sw) && !STR_EN) {
        // Enable FAN if not already enabled
        if (!FAN_EN) {
          uint16_t peak_wpr_val = peak_wiper_values[FAN_IDX];
          set_rheo(FAN_IDX, peak_wpr_val);
          fb_resistances[FAN_IDX] = WPR_TO_RES(peak_wpr_val);
          peak_state[FAN_IDX] = 1;
          EN_FAN_LAT = PWR_ON;
          fan_peak_tmr = millis;
        }
      } else {
        // Disable FAN
        EN_FAN_LAT = PWR_OFF;
      }
    }

    // STR
    if (STR_SW && (millis - str_en_tmr < STR_MAX_DUR)) {
      if (!STR_EN) {
        EN_STR_LAT = PWR_ON;
        str_en_tmr = millis;
      }
    } else {
      if (!STR_SW) {
        // Reset str_en_tmr if the start switch is in the off position
        str_en_tmr = millis;
      }

      EN_STR_LAT = PWR_OFF;
    }

    /**
     * Check peak timers and reset to normal mode if enough time has past
     */
    //FUEL
    if (FUEL_EN && peak_state[FUEL_IDX] && (millis - fuel_peak_tmr > FUEL_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[FUEL_IDX];
      set_rheo(FUEL_IDX, wpr_val);
      fb_resistances[FUEL_IDX] = WPR_TO_RES(wpr_val);
      peak_state[FUEL_IDX] = 0;
    }

    //WTR
    if (WTR_EN && peak_state[WTR_IDX] && (millis - wtr_peak_tmr > WTR_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[WTR_IDX];
      set_rheo(WTR_IDX, wpr_val);
      fb_resistances[WTR_IDX] = WPR_TO_RES(wpr_val);
      peak_state[WTR_IDX] = 0;
    }

    //FAN
    if (FAN_EN && peak_state[FAN_IDX] && (millis - fan_peak_tmr > FAN_PEAK_DUR)) {
      uint16_t wpr_val = wiper_values[FAN_IDX];
      set_rheo(FAN_IDX, wpr_val);
      fb_resistances[FAN_IDX] = WPR_TO_RES(wpr_val);
      peak_state[FAN_IDX] = 0;
    }

    //TODO: Overcurrent detection

    /**
     * Send diagnostic CAN messages
     */
    if (millis - diag_send_tmr >= DIAG_MSG_SEND) {
      CAN_data data = {0};
      data.halfword0 = (uint16_t ) seconds;
      data.halfword1 = total_current_draw;
      data.word1 = millis; //TODO: Change this to PCB temp and IC temp
      CAN_send_message(0x300, 8, data);

      diag_send_tmr = millis;
    }

    /**
     * Sample load current data and send results on CAN
     */
    if(millis - load_current_send_tmr >= LOAD_CUR_SEND) {
      uint32_t fb_volt_ign = read_adc_chn(ADC_IGN_CHN);
      uint16_t current_ign = (((((double) fb_volt_ign) / 4095.0) * 3.3 * 1.5)
          * IGN_SCLINV * IGN_RATIO) / fb_resistances[IGN_IDX];

      uint32_t fb_volt_inj = read_adc_chn(ADC_INJ_CHN);
      uint16_t current_inj = (((((double) fb_volt_inj) / 4095.0) * 3.3 * 1.5)
          * INJ_SCLINV * INJ_RATIO) / fb_resistances[INJ_IDX];

      uint32_t fb_volt_fuel = read_adc_chn(ADC_FUEL_CHN);
      uint16_t current_fuel = (((((double) fb_volt_fuel) / 4095.0) * 3.3 * 1.5)
          * FUEL_SCLINV * FUEL_RATIO) / fb_resistances[FUEL_IDX];

      uint32_t fb_volt_ecu = read_adc_chn(ADC_ECU_CHN);
      uint16_t current_ecu = (((((double) fb_volt_ecu) / 4095.0) * 3.3 * 1.5)
          * ECU_SCLINV * ECU_RATIO) / fb_resistances[ECU_IDX];

      CAN_data load_current_data = {0};
      load_current_data.halfword0 = current_ign;
      load_current_data.halfword1 = current_inj;
      load_current_data.halfword2 = current_fuel;
      load_current_data.halfword3 = current_ecu;
      CAN_send_message(0x304, 8, load_current_data);

      uint32_t fb_volt_wtr = read_adc_chn(ADC_WTR_CHN);
      uint16_t current_wtr = (((((double) fb_volt_wtr) / 4095.0) * 3.3 * 1.5)
          * WTR_SCLINV * WTR_RATIO) / fb_resistances[WTR_IDX];

      uint32_t fb_volt_fan = read_adc_chn(ADC_FAN_CHN);
      uint16_t current_fan = (((((double) fb_volt_fan) / 4095.0) * 3.3 * 1.5)
          * FAN_SCLINV * FAN_RATIO) / fb_resistances[FAN_IDX];

      uint32_t fb_volt_aux = read_adc_chn(ADC_AUX_CHN);
      uint16_t current_aux = (((((double) fb_volt_aux) / 4095.0) * 3.3 * 1.5)
          * AUX_SCLINV * AUX_RATIO) / fb_resistances[AUX_IDX];

      uint32_t fb_volt_pdlu = read_adc_chn(ADC_PDLU_CHN);
      uint16_t current_pdlu = (((((double) fb_volt_pdlu) / 4095.0) * 3.3 * 1.5)
          * PDLU_SCLINV * PDLU_RATIO) / fb_resistances[PDLU_IDX];

      load_current_data.doubleword = 0;
      load_current_data.halfword0 = current_wtr;
      load_current_data.halfword1 = current_fan;
      load_current_data.halfword2 = current_aux;
      load_current_data.halfword3 = current_pdlu;
      CAN_send_message(0x305, 8, load_current_data);

      uint32_t fb_volt_pdld = read_adc_chn(ADC_PDLD_CHN);
      uint16_t current_pdld = (((((double) fb_volt_pdld) / 4095.0) * 3.3 * 1.5)
          * PDLD_SCLINV * PDLD_RATIO) / fb_resistances[PDLD_IDX];

      uint32_t fb_volt_b5v5 = read_adc_chn(ADC_B5V5_CHN);
      uint16_t current_b5v5 = (((((double) fb_volt_b5v5) / 4095.0) * 3.3 * 1.5)
          * B5V5_SCLINV * B5V5_RATIO) / fb_resistances[B5V5_IDX];

      uint32_t fb_volt_bvbat = read_adc_chn(ADC_BVBAT_CHN);
      uint16_t current_bvbat = (((((double) fb_volt_bvbat) / 4095.0) * 3.3 * 1.5)
          * BVBAT_SCLINV * BVBAT_RATIO) / fb_resistances[BVBAT_IDX];

      load_current_data.doubleword = 0;
      load_current_data.halfword0 = current_pdld;
      load_current_data.halfword1 = current_b5v5;
      load_current_data.halfword2 = current_bvbat;
      CAN_send_message(0x306, 6, load_current_data);

      uint32_t fb_volt_str0 = read_adc_chn(ADC_STR0_CHN);
      uint16_t current_str0 = (((((double) fb_volt_str0) / 4095.0) * 3.3 * 1.5)
          * STR0_SCLINV * STR0_RATIO) / fb_resistances[STR0_IDX];

      uint32_t fb_volt_str1 = read_adc_chn(ADC_STR1_CHN);
      uint16_t current_str1 = (((((double) fb_volt_str1) / 4095.0) * 3.3 * 1.5)
          * STR1_SCLINV * STR1_RATIO) / fb_resistances[STR1_IDX];

      uint32_t fb_volt_str2 = read_adc_chn(ADC_STR2_CHN);
      uint16_t current_str2 = (((((double) fb_volt_str2) / 4095.0) * 3.3 * 1.5)
          * STR2_SCLINV * STR2_RATIO) / fb_resistances[STR2_IDX];

      uint16_t current_str_total = current_str0 + current_str1 + current_str2;

      load_current_data.doubleword = 0;
      load_current_data.halfword0 = current_str0;
      load_current_data.halfword1 = current_str1;
      load_current_data.halfword2 = current_str2;
      load_current_data.halfword3 = current_str_total;
      CAN_send_message(0x307, 8, load_current_data);

      load_current_send_tmr = millis;

      // Calculate total current consumption
      double current_total = ((double) current_ign) / (IGN_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_inj) / (INJ_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_fuel) / (FUEL_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_ecu) / (ECU_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_wtr) / (WTR_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_fan) / (FAN_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_aux) / (AUX_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_pdlu) / (PDLU_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_pdld) / (PDLD_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_b5v5) / (B5V5_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_bvbat) / (BVBAT_SCLINV / TOTAL_SCLINV);
      current_total += ((double) current_str_total) / (STR_SCLINV / TOTAL_SCLINV);
      total_current_draw = current_total;
    }

    /**
     * Sample voltage rail data and send results on CAN
     */
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
      CAN_send_message(0x302, 8, rail_voltage_data);

      rail_voltage_data.doubleword = 0;
      rail_voltage_data.halfword0 = rail_3v3_conv;
      CAN_send_message(0x303, 2, rail_voltage_data);

      rail_volt_send_tmr = millis;
    }

    /**
     *TODO: Send current value of peak mode current cutoffs
     */

    /**
     *TODO: Send current value of normal mode current cutoffs
     */

    //TODO: ???
    //TODO: Profit
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
  if(ADCCON2bits.EOSRDY) {
    ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion
  }

  IFS0CLR = _IFS0_T2IF_MASK; // Clear TMR2 Interrupt Flag
}

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  CAN_recv_tmr = millis; // Record time of latest received CAN message

  switch (msg.id) {
    case MOTEC0_ID:
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) |
          msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      oil_pres = ((double) ((msg.data[OIL_PRES_BYTE] << 8) |
          msg.data[OIL_PRES_BYTE + 1])) * OIL_PRES_SCL;
      oil_temp = ((double) ((msg.data[OIL_TEMP_BYTE] << 8) |
          msg.data[OIL_TEMP_BYTE + 1])) * OIL_TEMP_SCL;

      motec0_recv_tmr = millis;
      break;
    case MOTEC1_ID:
      eng_temp = ((double) ((msg.data[ENG_TEMP_BYTE] << 8) |
          msg.data[ENG_TEMP_BYTE + 1])) * ENG_TEMP_SCL;
      bat_volt_ecu = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) |
          msg.data[VOLT_ECU_BYTE + 1])) * VOLT_ECU_SCL;

      motec1_recv_tmr = millis;
      break;

    //TODO: Accept new values for CAN current cut-offs
    //TODO: Get WTR/FAN override switch state from wheel CAN messages
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
    case STR1_IDX: CS_STR1_LAT = 0; break;
    case STR2_IDX: CS_STR2_LAT = 0; break;
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
    case STR1_IDX: CS_STR1_LAT = 1; break;
    case STR2_IDX: CS_STR2_LAT = 1; break;
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
  CS_STR1_LAT = 0;
  CS_STR2_LAT = 0;

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
  CS_STR1_LAT = 1;
  CS_STR2_LAT = 1;
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
  ADC_STR1_CSS = 1;
  ADC_STR2_CSS = 1;
  ADC_3V3_CSS = 1;
  ADC_5V_CSS = 1;
  ADC_5V5_CSS = 1;
  ADC_12V_CSS = 1;
  ADC_VBAT_CSS = 1;
}
