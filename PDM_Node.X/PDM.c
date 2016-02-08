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
uint16_t wiper_values[NUM_LOADS] = {0};
uint16_t peak_wiper_values[NUM_LOADS] = {0};

// State variables determined by various sources
uint8_t fuel_prime_flag = 0;
uint8_t over_temp_flag = 0;

// Timing interval variables
volatile uint32_t CAN_recv_tmr, motec0_recv_tmr, motec1_recv_tmr = 0;
uint32_t fuel_prime_tmr = 0;
uint32_t str_en_tmr = 0;
uint32_t diag_send_tmr, rail_volt_send_tmr = 0;

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
  SW8_TRIS = INPUT;
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

  // Set all rheostats to 2.5k
  //TODO: Should actually set them to value stored in flash memory
  int i;
  for (i = 0; i < NUM_LOADS; i++) {
    wiper_values[i] = 0x80;
    peak_wiper_values[i] = 0x80;
  }
  send_all_rheo(0x80);

  // Turn on state-independent loads
  EN_ECU_LAT = PWR_ON;
  EN_AUX_LAT = PWR_ON;
  EN_B5V5_LAT = PWR_ON;
  EN_BVBAT_LAT = PWR_ON;

  STI(); // Enable interrupts

  // Main loop
  while (1) {

    // Determine if the fuel pump should be priming
    CLI();
    if (ON_SW) {
      fuel_prime_flag = 1;
    } else if (millis - fuel_prime_tmr > FUEL_PRIME_DUR && FUEL_EN) {
      fuel_prime_flag = 0;
    }
    STI();

    // Determine if the car is experiencing an over-temperature condition
    CLI();
    if (over_temp_flag) {
      if (eng_temp < FAN_THRESHOLD_L) {
        over_temp_flag = 0;
      }
    } else {
      if (eng_temp > FAN_THRESHOLD_H) {
        over_temp_flag = 1;
      }
    }
    STI();

    /**
     * Toggle state-dependent loads
     *
     * When enabling an inductive load, set the peak current limit and set the
     * peak timer. The peak current limit will be disabled at a set time later.
     *
     * Only power on or power off a load if it is not already on or off, even if
     * the conditions match.
     */
    CLI();
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
        //TODO: Enable IGN
        EN_IGN_LAT = PWR_ON;

        //TODO: Enable INJ
        EN_INJ_LAT = PWR_ON;

        //TODO: Enable FUEL
        EN_FUEL_LAT = PWR_ON;

        // If STR load is on, disable WATER and FAN. Otherwise, enable them.
        if (STR_EN) {
          //TODO: Disable WTR
          EN_WTR_LAT = PWR_OFF;

          //TODO: Disable FAN
          EN_FAN_LAT = PWR_OFF;
        } else {
          //TODO: Enable WTR
          EN_WTR_LAT = PWR_ON;

          //TODO: Enable FAN
          EN_FAN_LAT = PWR_ON;
        }

      } else {
        //TODO: Disable IGN
        EN_IGN_LAT = PWR_OFF;

        //TODO: Disable INJ
        EN_INJ_LAT = PWR_OFF;

        //TODO: Disable FUEL
        EN_FUEL_LAT = PWR_OFF;

        //TODO: Disable WTR
        EN_WTR_LAT = PWR_OFF;

        //TODO: Disable FAN
        EN_FAN_LAT = PWR_OFF;
      }
    } else {

      /**
       * Perform regular load control
       */

      //TODO: Toggle IGN
      //TODO: Toggle INJ
      //TODO: Toggle FUEL
      //TODO: Toggle WTR
      //TODO: Toggle FAN
    }
    STI();

    // STR
    CLI();
    if (STR_SW && (millis - str_en_tmr < STR_MAX_DUR)) {
      if (!STR_EN) {
        set_rheo(STR0_IDX, wiper_values[STR0_IDX]);
        set_rheo(STR1_IDX, wiper_values[STR1_IDX]);
        set_rheo(STR2_IDX, wiper_values[STR2_IDX]);
        EN_STR_LAT = PWR_ON;
        str_en_tmr = millis;
      }
    } else {
      if (!STR_SW) {
        // Reset str_en_tmr if the start switch is in the off position
        str_en_tmr = millis;
      }

      if (STR_EN) {
        EN_STR_LAT = PWR_OFF;
      }
    }
    STI();

    //TODO: Check peak timers

    CLI();
    /**
     * Send diagnostic CAN messages
     */
    if (millis - diag_send_tmr >= DIAG_MSG_SEND) {
      STI();

      CAN_data data = {0};
      data.halfword0 = (uint16_t ) seconds;
      data.halfword1 = millis; //TODO: Change this to total current draw
      CAN_send_message(0x211, 8, data);

      diag_send_tmr = millis;
    }
    STI();

    //TODO: Sample load current data
    //TODO: Send out load current data

    /**
     * Sample voltage rail data and send results on CAN
     */
    CLI();
    if(millis - rail_volt_send_tmr >= RAIL_VOLT_SEND) {
      STI();

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
      CAN_send_message(0x303, 8, rail_voltage_data);

      rail_volt_send_tmr = millis;
    }
    STI();

    //TODO: Overcurrent detection

    //TODO: Control PDLU/PDLD

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
  ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion

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
