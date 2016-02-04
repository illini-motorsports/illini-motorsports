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
double eng_rpm, oil_pres, oil_temp, eng_temp, bat_volt_ecu = 0;

// Stores wiper values for each load
uint16_t wiper_values[NUM_LOADS] = {0};
uint16_t peak_wiper_values[NUM_LOADS] = {0};

// State variables determined by various sources
uint8_t fuel_prime_flag = 0;
uint8_t over_temp_flag = 0;

// Timing interval variables
uint32_t CAN_recv_tmr, motec0_recv_tmr, motec1_recv_tmr = 0;
uint32_t fuel_prime_tmr = 0;
uint32_t str_en_tmr = 0;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(); // Initialize oscillator configuration bits
  init_timer1(); // Initialize timer1
  init_spi(); // Initialize SPI interface
  init_adc(); // Initialize ADC module
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
  for(i = 0; i < NUM_LOADS; i++) {
    wiper_values[i] = 0x80;
    peak_wiper_values[i] = 0x80;

  }
  send_all_rheo(0x80);

  // Set TRIS registers - ADC

  // Turn on state-independent loads
  EN_ECU_LAT = PWR_ON;
  EN_AUX_LAT = PWR_ON;
  EN_B5V5_LAT = PWR_ON;
  EN_BVBAT_LAT = PWR_ON;

  STI(); // Enable interrupts

  // Main loop
  while(1) {

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

      if(ON_SW) {
        //TODO: Enable IGN
        EN_IGN_LAT = PWR_ON;

        //TODO: Enable INJ
        EN_INJ_LAT = PWR_ON;

        //TODO: Enable FUEL
        EN_FUEL_LAT = PWR_ON;

        // If STR load is on, disable WATER and FAN. Otherwise, enable them.
        if(STR_EN) {
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
    //TODO: Sample current data
    //TODO: Send out current data
    //TODO: Overcurrent detection
    //TODO: Control PDLU/PDLD
    //TODO: ???
    //TODO: Profit
  }
}

/**
 * TMR1 Interrupt Handler
 *
 * Fires once every second.
 *
 * TODO: Fix for actual PDM code.
 */
void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
  seconds++;
  millis += 1000; // TODO: Actually make a milliseconds interrupt

  // Send test CAN message with header and current time in seconds
  /*
  uint8_t message[8] = {0xF, 0xE, 0xD, 0xC, 0, 0, 0, 0};
  ((uint32_t*) message)[1] = seconds;
  CAN_send_message(0x212, 8, message);
   */

  EN_B5V5_LAT = !EN_B5V5_PORT;

  IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}

/**
 * CAN1 Interrupt Handler
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL6SRS))) can_inthnd(void) {
  if(C1INTbits.RBIF) {
    CAN_recv_messages(process_CAN_msg); // Process all available CAN messages
  }

  if(C1INTbits.RBOVIF) {
    CAN_rx_ovf++;
  }

  IFS4bits.CAN1IF = 0; // Clear CAN1 Interrupt Flag
}

/**
 * Handler function for each received CAN message.
 *
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  CAN_recv_tmr = millis; // Record time of latest received CAN message

  switch(msg.id) {
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
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Select specific !CS signal
  switch(load_idx) {
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
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module

  // Deselect specific !CS signal
  switch(load_idx) {
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
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module

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
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module

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
