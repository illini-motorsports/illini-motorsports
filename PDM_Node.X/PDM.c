/**
 * PDM
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "PDM.h"

static volatile uint32_t seconds = 0;
static volatile uint8_t res_flag = 0;
    
// Car status variables reported over can from the ECU
double eng_rpm, oil_pres, oil_temp, eng_temp, bat_volt_ecu = 0;

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

  // Set all rheostats to 5k
  //TODO: Should actually set them to value stored in flash memory
  send_all_rheo(0xFF);

  // Set TRIS registers - ADC
  // Set TRIS registers - !SW

  // Turn on engine-off loads
  EN_ECU_LAT = PWR_ON;
  EN_AUX_LAT = PWR_ON;
  EN_B5V5_LAT = PWR_ON;
  EN_BVBAT_LAT = PWR_ON;

  asm volatile("ei"); // Enable interrupts
  
  // Main loop
  while(1);
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
  LATEbits.LATE5 = LATEbits.LATE5 ? 0 : 1; // Invert LATE5 - Toggle the LED

  // Send test CAN message with header and current time in seconds
  uint8_t message[8] = {0xF, 0xE, 0xD, 0xC, 0, 0, 0, 0};
  ((uint32_t*) message)[1] = seconds;
  CAN_send_message(0x212, 8, message);

  // Flip resistance every 3 seconds
  if(seconds % 3 == 0) {
    if(res_flag) {
      EN_FAN_LAT = PWR_ON; // FAN On
      send_all_rheo(0x0080); // Half resistance
      res_flag = 0;
    } else {
      EN_FAN_LAT = PWR_OFF; // FAN Off
      send_all_rheo(0x00FF); // Maximum resistance
      res_flag = 1;
    }
  }
  
  CAN_recv_messages(process_CAN_msg);
  
  IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}

/**
 * CAN1 Interrupt Handler
 *
 * TODO: Fix for actual PDM code
 */
void __attribute__((vector(_CAN1_VECTOR), interrupt(IPL6SRS))) can_inthnd(void) {
  IFS4bits.CAN1IF = 0;
}

/**
 * Handler function for each received CAN message.
 * 
 * @param msg The received CAN message
 */
void process_CAN_msg(CAN_message msg) {
  switch(msg.id) {
    case MOTEC0_ID:
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) | 
          msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      oil_pres = ((double) ((msg.data[OIL_PRES_BYTE] << 8) | 
          msg.data[OIL_PRES_BYTE + 1])) * OIL_PRES_SCL;
      oil_temp = ((double) ((msg.data[OIL_TEMP_BYTE] << 8) | 
          msg.data[OIL_TEMP_BYTE + 1])) * OIL_TEMP_SCL;
      break;
    case MOTEC1_ID:
      eng_temp = ((double) ((msg.data[ENG_TEMP_BYTE] << 8) | 
          msg.data[ENG_TEMP_BYTE + 1])) * ENG_TEMP_SCL;
      bat_volt_ecu = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) | 
          msg.data[VOLT_ECU_BYTE + 1])) * VOLT_ECU_SCL;
      break;
  }
}

/**
 * TODO: Fix for actual PDM code
 */
void send_rheo(uint16_t msg) {
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATGbits.LATG15 = 0; // CS selected
  SPI1BUF = msg;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATGbits.LATG15 = 1; // CS deselected
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
