/**
 * Logger
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#include "Logger.h"

// Count number of seconds and milliseconds since start of code execution
volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;

// State/status variables determined by various sources
volatile double eng_rpm = 0; // From ECU
int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]
double rail_vbat, rail_vbak, rail_vsup, rail_5v, rail_3v3 = 0.0;

// Timing interval variables
volatile uint32_t CAN_recv_tmr = 0;
uint32_t diag_send_tmr, rail_send_tmr = 0;
uint32_t temp_samp_tmr, rail_samp_tmr = 0;

// SPI Connections
SPIConn* spi_nvm = NULL;

/**
 * Main function
 */
void main(void) {
  init_general(); // Set general runtime configuration bits
  init_gpio_pins(); // Set all I/O pins to low outputs
  //init_peripheral_modules(); // Disable unused peripheral modules
  init_oscillator(0); // Initialize oscillator configuration bits
  init_timer2(); // Initialize timer2 (millis)
  init_adc(init_adc_logger); // Initialize ADC module
  init_termination(TERMINATING); // Initialize programmable CAN termination
  init_can(); // Initialize CAN

  spi_nvm = init_nvm_std(); // Initialize NVM module
  //TODO: USB

  //init_ltc3350(); // Initialize supercapacitor charger IC


  //TODO: RTC
  //TODO: SD
  //TODO: RF

  // Initialize pins
  SHDN_TRIS = OUTPUT;
  SHDN_LAT = 0; // Default to immediate shutdown

  // Trigger initial ADC conversion
  ADCCON3bits.GSWTRG = 1;

  STI(); // Enable interrupts

  // Main loop
  while (1) {
    STI(); // Enable interrupts (in case anything disabled without re-enabling)

    /**
     * Call helper functions
     */

    // Separate logic functions

    // Analog sampling functions
    sample_temp();
    sample_rail();

    // CAN message sending functions
    send_diag_can();
    send_rail_can();
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
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
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
 * void sample_rail(void)
 *
 * Samples all the various voltage rails on the PCB and scales the samples to
 * correct for voltage dividers.
 */
void sample_rail(void) {
  if(millis - rail_samp_tmr >= RAIL_SAMP_INTV) {
    uint32_t rail_vbat_samp = read_adc_chn(ADC_VBAT_CHN);
    rail_vbat = ((((double) rail_vbat_samp) / 4095.0) * 3.3 * 5);

    uint32_t rail_vbak_samp = read_adc_chn(ADC_VBAK_CHN);
    rail_vbak = ((((double) rail_vbak_samp) / 4095.0) * 3.3 * 5);

    uint32_t rail_vsup_samp = read_adc_chn(ADC_VSUP_CHN);
    rail_vsup = ((((double) rail_vsup_samp) / 4095.0) * 3.3 * 5);

    uint32_t rail_5v_samp = read_adc_chn(ADC_5V_CHN);
    rail_5v = ((((double) rail_5v_samp) / 4095.0) * 3.3 * 2);

    uint32_t rail_3v3_samp = read_adc_chn(ADC_3V3_CHN);
    rail_3v3 = ((((double) rail_3v3_samp) / 4095.0) * 3.3 * 2);

    rail_samp_tmr = millis;
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

    CAN_send_message(LOGGER_ID + 0, 6, data);
    diag_send_tmr = millis;
  }
}

/**
 * void send_rail_can(void)
 *
 * Sends the rail voltage CAN message if the interval has passed.
 */
void send_rail_can(void) {
  if (millis - rail_send_tmr >= RAIL_SEND) {
    CAN_data data = {0};
    data.halfword0 = (uint16_t) (rail_vbat * 1000.0);
    data.halfword1 = (uint16_t) (rail_vbak * 1000.0);
    data.halfword2 = (uint16_t) (rail_vsup * 1000.0);
    data.halfword3 = (uint16_t) (rail_5v * 1000.0);
    CAN_send_message(LOGGER_ID + 1, 8, data);

    data.halfword0 = (uint16_t) (rail_3v3 * 1000.0);
    CAN_send_message(LOGGER_ID + 2, 2, data);

    rail_send_tmr = millis;
  }
}

//========================== UTILITY FUNCTIONS =================================

/**
 * void init_adc_logger(void)
 *
 * Initializes ADC1 & ADC2 modules so that AN46 & AN46 will work, along with
 * all other pins.
 */
void init_adc_logger(void) {
  // Initialize pins
  ADC_VBAT_TRIS = INPUT;
  ADC_VBAT_ANSEL = AN_INPUT;
  ADC_VBAT_CSS = 1;
  ADC_VBAK_TRIS = INPUT;
  ADC_VBAK_ANSEL = AN_INPUT;
  ADC_VBAK_CSS = 1;
  ADC_VSUP_TRIS = INPUT;
  ADC_VSUP_ANSEL = AN_INPUT;
  ADC_VSUP_CSS = 1;
  ADC_5V_TRIS = INPUT;
  ADC_5V_ANSEL = AN_INPUT;
  ADC_5V_CSS = 1;
  ADC_3V3_TRIS = INPUT;
  ADC_3V3_ANSEL = AN_INPUT;
  ADC_3V3_CSS = 1;

  ADCTRG1bits.TRGSRC1 = SCAN_TRIGGER; // Software triggering for AN46
  ADCTRG1bits.TRGSRC2 = SCAN_TRIGGER; // Software triggering for AN47

  ADCTRGMODEbits.SH1ALT = 0b01; // Use AN46 instead of AN1
  ADCTRGMODEbits.SH2ALT = 0b01; // Use AN47 instead of AN2

  // Enable ADC1
  ADCANCONbits.ANEN1 = 1;      // ADC1 Analog and Bias Circuitry Enable (Enabled)
  while(!ADCANCONbits.WKRDY1); // Wait until ADC1 is ready
  ADCCON3bits.DIGEN1 = 1;      // ADC1 Digital Enable (Enabled)

  // Enable ADC2
  ADCANCONbits.ANEN2 = 1;      // ADC2 Analog and Bias Circuitry Enable (Enabled)
  while(!ADCANCONbits.WKRDY2); // Wait until ADC2 is ready
  ADCCON3bits.DIGEN2 = 1;      // ADC2 Digital Enable (Enabled)
}
