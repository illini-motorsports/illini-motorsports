/**
 * Wheel
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#include "Wheel.h"

// Count number of milliseconds since start of code execution
volatile uint32_t CANswStateMillis, CANswADLMillis, CANdiagMillis;
volatile uint8_t darkState;
volatile uint8_t auxState;

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(1);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_spi();// Initialize SPI interface
  init_adc(NULL);
  init_termination(TERMINATING);
  init_tlc5955();

  ADCCON3bits.GSWTRG = 1; // Initial ADC Conversion?
  STI();// Enable interrupts

  millis = 0;
  CANswStateMillis = CANswADLMillis = CANdiagMillis = 0;
  auxState = 0;
  auxNumber = 0;

  // Init Relevant Pins
  LCD_CS_LAT = 1;
  LCD_CS_TRIS = OUTPUT;
  LCD_RST_LAT = 1;
  LCD_RST_TRIS = OUTPUT;
  LCD_PWM_LAT = 1; //TODO: This is full brightness, PWM for other settings
  LCD_PWM_TRIS = OUTPUT;

  SW1_TRIS = INPUT;
  SW2_TRIS = INPUT;
  SW3_TRIS = INPUT;
  SW4_TRIS = INPUT;
  MOM1_TRIS = INPUT;
  MOM2_TRIS = INPUT;
  MOM3_TRIS = INPUT;
  MOM4_TRIS = INPUT;

  ROT1_TRIS = INPUT;
  ROT1_ANSEL = AN_INPUT;
  ROT1_CSS = 1;
  ROT2_TRIS = INPUT;
  ROT2_ANSEL = AN_INPUT;
  ROT2_CSS = 1;
  ROT3_TRIS = INPUT;
  ROT3_ANSEL = AN_INPUT;
  ROT3_CSS = 1;
  TROT1_TRIS = INPUT;
  TROT1_ANSEL = AN_INPUT;
  TROT1_CSS = 1;
  TROT2_TRIS = INPUT;
  TROT2_ANSEL = AN_INPUT;
  TROT2_CSS = 1;

  // Initialize RA8875
  reset();
  initialize();
  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX

  //fillScreen(RA8875_BLACK);

  // Initialize All the data streams
  initDataItems();
  init_can();
  updateSwVals();
  initAllScreens();
  changeScreen(RACE_SCREEN);

  tlc5955_startup();

  while(1) {
    // Send CAN messages with the correct frequency
    if(millis - CANswStateMillis >= CAN_SW_STATE_FREQ){
      CANswitchStates();
      CANswStateMillis = millis;
    }
    if(millis - CANswADLMillis >= CAN_SW_ADL_FREQ){
      CANswitchADL();
      CANswADLMillis = millis;
    }
    if(millis - CANdiagMillis >= CAN_DIAG_FREQ){
      CANdiag();
      CANdiagMillis = millis;
    }
    /*
    // Refresh Screen
    if(tRotary[1].value == 1 && screenNumber != PDM_DRAW_SCREEN){
    changeScreen(PDM_DRAW_SCREEN);
    }
    else if(tRotary[1].value == 2 && screenNumber != PDM_CUT_SCREEN){
    changeScreen(PDM_CUT_SCREEN);
    }
    else if(tRotary[1].value == 3 && screenNumber != RACE_SCREEN){
    changeScreen(RACE_SCREEN);
    }
    else if(tRotary[1].value == 4 && screenNumber != BRAKE_SCREEN){
    changeScreen(BRAKE_SCREEN);
    }
    // Change AUX State
    if(auxState != momentaries[0].value){
    if(momentaries[0].value == 1){
    changeAUXType((auxNumber+1)%3);
    }
    auxState = momentaries[0].value;
    }
    */

    refreshScreenItems();
  }
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count

  if (!(millis%25)){
    if (ADCCON2bits.EOSRDY) {
      ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion
    }
    updateSwVals();
    CANswitchStates();
    tlc5955_check_timers();
  }

  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

void delay(uint32_t num) {
  uint32_t start = millis;
  while (millis - start < num);
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

void process_CAN_msg(CAN_message msg){
  uint16_t * lsbArray = (uint16_t *) msg.data;
  switch (msg.id) {

    /*Motec*/
    case MOTEC_ID + 0:
      motecDataItems[ENG_RPM_IDX].value = parseMsgMotec(&msg, ENG_RPM_BYTE, ENG_RPM_SCL);
      motecDataItems[THROTTLE_POS_IDX].value = parseMsgMotec(&msg, THROTTLE_POS_BYTE, THROTTLE_POS_SCL);
      motecDataItems[LAMBDA_IDX].value = parseMsgMotec(&msg, LAMBDA_BYTE, LAMBDA_SCL);
      motecDataItems[VOLT_ECU_IDX].value = parseMsgMotec(&msg, VOLT_ECU_BYTE, VOLT_ECU_SCL);
      break;
    case MOTEC_ID + 1:
      motecDataItems[ENG_TEMP_IDX].value = parseMsgMotec(&msg, ENG_TEMP_BYTE, ENG_TEMP_SCL);
      motecDataItems[OIL_TEMP_IDX].value = parseMsgMotec(&msg, OIL_TEMP_BYTE, OIL_TEMP_SCL);
      motecDataItems[MANIFOLD_TEMP_IDX].value = parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL);
      motecDataItems[FUEL_TEMP_IDX].value = parseMsgMotec(&msg, FUEL_TEMP_BYTE, FUEL_TEMP_SCL);
      break;
    case MOTEC_ID + 2:
      motecDataItems[AMBIENT_PRES_IDX].value = parseMsgMotec(&msg, AMBIENT_PRES_BYTE, AMBIENT_PRES_SCL);
      motecDataItems[OIL_PRES_IDX].value = parseMsgMotec(&msg, OIL_PRES_BYTE, OIL_PRES_SCL);
      motecDataItems[MANIFOLD_PRES_IDX].value = parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL);
      motecDataItems[FUEL_PRES_IDX].value = parseMsgMotec(&msg, FUEL_PRES_BYTE, FUEL_PRES_SCL);
      break;
    case MOTEC_ID + 3:
      motecDataItems[WHEELSPEED_FL_IDX].value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
      motecDataItems[WHEELSPEED_FR_IDX].value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
      motecDataItems[WHEELSPEED_RL_IDX].value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
      motecDataItems[WHEELSPEED_RR_IDX].value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
      break;
    case MOTEC_ID + 4:
      motecDataItems[DRIVE_SPEED_IDX].value = parseMsgMotec(&msg, DRIVE_SPEED_BYTE, DRIVE_SPEED_SCL);
      motecDataItems[GROUND_SPEED_IDX].value = parseMsgMotec(&msg, GROUND_SPEED_BYTE, GROUND_SPEED_SCL);
      motecDataItems[GPS_SPEED_IDX].value = parseMsgMotec(&msg, GPS_SPEED_BYTE, GPS_SPEED_SCL);
      motecDataItems[GPS_ALT_IDX].value = parseMsgMotec(&msg, GPS_ALT_BYTE, GPS_ALT_SCL);
      break;
    case MOTEC_ID + 5:
      break;
    case MOTEC_ID + 6:
      motecDataItems[GPS_TIME_IDX].value = (double) ((msg.data[GPS_TIME_BYTE] << 24)|(msg.data[GPS_TIME_BYTE+1] << 16) |(msg.data[GPS_TIME_BYTE+2] << 8)|msg.data[GPS_TIME_BYTE+3]) * GPS_TIME_SCL;
      motecDataItems[RUN_TIME_IDX].value = parseMsgMotec(&msg, RUN_TIME_BYTE, RUN_TIME_SCL);
      motecDataItems[FUEL_USED_IDX].value = parseMsgMotec(&msg, FUEL_USED_BYTE, FUEL_USED_SCL);
      break;
    case MOTEC_ID + 7:
      motecDataItems[FUEL_INJ_DUTY_IDX].value = parseMsgMotec(&msg, FUEL_INJ_DUTY_BYTE, FUEL_INJ_DUTY_SCL);
      motecDataItems[FUEL_TRIM_IDX].value = parseMsgMotec(&msg, FUEL_TRIM_BYTE, FUEL_TRIM_SCL);
      motecDataItems[SHIFT_FORCE_IDX].value = parseMsgMotec(&msg, SHIFT_FORCE_BYTE, SHIFT_FORCE_SCL);
      motecDataItems[AIR_TEMP_IDX].value = parseMsgMotec(&msg, AIR_TEMP_BYTE, AIR_TEMP_SCL);
      break;

      /*GCM*/
    case GCM_ID:
      gcmDataItems[UPTIME_IDX].value = (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL;
      gcmDataItems[PCB_TEMP_IDX].value = (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL;
      gcmDataItems[IC_TEMP_IDX].value = (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL;
      break;
    case GCM_ID + 1:
      gcmDataItems[GEAR_IDX].value = (uint8_t) (msg.data[GEAR_BYTE]) * GEAR_SCL;
      gcmDataItems[GEAR_VOLT_IDX].value = (uint16_t) (lsbArray[GEAR_VOLT_BYTE/2]) * GEAR_VOLT_SCL;
      gcmDataItems[FORCE_IDX].value = (int16_t) (lsbArray[FORCE_BYTE/2]) * FORCE_SCL;
      break;
    case GCM_ID + 2:
      gcmDataItems[PADDLE_UP_SW_IDX].value = msg.data[GCM_SWITCH_BYTE] & PADDLE_UP_GCM_SW_MASK;
      gcmDataItems[PADDLE_DOWN_SW_IDX].value = msg.data[GCM_SWITCH_BYTE] & PADDLE_DOWN_GCM_SW_MASK;
      gcmDataItems[NEUTRAL_SW_IDX].value = msg.data[GCM_SWITCH_BYTE] & NEUTRAL_GCM_SW_MASK;
      gcmDataItems[QUEUE_UP_IDX].value = (uint8_t) (msg.data[QUEUE_UP_BYTE]) * QUEUE_UP_SCL;
      gcmDataItems[QUEUE_DN_IDX].value = (uint8_t) (msg.data[QUEUE_DN_BYTE]) * QUEUE_DN_SCL;
      gcmDataItems[QUEUE_NT_IDX].value = (uint8_t) (msg.data[QUEUE_NT_BYTE]) * QUEUE_NT_SCL;
      break;

      /*PDM*/
    case PDM_ID:
      pdmDataItems[UPTIME_IDX].value = (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL;
      pdmDataItems[PCB_TEMP_IDX].value = (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL;
      pdmDataItems[IC_TEMP_IDX].value = (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL;
      break;
    case PDM_ID + 1:

      // Enablity
      pdmDataItems[STR_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & STR_ENBL_MASK;
      pdmDataItems[BVBAT_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & BVBAT_ENBL_MASK;
      pdmDataItems[AUX_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & AUX_ENBL_MASK;
      pdmDataItems[ECU_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & ECU_ENBL_MASK;
      pdmDataItems[WTR_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & WTR_ENBL_MASK;
      pdmDataItems[FAN_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & FAN_ENBL_MASK;
      pdmDataItems[PDLD_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & PDLD_ENBL_MASK;
      pdmDataItems[PDLU_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & PDLU_ENBL_MASK;
      pdmDataItems[ABS_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & ABS_ENBL_MASK;
      pdmDataItems[INJ_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & INJ_ENBL_MASK;
      pdmDataItems[IGN_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & IGN_ENBL_MASK;
      pdmDataItems[FUEL_ENABLITY_IDX].value = lsbArray[LOAD_ENABLITY_BYTE/2] & FUEL_ENBL_MASK;

      // Peak Mode
      pdmDataItems[STR_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & STR_PEAKM_MASK;
      pdmDataItems[BVBAT_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & BVBAT_PEAKM_MASK;
      pdmDataItems[AUX_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & AUX_PEAKM_MASK;
      pdmDataItems[ECU_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & ECU_PEAKM_MASK;
      pdmDataItems[WTR_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & WTR_PEAKM_MASK;
      pdmDataItems[FAN_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & FAN_PEAKM_MASK;
      pdmDataItems[PDLD_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & PDLD_PEAKM_MASK;
      pdmDataItems[PDLU_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & PDLU_PEAKM_MASK;
      pdmDataItems[ABS_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & ABS_PEAKM_MASK;
      pdmDataItems[INJ_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & INJ_PEAKM_MASK;
      pdmDataItems[IGN_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & IGN_PEAKM_MASK;
      pdmDataItems[FUEL_PEAK_MODE_IDX].value = lsbArray[LOAD_PEAK_BYTE/2] & FUEL_PEAKM_MASK;

      // Total current
      pdmDataItems[TOTAL_CURRENT_IDX].value = (uint16_t) (lsbArray[TOTAL_CURRENT_BYTE/2]) * TOTAL_CURRENT_SCL;

      // Switch bitmap
      pdmDataItems[AUX2_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & AUX2_PDM_SW_MASK;
      pdmDataItems[AUX1_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & AUX1_PDM_SW_MASK;
      pdmDataItems[ABS_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & ABS_PDM_SW_MASK;
      pdmDataItems[KILL_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & KILL_PDM_SW_MASK;
      pdmDataItems[ACT_DN_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & ACT_DN_PDM_SW_MASK;
      pdmDataItems[ACT_UP_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & ACT_UP_PDM_SW_MASK;
      pdmDataItems[ON_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & ON_PDM_SW_MASK;
      pdmDataItems[STR_SWITCH_IDX].value = msg.data[PDM_SWITCH_BYTE] & STR_PDM_SW_MASK;

      // Flags
      pdmDataItems[KILL_ENGINE_FLAG_IDX].value = msg.data[PDM_FLAG_BYTE] & KILL_ENGINE_PDM_FLAG_MASK;
      pdmDataItems[KILL_CAR_FLAG_IDX].value = msg.data[PDM_FLAG_BYTE] & KILL_CAR_PDM_FLAG_MASK;
      pdmDataItems[OVER_TEMP_FLAG_IDX].value = msg.data[PDM_FLAG_BYTE] & OVER_TEMP_PDM_FLAG_MASK;
      pdmDataItems[FUEL_PRIME_FLAG_IDX].value = msg.data[PDM_FLAG_BYTE] & FUEL_PRIME_PDM_FLAG_MASK;
      break;
    case PDM_ID + 2:
      pdmDataItems[VBAT_RAIL_IDX].value = (uint16_t) (lsbArray[VBAT_RAIL_BYTE/2]) * VBAT_RAIL_SCL;
      pdmDataItems[V12_RAIL_IDX].value = (uint16_t) (lsbArray[V12_RAIL_BYTE/2]) * V12_RAIL_SCL;
      pdmDataItems[V5_RAIL_IDX].value = (uint16_t) (lsbArray[V5_RAIL_BYTE/2]) * V5_RAIL_SCL;
      pdmDataItems[V3V3_RAIL_IDX].value = (uint16_t) (lsbArray[V3V3_RAIL_BYTE/2]) * V3V3_RAIL_SCL;
      break;
    case PDM_ID + 3:
      pdmDataItems[FUEL_DRAW_IDX].value = (uint16_t) (lsbArray[FUEL_DRAW_BYTE/2]) * FUEL_DRAW_SCL;
      pdmDataItems[IGN_DRAW_IDX].value = (uint16_t) (lsbArray[IGN_DRAW_BYTE/2]) * IGN_DRAW_SCL;
      pdmDataItems[INJ_DRAW_IDX].value = (uint16_t) (lsbArray[INJ_DRAW_BYTE/2]) * INJ_DRAW_SCL;
      pdmDataItems[ABS_DRAW_IDX].value = (uint16_t) (lsbArray[ABS_DRAW_BYTE/2]) * ABS_DRAW_SCL;
      break;
    case PDM_ID + 4:
      pdmDataItems[PDLU_DRAW_IDX].value = (uint16_t) (lsbArray[PDLU_DRAW_BYTE/2]) * PDLU_DRAW_SCL;
      pdmDataItems[PDLD_DRAW_IDX].value = (uint16_t) (lsbArray[PDLD_DRAW_BYTE/2]) * PDLD_DRAW_SCL;
      pdmDataItems[FAN_DRAW_IDX].value = (uint16_t) (lsbArray[FAN_DRAW_BYTE/2]) * FAN_DRAW_SCL;
      pdmDataItems[WTR_DRAW_IDX].value = (uint16_t) (lsbArray[WTR_DRAW_BYTE/2]) * WTR_DRAW_SCL;
      break;
    case PDM_ID + 5:
      pdmDataItems[ECU_DRAW_IDX].value = (uint16_t) (lsbArray[ECU_DRAW_BYTE/2]) * ECU_DRAW_SCL;
      pdmDataItems[AUX_DRAW_IDX].value = (uint16_t) (lsbArray[AUX_DRAW_BYTE/2]) * AUX_DRAW_SCL;
      pdmDataItems[BVBAT_DRAW_IDX].value = (uint16_t) (lsbArray[BVBAT_DRAW_BYTE/2]) * BVBAT_DRAW_SCL;
      pdmDataItems[STR_DRAW_IDX].value = (uint16_t) (lsbArray[STR_DRAW_BYTE/2]) * STR_DRAW_SCL;
      break;
    case PDM_ID + 6:
      pdmDataItems[FUEL_CUT_IDX].value = (uint16_t) (lsbArray[FUEL_CUT_BYTE/2]) * FUEL_CUT_SCL;
      pdmDataItems[IGN_CUT_IDX].value = (uint16_t) (lsbArray[IGN_CUT_BYTE/2]) * IGN_CUT_SCL;
      pdmDataItems[INJ_CUT_IDX].value = (uint16_t) (lsbArray[INJ_CUT_BYTE/2]) * INJ_CUT_SCL;
      pdmDataItems[ABS_CUT_IDX].value = (uint16_t) (lsbArray[ABS_CUT_BYTE/2]) * ABS_CUT_SCL;
      break;
    case PDM_ID + 7:
      pdmDataItems[PDLU_CUT_IDX].value = (uint16_t) (lsbArray[PDLU_CUT_BYTE/2]) * PDLU_CUT_SCL;
      pdmDataItems[PDLD_CUT_IDX].value = (uint16_t) (lsbArray[PDLD_CUT_BYTE/2]) * PDLD_CUT_SCL;
      pdmDataItems[FAN_CUT_IDX].value = (uint16_t) (lsbArray[FAN_CUT_BYTE/2]) * FAN_CUT_SCL;
      pdmDataItems[WTR_CUT_IDX].value = (uint16_t) (lsbArray[WTR_CUT_BYTE/2]) * WTR_CUT_SCL;
      break;
    case PDM_ID + 8:
      pdmDataItems[ECU_CUT_IDX].value = (uint16_t) (lsbArray[ECU_CUT_BYTE/2]) * ECU_CUT_SCL;
      pdmDataItems[AUX_CUT_IDX].value = (uint16_t) (lsbArray[AUX_CUT_BYTE/2]) * AUX_CUT_SCL;
      pdmDataItems[BVBAT_CUT_IDX].value = (uint16_t) (lsbArray[BVBAT_CUT_BYTE/2]) * BVBAT_CUT_SCL;
      break;
    case PDM_ID + 9:
      pdmDataItems[FUEL_CUT_P_IDX].value = (uint16_t) (lsbArray[FUEL_CUT_P_BYTE/2]) * FUEL_CUT_P_SCL;
      pdmDataItems[FAN_CUT_P_IDX].value = (uint16_t) (lsbArray[FAN_CUT_P_BYTE/2]) * FAN_CUT_P_SCL;
      pdmDataItems[WTR_CUT_P_IDX].value = (uint16_t) (lsbArray[WTR_CUT_P_BYTE/2]) * WTR_CUT_P_SCL;
      pdmDataItems[ECU_CUT_P_IDX].value = (uint16_t) (lsbArray[ECU_CUT_P_BYTE/2]) * ECU_CUT_P_SCL;
      break;
    case PDM_ID + 10:
      pdmDataItems[FUEL_OC_COUNT_IDX].value = (uint16_t) (msg.data[FUEL_OC_COUNT_BYTE]);
      pdmDataItems[IGN_OC_COUNT_IDX].value = (uint16_t) (msg.data[IGN_OC_COUNT_BYTE]);
      pdmDataItems[INJ_OC_COUNT_IDX].value = (uint16_t) (msg.data[INJ_OC_COUNT_BYTE]);
      pdmDataItems[ABS_OC_COUNT_IDX].value = (uint16_t) (msg.data[ABS_OC_COUNT_BYTE]);
      pdmDataItems[PDLU_OC_COUNT_IDX].value = (uint16_t) (msg.data[PDLU_OC_COUNT_BYTE]);
      pdmDataItems[PDLD_OC_COUNT_IDX].value = (uint16_t) (msg.data[PDLD_OC_COUNT_BYTE]);
      pdmDataItems[FAN_OC_COUNT_IDX].value = (uint16_t) (msg.data[FAN_OC_COUNT_BYTE]);
      pdmDataItems[WTR_OC_COUNT_IDX].value = (uint16_t) (msg.data[WTR_OC_COUNT_BYTE]);
      break;
    case PDM_ID + 11:
      pdmDataItems[ECU_OC_COUNT_IDX].value = (uint16_t) (msg.data[ECU_OC_COUNT_BYTE]);
      pdmDataItems[AUX_OC_COUNT_IDX].value = (uint16_t) (msg.data[AUX_OC_COUNT_BYTE]);
      pdmDataItems[BVBAT_OC_COUNT_IDX].value = (uint16_t) (msg.data[BVBAT_OC_COUNT_BYTE]);
      pdmDataItems[STR_OC_COUNT_IDX].value = (uint16_t) (msg.data[STR_OC_COUNT_BYTE]);
      break;

      /*Tire Temps*/
    case TIRE_TEMP_FL_ID:
      tireTempDataItems[FL0].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL1].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL2].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL3].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL].value = (tireTempDataItems[FL0].value + tireTempDataItems[FL1].value + tireTempDataItems[FL2].value + tireTempDataItems[FL3].value)/4.0;
      break;
    case TIRE_TEMP_FR_ID:
      tireTempDataItems[FR0].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR1].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR2].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR3].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR].value = (tireTempDataItems[FR0].value + tireTempDataItems[FR1].value + tireTempDataItems[FR2].value + tireTempDataItems[FR3].value)/4.0;
      break;
    case TIRE_TEMP_RL_ID:
      tireTempDataItems[RL0].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL1].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL2].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL3].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL].value = (tireTempDataItems[RL0].value + tireTempDataItems[RL1].value + tireTempDataItems[RL2].value + tireTempDataItems[RL3].value)/4.0;
      break;
    case TIRE_TEMP_RR_ID:
      tireTempDataItems[RR0].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR1].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR2].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR3].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR].value = (tireTempDataItems[RR0].value + tireTempDataItems[RR1].value + tireTempDataItems[RR2].value + tireTempDataItems[RR3].value)/4.0;
      break;

      // SPM
    case SPM_ID:
      spmDataItems[UPTIME_IDX].value = (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL;
      spmDataItems[PCB_TEMP_IDX].value = (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL;
      spmDataItems[IC_TEMP_IDX].value = (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL;
      break;
    case SPM_ID + 1:
      spmDataItems[ANALOG_CHAN_0_IDX].value = (double) (lsbArray[ANALOG_CHAN_0_BYTE/2] * ANALOG_CHAN_0_SCL);
      spmDataItems[ANALOG_CHAN_1_IDX].value = (double) (lsbArray[ANALOG_CHAN_1_BYTE/2] * ANALOG_CHAN_1_SCL);
      spmDataItems[ANALOG_CHAN_2_IDX].value = (double) (lsbArray[ANALOG_CHAN_2_BYTE/2] * ANALOG_CHAN_2_SCL);
      spmDataItems[ANALOG_CHAN_3_IDX].value = (double) (lsbArray[ANALOG_CHAN_3_BYTE/2] * ANALOG_CHAN_3_SCL);
      break;
    case SPM_ID + 2:
      spmDataItems[ANALOG_CHAN_4_IDX].value = (double) (lsbArray[ANALOG_CHAN_4_BYTE/2] * ANALOG_CHAN_4_SCL);
      spmDataItems[ANALOG_CHAN_5_IDX].value = (double) (lsbArray[ANALOG_CHAN_5_BYTE/2] * ANALOG_CHAN_5_SCL);
      spmDataItems[ANALOG_CHAN_6_IDX].value = (double) (lsbArray[ANALOG_CHAN_6_BYTE/2] * ANALOG_CHAN_6_SCL);
      spmDataItems[ANALOG_CHAN_7_IDX].value = (double) (lsbArray[ANALOG_CHAN_7_BYTE/2] * ANALOG_CHAN_7_SCL);
      break;
    case SPM_ID + 3:
      spmDataItems[ANALOG_CHAN_8_IDX].value = (double) (lsbArray[ANALOG_CHAN_8_BYTE/2] * ANALOG_CHAN_8_SCL);
      spmDataItems[ANALOG_CHAN_9_IDX].value = (double) (lsbArray[ANALOG_CHAN_9_BYTE/2] * ANALOG_CHAN_9_SCL);
      spmDataItems[ANALOG_CHAN_10_IDX].value = (double) (lsbArray[ANALOG_CHAN_10_BYTE/2] * ANALOG_CHAN_10_SCL);
      spmDataItems[ANALOG_CHAN_11_IDX].value = (double) (lsbArray[ANALOG_CHAN_11_BYTE/2] * ANALOG_CHAN_11_SCL);
      break;
    case SPM_ID + 4:
      spmDataItems[ANALOG_CHAN_12_IDX].value = (double) (lsbArray[ANALOG_CHAN_12_BYTE/2] * ANALOG_CHAN_12_SCL);
      spmDataItems[ANALOG_CHAN_13_IDX].value = (double) (lsbArray[ANALOG_CHAN_13_BYTE/2] * ANALOG_CHAN_13_SCL);
      spmDataItems[ANALOG_CHAN_14_IDX].value = (double) (lsbArray[ANALOG_CHAN_14_BYTE/2] * ANALOG_CHAN_14_SCL);
      spmDataItems[ANALOG_CHAN_15_IDX].value = (double) (lsbArray[ANALOG_CHAN_15_BYTE/2] * ANALOG_CHAN_15_SCL);
      break;
    case SPM_ID + 5:
      spmDataItems[ANALOG_CHAN_16_IDX].value = (double) (lsbArray[ANALOG_CHAN_16_BYTE/2] * ANALOG_CHAN_16_SCL);
      spmDataItems[ANALOG_CHAN_17_IDX].value = (double) (lsbArray[ANALOG_CHAN_17_BYTE/2] * ANALOG_CHAN_17_SCL);
      spmDataItems[ANALOG_CHAN_18_IDX].value = (double) (lsbArray[ANALOG_CHAN_18_BYTE/2] * ANALOG_CHAN_18_SCL);
      spmDataItems[ANALOG_CHAN_19_IDX].value = (double) (lsbArray[ANALOG_CHAN_19_BYTE/2] * ANALOG_CHAN_19_SCL);
      break;
    case SPM_ID + 6:
      spmDataItems[ANALOG_CHAN_20_IDX].value = (double) (lsbArray[ANALOG_CHAN_20_BYTE/2] * ANALOG_CHAN_20_SCL);
      spmDataItems[ANALOG_CHAN_21_IDX].value = (double) (lsbArray[ANALOG_CHAN_21_BYTE/2] * ANALOG_CHAN_21_SCL);
      spmDataItems[ANALOG_CHAN_22_IDX].value = (double) (lsbArray[ANALOG_CHAN_22_BYTE/2] * ANALOG_CHAN_22_SCL);
      spmDataItems[ANALOG_CHAN_23_IDX].value = (double) (lsbArray[ANALOG_CHAN_23_BYTE/2] * ANALOG_CHAN_23_SCL);
      break;
    case SPM_ID + 7:
      spmDataItems[ANALOG_CHAN_24_IDX].value = (double) (lsbArray[ANALOG_CHAN_24_BYTE/2] * ANALOG_CHAN_24_SCL);
      spmDataItems[ANALOG_CHAN_25_IDX].value = (double) (lsbArray[ANALOG_CHAN_25_BYTE/2] * ANALOG_CHAN_25_SCL);
      spmDataItems[ANALOG_CHAN_26_IDX].value = (double) (lsbArray[ANALOG_CHAN_26_BYTE/2] * ANALOG_CHAN_26_SCL);
      spmDataItems[ANALOG_CHAN_27_IDX].value = (double) (lsbArray[ANALOG_CHAN_27_BYTE/2] * ANALOG_CHAN_27_SCL);
      break;
    case SPM_ID + 8:
      spmDataItems[ANALOG_CHAN_28_IDX].value = (double) (lsbArray[ANALOG_CHAN_28_BYTE/2] * ANALOG_CHAN_28_SCL);
      spmDataItems[ANALOG_CHAN_29_IDX].value = (double) (lsbArray[ANALOG_CHAN_29_BYTE/2] * ANALOG_CHAN_29_SCL);
      spmDataItems[ANALOG_CHAN_30_IDX].value = (double) (lsbArray[ANALOG_CHAN_30_BYTE/2] * ANALOG_CHAN_30_SCL);
      spmDataItems[ANALOG_CHAN_31_IDX].value = (double) (lsbArray[ANALOG_CHAN_31_BYTE/2] * ANALOG_CHAN_31_SCL);
      break;
    case SPM_ID + 9:
      spmDataItems[ANALOG_CHAN_32_IDX].value = (double) (lsbArray[ANALOG_CHAN_32_BYTE/2] * ANALOG_CHAN_32_SCL);
      spmDataItems[ANALOG_CHAN_33_IDX].value = (double) (lsbArray[ANALOG_CHAN_33_BYTE/2] * ANALOG_CHAN_33_SCL);
      spmDataItems[ANALOG_CHAN_34_IDX].value = (double) (lsbArray[ANALOG_CHAN_34_BYTE/2] * ANALOG_CHAN_34_SCL);
      spmDataItems[ANALOG_CHAN_35_IDX].value = (double) (lsbArray[ANALOG_CHAN_35_BYTE/2] * ANALOG_CHAN_35_SCL);
      break;
    case SPM_ID + 10:
      spmDataItems[TCOUPLE_0_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_0_BYTE/2]) * TCOUPLE_SCL);
      spmDataItems[TCOUPLE_1_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_1_BYTE/2]) * TCOUPLE_SCL);
      spmDataItems[TCOUPLE_2_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_2_BYTE/2]) * TCOUPLE_SCL);
      spmDataItems[TCOUPLE_3_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_3_BYTE/2]) * TCOUPLE_SCL);
      break;
    case SPM_ID + 11:
      spmDataItems[TCOUPLE_4_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_4_BYTE/2]) * TCOUPLE_SCL); 
      spmDataItems[TCOUPLE_5_IDX].value = (double) ((int16_t) (lsbArray[TCOUPLE_5_BYTE/2]) * TCOUPLE_SCL);
      spmDataItems[AVG_JUNCT_TEMP_IDX].value = (double) ((int16_t) (lsbArray[AVG_JUNCT_TEMP_BYTE/2]) * AVG_JUNCT_TEMP_SCL);
      spmDataItems[TCOUPLE_1_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_0_FAULT_MASK;
      spmDataItems[TCOUPLE_1_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_1_FAULT_MASK;
      spmDataItems[TCOUPLE_2_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_2_FAULT_MASK;
      spmDataItems[TCOUPLE_3_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_3_FAULT_MASK;
      spmDataItems[TCOUPLE_4_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_4_FAULT_MASK;
      spmDataItems[TCOUPLE_5_FAULT_IDX].value = lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_5_FAULT_MASK;
      break;
    case SPM_ID + 12:
      spmDataItems[DIGITAL_INPUT_0_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_0_MASK;
      spmDataItems[DIGITAL_INPUT_1_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_1_MASK;
      spmDataItems[DIGITAL_INPUT_2_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_2_MASK;
      spmDataItems[DIGITAL_INPUT_3_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_3_MASK;
      spmDataItems[DIGITAL_INPUT_4_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_4_MASK;
      spmDataItems[DIGITAL_INPUT_5_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_5_MASK;
      spmDataItems[DIGITAL_INPUT_6_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_6_MASK;
      spmDataItems[DIGITAL_INPUT_7_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_7_MASK;
      spmDataItems[DIGITAL_INPUT_8_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_8_MASK;
      spmDataItems[DIGITAL_INPUT_9_IDX ].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_9_MASK;
      spmDataItems[DIGITAL_INPUT_10_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_10_MASK;
      spmDataItems[DIGITAL_INPUT_11_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_11_MASK;
      spmDataItems[DIGITAL_INPUT_12_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_12_MASK;
      spmDataItems[DIGITAL_INPUT_13_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_13_MASK;
      spmDataItems[DIGITAL_INPUT_14_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_14_MASK;
      spmDataItems[DIGITAL_INPUT_15_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_15_MASK;
      spmDataItems[FREQ_COUNT_0_IDX].value = (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL);
      spmDataItems[FREQ_COUNT_1_IDX].value = (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL);
      spmDataItems[FREQ_COUNT_2_IDX].value = (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL);
      break;
    case SPM_ID + 13:
      spmDataItems[PGA_0_SETTINGS_IDX ].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_0_SETTINGS_MASK) >> PGA_0_SETTINGS_SHF;
      spmDataItems[PGA_1_SETTINGS_IDX ].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_1_SETTINGS_MASK) >> PGA_1_SETTINGS_SHF;
      spmDataItems[PGA_2_SETTINGS_IDX ].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_2_SETTINGS_MASK) >> PGA_2_SETTINGS_SHF;
      spmDataItems[PGA_3_SETTINGS_IDX ].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_3_SETTINGS_MASK) >> PGA_3_SETTINGS_SHF;
      spmDataItems[FREQ_0_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_0_SETTINGS_MASK) >> FREQ_0_SETTINGS_SHF;
      spmDataItems[FREQ_1_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_1_SETTINGS_MASK) >> FREQ_1_SETTINGS_SHF;
      spmDataItems[FREQ_2_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_2_SETTINGS_MASK) >> FREQ_2_SETTINGS_SHF;
      break;

      /*Front Analog Hub*/
    case ANALOG_FRONT_ID + 1:
      brakePressFront.value = (double) ((uint16_t) (lsbArray[BPF_BYTE/2]) * BRK_PRS_SCL);
      if(brakePressFront.value > brakeMaxFront.value)
      {
        brakeMaxFront.value = brakePressFront.value;
      }
      if (brakePressFront.value < brakeMinFront.value)
      {
        brakeMinFront.value = brakePressFront.value;
      }
      brakePressRear.value = (double) ((uint16_t) (lsbArray[BPR_BYTE/2]) * BRK_PRS_SCL);
      if(brakePressRear.value > brakeMaxRear.value)
      {
        brakeMaxRear.value = brakePressRear.value;
      }
      if (brakePressRear.value < brakeMinRear.value)
      {
        brakeMinRear.value = brakePressRear.value;
      }
  }
}

void CANswitchStates(void){
  CAN_data switchData = {0};
  uint8_t bitMask = (uint8_t)momentaries[0].value|(uint8_t)momentaries[1].value << 1|
    (uint8_t)momentaries[2].value << 2|(uint8_t)momentaries[3].value << 3|
    (uint8_t)switches[0].value << 4|(uint8_t)switches[1].value << 5|
    (uint8_t)switches[2].value << 6|(uint8_t)switches[3].value << 7;
  switchData.byte0 = bitMask;
  switchData.byte1 = ((uint8_t)rotary[0].value << 4) | (uint8_t)rotary[1].value;
  switchData.byte2 = ((uint8_t)rotary[2].value << 4) | (uint8_t)tRotary[0].value;
  switchData.byte3 = (uint8_t)tRotary[1].value << 4;
  //CAN_send_message(WHEEL_ID + 0x1, 4, switchData);
}

void CANswitchADL(void){
  CAN_data rotaries = {0};
  rotaries.halfword0 = 0x0000;
  rotaries.halfword1 = (uint16_t) rotary[0].value;
  rotaries.halfword2 = (uint16_t) rotary[1].value;
  rotaries.halfword3 = (uint16_t) rotary[2].value;
  //CAN_send_message(ADL_ID, 8, rotaries);
}

void CANdiag(void){
  return;
}

double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl){
  return ((double) ((msg->data[byte] << 8) | msg->data[byte + 1])) * scl;
}

void updateSwVals(void){
  // Update Switches
  switches[0].value = !SW1_PORT;
  switches[1].value = !SW2_PORT;
  switches[2].value = !SW3_PORT;
  switches[3].value = !SW4_PORT;
  // Update Momentaries
  momentaries[0].value = !MOM1_PORT;
  momentaries[1].value = !MOM2_PORT;
  momentaries[2].value = !MOM3_PORT;
  momentaries[3].value = !MOM4_PORT;
  // Update rotaries
  rotary[0].value = getRotaryPosition(read_adc_chn(ROT1_CHN));
  rotary[1].value = getRotaryPosition(read_adc_chn(ROT2_CHN));
  rotary[2].value = getRotaryPosition(read_adc_chn(ROT3_CHN));
  tRotary[0].value = getRotaryPosition(read_adc_chn(TROT1_CHN));
  tRotary[1].value = getRotaryPosition(read_adc_chn(TROT2_CHN));
}

// 10 is error
uint8_t getRotaryPosition(uint32_t adcValue){
  if(adcValue >= ROT_RANGE_LOW && adcValue < ROT_RANGE_0) {return 0;}
  if(adcValue >= ROT_RANGE_0 && adcValue < ROT_RANGE_1) {return 1;}
  if(adcValue >= ROT_RANGE_1 && adcValue < ROT_RANGE_2) {return 2;}
  if(adcValue >= ROT_RANGE_2 && adcValue < ROT_RANGE_3) {return 3;}
  if(adcValue >= ROT_RANGE_3 && adcValue < ROT_RANGE_4) {return 4;}
  if(adcValue >= ROT_RANGE_4 && adcValue < ROT_RANGE_5) {return 5;}
  if(adcValue >= ROT_RANGE_5 && adcValue < ROT_RANGE_6) {return 6;}
  if(adcValue >= ROT_RANGE_6 && adcValue < ROT_RANGE_7) {return 7;}
  if(adcValue >= ROT_RANGE_7 && adcValue < ROT_RANGE_8) {return 8;}
  if(adcValue >= ROT_RANGE_8 && adcValue < ROT_RANGE_HIGH) {return 9;}
  return 10;
}

void init_spi(){
  unlock_config();

  // Initialize SDI1/SDO1 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB9 = INPUT;
  ANSELBbits.ANSB9 = DIG_INPUT;
  SDI1Rbits.SDI1R = 0b0101; // RPB9
  TRISBbits.TRISB10 = OUTPUT;
  RPB10Rbits.RPB10R = 0b0101; // SDO1
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK1 and !CS_NVM pins
  TRISDbits.TRISD1 = OUTPUT; // SCK1

  // Disable interrupts
  IEC3bits.SPI1EIE = 0;
  IEC3bits.SPI1RXIE = 0;
  IEC3bits.SPI1TXIE = 0;

  // Disable SPI1 module
  SPI1CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI1BUF;

  // Use standard buffer mode
  SPI1CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI1BRG = 24;

  SPI1STATbits.SPIROV = 0;

  SPI1CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI1CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI1CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI1CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI1CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI1CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI1CONbits.DISSDI = 0;
  SPI1CONbits.DISSDO = 0;
  SPI1CONbits.SMP = 1;
  SPI1CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI1 module
  SPI1CONbits.ON = 1;

  lock_config();
}
