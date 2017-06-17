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

  SW0_TRIS = INPUT;
  SW1_TRIS = INPUT;
  SW2_TRIS = INPUT;
  SW3_TRIS = INPUT;
  MOM0_TRIS = INPUT;
  MOM1_TRIS = INPUT;
  MOM2_TRIS = INPUT;
  MOM3_TRIS = INPUT;

  ROT0_TRIS = INPUT;
  ROT0_ANSEL = AN_INPUT;
  ROT0_CSS = 1;
  ROT1_TRIS = INPUT;
  ROT1_ANSEL = AN_INPUT;
  ROT1_CSS = 1;
  ROT1_TRG = SOFTWARE;
  ROT2_TRIS = INPUT;
  ROT2_ANSEL = AN_INPUT;
  ROT2_CSS = 1;
  ROT2_TRG = SOFTWARE;
  TROT0_TRIS = INPUT;
  TROT0_ANSEL = AN_INPUT;
  TROT0_CSS = 1;
  TROT0_TRG = SOFTWARE;
  TROT1_TRIS = INPUT;
  TROT1_ANSEL = AN_INPUT;
  TROT1_CSS = 1;

  // Initialize RA8875
  init_ra8875(1, LCD_CS_LATBITS, LCD_CS_LATNUM); // Don't store SPIConn pointer since will always use default
  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX

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

    checkChangeScreen(); // Check to see if the screen should be changed

    refreshScreenItems(); // Refresh items on screen
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
      tireTempDataItems[FL0_IDX].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL1_IDX].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL2_IDX].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL3_IDX].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FL_IDX].value = (tireTempDataItems[FL0_IDX].value + tireTempDataItems[FL1_IDX].value + tireTempDataItems[FL2_IDX].value + tireTempDataItems[FL3_IDX].value)/4.0;
      break;
    case TIRE_TEMP_FR_ID:
      tireTempDataItems[FR0_IDX].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR1_IDX].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR2_IDX].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR3_IDX].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[FR_IDX].value = (tireTempDataItems[FR0_IDX].value + tireTempDataItems[FR1_IDX].value + tireTempDataItems[FR2_IDX].value + tireTempDataItems[FR3_IDX].value)/4.0;
      break;
    case TIRE_TEMP_RL_ID:
      tireTempDataItems[RL0_IDX].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL1_IDX].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL2_IDX].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL3_IDX].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RL_IDX].value = (tireTempDataItems[RL0_IDX].value + tireTempDataItems[RL1_IDX].value + tireTempDataItems[RL2_IDX].value + tireTempDataItems[RL3_IDX].value)/4.0;
      break;
    case TIRE_TEMP_RR_ID:
      tireTempDataItems[RR0_IDX].value = (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR1_IDX].value = (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR2_IDX].value = (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR3_IDX].value = (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL);
      tireTempDataItems[RR_IDX].value = (tireTempDataItems[RR0_IDX].value + tireTempDataItems[RR1_IDX].value + tireTempDataItems[RR2_IDX].value + tireTempDataItems[RR3_IDX].value)/4.0;
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
      spmDataItems[DIGITAL_INPUT_0_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_0_MASK;
      spmDataItems[DIGITAL_INPUT_1_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_1_MASK;
      spmDataItems[DIGITAL_INPUT_2_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_2_MASK;
      spmDataItems[DIGITAL_INPUT_3_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_3_MASK;
      spmDataItems[DIGITAL_INPUT_4_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_4_MASK;
      spmDataItems[DIGITAL_INPUT_5_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_5_MASK;
      spmDataItems[DIGITAL_INPUT_6_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_6_MASK;
      spmDataItems[DIGITAL_INPUT_7_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_7_MASK;
      spmDataItems[DIGITAL_INPUT_8_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_8_MASK;
      spmDataItems[DIGITAL_INPUT_9_IDX].value = lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_9_MASK;
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
      spmDataItems[PGA_0_SETTINGS_IDX].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_0_SETTINGS_MASK) >> PGA_0_SETTINGS_SHF;
      spmDataItems[PGA_1_SETTINGS_IDX].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_1_SETTINGS_MASK) >> PGA_1_SETTINGS_SHF;
      spmDataItems[PGA_2_SETTINGS_IDX].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_2_SETTINGS_MASK) >> PGA_2_SETTINGS_SHF;
      spmDataItems[PGA_3_SETTINGS_IDX].value = (lsbArray[PGA_SETTINGS_BYTE] & PGA_3_SETTINGS_MASK) >> PGA_3_SETTINGS_SHF;
      spmDataItems[FREQ_0_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_0_SETTINGS_MASK) >> FREQ_0_SETTINGS_SHF;
      spmDataItems[FREQ_1_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_1_SETTINGS_MASK) >> FREQ_1_SETTINGS_SHF;
      spmDataItems[FREQ_2_SETTINGS_IDX].value = (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_2_SETTINGS_MASK) >> FREQ_2_SETTINGS_SHF;
      break;
  }
}

void CANswitchStates(void){
  CAN_data switchData = {0};
  int bitMask = 0;
  int i;
  for(i = MOM_0_IDX;i < SWITCH_3_IDX; i++) {
    bitMask |= (uint8_t) wheelDataItems[i].value << (i - MOM_0_IDX);
  }
  switchData.byte0 = bitMask;
  switchData.byte1 = (uint8_t)wheelDataItems[ROTARY_0_IDX].value << 4;
  switchData.byte1 |= (uint8_t)wheelDataItems[ROTARY_1_IDX].value;
  switchData.byte2 = (uint8_t)wheelDataItems[ROTARY_2_IDX].value << 4;
  switchData.byte2 |= (uint8_t)wheelDataItems[TROTARY_0_IDX].value;
  switchData.byte3 = (uint8_t)wheelDataItems[TROTARY_1_IDX].value << 4;
  CAN_send_message(WHEEL_ID + 0x1, 4, switchData);
}

void CANswitchADL(void){
  CAN_data rotaries = {0};
  rotaries.halfword0 = 0x0000;
  rotaries.halfword1 = (uint16_t) wheelDataItems[ROTARY_0_IDX].value;
  rotaries.halfword2 = (uint16_t) wheelDataItems[ROTARY_1_IDX].value;
  rotaries.halfword3 = (uint16_t) wheelDataItems[ROTARY_2_IDX].value;
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
  wheelDataItems[SWITCH_0_IDX].value = !SW0_PORT;
  wheelDataItems[SWITCH_1_IDX].value = !SW1_PORT;
  wheelDataItems[SWITCH_2_IDX].value = !SW2_PORT;
  wheelDataItems[SWITCH_3_IDX].value = !SW3_PORT;
  // Update Momentaries
  wheelDataItems[MOM_0_IDX].value = !MOM0_PORT;
  wheelDataItems[MOM_1_IDX].value = !MOM1_PORT;
  wheelDataItems[MOM_2_IDX].value = !MOM2_PORT;
  wheelDataItems[MOM_3_IDX].value = !MOM3_PORT;
  // Update rotaries
  wheelDataItems[ROTARY_0_IDX].value = getRotaryPosition(read_adc_chn(ROT0_CHN));
  wheelDataItems[ROTARY_1_IDX].value = getRotaryPosition(read_adc_chn(ROT1_CHN));
  wheelDataItems[ROTARY_2_IDX].value = getRotaryPosition(read_adc_chn(ROT2_CHN));
  wheelDataItems[TROTARY_0_IDX].value = getRotaryPosition(read_adc_chn(TROT0_CHN));
  wheelDataItems[TROTARY_1_IDX].value = getRotaryPosition(read_adc_chn(TROT1_CHN));
}

uint8_t getRotaryPosition(uint32_t adcValue){
  return (uint8_t) (adcValue / 409.6);
}

void checkChangeScreen(void) {
  uint8_t rotVal = wheelDataItems[TROTARY_1_IDX].value;
  uint8_t screenIdx;

  switch (rotVal) {
    case 0:
      screenIdx = RACE_SCREEN;
    default:
      screenIdx = RACE_SCREEN;
  }

  if(screenIdx != screenNumber) {
    changeScreen(screenIdx);
    screenNumber = screenIdx;
  }
}
