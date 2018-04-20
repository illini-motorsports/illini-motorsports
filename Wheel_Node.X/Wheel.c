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
volatile uint32_t CANswStateMillis, CANswADLMillis, CANdiagMillis, tempSampMillis;
volatile uint8_t nightModeState;

int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(1);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_adc(NULL);
  init_termination(TERMINATING);
  init_tlc5955();

  ADCCON3bits.GSWTRG = 1; // Initial ADC Conversion
  STI();// Enable interrupts

  millis = 0;
  CANswStateMillis = CANswADLMillis = CANdiagMillis = 0;
  auxNumber = 0;
  warnCount = 0;

  // Initialize specific GPIO pins
  init_gpio_wheel();

  // Initialize RA8875
  init_ra8875(1, LCD_CS_LATBITS, LCD_CS_LATNUM); // Don't store SPIConn pointer since will always use default
  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX

  // Initialize All the data streams
  initDataItems();
  updateSwVals();
  init_can();
  initAllScreens();
  nightModeState = wheelDataItems[SW_ND_IDX].value;
  initNightMode(nightModeState);
  screenNumber = RACE_SCREEN;
  changeScreen(screenNumber);

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

    sample_temp(); // Sample internal and external temperature sensors

    checkChangeScreen(); // Check to see if the screen should be changed

    refreshScreenItems(); // Refresh items on screen
  }
}

/*
 * Initializes all the specific GPIO pins for the wheel board
 */
void init_gpio_wheel(void) {
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

  SW2_ANSEL = DIG_INPUT;
  SW3_ANSEL = DIG_INPUT;
  MOM2_ANSEL = DIG_INPUT;
  MOM3_ANSEL = DIG_INPUT;

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
      updateDataItem(&motecDataItems[ENG_RPM_IDX], parseMsgMotec(&msg, ENG_RPM_BYTE, ENG_RPM_SCL));
      updateDataItem(&motecDataItems[THROTTLE_POS_IDX], parseMsgMotec(&msg, THROTTLE_POS_BYTE, THROTTLE_POS_SCL));
      updateDataItem(&motecDataItems[LAMBDA_IDX], parseMsgMotec(&msg, LAMBDA_BYTE, LAMBDA_SCL));
      updateDataItem(&motecDataItems[VOLT_ECU_IDX], parseMsgMotec(&msg, VOLT_ECU_BYTE, VOLT_ECU_SCL));
      break;
    case MOTEC_ID + 1:
      updateDataItem(&motecDataItems[ENG_TEMP_IDX], parseMsgMotec(&msg, ENG_TEMP_BYTE, ENG_TEMP_SCL));
      updateDataItem(&motecDataItems[OIL_TEMP_IDX], parseMsgMotec(&msg, OIL_TEMP_BYTE, OIL_TEMP_SCL));
      updateDataItem(&motecDataItems[MANIFOLD_TEMP_IDX], parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL));
      updateDataItem(&motecDataItems[FUEL_TEMP_IDX], parseMsgMotec(&msg, FUEL_TEMP_BYTE, FUEL_TEMP_SCL));
      break;
    case MOTEC_ID + 2:
      updateDataItem(&motecDataItems[AMBIENT_PRES_IDX], parseMsgMotec(&msg, AMBIENT_PRES_BYTE, AMBIENT_PRES_SCL));
      updateDataItem(&motecDataItems[OIL_PRES_IDX], parseMsgMotec(&msg, OIL_PRES_BYTE, OIL_PRES_SCL));
      updateDataItem(&motecDataItems[MANIFOLD_PRES_IDX], parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL));
      updateDataItem(&motecDataItems[FUEL_PRES_IDX], parseMsgMotec(&msg, FUEL_PRES_BYTE, FUEL_PRES_SCL));
      break;
    case MOTEC_ID + 3:
      updateDataItem(&motecDataItems[WHEELSPEED_FL_IDX], parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL));
      updateDataItem(&motecDataItems[WHEELSPEED_FR_IDX], parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL));
      updateDataItem(&motecDataItems[WHEELSPEED_RL_IDX], parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL));
      updateDataItem(&motecDataItems[WHEELSPEED_RR_IDX], parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL));
      double whlSpdAvg = (motecDataItems[WHEELSPEED_FL_IDX].value + motecDataItems[WHEELSPEED_FR_IDX].value + motecDataItems[WHEELSPEED_RL_IDX].value + motecDataItems[WHEELSPEED_RR_IDX].value)/4.0;
      updateDataItem(&motecDataItems[WHEELSPEED_AVG_IDX], whlSpdAvg);
      break;
    case MOTEC_ID + 4:
      updateDataItem(&motecDataItems[DRIVE_SPEED_IDX], parseMsgMotec(&msg, DRIVE_SPEED_BYTE, DRIVE_SPEED_SCL));
      updateDataItem(&motecDataItems[GROUND_SPEED_IDX], parseMsgMotec(&msg, GROUND_SPEED_BYTE, GROUND_SPEED_SCL));
      updateDataItem(&motecDataItems[GPS_SPEED_IDX], parseMsgMotec(&msg, GPS_SPEED_BYTE, GPS_SPEED_SCL));
      updateDataItem(&motecDataItems[GPS_ALT_IDX], parseMsgMotec(&msg, GPS_ALT_BYTE, GPS_ALT_SCL));
      break;
    case MOTEC_ID + 5:
      break;
    case MOTEC_ID + 6:
      updateDataItem(&motecDataItems[GPS_TIME_IDX], (double) ((msg.data[GPS_TIME_BYTE] << 24)|(msg.data[GPS_TIME_BYTE+1] << 16) |(msg.data[GPS_TIME_BYTE+2] << 8)|msg.data[GPS_TIME_BYTE+3]) * GPS_TIME_SCL);
      updateDataItem(&motecDataItems[RUN_TIME_IDX], parseMsgMotec(&msg, RUN_TIME_BYTE, RUN_TIME_SCL));
      updateDataItem(&motecDataItems[FUEL_USED_IDX], parseMsgMotec(&msg, FUEL_USED_BYTE, FUEL_USED_SCL));
      break;
    case MOTEC_ID + 7:
      updateDataItem(&motecDataItems[FUEL_INJ_DUTY_IDX], parseMsgMotec(&msg, FUEL_INJ_DUTY_BYTE, FUEL_INJ_DUTY_SCL));
      updateDataItem(&motecDataItems[FUEL_TRIM_IDX], parseMsgMotec(&msg, FUEL_TRIM_BYTE, FUEL_TRIM_SCL));
      updateDataItem(&motecDataItems[SHIFT_FORCE_IDX], parseMsgMotec(&msg, SHIFT_FORCE_BYTE, SHIFT_FORCE_SCL));
      updateDataItem(&motecDataItems[AIR_TEMP_IDX], parseMsgMotec(&msg, AIR_TEMP_BYTE, AIR_TEMP_SCL));
      break;

      /*GCM*/
    case GCM_ID:
      updateDataItem(&gcmDataItems[UPTIME_IDX], (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL);
      updateDataItem(&gcmDataItems[PCB_TEMP_IDX], (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL);
      updateDataItem(&gcmDataItems[IC_TEMP_IDX], (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL);
      break;
    case GCM_ID + 1:
      updateDataItem(&gcmDataItems[GEAR_IDX], (uint8_t) (msg.data[GEAR_BYTE]) * GEAR_SCL);
      updateDataItem(&gcmDataItems[MODE_IDX], (uint8_t) (msg.data[MODE_BYTE]));
      updateDataItem(&gcmDataItems[GEAR_VOLT_IDX], (uint16_t) (lsbArray[GEAR_VOLT_BYTE/2]) * GEAR_VOLT_SCL);
      updateDataItem(&gcmDataItems[FORCE_IDX], (int16_t) (lsbArray[FORCE_BYTE/2]) * FORCE_SCL);
      break;
    case GCM_ID + 2:
      updateDataItem(&gcmDataItems[PADDLE_UP_SW_IDX], msg.data[GCM_SWITCH_BYTE] & PADDLE_UP_GCM_SW_MASK);
      updateDataItem(&gcmDataItems[PADDLE_DOWN_SW_IDX], msg.data[GCM_SWITCH_BYTE] & PADDLE_DOWN_GCM_SW_MASK);
      updateDataItem(&gcmDataItems[NEUTRAL_SW_IDX], msg.data[GCM_SWITCH_BYTE] & NEUTRAL_GCM_SW_MASK);
      updateDataItem(&gcmDataItems[QUEUE_UP_IDX], (uint8_t) (msg.data[QUEUE_UP_BYTE]) * QUEUE_UP_SCL);
      updateDataItem(&gcmDataItems[QUEUE_DN_IDX], (uint8_t) (msg.data[QUEUE_DN_BYTE]) * QUEUE_DN_SCL);
      updateDataItem(&gcmDataItems[QUEUE_NT_IDX], (uint8_t) (msg.data[QUEUE_NT_BYTE]) * QUEUE_NT_SCL);
      break;

      /*PDM*/
    case PDM_ID:
      updateDataItem(&pdmDataItems[UPTIME_IDX], (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL);
      updateDataItem(&pdmDataItems[PCB_TEMP_IDX], (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL);
      updateDataItem(&pdmDataItems[IC_TEMP_IDX], (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL);
      break;
    case PDM_ID + 1:

      // Enablity
      updateDataItem(&pdmDataItems[STR_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & STR_ENBL_MASK);
      updateDataItem(&pdmDataItems[BVBAT_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & BVBAT_ENBL_MASK);
      updateDataItem(&pdmDataItems[AUX_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & AUX_ENBL_MASK);
      updateDataItem(&pdmDataItems[ECU_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & ECU_ENBL_MASK);
      updateDataItem(&pdmDataItems[WTR_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & WTR_ENBL_MASK);
      updateDataItem(&pdmDataItems[FAN_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & FAN_ENBL_MASK);
      updateDataItem(&pdmDataItems[PDLD_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & PDLD_ENBL_MASK);
      updateDataItem(&pdmDataItems[PDLU_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & PDLU_ENBL_MASK);
      updateDataItem(&pdmDataItems[ABS_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & ABS_ENBL_MASK);
      updateDataItem(&pdmDataItems[INJ_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & INJ_ENBL_MASK);
      updateDataItem(&pdmDataItems[IGN_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & IGN_ENBL_MASK);
      updateDataItem(&pdmDataItems[FUEL_ENABLITY_IDX], lsbArray[LOAD_ENABLITY_BYTE/2] & FUEL_ENBL_MASK);

      // Peak Mode
      updateDataItem(&pdmDataItems[STR_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & STR_PEAKM_MASK);
      updateDataItem(&pdmDataItems[BVBAT_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & BVBAT_PEAKM_MASK);
      updateDataItem(&pdmDataItems[AUX_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & AUX_PEAKM_MASK);
      updateDataItem(&pdmDataItems[ECU_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & ECU_PEAKM_MASK);
      updateDataItem(&pdmDataItems[WTR_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & WTR_PEAKM_MASK);
      updateDataItem(&pdmDataItems[FAN_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & FAN_PEAKM_MASK);
      updateDataItem(&pdmDataItems[PDLD_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & PDLD_PEAKM_MASK);
      updateDataItem(&pdmDataItems[PDLU_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & PDLU_PEAKM_MASK);
      updateDataItem(&pdmDataItems[ABS_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & ABS_PEAKM_MASK);
      updateDataItem(&pdmDataItems[INJ_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & INJ_PEAKM_MASK);
      updateDataItem(&pdmDataItems[IGN_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & IGN_PEAKM_MASK);
      updateDataItem(&pdmDataItems[FUEL_PEAK_MODE_IDX], lsbArray[LOAD_PEAK_BYTE/2] & FUEL_PEAKM_MASK);

      // Total current
      updateDataItem(&pdmDataItems[TOTAL_CURRENT_IDX], (uint16_t) (lsbArray[TOTAL_CURRENT_BYTE/2]) * TOTAL_CURRENT_SCL);

      // Switch bitmap
      updateDataItem(&pdmDataItems[AUX2_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & AUX2_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[AUX1_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & AUX1_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[ABS_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & ABS_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[KILL_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & KILL_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[ACT_DN_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & ACT_DN_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[ACT_UP_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & ACT_UP_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[ON_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & ON_PDM_SW_MASK);
      updateDataItem(&pdmDataItems[STR_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & STR_PDM_SW_MASK);

      // Flags
      updateDataItem(&pdmDataItems[KILL_ENGINE_FLAG_IDX], msg.data[PDM_FLAG_BYTE] & KILL_ENGINE_PDM_FLAG_MASK);
      updateDataItem(&pdmDataItems[KILL_CAR_FLAG_IDX], msg.data[PDM_FLAG_BYTE] & KILL_CAR_PDM_FLAG_MASK);
      updateDataItem(&pdmDataItems[OVER_TEMP_FLAG_IDX], msg.data[PDM_FLAG_BYTE] & OVER_TEMP_PDM_FLAG_MASK);
      updateDataItem(&pdmDataItems[FUEL_PRIME_FLAG_IDX], msg.data[PDM_FLAG_BYTE] & FUEL_PRIME_PDM_FLAG_MASK);
      break;
    case PDM_ID + 2:
      updateDataItem(&pdmDataItems[VBAT_RAIL_IDX], (uint16_t) (lsbArray[VBAT_RAIL_BYTE/2]) * VBAT_RAIL_SCL);
      updateDataItem(&pdmDataItems[V12_RAIL_IDX], (uint16_t) (lsbArray[V12_RAIL_BYTE/2]) * V12_RAIL_SCL);
      updateDataItem(&pdmDataItems[V5_RAIL_IDX], (uint16_t) (lsbArray[V5_RAIL_BYTE/2]) * V5_RAIL_SCL);
      updateDataItem(&pdmDataItems[V3V3_RAIL_IDX], (uint16_t) (lsbArray[V3V3_RAIL_BYTE/2]) * V3V3_RAIL_SCL);
      break;
    case PDM_ID + 3:
      updateDataItem(&pdmDataItems[FUEL_DRAW_IDX], (uint16_t) (lsbArray[FUEL_DRAW_BYTE/2]) * FUEL_DRAW_SCL);
      updateDataItem(&pdmDataItems[IGN_DRAW_IDX], (uint16_t) (lsbArray[IGN_DRAW_BYTE/2]) * IGN_DRAW_SCL);
      updateDataItem(&pdmDataItems[INJ_DRAW_IDX], (uint16_t) (lsbArray[INJ_DRAW_BYTE/2]) * INJ_DRAW_SCL);
      updateDataItem(&pdmDataItems[ABS_DRAW_IDX], (uint16_t) (lsbArray[ABS_DRAW_BYTE/2]) * ABS_DRAW_SCL);
      break;
    case PDM_ID + 4:
      updateDataItem(&pdmDataItems[PDLU_DRAW_IDX], (uint16_t) (lsbArray[PDLU_DRAW_BYTE/2]) * PDLU_DRAW_SCL);
      updateDataItem(&pdmDataItems[PDLD_DRAW_IDX], (uint16_t) (lsbArray[PDLD_DRAW_BYTE/2]) * PDLD_DRAW_SCL);
      updateDataItem(&pdmDataItems[FAN_DRAW_IDX], (uint16_t) (lsbArray[FAN_DRAW_BYTE/2]) * FAN_DRAW_SCL);
      updateDataItem(&pdmDataItems[WTR_DRAW_IDX], (uint16_t) (lsbArray[WTR_DRAW_BYTE/2]) * WTR_DRAW_SCL);
      break;
    case PDM_ID + 5:
      updateDataItem(&pdmDataItems[ECU_DRAW_IDX], (uint16_t) (lsbArray[ECU_DRAW_BYTE/2]) * ECU_DRAW_SCL);
      updateDataItem(&pdmDataItems[AUX_DRAW_IDX], (uint16_t) (lsbArray[AUX_DRAW_BYTE/2]) * AUX_DRAW_SCL);
      updateDataItem(&pdmDataItems[BVBAT_DRAW_IDX], (uint16_t) (lsbArray[BVBAT_DRAW_BYTE/2]) * BVBAT_DRAW_SCL);
      updateDataItem(&pdmDataItems[STR_DRAW_IDX], (uint16_t) (lsbArray[STR_DRAW_BYTE/2]) * STR_DRAW_SCL);
      break;
    case PDM_ID + 6:
      updateDataItem(&pdmDataItems[FUEL_CUT_IDX], (uint16_t) (lsbArray[FUEL_CUT_BYTE/2]) * FUEL_CUT_SCL);
      updateDataItem(&pdmDataItems[IGN_CUT_IDX], (uint16_t) (lsbArray[IGN_CUT_BYTE/2]) * IGN_CUT_SCL);
      updateDataItem(&pdmDataItems[INJ_CUT_IDX], (uint16_t) (lsbArray[INJ_CUT_BYTE/2]) * INJ_CUT_SCL);
      updateDataItem(&pdmDataItems[ABS_CUT_IDX], (uint16_t) (lsbArray[ABS_CUT_BYTE/2]) * ABS_CUT_SCL);
      break;
    case PDM_ID + 7:
      updateDataItem(&pdmDataItems[PDLU_CUT_IDX], (uint16_t) (lsbArray[PDLU_CUT_BYTE/2]) * PDLU_CUT_SCL);
      updateDataItem(&pdmDataItems[PDLD_CUT_IDX], (uint16_t) (lsbArray[PDLD_CUT_BYTE/2]) * PDLD_CUT_SCL);
      updateDataItem(&pdmDataItems[FAN_CUT_IDX], (uint16_t) (lsbArray[FAN_CUT_BYTE/2]) * FAN_CUT_SCL);
      updateDataItem(&pdmDataItems[WTR_CUT_IDX], (uint16_t) (lsbArray[WTR_CUT_BYTE/2]) * WTR_CUT_SCL);
      break;
    case PDM_ID + 8:
      updateDataItem(&pdmDataItems[ECU_CUT_IDX], (uint16_t) (lsbArray[ECU_CUT_BYTE/2]) * ECU_CUT_SCL);
      updateDataItem(&pdmDataItems[AUX_CUT_IDX], (uint16_t) (lsbArray[AUX_CUT_BYTE/2]) * AUX_CUT_SCL);
      updateDataItem(&pdmDataItems[BVBAT_CUT_IDX], (uint16_t) (lsbArray[BVBAT_CUT_BYTE/2]) * BVBAT_CUT_SCL);
      break;
    case PDM_ID + 9:
      updateDataItem(&pdmDataItems[FUEL_CUT_P_IDX], (uint16_t) (lsbArray[FUEL_CUT_P_BYTE/2]) * FUEL_CUT_P_SCL);
      updateDataItem(&pdmDataItems[FAN_CUT_P_IDX], (uint16_t) (lsbArray[FAN_CUT_P_BYTE/2]) * FAN_CUT_P_SCL);
      updateDataItem(&pdmDataItems[WTR_CUT_P_IDX], (uint16_t) (lsbArray[WTR_CUT_P_BYTE/2]) * WTR_CUT_P_SCL);
      updateDataItem(&pdmDataItems[ECU_CUT_P_IDX], (uint16_t) (lsbArray[ECU_CUT_P_BYTE/2]) * ECU_CUT_P_SCL);
      break;
    case PDM_ID + 10:
      updateDataItem(&pdmDataItems[FUEL_OC_COUNT_IDX], (uint16_t) (msg.data[FUEL_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[IGN_OC_COUNT_IDX], (uint16_t) (msg.data[IGN_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[INJ_OC_COUNT_IDX], (uint16_t) (msg.data[INJ_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[ABS_OC_COUNT_IDX], (uint16_t) (msg.data[ABS_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[PDLU_OC_COUNT_IDX], (uint16_t) (msg.data[PDLU_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[PDLD_OC_COUNT_IDX], (uint16_t) (msg.data[PDLD_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[FAN_OC_COUNT_IDX], (uint16_t) (msg.data[FAN_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[WTR_OC_COUNT_IDX], (uint16_t) (msg.data[WTR_OC_COUNT_BYTE]));
      break;
    case PDM_ID + 11:
      updateDataItem(&pdmDataItems[ECU_OC_COUNT_IDX], (uint16_t) (msg.data[ECU_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[AUX_OC_COUNT_IDX], (uint16_t) (msg.data[AUX_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[BVBAT_OC_COUNT_IDX], (uint16_t) (msg.data[BVBAT_OC_COUNT_BYTE]));
      updateDataItem(&pdmDataItems[STR_OC_COUNT_IDX], (uint16_t) (msg.data[STR_OC_COUNT_BYTE]));
      break;

      /*Tire Temps*/
    case TIRE_TEMP_FL_ID:
      updateDataItem(&tireTempDataItems[FL0_IDX], (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FL1_IDX], (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FL2_IDX], (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FL3_IDX], (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FL_IDX], (tireTempDataItems[FL0_IDX].value + tireTempDataItems[FL1_IDX].value + tireTempDataItems[FL2_IDX].value + tireTempDataItems[FL3_IDX].value)/4.0);
      break;
    case TIRE_TEMP_FR_ID:
      updateDataItem(&tireTempDataItems[FR0_IDX], (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FR1_IDX], (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FR2_IDX], (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FR3_IDX], (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[FR_IDX], (tireTempDataItems[FR0_IDX].value + tireTempDataItems[FR1_IDX].value + tireTempDataItems[FR2_IDX].value + tireTempDataItems[FR3_IDX].value)/4.0);
      break;
    case TIRE_TEMP_RL_ID:
      updateDataItem(&tireTempDataItems[RL0_IDX], (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RL1_IDX], (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RL2_IDX], (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RL3_IDX], (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RL_IDX], (tireTempDataItems[RL0_IDX].value + tireTempDataItems[RL1_IDX].value + tireTempDataItems[RL2_IDX].value + tireTempDataItems[RL3_IDX].value)/4.0);
      break;
    case TIRE_TEMP_RR_ID:
      updateDataItem(&tireTempDataItems[RR0_IDX], (double) (lsbArray[TIRE_TEMP_1_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RR1_IDX], (double) (lsbArray[TIRE_TEMP_2_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RR2_IDX], (double) (lsbArray[TIRE_TEMP_3_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RR3_IDX], (double) (lsbArray[TIRE_TEMP_4_BYTE/2]*TIRE_TEMP_SCL));
      updateDataItem(&tireTempDataItems[RR_IDX], (tireTempDataItems[RR0_IDX].value + tireTempDataItems[RR1_IDX].value + tireTempDataItems[RR2_IDX].value + tireTempDataItems[RR3_IDX].value)/4.0);
      break;

      // SPM
    case SPM_ID:
      updateDataItem(&spmDataItems[UPTIME_IDX], (uint16_t) (lsbArray[UPTIME_BYTE/2]) * UPTIME_SCL);
      updateDataItem(&spmDataItems[PCB_TEMP_IDX], (int16_t) (lsbArray[PCB_TEMP_BYTE/2]) * PCB_TEMP_SCL);
      updateDataItem(&spmDataItems[IC_TEMP_IDX], (int16_t) (lsbArray[IC_TEMP_BYTE/2]) * IC_TEMP_SCL);
      break;
    case SPM_ID + 1:
      updateDataItem(&spmDataItems[ANALOG_CHAN_0_IDX], (double) (lsbArray[ANALOG_CHAN_0_BYTE/2] * ANALOG_CHAN_0_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_1_IDX], (double) (lsbArray[ANALOG_CHAN_1_BYTE/2] * ANALOG_CHAN_1_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_2_IDX], (double) (lsbArray[ANALOG_CHAN_2_BYTE/2] * ANALOG_CHAN_2_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_3_IDX], (double) (lsbArray[ANALOG_CHAN_3_BYTE/2] * ANALOG_CHAN_3_SCL));
      break;
    case SPM_ID + 2:
      updateDataItem(&spmDataItems[ANALOG_CHAN_4_IDX], (double) (lsbArray[ANALOG_CHAN_4_BYTE/2] * ANALOG_CHAN_4_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_5_IDX], (double) (lsbArray[ANALOG_CHAN_5_BYTE/2] * ANALOG_CHAN_5_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_6_IDX], (double) (lsbArray[ANALOG_CHAN_6_BYTE/2] * ANALOG_CHAN_6_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_7_IDX], (double) (lsbArray[ANALOG_CHAN_7_BYTE/2] * ANALOG_CHAN_7_SCL));
      break;
    case SPM_ID + 3:
      updateDataItem(&spmDataItems[ANALOG_CHAN_8_IDX], (double) (lsbArray[ANALOG_CHAN_8_BYTE/2] * ANALOG_CHAN_8_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_9_IDX], (double) (lsbArray[ANALOG_CHAN_9_BYTE/2] * ANALOG_CHAN_9_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_10_IDX], (double) (lsbArray[ANALOG_CHAN_10_BYTE/2] * ANALOG_CHAN_10_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_11_IDX], (double) (lsbArray[ANALOG_CHAN_11_BYTE/2] * ANALOG_CHAN_11_SCL));
      break;
    case SPM_ID + 4:
      updateDataItem(&spmDataItems[ANALOG_CHAN_12_IDX], (double) (lsbArray[ANALOG_CHAN_12_BYTE/2] * ANALOG_CHAN_12_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_13_IDX], (double) (lsbArray[ANALOG_CHAN_13_BYTE/2] * ANALOG_CHAN_13_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_14_IDX], (double) (lsbArray[ANALOG_CHAN_14_BYTE/2] * ANALOG_CHAN_14_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_15_IDX], (double) (lsbArray[ANALOG_CHAN_15_BYTE/2] * ANALOG_CHAN_15_SCL));
      break;
    case SPM_ID + 5:
      updateDataItem(&spmDataItems[ANALOG_CHAN_16_IDX], (double) (lsbArray[ANALOG_CHAN_16_BYTE/2] * ANALOG_CHAN_16_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_17_IDX], (double) (lsbArray[ANALOG_CHAN_17_BYTE/2] * ANALOG_CHAN_17_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_18_IDX], (double) (lsbArray[ANALOG_CHAN_18_BYTE/2] * ANALOG_CHAN_18_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_19_IDX], (double) (lsbArray[ANALOG_CHAN_19_BYTE/2] * ANALOG_CHAN_19_SCL));
      break;
    case SPM_ID + 6:
      updateDataItem(&spmDataItems[ANALOG_CHAN_20_IDX], (double) (lsbArray[ANALOG_CHAN_20_BYTE/2] * ANALOG_CHAN_20_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_21_IDX], (double) (lsbArray[ANALOG_CHAN_21_BYTE/2] * ANALOG_CHAN_21_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_22_IDX], (double) (lsbArray[ANALOG_CHAN_22_BYTE/2] * ANALOG_CHAN_22_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_23_IDX], (double) (lsbArray[ANALOG_CHAN_23_BYTE/2] * ANALOG_CHAN_23_SCL));
      break;
    case SPM_ID + 7:
      updateDataItem(&spmDataItems[ANALOG_CHAN_24_IDX], (double) (lsbArray[ANALOG_CHAN_24_BYTE/2] * ANALOG_CHAN_24_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_25_IDX], (double) (lsbArray[ANALOG_CHAN_25_BYTE/2] * ANALOG_CHAN_25_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_26_IDX], (double) (lsbArray[ANALOG_CHAN_26_BYTE/2] * ANALOG_CHAN_26_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_27_IDX], (double) (lsbArray[ANALOG_CHAN_27_BYTE/2] * ANALOG_CHAN_27_SCL));
      break;
    case SPM_ID + 8:
      updateDataItem(&spmDataItems[ANALOG_CHAN_28_IDX], (double) (lsbArray[ANALOG_CHAN_28_BYTE/2] * ANALOG_CHAN_28_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_29_IDX], (double) (lsbArray[ANALOG_CHAN_29_BYTE/2] * ANALOG_CHAN_29_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_30_IDX], (double) (lsbArray[ANALOG_CHAN_30_BYTE/2] * ANALOG_CHAN_30_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_31_IDX], (double) (lsbArray[ANALOG_CHAN_31_BYTE/2] * ANALOG_CHAN_31_SCL));
      break;
    case SPM_ID + 9:
      updateDataItem(&spmDataItems[ANALOG_CHAN_32_IDX], (double) (lsbArray[ANALOG_CHAN_32_BYTE/2] * ANALOG_CHAN_32_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_33_IDX], (double) (lsbArray[ANALOG_CHAN_33_BYTE/2] * ANALOG_CHAN_33_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_34_IDX], (double) (lsbArray[ANALOG_CHAN_34_BYTE/2] * ANALOG_CHAN_34_SCL));
      updateDataItem(&spmDataItems[ANALOG_CHAN_35_IDX], (double) (lsbArray[ANALOG_CHAN_35_BYTE/2] * ANALOG_CHAN_35_SCL));
      break;
    case SPM_ID + 10:
      updateDataItem(&spmDataItems[TCOUPLE_0_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_0_BYTE/2]) * TCOUPLE_SCL));
      updateDataItem(&spmDataItems[TCOUPLE_1_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_1_BYTE/2]) * TCOUPLE_SCL));
      updateDataItem(&spmDataItems[TCOUPLE_2_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_2_BYTE/2]) * TCOUPLE_SCL));
      updateDataItem(&spmDataItems[TCOUPLE_3_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_3_BYTE/2]) * TCOUPLE_SCL));
      break;
    case SPM_ID + 11:
      updateDataItem(&spmDataItems[TCOUPLE_4_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_4_BYTE/2]) * TCOUPLE_SCL));
      updateDataItem(&spmDataItems[TCOUPLE_5_IDX], (double) ((int16_t) (lsbArray[TCOUPLE_5_BYTE/2]) * TCOUPLE_SCL));
      updateDataItem(&spmDataItems[AVG_JUNCT_TEMP_IDX], (double) ((int16_t) (lsbArray[AVG_JUNCT_TEMP_BYTE/2]) * AVG_JUNCT_TEMP_SCL));
      updateDataItem(&spmDataItems[TCOUPLE_1_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_0_FAULT_MASK);
      updateDataItem(&spmDataItems[TCOUPLE_1_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_1_FAULT_MASK);
      updateDataItem(&spmDataItems[TCOUPLE_2_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_2_FAULT_MASK);
      updateDataItem(&spmDataItems[TCOUPLE_3_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_3_FAULT_MASK);
      updateDataItem(&spmDataItems[TCOUPLE_4_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_4_FAULT_MASK);
      updateDataItem(&spmDataItems[TCOUPLE_5_FAULT_IDX], lsbArray[TCOUPLE_FAULT_BYTE] & TCOUPLE_5_FAULT_MASK);
      break;
    case SPM_ID + 12:
      updateDataItem(&spmDataItems[DIGITAL_INPUT_0_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_0_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_1_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_1_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_2_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_2_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_3_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_3_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_4_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_4_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_5_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_5_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_6_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_6_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_7_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_7_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_8_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_8_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_9_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_9_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_10_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_10_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_11_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_11_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_12_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_12_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_13_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_13_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_14_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_14_MASK);
      updateDataItem(&spmDataItems[DIGITAL_INPUT_15_IDX], lsbArray[DIGITAL_INPUT_BYTE] & DIGITAL_INPUT_15_MASK);
      updateDataItem(&spmDataItems[FREQ_COUNT_0_IDX], (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL));
      updateDataItem(&spmDataItems[FREQ_COUNT_1_IDX], (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL));
      updateDataItem(&spmDataItems[FREQ_COUNT_2_IDX], (double) (lsbArray[FREQ_COUNT_0_BYTE] * FREQ_COUNT_0_SCL));
      break;
    case SPM_ID + 13:
      updateDataItem(&spmDataItems[PGA_0_SETTINGS_IDX], (lsbArray[PGA_SETTINGS_BYTE] & PGA_0_SETTINGS_MASK) >> PGA_0_SETTINGS_SHF);
      updateDataItem(&spmDataItems[PGA_1_SETTINGS_IDX], (lsbArray[PGA_SETTINGS_BYTE] & PGA_1_SETTINGS_MASK) >> PGA_1_SETTINGS_SHF);
      updateDataItem(&spmDataItems[PGA_2_SETTINGS_IDX], (lsbArray[PGA_SETTINGS_BYTE] & PGA_2_SETTINGS_MASK) >> PGA_2_SETTINGS_SHF);
      updateDataItem(&spmDataItems[PGA_3_SETTINGS_IDX], (lsbArray[PGA_SETTINGS_BYTE] & PGA_3_SETTINGS_MASK) >> PGA_3_SETTINGS_SHF);
      updateDataItem(&spmDataItems[FREQ_0_SETTINGS_IDX], (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_0_SETTINGS_MASK) >> FREQ_0_SETTINGS_SHF);
      updateDataItem(&spmDataItems[FREQ_1_SETTINGS_IDX], (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_1_SETTINGS_MASK) >> FREQ_1_SETTINGS_SHF);
      updateDataItem(&spmDataItems[FREQ_2_SETTINGS_IDX], (lsbArray[FREQ_SETTINGS_BYTE] & FREQ_2_SETTINGS_MASK) >> FREQ_2_SETTINGS_SHF);
      break;

      // IMU
    case IMU_FIRST_ID:
      updateDataItem(&imuDataItems[YAW_RATE_IDX],  (double) ((lsbArray[YAW_RATE_BYTE/2]  * YAW_RATE_SCL)  + YAW_RATE_OFFSET));
      updateDataItem(&imuDataItems[LATERAL_G_IDX], (double) ((lsbArray[LATERAL_G_BYTE/2] * LATERAL_G_SCL) + LATERAL_G_OFFSET));
      break;
    case IMU_SECOND_ID:
      updateDataItem(&imuDataItems[YAW_ACCEL_IDX],      (double) ((lsbArray[YAW_ACCEL_BYTE/2]      * YAW_ACCEL_SCL)  + YAW_ACCEL_OFFSET));
      updateDataItem(&imuDataItems[LONGITUDINAL_G_IDX], (double) ((lsbArray[LONGITUDINAL_G_BYTE/2] * LONGITUDINAL_G_SCL) + LONGITUDINAL_G_OFFSET));
      break;
  }
}

void updateDataItem(volatile dataItem * data, double value) {
  data->value = value;
  data->refreshTime = millis;
}

//TODO: Use more of the MASK/BITPOS definitions from CAN.h
void CANswitchStates(void){
  CAN_data switchData = {0};

  uint8_t bitMask = 0;
  bitMask |= (((uint8_t) wheelDataItems[MOM_RDO_IDX].value) & 0x1) << (RADIO_BTN_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[MOM_ACK_IDX].value) & 0x1) << (ACK_BTN_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[MOM_AUX_IDX].value) & 0x1) << (AUX_BTN_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[SW_FUEL_IDX].value) & 0x1) << (FUEL_OVR_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[SW_FAN_IDX].value) & 0x1) << (FAN_OVR_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[SW_WTR_IDX].value) & 0x1) << (WTR_OVR_BITPOS - 1);
  bitMask |= (((uint8_t) wheelDataItems[SW_ND_IDX].value) & 0x1) << (ND_SW_BITPOS - 1);

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
  rotaries.halfword0 = ADL_IDX_1_3;
  rotaries.halfword1 = (uint16_t) wheelDataItems[ROTARY_0_IDX].value;
  rotaries.halfword2 = (uint16_t) wheelDataItems[ROTARY_1_IDX].value;
  rotaries.halfword3 = (uint16_t) wheelDataItems[ROTARY_2_IDX].value;
  CAN_send_message(ADL_ID, 8, rotaries);
}

void CANdiag(void){
  CAN_data data = {0};
  data.halfword0 = (uint16_t) (millis / 1000);
  data.halfword1 = pcb_temp;
  data.halfword2 = junc_temp;
  CAN_send_message(WHEEL_ID + 0, 6, data);
}

double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl){
  return ((double) ((msg->data[byte] << 8) | msg->data[byte + 1])) * scl;
}

void updateSwVals(void){
  // Update Switches
  wheelDataItems[SW_FAN_IDX].value = !SW0_PORT;
  wheelDataItems[SW_ND_IDX].value = !SW1_PORT;
  wheelDataItems[SW_WTR_IDX].value = 0;//!SW2_PORT;
  wheelDataItems[SW_FUEL_IDX].value = 0;//!SW3_PORT;
  // Update Momentaries
  wheelDataItems[MOM_AUX_IDX].value = !MOM0_PORT;
  wheelDataItems[MOM_ACK_IDX].value = !MOM1_PORT;
  wheelDataItems[MOM_RDO_IDX].value = 0;//!MOM2_PORT;
  wheelDataItems[MOM_NEU_IDX].value = 0;//!MOM3_PORT;
  // Update rotaries
  wheelDataItems[ROTARY_0_IDX].value = getRotaryPosition(read_adc_chn(ROT0_CHN));
  wheelDataItems[ROTARY_1_IDX].value = getRotaryPosition(read_adc_chn(ROT1_CHN));
  wheelDataItems[ROTARY_2_IDX].value = getRotaryPosition(read_adc_chn(ROT2_CHN));
  wheelDataItems[TROTARY_0_IDX].value = getRotaryPosition(read_adc_chn(TROT0_CHN));
  wheelDataItems[TROTARY_1_IDX].value = getRotaryPosition(read_adc_chn(TROT1_CHN));
}

uint8_t getRotaryPosition(uint32_t adcValue){
  return 9 - (uint8_t) (adcValue / 409.6);
}

void checkChangeScreen(void) {
  uint8_t rotVal = wheelDataItems[TROTARY_1_IDX].value;
  uint8_t screenIdx;

  if(nightModeState != wheelDataItems[SW_ND_IDX].value) {
    nightModeState = wheelDataItems[SW_ND_IDX].value;
    nightMode(nightModeState);
  }
  switch (rotVal) {
    case 0:
      screenIdx = RACE_SCREEN;
      break;
    case 1:
      screenIdx = PDM_DRAW_SCREEN;
      break;
    case 2:
      screenIdx = PDM_GRID_SCREEN;
      break;
    case 3:
      screenIdx = LAMBDA_SCREEN;
      break;
    case 4:
      screenIdx = THROTTLE_SCREEN;
      break;
    case 5:
      screenIdx = IMU_SCREEN;
      break;
    default:
      screenIdx = RACE_SCREEN;
      break;
  }

  if(screenIdx != screenNumber) {
    changeScreen(screenIdx);
    screenNumber = screenIdx;
  }
}

/**
 * void sample_temp(void)
 *
 * Samples the PCB temp sensor and internal die temp sensor, then updates
 * variables if the interval has passed.
 */
void sample_temp(void) {
  if(millis - tempSampMillis >= TEMP_SAMP_INTV) {

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

    tempSampMillis = millis;
  }
}
