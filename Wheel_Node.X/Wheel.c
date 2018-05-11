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
      break;
    case MOTEC_ID + 1:
      updateDataItem(&motecDataItems[ENG_TEMP_IDX], parseMsgMotec(&msg, ENG_TEMP_BYTE, ENG_TEMP_SCL));
      updateDataItem(&motecDataItems[OIL_TEMP_IDX], parseMsgMotec(&msg, OIL_TEMP_BYTE, OIL_TEMP_SCL));
      break;
    case MOTEC_ID + 2:
      updateDataItem(&motecDataItems[OIL_PRES_IDX], parseMsgMotec(&msg, OIL_PRES_BYTE, OIL_PRES_SCL));
      break;

      /*GCM*/
    case GCM_ID + 1:
      updateDataItem(&gcmDataItems[GEAR_IDX], (uint8_t) (msg.data[GEAR_BYTE]) * GEAR_SCL);
      updateDataItem(&gcmDataItems[MODE_IDX], msg.data[MODE_BYTE]);
      break;

      /*PDM*/
    case PDM_ID + 1:
      updateDataItem(&pdmDataItems[KILL_SWITCH_IDX], msg.data[PDM_SWITCH_BYTE] & KILL_PDM_SW_MASK);
      break;
    case PDM_ID + 2:
      updateDataItem(&pdmDataItems[VBAT_RAIL_IDX], (uint16_t) (lsbArray[VBAT_RAIL_BYTE/2]) * VBAT_RAIL_SCL);
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
