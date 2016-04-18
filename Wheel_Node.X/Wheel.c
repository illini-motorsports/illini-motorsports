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

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_spi();// Initialize SPI interface
	init_adc(initADCWheel);

  ADCCON3bits.GSWTRG = 1; // Initial ADC Conversion?
  STI();// Enable interrupts
  
	// Init Relevant Pins
	millis = 0;
	CANswStateMillis = CANswADLMillis = CANdiagMillis = 0;
  LCD_CS_TRIS = OUTPUT;
  LCD_CS_LAT = 1;
  LCD_RST_TRIS = OUTPUT;
	LCD_RST_LAT = 1;
	TRISEbits.TRISE8 = OUTPUT;
	LATEbits.LATE8 = 0;
	SW1_TRIS = INPUT;
	SW2_TRIS = INPUT;
	SW3_TRIS = INPUT;
	SW4_TRIS = INPUT;
	MOM1_TRIS = INPUT;
	MOM2_TRIS = INPUT;
	MOM3_TRIS = INPUT;
	MOM4_TRIS = INPUT;
  
	// Initialize RA8875
  reset();
  initialize();
  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX
	PWM1config(1, RA8875_PWM_CLK_DIV1024);// PWM output for backlight
	PWM1out(255);
	fillScreen(RA8875_WHITE);
	//drawChevron(150,15,130,200,RA8875_BLACK,RA8875_WHITE);
	
	// Initialize All the data streams
	initDataItems();
  init_can();
  initAllScreens();
	changeScreen(RACE_SCREEN);
	
  while(1){
		// Send CAN messages with the correct frequency
		if(CANswStateMillis >= CAN_SW_STATE_FREQ){
			CANswitchStates();
			CANswStateMillis = 0;
		}	
		if(CANswADLMillis >= CAN_SW_ADL_FREQ){
			CANswitchADL();
			CANswADLMillis = 0;
		}
		if(CANdiagMillis >= CAN_DIAG_FREQ){
			CANdiag();
			CANdiagMillis = 0;
		}
		// Refresh Screen
		delay(50);
		oilPress.value += 0.01;
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
	CANswStateMillis++;
	CANswADLMillis++;
	CANdiagMillis++;

	// Should I be doing this every millisecond?
	if (ADCCON2bits.EOSRDY) {
    ADCCON3bits.GSWTRG = 1; // Trigger an ADC conversion
  }

	if (!(millis%25)){
		updateSwVals();
		CANswitchStates();
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
  switch (msg.id) {
    case MOTEC_ID + 0:
			rpm.value = parseMsgMotec(&msg, ENG_RPM_BYTE, ENG_RPM_SCL);
			throtPos.value = parseMsgMotec(&msg, THROTTLE_POS_BYTE, THROTTLE_POS_SCL);
			lambda.value = parseMsgMotec(&msg, LAMBDA_BYTE, LAMBDA_SCL);
      batVoltage.value = parseMsgMotec(&msg, VOLT_ECU_BYTE, VOLT_ECU_SCL);
      break;
    case MOTEC_ID + 1:
      waterTemp.value = parseMsgMotec(&msg, ENG_TEMP_BYTE, ENG_TEMP_SCL);
      oilTemp.value = parseMsgMotec(&msg, OIL_TEMP_BYTE, OIL_TEMP_SCL);
			manifoldTemp.value = parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL);
			fuelTemp.value = parseMsgMotec(&msg, FUEL_TEMP_BYTE, FUEL_TEMP_SCL);
      break;
    case MOTEC_ID + 2:
			ambientPress.value = parseMsgMotec(&msg, AMBIENT_PRES_BYTE, AMBIENT_PRES_SCL);
      oilPress.value = parseMsgMotec(&msg, OIL_PRES_BYTE, OIL_PRES_SCL);
			manifoldPress.value = parseMsgMotec(&msg, MANIFOLD_TEMP_BYTE, MANIFOLD_TEMP_SCL);
			fuelPress.value = parseMsgMotec(&msg, FUEL_PRES_BYTE, FUEL_PRES_SCL);
      break;
		case MOTEC_ID + 3:
			wheelSpeedFL.value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
			wheelSpeedFR.value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
			wheelSpeedRL.value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
			wheelSpeedRR.value = parseMsgMotec(&msg, WHEELSPEED_FL_BYTE, WHEELSPEED_FL_SCL);
			break;
		case MOTEC_ID + 4:
			driveSpeed.value = parseMsgMotec(&msg, DRIVE_SPEED_BYTE, DRIVE_SPEED_SCL);
			groundSpeed.value = parseMsgMotec(&msg, GROUND_SPEED_BYTE, GROUND_SPEED_SCL);
			gpsSpeed.value = parseMsgMotec(&msg, GPS_SPEED_BYTE, GPS_SPEED_SCL);
			gpsAltitude.value = parseMsgMotec(&msg, GPS_ALT_BYTE, GPS_ALT_SCL);
			break;
		case MOTEC_ID + 5:
			gpsLat.value = (double) ((msg.data[GPS_LAT_BYTE] << 24) |
							(msg.data[GPS_LAT_BYTE+1] << 16) | (msg.data[GPS_LAT_BYTE+2] << 8) | 
							msg.data[GPS_LAT_BYTE+3]) * GPS_LAT_SCL;
			gpsLong.value = (double) ((msg.data[GPS_LONG_BYTE] << 24) | 
							(msg.data[GPS_LONG_BYTE+1] << 16) | (msg.data[GPS_LONG_BYTE+2] << 8) | 
							msg.data[GPS_LONG_BYTE+3]) * GPS_LONG_SCL;
			break;
		case MOTEC_ID + 6:
			gpsTime.value = (double) ((msg.data[GPS_TIME_BYTE] << 24) | 
							(msg.data[GPS_TIME_BYTE+1] << 16) | (msg.data[GPS_TIME_BYTE+2] << 8) | 
							msg.data[GPS_TIME_BYTE+3]) * GPS_TIME_SCL;
			runTime.value = parseMsgMotec(&msg, RUN_TIME_BYTE, RUN_TIME_SCL);
			fuelConsum.value = parseMsgMotec(&msg, FUEL_USED_BYTE, FUEL_USED_SCL);
			break;
		case MOTEC_ID + 7:
			fuelInjDuty.value = parseMsgMotec(&msg, FUEL_INJ_DUTY_BYTE, FUEL_INJ_DUTY_SCL);
			fuelTrim.value = parseMsgMotec(&msg, FUEL_TRIM_BYTE, FUEL_TRIM_SCL);
			break;
		case PADDLE_ID:
			paddleUptime.value = (double) ((uint16_t) msg.data[PADDLE_UPTIME_BYTE]) * 
							PADDLE_UPTIME_SCL;
			paddleTemp.value = (double) ((uint16_t) msg.data[PADDLE_TEMP_BYTE]) * 
							PADDLE_TEMP_SCL;
			neutQueue.value = msg.data[QUEUE_NT_BYTE] * QUEUE_NT_SCL;
			upQueue.value = msg.data[QUEUE_UP_BYTE] * QUEUE_UP_SCL;
			downQueue.value =  msg.data[QUEUE_DN_BYTE] * QUEUE_DN_SCL;
			break;
		case PADDLE_ID + 1:
			gearVoltage.value = (double) ((uint16_t) msg.data[GEAR_VOLT_BYTE]) * 
							GEAR_VOLT_SCL;
			gearPos.value = msg.data[GEAR_BYTE];
			break;
		case PDM_ID:
			pdmUptime.value = (uint16_t) (msg.data[PDM_UPTIME_BYTE]) * PDM_UPTIME_SCL;
			pdmTemp.value = (int16_t) (msg.data[PDM_PCB_TEMP_BYTE]) * PDM_PCB_TEMP_SCL;
			pdmICTemp.value = (int16_t) (msg.data[PDM_IC_TEMP_BYTE]) * PDM_IC_TEMP_SCL;
			pdmCurrentDraw.value = (uint16_t) (msg.data[TOTAL_CURRENT_BYTE]) * TOTAL_CURRENT_SCL;
			break;
		case PDM_ID + 1:
			// TODO: Create special cases for Bitmaps
			// Load Enability
			// Load Peak Mode
			// Switch Bitmap
			break;
		case PDM_ID + 2:
			pdmVBat.value = (uint16_t) (msg.data[VBAT_RAIL_BYTE]) * VBAT_RAIL_SCL;
			pdm12v.value = (uint16_t) (msg.data[V12_RAIL_BYTE]) * V12_RAIL_SCL;
			pdm5v5.value = (uint16_t) (msg.data[V5V5_RAIL_BYTE]) * V5V5_RAIL_SCL;
			pdm5v.value = (uint16_t) (msg.data[V5_RAIL_BYTE]) * V5_RAIL_SCL;
			break;
		case PDM_ID + 3:
			pdm3v3.value = (uint16_t) (msg.data[V3V3_RAIL_BYTE]) * V3V3_RAIL_SCL;
			break;
		case PDM_ID + 4:
			pdmIGNdraw.value = (uint16_t) (msg.data[IGN_DRAW_BYTE]) * IGN_DRAW_SCL;
			pdmINJdraw.value = (uint16_t) (msg.data[INJ_DRAW_BYTE]) * INJ_DRAW_SCL;
			pdmFUELdraw.value = (uint16_t) (msg.data[FUEL_DRAW_BYTE]) * FUEL_DRAW_SCL;
			pdmECUdraw.value = (uint16_t) (msg.data[ECU_DRAW_BYTE]) * ECU_DRAW_SCL;
			break;
		case PDM_ID + 5:
			pdmWTRdraw.value = (uint16_t) (msg.data[WTR_DRAW_BYTE]) * WTR_DRAW_SCL;
			pdmFANdraw.value = (uint16_t) (msg.data[FAN_DRAW_BYTE]) * FAN_DRAW_SCL;
			pdmAUXdraw.value = (uint16_t) (msg.data[AUX_DRAW_BYTE]) * AUX_DRAW_SCL;
			pdmPDLUdraw.value = (uint16_t) (msg.data[PDLU_DRAW_BYTE]) * PDLU_DRAW_SCL;
			break;
		case PDM_ID + 6:
			pdmPDLDdraw.value = (uint16_t) (msg.data[PDLD_DRAW_BYTE]) * PDLD_DRAW_SCL;
			pdm5v5draw.value = (uint16_t) (msg.data[B5V5_DRAW_BYTE]) * B5V5_DRAW_SCL;
			pdmBATdraw.value = (uint16_t) (msg.data[VBAT_DRAW_BYTE]) * VBAT_DRAW_BYTE;
			break;
		case PDM_ID + 7:
			pdmSTR0draw.value = (uint16_t) (msg.data[STR0_DRAW_BYTE]) * STR0_DRAW_SCL;
			pdmSTR1draw.value = (uint16_t) (msg.data[STR1_DRAW_BYTE]) * STR1_DRAW_SCL;
			pdmSTR2draw.value = (uint16_t) (msg.data[STR2_DRAW_BYTE]) * STR2_DRAW_SCL;
			pdmSTRdraw.value = (uint16_t) (msg.data[STR_DRAW_BYTE]) * STR_DRAW_SCL;
			break;
		case PDM_ID + 8:
			pdmIGNcut.value = (uint16_t) (msg.data[IGN_CUT_BYTE]) * IGN_CUT_SCL;
			pdmINJcut.value = (uint16_t) (msg.data[INJ_CUT_BYTE]) * INJ_CUT_SCL;
			pdmAUXcut.value = (uint16_t) (msg.data[AUX_CUT_BYTE]) * AUX_CUT_SCL;
			pdmPDLUcut.value = (uint16_t) (msg.data[PDLU_CUT_BYTE]) * PDLU_CUT_SCL;
			break;
		case PDM_ID + 9:
			pdmPDLDcut.value = (uint16_t) (msg.data[PDLD_CUT_BYTE]) * PDLD_CUT_SCL;
			pdm5v5cut.value = (uint16_t) (msg.data[B5V5_CUT_BYTE]) * B5V5_CUT_SCL;
			pdmBATcut.value = (uint16_t) (msg.data[BVBAT_CUT_BYTE]) * BVBAT_CUT_SCL;
			break;
		case PDM_ID + 10:
			pdmSTR0cut.value = (uint16_t) (msg.data[STR0_CUT_BYTE]) * STR0_CUT_SCL;
			pdmSTR1cut.value = (uint16_t) (msg.data[STR1_CUT_BYTE]) * STR1_CUT_SCL;
			pdmSTR2cut.value = (uint16_t) (msg.data[STR2_CUT_BYTE]) * STR2_CUT_SCL;
			break;
		case PDM_ID + 11:
			pdmFUELNcut.value = (uint16_t) (msg.data[FUEL_CUT_N_BYTE]) * FUEL_CUT_N_SCL;
			pdmWTRNcut.value = (uint16_t) (msg.data[WTR_CUT_N_BYTE]) * WTR_CUT_N_SCL;
			pdmFANNcut.value = (uint16_t) (msg.data[FAN_CUT_N_BYTE]) * FAN_CUT_N_SCL;
			pdmECUNcut.value = (uint16_t) (msg.data[ECU_CUT_N_BYTE]) * ECU_CUT_N_SCL;
			break;
		case PDM_ID + 12:
			pdmFUELPcut.value = (uint16_t) (msg.data[FUEL_CUT_P_BYTE]) * FUEL_CUT_P_SCL;
			pdmWTRPcut.value = (uint16_t) (msg.data[WTR_CUT_P_BYTE]) * WTR_CUT_P_SCL;
			pdmFANPcut.value = (uint16_t) (msg.data[FAN_CUT_P_BYTE]) * FAN_CUT_P_SCL;
			pdmECUPcut.value = (uint16_t) (msg.data[ECU_CUT_P_BYTE]) * ECU_CUT_P_SCL;
			break;
		case TIRE_TEMP_FL_ID:
			ttFL1.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE]) * TIRE_TEMP_SCL);
			ttFL2.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE]) * TIRE_TEMP_SCL);
			ttFL3.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE]) * TIRE_TEMP_SCL);
			ttFL4.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE]) * TIRE_TEMP_SCL);
			ttFL.value = (ttFL1.value + ttFL2.value + ttFL3.value + ttFL4.value)/4.0;
		case TIRE_TEMP_FR_ID:
			ttFR1.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE]) * TIRE_TEMP_SCL);
			ttFR2.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE]) * TIRE_TEMP_SCL);
			ttFR3.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE]) * TIRE_TEMP_SCL);
			ttFR4.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE]) * TIRE_TEMP_SCL);
			ttFR.value = (ttFR1.value + ttFR2.value + ttFR3.value + ttFR4.value)/4.0;
		case TIRE_TEMP_RL_ID:
			ttRL1.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE]) * TIRE_TEMP_SCL);
			ttRL2.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE]) * TIRE_TEMP_SCL);
			ttRL3.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE]) * TIRE_TEMP_SCL);
			ttRL4.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE]) * TIRE_TEMP_SCL);
			ttRL.value = (ttRL1.value + ttRL2.value + ttRL3.value + ttRL4.value)/4.0;
		case TIRE_TEMP_RR_ID:
			ttRR1.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE]) * TIRE_TEMP_SCL);
			ttRR2.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE]) * TIRE_TEMP_SCL);
			ttRR3.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE]) * TIRE_TEMP_SCL);
			ttRR4.value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE]) * TIRE_TEMP_SCL);
			ttRR.value = (ttRR1.value + ttRR2.value + ttRR3.value + ttRR4.value)/4.0;
  }
}

void CANswitchStates(void){
	CAN_data switchData = {0};
	switchData.byte0 = (swBitmask << 4) | momBitmask;
	switchData.byte1 = (rotary[0] << 4) | rotary[2];
	switchData.byte2 = rotary[3] << 4;
	CAN_send_message(WHEEL_ID + 1,3,switchData);
}

void CANswitchADL(void){
	CAN_data rotaries = {0};
	rotaries.halfword0 = 0x0000;
	rotaries.halfword1 = rotary[0];
	rotaries.halfword2 = rotary[1];
	rotaries.halfword3 = rotary[2];
	CAN_send_message(ADL_ID, 8, rotaries);
}

void CANdiag(void){
	return;
}

double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl){
	return ((double) ((msg->data[byte] << 8) | msg->data[byte + 1])) * scl;
}

void initADCWheel(void){
	ROT1_TRIS = INPUT;
	ROT2_TRIS = INPUT;
	ROT3_TRIS = INPUT;
	TROT1_TRIS = INPUT;
	TROT2_TRIS = INPUT;
	ROT1_ANSEL = AN_INPUT;
	ROT2_ANSEL = AN_INPUT;
	ROT3_ANSEL = AN_INPUT;
	TROT1_ANSEL = AN_INPUT;
	TROT2_ANSEL = AN_INPUT;
	ROT1_CSS = 1;
	ROT2_CSS = 1;
	ROT3_CSS = 1;
	TROT1_CSS = 1;
	TROT2_CSS = 1;
}

void updateSwVals(void){
	// Update Switches
	swBitmask = 0;
	if(SW1_PORT){swBitmask |= SW1_BIT;}
	if(SW2_PORT){swBitmask |= SW2_BIT;}
	if(SW3_PORT){swBitmask |= SW3_BIT;}
	if(SW4_PORT){swBitmask |= SW4_BIT;}
	// Update Momentaries
	momBitmask = 0;
	if(MOM1_PORT){momBitmask |= MOM1_BIT;}
	if(MOM2_PORT){momBitmask |= MOM2_BIT;}
	if(MOM3_PORT){momBitmask |= MOM3_BIT;}
	if(MOM4_PORT){momBitmask |= MOM4_BIT;}
	
	// Update rotaries
	rotary[0] = getRotaryPosition(read_adc_chn(ROT1_CHN));
	rotary[1] = getRotaryPosition(read_adc_chn(ROT2_CHN));
	rotary[2] = getRotaryPosition(read_adc_chn(ROT3_CHN));
	tRotary[0] = getRotaryPosition(read_adc_chn(TROT1_CHN));
	tRotary[1] = getRotaryPosition(read_adc_chn(TROT2_CHN));
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