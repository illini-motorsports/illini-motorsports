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
volatile uint32_t millis = 0;

// Declare CAN Vars;
volatile double eng_rpm = 0, bat_volt_ecu = 0, eng_temp = 0, oil_temp = 0, oil_pres = 0;
volatile uint8_t gear_pos = 0;

void main(void) {
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator();// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_spi();// Initialize SPI interface
  STI();// Enable interrupts
  
  LCD_CS_TRIS = OUTPUT;
  LCD_CS_LAT = 1;
  LCD_RST_TRIS = OUTPUT;
  TRISBbits.TRISB6 = OUTPUT;
  TRISBbits.TRISB7 = OUTPUT;
  LATBbits.LATB6 = 1;
  LATBbits.LATB7 = 0;

	// Initialize RA8875
  reset();
  initialize();
  displayOn(1);
  GPIOX(1);// Enable TFT - display enable tied to GPIOX
  PWM1config(1, RA8875_PWM_CLK_DIV1024);// PWM output for backlight
  PWM1out(255);

	/*
	while(1){
		fillScreen(RA8875_RED);
		delay(500);
	}
	*/

	// Initialize All the data streams
	// This needs to be done before CAN starts
	initDataItems();
  init_can();

  initAllScreens();
	changeScreen(RACE_SCREEN);
	
  while(1){
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
  }
}

double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl){
	return ((double) ((msg->data[byte] << 8) | msg->data[byte + 1])) * scl;
}