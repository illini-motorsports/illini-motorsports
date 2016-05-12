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
	init_oscillator();// Initialize oscillator configuration bits
	init_timer2();// Initialize timer2 (millis)
	init_spi();// Initialize SPI interface
	init_adc(initADCWheel);

	ADCCON3bits.GSWTRG = 1; // Initial ADC Conversion?
	STI();// Enable interrupts

	// Init Relevant Pins
	millis = 0;
	CANswStateMillis = CANswADLMillis = CANdiagMillis = 0;
	auxState = 0;
	auxNumber = 0;
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
	//fillScreen(RA8875_WHITE);
	//drawChevron(150,15,130,200,RA8875_BLACK,RA8875_WHITE);

	// Initialize All the data streams
	initDataItems();
	init_can();
	updateSwVals();
	initAllScreens();
	changeScreen(PDM_CUT_SCREEN);

	while(1){
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
		// Refresh Screen
		if(tRotary[1].value == 1 && screenNumber != PDM_DRAW_SCREEN){
			changeScreen(PDM_DRAW_SCREEN);
		}
		else if(tRotary[1].value == 2 && screenNumber != PDM_CUT_SCREEN){
			changeScreen(PDM_CUT_SCREEN);
		}
		else if(tRotary[1].value != 2 && tRotary[1].value != 1 && screenNumber != RACE_SCREEN){
			changeScreen(RACE_SCREEN);
		}

		// Change AUX State
		if(auxState != momentaries[0].value){
			if(momentaries[0].value == 1){
				changeAUXType((auxNumber+1)%3);
			}
			auxState = momentaries[0].value;
		}

		/*
		if(tRotary[0].value > 2){
			changeAUXType(0);
		} else{
			changeAUXType(tRotary[0].value);
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

		/*Motec Paddle Shifting*/
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
			break;
		case MOTEC_ID + 6:
			gpsTime.value = (double) ((msg.data[GPS_TIME_BYTE] << 24)|(msg.data[GPS_TIME_BYTE+1] << 16)|(msg.data[GPS_TIME_BYTE+2] << 8)|msg.data[GPS_TIME_BYTE+3]) * GPS_TIME_SCL;
			runTime.value = parseMsgMotec(&msg, RUN_TIME_BYTE, RUN_TIME_SCL);
			fuelConsum.value = parseMsgMotec(&msg, FUEL_USED_BYTE, FUEL_USED_SCL);
			break;
		case MOTEC_ID + 7:
			fuelInjDuty.value = parseMsgMotec(&msg, FUEL_INJ_DUTY_BYTE, FUEL_INJ_DUTY_SCL);
			fuelTrim.value = parseMsgMotec(&msg, FUEL_TRIM_BYTE, FUEL_TRIM_SCL);
			break;

		/*Paddle Shifting ID's*/
		case PADDLE_ID:
			paddleUptime.value = (double) ((uint16_t) msg.data[PADDLE_UPTIME_BYTE])*PADDLE_UPTIME_SCL;
			paddleTemp.value = (double) ((uint16_t) msg.data[PADDLE_TEMP_BYTE])*PADDLE_TEMP_SCL;
			neutQueue.value = msg.data[QUEUE_NT_BYTE] * QUEUE_NT_SCL;
			upQueue.value = msg.data[QUEUE_UP_BYTE] * QUEUE_UP_SCL;
			downQueue.value =  msg.data[QUEUE_DN_BYTE] * QUEUE_DN_SCL;
			break;
		case PADDLE_ID + 1:
			gearVoltage.value = (double) ((uint16_t) msg.data[GEAR_VOLT_BYTE])*GEAR_VOLT_SCL;
			gearPos.value = msg.data[GEAR_BYTE];
			break;

		/*PDM ID's*/
		case PDM_ID:
			pdmUptime.value = (uint16_t) (msg.data[PDM_UPTIME_BYTE]) * PDM_UPTIME_SCL;
			pdmTemp.value = (int16_t) (msg.data[PDM_PCB_TEMP_BYTE]) * PDM_PCB_TEMP_SCL;
			pdmICTemp.value = (int16_t) (msg.data[PDM_IC_TEMP_BYTE]) * PDM_IC_TEMP_SCL;
			pdmCurrentDraw.value = (uint16_t) (msg.data[TOTAL_CURRENT_BYTE]) * TOTAL_CURRENT_SCL;
			break;
		case PDM_ID + 1:
			STRenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & STR_ENBL_BIT;
			BVBATenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & BVBAT_ENBL_BIT;
			B5V5enabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & B5v5_ENBL_BIT;
			PDLDenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & PDLD_ENBL_BIT;
			PDLUenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & PDLU_ENBL_BIT;
			AUXenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & AUX_ENBL_BIT;
			FANenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & FAN_ENBL_BIT;
			WTRenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & WTR_ENBL_BIT;
			ECUenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & ECU_ENBL_BIT;
			FUELenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & FUEL_ENBL_BIT;
			INJenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & INJ_ENBL_BIT;
			IGNenabl.value = lsbArray[LOAD_ENABLITY_BYTE/2] & IGN_ENBL_BIT;
			STR2pm.value = lsbArray[LOAD_PEAK_BYTE/2] & STR2_PEAKM_BIT;
			STR1pm.value = lsbArray[LOAD_PEAK_BYTE/2] & STR1_PEAKM_BIT;
			STR0pm.value = lsbArray[LOAD_PEAK_BYTE/2] & STR0_PEAKM_BIT;
			BVBATpm.value = lsbArray[LOAD_PEAK_BYTE/2] & BVBAT_PEAKM_BIT;
			B5V5pm.value = lsbArray[LOAD_PEAK_BYTE/2] & B5v5_PEAKM_BIT;
			PDLDpm.value = lsbArray[LOAD_PEAK_BYTE/2] & PDLD_PEAKM_BIT;
			PDLUpm.value = lsbArray[LOAD_PEAK_BYTE/2] & PDLU_PEAKM_BIT;
			AUXpm.value = lsbArray[LOAD_PEAK_BYTE/2] & AUX_PEAKM_BIT;
			FANpm.value = lsbArray[LOAD_PEAK_BYTE/2] & FAN_PEAKM_BIT;
			WTRpm.value = lsbArray[LOAD_PEAK_BYTE/2] & WTR_PEAKM_BIT;
			ECUpm.value = lsbArray[LOAD_PEAK_BYTE/2] & ECU_PEAKM_BIT;
			FUELpm.value = lsbArray[LOAD_PEAK_BYTE/2] & FUEL_PEAKM_BIT;
			INJpm.value = lsbArray[LOAD_PEAK_BYTE/2] & INJ_PEAKM_BIT;
			IGNpm.value = lsbArray[LOAD_PEAK_BYTE/2] & IGN_PEAKM_BIT;
			KILLpdmSw.value = msg.data[PDM_SWITCH_BYTE] & KILL_PDM_SW_BIT;
			ACT_DNpdmSw.value = msg.data[PDM_SWITCH_BYTE] & ACT_DN_PDM_SW_BIT;
			ACT_UPpdmSw.value = msg.data[PDM_SWITCH_BYTE] & ACT_UP_PDM_SW_BIT;
			ONpdmSw.value = msg.data[PDM_SWITCH_BYTE] & ON_PDM_SW_BIT;
			STRpdmSw.value = msg.data[PDM_SWITCH_BYTE] & STR_PDM_SW_BIT;
			break;
		case PDM_ID + 2:
			pdmVBat.value = (uint16_t) (lsbArray[VBAT_RAIL_BYTE/2]) * VBAT_RAIL_SCL;
			pdm12v.value = (uint16_t) (lsbArray[V12_RAIL_BYTE/2]) * V12_RAIL_SCL;
			pdm5v5.value = (uint16_t) (lsbArray[V5V5_RAIL_BYTE/2]) * V5V5_RAIL_SCL;
			pdm5v.value = (uint16_t) (lsbArray[V5_RAIL_BYTE/2]) * V5_RAIL_SCL;
			break;
		case PDM_ID + 3:
			pdm3v3.value = (uint16_t) (lsbArray[V3V3_RAIL_BYTE/2]) * V3V3_RAIL_SCL;
			break;
		case PDM_ID + 4:
			pdmIGNdraw.value = (uint16_t) (lsbArray[IGN_DRAW_BYTE/2]) * IGN_DRAW_SCL;
			pdmINJdraw.value = (uint16_t) (lsbArray[INJ_DRAW_BYTE/2]) * INJ_DRAW_SCL;
			pdmFUELdraw.value = (uint16_t) (lsbArray[FUEL_DRAW_BYTE/2]) * FUEL_DRAW_SCL;
			pdmECUdraw.value = (uint16_t) (lsbArray[ECU_DRAW_BYTE/2]) * ECU_DRAW_SCL;
			break;
		case PDM_ID + 5:
			pdmWTRdraw.value = (uint16_t) (lsbArray[WTR_DRAW_BYTE/2]) * WTR_DRAW_SCL;
			pdmFANdraw.value = (uint16_t) (lsbArray[FAN_DRAW_BYTE/2]) * FAN_DRAW_SCL;
			pdmAUXdraw.value = (uint16_t) (lsbArray[AUX_DRAW_BYTE/2]) * AUX_DRAW_SCL;
			pdmPDLUdraw.value = (uint16_t) (lsbArray[PDLU_DRAW_BYTE/2]) * PDLU_DRAW_SCL;
			break;
		case PDM_ID + 6:
			pdmPDLDdraw.value = (uint16_t) (lsbArray[PDLD_DRAW_BYTE/2]) * PDLD_DRAW_SCL;
			pdm5v5draw.value = (uint16_t) (lsbArray[B5V5_DRAW_BYTE/2]) * B5V5_DRAW_SCL;
			pdmBATdraw.value = (uint16_t) (lsbArray[VBAT_DRAW_BYTE/2]) * VBAT_DRAW_BYTE;
			break;
		case PDM_ID + 7:
			pdmSTR0draw.value = (uint16_t) (lsbArray[STR0_DRAW_BYTE/2]) * STR0_DRAW_SCL;
			pdmSTR1draw.value = (uint16_t) (lsbArray[STR1_DRAW_BYTE/2]) * STR1_DRAW_SCL;
			pdmSTR2draw.value = (uint16_t) (lsbArray[STR2_DRAW_BYTE/2]) * STR2_DRAW_SCL;
			pdmSTRdraw.value = (uint16_t) (lsbArray[STR_DRAW_BYTE/2]) * STR_DRAW_SCL;
			break;
		case PDM_ID + 8:
			pdmIGNcut.value = (uint16_t) (lsbArray[IGN_CUT_BYTE/2]) * IGN_CUT_SCL;
			pdmINJcut.value = (uint16_t) (lsbArray[INJ_CUT_BYTE/2]) * INJ_CUT_SCL;
			pdmAUXcut.value = (uint16_t) (lsbArray[AUX_CUT_BYTE/2]) * AUX_CUT_SCL;
			pdmPDLUcut.value = (uint16_t) (lsbArray[PDLU_CUT_BYTE/2]) * PDLU_CUT_SCL;
			break;
		case PDM_ID + 9:
			pdmPDLDcut.value = (uint16_t) (lsbArray[PDLD_CUT_BYTE/2]) * PDLD_CUT_SCL;
			pdm5v5cut.value = (uint16_t) (lsbArray[B5V5_CUT_BYTE/2]) * B5V5_CUT_SCL;
			pdmBATcut.value = (uint16_t) (lsbArray[BVBAT_CUT_BYTE/2]) * BVBAT_CUT_SCL;
			break;
		case PDM_ID + 10:
			pdmSTR0cut.value = (uint16_t) (lsbArray[STR0_CUT_BYTE/2]) * STR0_CUT_SCL;
			pdmSTR1cut.value = (uint16_t) (lsbArray[STR1_CUT_BYTE/2]) * STR1_CUT_SCL;
			pdmSTR2cut.value = (uint16_t) (lsbArray[STR2_CUT_BYTE/2]) * STR2_CUT_SCL;
			break;
		case PDM_ID + 11:
			pdmFUELNcut.value = (uint16_t) (lsbArray[FUEL_CUT_N_BYTE/2]) * FUEL_CUT_N_SCL;
			pdmWTRNcut.value = (uint16_t) (lsbArray[WTR_CUT_N_BYTE/2]) * WTR_CUT_N_SCL;
			pdmFANNcut.value = (uint16_t) (lsbArray[FAN_CUT_N_BYTE/2]) * FAN_CUT_N_SCL;
			pdmECUNcut.value = (uint16_t) (lsbArray[ECU_CUT_N_BYTE/2]) * ECU_CUT_N_SCL;
			break;
		case PDM_ID + 12:
			pdmFUELPcut.value = (uint16_t) (lsbArray[FUEL_CUT_P_BYTE/2]) * FUEL_CUT_P_SCL;
			pdmWTRPcut.value = (uint16_t) (lsbArray[WTR_CUT_P_BYTE/2]) * WTR_CUT_P_SCL;
			pdmFANPcut.value = (uint16_t) (lsbArray[FAN_CUT_P_BYTE/2]) * FAN_CUT_P_SCL;
			pdmECUPcut.value = (uint16_t) (lsbArray[ECU_CUT_P_BYTE/2]) * ECU_CUT_P_SCL;
			break;

		/*Tire Temps*/
		case TIRE_TEMP_FL_ID:
			ttFLA[0].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE])*TIRE_TEMP_SCL);
			ttFLA[1].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE])*TIRE_TEMP_SCL);
			ttFLA[2].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE])*TIRE_TEMP_SCL);
			ttFLA[3].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE])*TIRE_TEMP_SCL);
			ttFL.value = (ttFLA[0].value+ttFLA[1].value+ttFLA[2].value+ttFLA[3].value)/4.0;
		case TIRE_TEMP_FR_ID:
			ttFRA[0].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE])*TIRE_TEMP_SCL);
			ttFRA[1].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE])*TIRE_TEMP_SCL);
			ttFRA[2].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE])*TIRE_TEMP_SCL);
			ttFRA[3].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE])*TIRE_TEMP_SCL);
			ttFR.value = (ttFRA[0].value+ttFRA[1].value+ttFRA[2].value+ttFRA[3].value)/4.0;
		case TIRE_TEMP_RL_ID:
			ttRLA[0].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE])*TIRE_TEMP_SCL);
			ttRLA[1].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE])*TIRE_TEMP_SCL);
			ttRLA[2].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE])*TIRE_TEMP_SCL);
			ttRLA[3].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE])*TIRE_TEMP_SCL);
			ttRL.value = (ttRLA[0].value+ttRLA[1].value+ttRLA[2].value+ttRLA[3].value)/4.0;
		case TIRE_TEMP_RR_ID:
			ttRRA[0].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_1_BYTE])*TIRE_TEMP_SCL);
			ttRRA[1].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_2_BYTE])*TIRE_TEMP_SCL);
			ttRRA[2].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_3_BYTE])*TIRE_TEMP_SCL);
			ttRRA[3].value = (double) ((uint16_t) (msg.data[TIRE_TEMP_4_BYTE])*TIRE_TEMP_SCL);
			ttRR.value = (ttRRA[0].value+ttRRA[1].value+ttRRA[2].value+ttRRA[3].value)/4.0;
	}
}

void CANswitchStates(void){
	CAN_data switchData = {0};
	uint8_t bitMask = (uint8_t)momentaries[0].value|(uint8_t)momentaries[1].value << 1|(uint8_t)momentaries[2].value << 2|(uint8_t)momentaries[3].value << 3|(uint8_t)switches[0].value << 4|(uint8_t)switches[1].value << 5|(uint8_t)switches[2].value << 6|(uint8_t)switches[3].value << 7;
	switchData.byte0 = bitMask;
	switchData.byte1 = ((uint8_t)rotary[0].value << 4) | (uint8_t)rotary[1].value;
	switchData.byte2 = ((uint8_t)rotary[2].value << 4) | (uint8_t)tRotary[0].value;
	switchData.byte3 = (uint8_t)tRotary[1].value << 4;
	CAN_send_message(WHEEL_ID + 0x1, 4, switchData);
}

void CANswitchADL(void){
	CAN_data rotaries = {0};
	rotaries.halfword0 = 0x0000;
	rotaries.halfword1 = (uint16_t) rotary[0].value;
	rotaries.halfword2 = (uint16_t) rotary[1].value;
	rotaries.halfword3 = (uint16_t) rotary[2].value;
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
