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
			neutQueue.value = 0;
			upQueue.value = 0;
			downQueue.value = 0;
			break;
		case PADDLE_ID + 1:
			gearPos.value = msg.data[GEAR_BYTE];
			break;
		case PDM_ID:
			// Uptime
			// PCB Temp
			// IC Temp
			// Total Current
			break;
		case PDM_ID + 1:
			// Load Enability
			// Load Peak Mode
			// Switch Bitmap
			break;
		case PDM_ID + 2:
			// VBAT Rail
			// 12v Rail
			// 5v5 Rail
			// 5v Rail
			break;
		case PDM_ID + 3:
			// 3v3 Rail
			break;
		case PDM_ID + 4:
			// Current Draws
			// IGN
			// INJ
			// FUEL
			// ECU
			break;
		case PDM_ID + 5:
			// Current Draws
			// WTR
			// FAN
			// AUX
			// PDLU
			break;
		case PDM_ID + 6:
			// Current Draws
			// PDLD
			// B5v5
			// BVBAT
			break;
		case PDM_ID + 7:
			// Current Draws
			// STR0
			// STR1
			// STR2
			// STR
			break;
		case PDM_ID + 8:
			// Current Cutoff
			// IGN
			// INJ
			// AUX
			// PDLU
			break;
		case PDM_ID + 9:
			// Current Cutoff
			// PDLU
			// B5v5
			// BVBAT
			break;
		case PDM_ID + 10:
			// Current Cutoff
			// STR0
			// STR1
			// STR2
			break;
		case PDM_ID + 11:
			// Normal Current Cutoff
			// FUEL
			// WTR
			// FAN
			// ECU
			break;
		case PDM_ID + 12:
			// Peak Current Cutoff
			// FUEL
			// WTR
			// FAN
			// ECU
			break;
  }
}

double parseMsgMotec(CAN_message * msg, uint8_t byte, double scl){
	return ((double) ((msg->data[byte] << 8) | msg->data[byte + 1])) * scl;
}