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
      eng_rpm = ((double) ((msg.data[ENG_RPM_BYTE] << 8) |
          msg.data[ENG_RPM_BYTE + 1])) * ENG_RPM_SCL;
      bat_volt_ecu = ((double) ((msg.data[VOLT_ECU_BYTE] << 8) |
          msg.data[VOLT_ECU_BYTE + 1])) * VOLT_ECU_SCL;

      break;
    case MOTEC_ID + 1:
      waterTemp.value = ((double) ((msg.data[ENG_TEMP_BYTE] << 8) |
          msg.data[ENG_TEMP_BYTE + 1])) * ENG_TEMP_SCL;
      oilTemp.value = ((double) ((msg.data[OIL_TEMP_BYTE] << 8) |
          msg.data[OIL_TEMP_BYTE + 1])) * OIL_TEMP_SCL;

      break;
    case MOTEC_ID + 2:
      oilPress.value = ((double) ((msg.data[OIL_PRES_BYTE] << 8) |
          msg.data[OIL_PRES_BYTE + 1])) * OIL_PRES_SCL;

      break;
	case PADDLE_ID + 1:
		gearPos.value = msg.data[GEAR_BYTE];
		break;
  }
}