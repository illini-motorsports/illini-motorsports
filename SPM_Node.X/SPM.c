#include "SPM.h"

double analog_channels[36] = {0};
SPIConn *analog_connections[6] = {0};
uint16_t digital_channels = 0;
SPIConn *digital_connection = {0};
volatile uint32_t millis = 0;
uint32_t canAnalogMillis = 0;

void main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(0);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_adcs();// Initialize all of the ADC's
  init_gpio(); // Initialize external digital GPIO

  while(1){
    //update_analog_channels();
    update_digital_channels();

    if(millis - canAnalogMillis > 50){
      //CANAnalogChannels();
      canAnalogMillis = millis;
    }
  }
}

void CANAnalogChannels(void){
  CAN_data data = {0};
  int i;
  for(i = 0;i<8;i++){
    data.halfword0 = (uint16_t) (analog_channels[i*4]*ANALOG_CAN_SCL);
    data.halfword1 = (uint16_t) (analog_channels[(i*4)+1]*ANALOG_CAN_SCL);
    data.halfword2 = (uint16_t) (analog_channels[(i*4)+2]*ANALOG_CAN_SCL);
    data.halfword3 = (uint16_t) (analog_channels[(i*4)+3]*ANALOG_CAN_SCL);
    CAN_send_message(SPM_ID + i + 1, 8, data);
  }
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count

  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

// reads from both ADC's at the same time, which kinda negates the point of the
// low speed vs high speed ADC's
void update_analog_channels(void){
  // Update ad7490 values
  uint16_t ad7490_values[32] = {0};
  ad7490_read_channels(ad7490_values, analog_connections[0]);
  ad7490_read_channels(ad7490_values+16, analog_connections[1]);
  int i;
  for(i = 0;i<32;i++){
    analog_channels[i] = 5*(ad7490_values[analogMappings[i]]/4095.0);
  }
}

void update_digital_channels(void) {
  digital_channels = mcp23s17_read_all(digital_connection) >> 8;
}

void init_adcs(void) {
  ADC_0_CS_LAT = 1;
  ADC_1_CS_LAT = 1;
  ADC_0_CS_TRIS = OUTPUT;
  ADC_1_CS_TRIS = OUTPUT;

  analog_connections[0] = init_ad7490(1, ADC_0_CS_LATBITS, ADC_0_CS_LATNUM);
  analog_connections[1] = init_ad7490(1, ADC_1_CS_LATBITS, ADC_1_CS_LATNUM);
}

void init_gpio(void){
  // Initialize all CS pins
  GPIO_CS_LAT = 1;
  GPIO_CS_TRIS = OUTPUT;

  // initialize all gpio chips
  digital_connection = init_mcp23s17(5, GPIO_CS_LATBITS, GPIO_CS_LATNUM);
}
