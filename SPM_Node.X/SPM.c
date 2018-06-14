#include "SPM.h"

double analog_channels[32] = {0};
SPIConn *analog_connections[2] = {0};
uint16_t digital_channels = 0;
SPIConn *digital_connection = {0};
volatile uint32_t millis = 0;
uint32_t canAnalogMillis = 0;
uint32_t canDiagMillis = 0;

uint32_t temp_samp_tmr = 0;
int16_t pcb_temp = 0; // PCB temperature reading in units of [C/0.005]
int16_t junc_temp = 0; // Junction temperature reading in units of [C/0.005]

void main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(0);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_adc(NULL); // Initialize ADC module
  init_termination(NOT_TERMINATING);
  init_gpio(); // Initialize external digital GPIO
  init_can(); // Initialize CAN

  ADCCON3bits.GSWTRG = 1; // Initial ADC Conversion
  STI();// Enable interrupts
  
  init_adcs();// Initialize all of the ADC's
  while(1){
    update_analog_channels();

    if(millis - canAnalogMillis > CAN_ANALOG_INTV){
      CANAnalogChannels();
      canAnalogMillis = millis;
    }

    if(millis - canDiagMillis > CAN_DIAG_INTV){
      CANDiag();
      canDiagMillis = millis;
    }

    sample_temp(); // Sample internal and external temperature sensors
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

void CANDiag(void){
  CAN_data data = {0};
  data.halfword0 = (uint16_t) (millis / 1000);
  data.halfword1 = pcb_temp;
  data.halfword2 = junc_temp;
  CAN_send_message(SPM_ID, 6, data);
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count

  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
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

void process_CAN_msg(CAN_message msg) {}

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

void sample_temp(void) {
  if(millis - temp_samp_tmr >= TEMP_SAMP_INTV) {

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

    temp_samp_tmr = millis;
  }
}
