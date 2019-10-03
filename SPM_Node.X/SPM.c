#include "SPM.h"

double analog_channels[32] = {0};
double strain_channels[24] = {0};
max31855_data thermocouple_channels[4] = {0};
SPIConn *analog_connections[2] = {0};
SPIConn *strain_connections[2] = {0};
SPIConn *thermocouple_connections[4] = {0};
uint16_t digital_channels = 0;
SPIConn *digital_connection = {0};
volatile uint32_t millis = 0;
uint32_t canAnalogMillis = 0;
uint32_t canDiagMillis = 0;
uint8_t prev_dig_val[4] = {0};
uint32_t DigMicros[4] = {0,0,0,0};
double wheel_speeds[4] = {0};
uint32_t counter = 0;

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
  init_thermocouples();
  
  while(1){ 
    //update_digital_channels();
    read_thermocouples();
    //calc_wheel_speeds();
    update_analog_channels();
    //update_strain_channels();
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
//  CAN_data data = {0};
//  int i;
//  for(i = 0;i<8;i++){
//    data.halfword0 = (uint16_t) (analog_channels[i*4]*ANALOG_CAN_SCL);
//    data.halfword1 = (uint16_t) (analog_channels[(i*4)+1]*ANALOG_CAN_SCL);
//    data.halfword2 = (uint16_t) (analog_channels[(i*4)+2]*ANALOG_CAN_SCL);
//    data.halfword3 = (uint16_t) (analog_channels[(i*4)+3]*ANALOG_CAN_SCL);
//    CAN_send_message(SPM_ID + i + 1, 8, data);
//  }
    CAN_data susFront = {0};
    CAN_data susRear = {0};
    CAN_data strain;
    CAN_data temps;
    CAN_data radTemps;
    double radIn = (analog_channels[7]) * ((10000) / (5 - analog_channels[7]));
    double radOut = (analog_channels[8]) * ((10000) / (5 - analog_channels[8]));
    susRear.halfword0 = (uint16_t) (analog_channels[0]*ANALOG_CAN_SCL);
    susRear.halfword1 = (uint16_t) (analog_channels[1]*ANALOG_CAN_SCL);
    susRear.halfword2 = (uint16_t) (analog_channels[2]*ANALOG_CAN_SCL);
    susRear.halfword3 = (uint16_t) (analog_channels[3]*ANALOG_CAN_SCL);
    CAN_send_message(SPM_ID + 1, 8, susRear);
    susFront.halfword0 = (uint16_t) (analog_channels[4]*ANALOG_CAN_SCL);
    susFront.halfword1 = (uint16_t) (analog_channels[5]*ANALOG_CAN_SCL);
    susFront.halfword2 = (uint16_t) (analog_channels[6]*ANALOG_CAN_SCL);
    susFront.halfword3 = (uint16_t) (analog_channels[7]*ANALOG_CAN_SCL);
    CAN_send_message(0x61, 8, susFront);
//    strain.halfword0 = (uint16_t) (strain_channels[0]*ANALOG_CAN_SCL);
//    strain.halfword1 = (uint16_t) (strain_channels[1]*ANALOG_CAN_SCL);
//    strain.halfword2 = (uint16_t) (strain_channels[2]*ANALOG_CAN_SCL);
//    strain.halfword3 = (uint16_t) (strain_channels[3]*ANALOG_CAN_SCL);
//    CAN_send_message(SGH_ID + 1, 8, strain);
    temps.halfword0 = (uint16_t) thermocouple_channels[0].thermocoupleTemp;
    temps.halfword1 = (uint16_t) thermocouple_channels[1].thermocoupleTemp;
    temps.halfword2 = (uint16_t) thermocouple_channels[2].thermocoupleTemp;
    temps.halfword3 = (uint16_t) thermocouple_channels[3].thermocoupleTemp;
    CAN_send_message(0x5A,8,temps);
    radTemps.word0 = (linearizeThermistor(radIn));
    radTemps.word1 = (linearizeThermistor(radOut));
    CAN_send_message(0x52,8,radTemps);
 }

void CANDiag(void){
  CAN_data data = {0};
  data.halfword0 = (uint16_t) (millis / 1000);
  data.halfword1 = pcb_temp;
  data.halfword2 = junc_temp;
  CAN_send_message(SPM_ID, 6, data);
}

int32_t linearizeThermistor(double inTherm)
{
    int i = 0;
    double outTherm = 666;
    if(inTherm >= 5896)
    {
        return (outTherm * (ANALOG_CAN_SCL / 10)); //Water is way too cold, send fault value
    }
    for(i = 0; i < 20; i++)
    {
        if((resRange[i] < inTherm) && (i != 19))
        {
            outTherm = tempRange[i-1] + ((tempRange[i] - tempRange[i-1])/(resRange[i] - resRange[i-1]))*(inTherm - resRange[i-1]);
            break;
        }
    }
    return (int32_t)(outTherm * (ANALOG_CAN_SCL / 10));
}

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL5SRS))) timer2_inthnd(void) {
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

void update_strain_channels(void){
    uint16_t strain_positive[16] = {0};
    uint16_t strain_negative[16] = {0};
    ad7490_read_channels(strain_positive, strain_connections[0]); //Read positive values
    ad7490_read_channels(strain_negative, strain_connections[1]); //Read negative values
    int i;
    for(i=0;i< 2; i++){
            strain_channels[i] = 5*(strain_positive[i]/4095.0);
            strain_channels[i+2] = 5*(strain_negative[i]/4095.0);
    }
    
    
//    for(i=0;i<12;i++){
//        if(strain_negative[i] > strain_positive[i]){
//            strain_channels[i] = 5*(strain_negative[i]/4095.0);
//        }
//        else if(strain_positive[i] > strain_negative[i]){
//            strain_channels[i] = 5*(strain_positive[i]/4095.0);
//        }
//        else{
//            strain_channels[i] = 0;
//        }
    //}
    return;
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
  digital_channels = mcp23s17_read_all(digital_connection);
  counter+=1;
}

void read_thermocouples(void){
    int k;
    for(k=0;k<4;k++){
    thermocouple_channels[k] = read_max31855_data(thermocouple_connections[k]);
    }
}

void calc_wheel_speeds(void) {
    int j;
    uint8_t thing[4];
    thing[0] = (digital_channels & 0b100000) >> 6;
    thing[1] = (digital_channels & 0b10000) >> 4;
    thing[2] = (digital_channels & 0b100) >> 2;
    thing[3] = digital_channels & 1;
    
    for(j =0; j < 4; j++)
    if(!prev_dig_val[j] && thing[j]){
        wheel_speeds[j] = 10000*TIRE_DIAMETER_PER_TR/(counter*SPI_DIFF_TIME); // wheel speed in fps
        counter=0;
    }
}



void init_adcs(void) {
  ADC_0_CS_LAT = 1;
  ADC_1_CS_LAT = 1;
  ADC_0_CS_TRIS = OUTPUT;
  ADC_1_CS_TRIS = OUTPUT;
  ADC_2_CS_LAT = 1;
  ADC_3_CS_LAT = 1;
  ADC_2_CS_TRIS = OUTPUT;
  ADC_3_CS_TRIS = OUTPUT;
//  analog_connections[0] = init_ad7490(1, ADC_0_CS_LATBITS, ADC_0_CS_LATNUM);
//  analog_connections[1] = init_ad7490(1, ADC_1_CS_LATBITS, ADC_1_CS_LATNUM);
  strain_connections[0] = init_ad7490(2, ADC_2_CS_LATBITS, ADC_2_CS_LATNUM);
  strain_connections[1] = init_ad7490(2, ADC_3_CS_LATBITS, ADC_3_CS_LATNUM);
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

void init_thermocouples(void){
    THERMOCOUPLE_CS_LAT1 = 1;
    THERMOCOUPLE_CS_LAT2 = 1;
    THERMOCOUPLE_CS_LAT3 = 1;
    THERMOCOUPLE_CS_LAT4 = 1;
    
    THERMOCOUPLE_CS_TRIS1 = OUTPUT;
    THERMOCOUPLE_CS_TRIS2 = OUTPUT;
    THERMOCOUPLE_CS_TRIS3 = OUTPUT;
    THERMOCOUPLE_CS_TRIS4 = OUTPUT;
    
    thermocouple_connections[0] = init_max31855(3,THERMOCOUPLE_CS_LATBITS1,THERMOCOUPLE_CS_LATNUM1);
    thermocouple_connections[1] = init_max31855(3,THERMOCOUPLE_CS_LATBITS2,THERMOCOUPLE_CS_LATNUM2);
    thermocouple_connections[2] = init_max31855(3,THERMOCOUPLE_CS_LATBITS3,THERMOCOUPLE_CS_LATNUM3);
    thermocouple_connections[3] = init_max31855(3,THERMOCOUPLE_CS_LATBITS4,THERMOCOUPLE_CS_LATNUM4);
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