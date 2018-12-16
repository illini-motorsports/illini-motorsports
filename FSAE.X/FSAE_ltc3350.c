/**
 * FSAE Library LTC3350
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Jacob Drewniak
 * Created:   2018-2019
 */
#include "FSAE_ltc3350.h"

/**
 */
void init_ltc3350(void) {
  // Initialize pins

  // Initialize I2C bus
  _ltc3350_init_i2c();

}

/**
 * Wait for idle bus state
 */
inline void _ltc3350_wait_idle(void) {
  while (I2C4CON & 0x1F || 
         I2C4STATbits.TRSTAT || 
         I2C4STATbits.S);
}

/**
 * 
 * S, Slave addr/wr, command, RS, Slave addr/rd, data low, data high, P
 */
uint16_t ltc3350_read(uint8_t addr) {
    start_i2c();
    delay();
    
    ack = send_data_i2c((LTC3350_DEV_ADDR << 1) & 0xFE);
    if (ack){
        return -1;
    }
    
    ack = send_data_i2c(addr);
     if (ack){
        return -2;
    }
    
    ack = send_data_i2c((LTC3350_DEV_ADDR << 1) & 0xFF);
    if (ack){
        return -3;
    }
    
    data = read_data_i2c();
    acknowledge();
    
  return data;
}

/**
 * 
 * S, Slave addr/wr, command, data low, data high, P 
 */
void ltc3350_write(uint8_t addr, uint16_t data) {
  _ltc3350_wait_idle();

// Generate Start Condition
  I2C4CONbits.SEN = 1;
  while(I2C4CONbits.SEN);

  //Send slave address
  I2C4TRN = (LTC3350_DEV_ADDR << 1) | 0xFE;
  while (I2C4STATbits.TBF); // Wait for empty buffer
  while (I2C4STATbits.ACKSTAT); // Wait for ACK
  while(I2C4CON & 0x001F);

  //Send address of register
  I2C4TRN = addr;
  while (I2C4STATbits.TBF); // Wait for empty buffer
  while (I2C4STATbits.ACKSTAT); // Wait for ACK
  while(I2C4CON & 0x001F);

  //Generate Repeat Start condition
  I2C4CONbits.RSEN = 1;
  while(I2C4CONbits.RSEN);

  //Send slave address
  I2C4TRN = (LTC3350_DEV_ADDR << 1) | 0xFF;
  while (I2C4STATbits.TBF); // Wait for empty buffer
  while (I2C4STATbits.ACKSTAT); // Wait for ACK
  while(I2C4CON & 0x001F);

  //Send data
  I2C4TRN = data & 0xFF00;
  while (I2C4STATbits.TBF); // Wait for empty buffer
  while (I2C4STATbits.ACKSTAT); // Wait for ACK
  while(I2C4CON & 0x001F);

  //Send data
  I2C4TRN = data & 0x00FF;
  while (I2C4STATbits.TBF); // Wait for empty buffer
  while (I2C4STATbits.ACKSTAT); // Wait for ACK
  while(I2C4CON & 0x001F);

  //Generate Stop condition
  while (I2C4CON & 0x1F); // Wait for master logic inactive
  I2C4CONbits.PEN = 1; // Generate stop event
  while (I2C4CONbits.PEN);

}

/**
 */
void _ltc3350_init_i2c(void) {
  //unlock_config();

  // Disable I2C4 Module
    I2C4CON = 0x00000000;

  //ANSELGbits.ANSG7 = DIG_INPUT;
  

  // SCL4
  //TRISGbits.TRISG8 = OUTPUT;
  
  
  CHG_PFO_TRIS = INPUT;
  CHG_PFO_ANSEL = DIG_INPUT;
  CHG_ALM_TRIS = INPUT;
  CHG_ALM_ANSEL = DIG_INPUT;
  CHG_CPGD_TRIS = INPUT;
  CHG_CPGD_ANSEL = DIG_INPUT;
  DATA_LINE_ANSEL = DIG_INPUT;
  CLK_LINE_ANSEL = DIG_INPUT;
  DATA_LINE_TRIS = OUTPUT;
  CLK_LINE_TRIS = INPUT;
  //CNPUGbits.CNPUG7 = 1;
  //CNPUGbits.CNPUG8 = 1;
  
  //Enable Interrupts
  IEC5bits.I2C4MIE = 0;
  IFS5bits.I2C4MIF = 0;
  IPC43bits.I2C4MIP = 7;

  /**
   * Baud rate (400kHz desired)
   * 
   * I2CxBRG = (F_FBCLK2 * ((1 / (2 * F_SCL)) - T_PGD)) - 2
   * I2CxBRG = (100Mhz * ((1.25 uS - 104nS)) - 2
   * I2CxBRG = 112
   */
  //I2C4BRG = 0x70; //400
  I2C4BRG = 0x1E8; //100 kHz
  //I2C4BRG = 0x9B8; // 20

  //I2C4ADD = LTC3350_DEV_ADDR;

  // I2C4CON
  //I2C4CONbits.A10M = 0; // 7-bit Slave Address
  //I2C4CONbits.SMEN = 1; // Compliant with SMBus Spec
  //I2C4CONbits.DISSLW = 1; //100kHz slew rate mode
  I2C4CONbits.SDAHT = 1;
  I2C4CON = 0x00008200;


  // Enable I2C4 Module
  //I2C4CONbits.ON = 1;
  //I2C4CONbits.SCLREL = 0;

}

uint16_t send_data_i2c(uint8_t blah){
    int i;
    for(i = 7; i >= 0; i--){
        DATA_LINE_LAT = (blah >> i) & 0x01;
        setup_time();
        CLK_LINE_LAT = 1;
        hold_time();
        CLK_LINE_LAT = 0;
        if (i > 0){
        setup_time();
        }
    }
    DATA_LINE_TRIS = INPUT;
    DATA_LINE_ANSEL = DIG_INPUT;
    setup_time();
    setup_time();
    CLK_LINE_LAT = 1;
    ack = 1;
    current_time = micros;
    while(micros - current_time < 20){
        if(!DATA_LINE_PORT){
            ack = 0;
            break;
        }
    }
    CLK_LINE_LAT = 0;
    return ack;
}

uint16_t read_data_i2c(){
    DATA_LINE_TRIS = INPUT;
    DATA_LINE_ANSEL = DIG_INPUT;
    
    uint16_t stuff = 0;
    int i;
    for(i = 0; i < 8; i++){
        CLK_LINE_LAT = 1;
        setup_time();
        stuff = ((stuff << i) | DATA_LINE_PORT);
        setup_time();
        CLK_LINE_LAT = 0;
        hold_time();
    }
    return stuff;
}

void acknowledge(){
    DATA_LINE_TRIS = OUTPUT;
    DATA_LINE_LAT = 0;
    setup_time();
    CLK_LINE_LAT = 1;
    hold_time();
    CLK_LINE_LAT = 0;
    setup_time();
    DATA_LINE_TRIS = INPUT;
    DATA_LINE_ANSEL = DIG_INPUT;
    setup_time();
}

void start_i2c(){
    micros = 0;
    init_timer6(1);
    DATA_LINE_TRIS = OUTPUT;
    DATA_LINE_LAT = 0;
    setup_time();
    CLK_LINE_TRIS = OUTPUT;
    CLK_LINE_LAT = 0;
}

void stop_i2c(){
    DATA_LINE_TRIS = INPUT;
    CLK_LINE_TRIS = INPUT;
    DATA_LINE_ANSEL = DIG_INPUT;
    CLK_LINE_ANSEL = DIG_INPUT;
    T6CONbits.ON = 0;
}

inline void hold_time(){
    current_time = micros;
    while(micros - current_time < HOLD_TIME);
}

inline void setup_time(){
    current_time = micros;
    while(micros - current_time < SETUP_TIME);
}

inline void delay(){
    current_time = micros;
    while(micros - current_time < 50000);
}