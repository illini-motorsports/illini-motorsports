#include <proc/p32mz2048efm100.h>

#include "Development.h"


kTemp kt;
int main(void){
  init_general();// Set general runtime configuration bits
  init_gpio_pins();// Set all I/O pins to low outputs
  init_oscillator(0);// Initialize oscillator configuration bits
  init_timer2();// Initialize timer2 (millis)
  init_timer6();
  init_pwm(0x4E1F,1); //Initialize pwm signal with given period (DRS page 2017-2018)
  init_spi();
  
  millis = 0;
  periods = 0;
  
  while(1);
  
  //PWM_TRIS = OUTPUT;
  
  PIC_LED_TRIS = OUTPUT;
  int i;
  while(1){
    for(i = 0;i<1000000;i++);
    PIC_LED_LAT = 1;
    //PWM_LAT = 1;
    get_temp(&kt);
    for(i = 0;i<1000000;i++);
    PIC_LED_LAT = 0;
    //PWM_LAT = 0;
  }
  return 0;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
  millis++;// Increment millis count
  //if (!(millis%250)){
  //    PIC_LED_LAT = !PIC_LED_LAT;
  //}
  
  IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
}

//Timer6 interrupt for PDM soft starting. Uses timer6. Will send signal to all 3 OCs and pins
//Hopefully switches will be turned on at some delay, maybe 2 seconds at most between flipping on WTR, FUEL, or FAN.
void __attribute__((vector(_TIMER_6_VECTOR), interrupt(IPL7SRS))) timer6_inthnd(void) {
  periods++;// Increment periods count
  //soft start multipliers start at .2 and end at 2.
  if(periods >= 15 && periods <= 74) {
      pwm_set(.2*PERD,7);
      pwm_set(.2*PERD,8);
      pwm_set(.2*PERD,9);
  }
  else if(periods >= 75 && periods <= 99) {
      pwm_set(.4*PERD,7);
      pwm_set(.4*PERD,8);
      pwm_set(.4*PERD,9);
  }
  else if (periods >= 100 && periods <= 149){
      pwm_set(1*PERD,7);    //TODO: Calculate hex values
      pwm_set(1*PERD,8); 
      pwm_set(1*PERD,9); 
  }
  else if (periods >= 150 && periods <= 199) {
      pwm_set(1.5*PERD,7);
      pwm_set(1.5*PERD,8);
      pwm_set(1.5*PERD,9);
  }
  else if (periods >= 200){
      pwm_set(2*PERD,7);
      pwm_set(2*PERD,8);
      pwm_set(2*PERD,9);
      periods = 0;              //reset periods
      set_load_pwm(FUEL_IDX,1); //set final load state.
  }
  IFS0CLR = _IFS0_T6IF_MASK;// Clear TMR6 Interrupt Flag
}

void __attribute__((vector(_TIMER_4_VECTOR), interrupt(IPL7SRS))) timer4_inthnd(void) {
  periods++;// Increment periods count
  if (periods == 100){
      pwm_set(0x03E8,1);    //base period
  } else if (periods == 200){
      pwm_set(0x07D0,1);    //2 * base period
      periods = 0;
  }
  
  IFS0CLR = _IFS0_T4IF_MASK;// Clear TMR4 Interrupt Flag
}

void __attribute__((vector(_TIMER_6_VECTOR), interrupt(IPL7SRS))) timer6_inthnd(void) {
  periods++;// Increment periods count
  if (periods == 100){
      pwm_set(0x03E8,7);
  } else if (periods == 200){
      pwm_set(0x07D0,7);
      periods = 0;
  }
  
  IFS0CLR = _IFS0_T6IF_MASK;// Clear TMR6 Interrupt Flag
}

void get_temp(kTemp *kt){
  uint32_t spi_bits = get_temp_spi();
  kt->fault = spi_bits & 0x0000000F;
  if(kt->fault){
    kt->temp = 0;
    kt->jTemp = 0;
  } else {
    kt->temp = (spi_bits >> 18)/4.0;
    kt->jTemp = ((spi_bits & 0x0000FFFF) >> 4)*0.0625;
  }
}

uint32_t get_temp_spi(){
  uint32_t resp = 0;
  while(SPI1STATbits.SPIBUSY);
  TEMP_CS_LAT = 0;
  SPI1BUF = 0;
  while(!SPI1STATbits.SPIRBF);
  resp = SPI1BUF;
  TEMP_CS_LAT = 1;
  return resp;
}

void init_spi(){
  unlock_config();

  // Initialize SDI6/SDO6 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB9 = INPUT;
  ANSELBbits.ANSB9 = 0;
  SDI1Rbits.SDI1R = 0b0101; // RPB9
  TRISBbits.TRISB10 = OUTPUT;
  RPB10Rbits.RPB10R = 0b0101; // SDO1
  CFGCONbits.IOLOCK = 1;

  // Initialize SCK6 and !CS_NVM pins
  TRISDbits.TRISD1 = OUTPUT; // SCK1
  TEMP_CS_TRIS = OUTPUT; // !CS_NVM
  TEMP_CS_LAT = 1;

  // Disable interrupts
  IEC3bits.SPI1EIE = 0;
  IEC3bits.SPI1RXIE = 0;
  IEC3bits.SPI1TXIE = 0;

  // Disable SPI1 module
  SPI1CONbits.ON = 0;

  // Clear receive buffer
  uint32_t readVal = SPI1BUF;

  // Use standard buffer mode
  SPI1CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI1BRG = 24;

  SPI1STATbits.SPIROV = 0;

  SPI1CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI1CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI1CONbits.MODE32 = 1;  // 32/16-Bit Communication Select bits (8-bit)
  SPI1CONbits.MODE16 = 0;  // 32/16-Bit Communication Select bits (8-bit)
  SPI1CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI1CONbits.CKE = 1;     // SPI Clock Edge Select (Serial output data changes on transition from active clock state to idle clock state)
  SPI1CONbits.DISSDI = 0;
  SPI1CONbits.DISSDO = 0;
  SPI1CONbits.SMP = 1;
  SPI1CONbits.CKP = 0;     // Clock Polarity Select (Idle state for clock is a low level)

  // Enable SPI1 module
  SPI1CONbits.ON = 1;

  lock_config();
}

void pwm_set(uint16_t duty_cycle, uint8_t OC){
    
    switch(OC){
        case 1:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC1R = duty_cycle; //becomes read only during pwm mode operation
            OC1RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
        
        case 2:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC2R = duty_cycle; //becomes read only during pwm mode operation
            OC2RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 3:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC3R = duty_cycle; //becomes read only during pwm mode operation
            OC3RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 4:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC4R = duty_cycle; //becomes read only during pwm mode operation
            OC4RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 5:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC5R = duty_cycle; //becomes read only during pwm mode operation
            OC5RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 6:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC6R = duty_cycle; //becomes read only during pwm mode operation
            OC6RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
        
        case 7:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC7R = duty_cycle; //becomes read only during pwm mode operation
            OC7RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 8:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC8R = duty_cycle; //becomes read only during pwm mode operation
            OC8RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
            
        case 9:
            unlock_config();
    
            CFGCONbits.IOLOCK = 0;
            //CFGCONbits.OCACLK = 1;
    
            OC9R = duty_cycle; //becomes read only during pwm mode operation
            OC9RS = duty_cycle; //sets to 5% duty cycle
    
            CFGCONbits.IOLOCK = 1;
    
            lock_config();
            break;
    }
}
    
    /*
int delay(int num) {
    uint32_t start = millis;
    while (millis - start < 1000*num);
    return 1;
}
    */
   