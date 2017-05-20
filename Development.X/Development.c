#include "Development.h"

kTemp kt;
int main(void){
    init_general();// Set general runtime configuration bits
    init_gpio_pins();// Set all I/O pins to low outputs
    init_oscillator(0);// Initialize oscillator configuration bits
    init_timer2();// Initialize timer2 (millis)
    init_spi();
    
    PIC_LED_TRIS = OUTPUT;
    int i;
    while(1){
	for(i = 0;i<1000000;i++);
    	PIC_LED_LAT = 1;
	get_temp(&kt);
	for(i = 0;i<1000000;i++);
    	PIC_LED_LAT = 0;
    }
  return 0;
}

/**
 * TMR2 Interrupt Handler
 *
 * Fires once every millisecond.
 */
void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL6SRS))) timer2_inthnd(void) {
	//millis++;// Increment millis count
	//if (!(millis%250)){
	//		PIC_LED_LAT = !PIC_LED_LAT;
	//}
	IFS0CLR = _IFS0_T2IF_MASK;// Clear TMR2 Interrupt Flag
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