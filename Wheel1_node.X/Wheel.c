#include "Wheel.h"

int flag;
uint16_t readMsg;
uint16_t writeMsg = 0x0000;
uint16_t readData = 0x0000;
uint16_t readVals[100];
int readCount = 0;

void main(void){
    init_general(); // Set general runtime configuration bits
    init_gpio_pins(); // Set all I/O pins to low outputs
    //init_peripheral_modules(); // Disable unused peripheral modules
    init_oscillator(); // Initialize oscillator configuration bits
    init_timer1(); // Initialize timer1
    init_spi(); // Initialize SPI interface
    asm volatile("ei"); // Enable interrupts
    int input = 0;
    flag = 0;
    TRISDbits.TRISD9 = OUTPUT; // LED pin
    LATDbits.LATD9 = 1;
    TRISAbits.TRISA7 = OUTPUT; // CS pin
    send_rheo(0b0100000011111011); //
    int i;
     while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATAbits.LATA7 = 0; // CS selected
  SPI1BUF = 0x00AA;
 	while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module 
  LATAbits.LATA7 = 1; // CS selected
    while(1){
	send_rheo(0x00AA);
    	for(i=0;i<100;i++);
    }
}

void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
    LATDbits.LATD9 = LATDbits.LATD9 ? 0 : 1; // Invert LATD1 - Toggle the LED
    IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
    
}

void send_rheo(uint16_t msg) {
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATAbits.LATA7 = 0; // CS selected
  //SPI1BUF = msg;
  //while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  SPI1STATbits.SPIROV = 0;
  SPI1BUF = 0x0C00;
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
 // while(!SPI1STATbits.SPIRBF); // Wait for idle SPI module
  readVals[readCount] = SPI1BUF;
  readCount++;
  if(readCount >= 100){
      int i;
    	for(i=0;i<100;i++);
  }
  //while(SPI1STATbits.SPIBUSY); // Weit for idle SPI module
  
  while(SPI1STATbits.SPIBUSY); // Wait for idle SPI module
  LATAbits.LATA7 = 1; // CS Deselected
}