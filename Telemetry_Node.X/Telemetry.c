/**
 * Telemetry
 *
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Author:          George Schwieters
 * Author:          Andrew Mass
 * Created:         2013-2014
 */

#include "p18F46K80.h"
#include "FSAE.h"
#include "Telemetry.h"
#include "timers.h"
#include "usart.h"
#include "ECAN.h"
#include "CAN.h"

/**
 * PIC18F46K80 Configuration Bits Settings
 */

// CONFIG1L
#pragma config RETEN = OFF      // VREG Sleep Enable bit (Ultra low-power regulator is Disabled (Controlled by REGSLP bit))
#pragma config INTOSCSEL = HIGH // LF-INTOSC Low-power Enable bit (LF-INTOSC in High-power mode during Sleep)
#pragma config SOSCSEL = HIGH   // SOSC Power Selection and mode Configuration bits (High Power SOSC circuit selected)
#pragma config XINST = OFF      // Extended Instruction Set (Disabled)

// CONFIG1H
#ifdef INTERNAL
    #pragma config FOSC = INTIO2    // Oscillator (Internal RC oscillator)
#else
    #pragma config FOSC = HS1       // Oscillator (HS oscillator (Medium power, 4 MHz - 16 MHz))
#endif

#pragma config PLLCFG = ON      // PLL x4 Enable bit (Enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Disabled)
#pragma config IESO = ON        // Internal External Oscillator Switch Over Mode (Enabled)

// CONFIG2L
#pragma config PWRTEN = OFF      // Power Up Timer (Disabled)
#pragma config BOREN = OFF      // Brown Out Detect (Disabled in hardware, SBOREN disabled)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (1.8V)
#pragma config BORPWR = ZPBORMV // BORMV Power level (ZPBORMV instead of BORMV is selected)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WDTPS = 1048576  // Watchdog Postscaler (1:1048576)

// CONFIG3H
#pragma config CANMX = PORTB    // ECAN Mux bit (ECAN TX and RX pins are located on RB2 and RB3, respectively)
#pragma config MSSPMSK = MSK7   // MSSP address masking (7 Bit address masking mode)
#pragma config MCLRE = ON       // Master Clear Enable (MCLR Enabled, RE3 Disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Overflow Reset (Enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size (2K word Boot Block size)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protect 00800-03FFF (Disabled)
#pragma config CP1 = OFF        // Code Protect 04000-07FFF (Disabled)
#pragma config CP2 = OFF        // Code Protect 08000-0BFFF (Disabled)
#pragma config CP3 = OFF        // Code Protect 0C000-0FFFF (Disabled)

// CONFIG5H
#pragma config CPB = OFF        // Code Protect Boot (Disabled)
#pragma config CPD = OFF        // Data EE Read Protect (Disabled)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protect 00800-03FFF (Disabled)
#pragma config WRT1 = OFF       // Table Write Protect 04000-07FFF (Disabled)
#pragma config WRT2 = OFF       // Table Write Protect 08000-0BFFF (Disabled)
#pragma config WRT3 = OFF       // Table Write Protect 0C000-0FFFF (Disabled)

// CONFIG6H
#pragma config WRTC = OFF       // Config. Write Protect (Disabled)
#pragma config WRTB = OFF       // Table Write Protect Boot (Disabled)
#pragma config WRTD = OFF       // Data EE Write Protect (Disabled)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protect 00800-03FFF (Disabled)
#pragma config EBTR1 = OFF      // Table Read Protect 04000-07FFF (Disabled)
#pragma config EBTR2 = OFF      // Table Read Protect 08000-0BFFF (Disabled)
#pragma config EBTR3 = OFF      // Table Read Protect 0C000-0FFFF (Disabled)

// CONFIG7H
#pragma config EBTRB = OFF      // Table Read Protect Boot (Disabled)

/**
 * Global Variable Declarations
 */

static volatile unsigned long millis = 0; // Holds timer0 rollover count

static unsigned char msg[4 + 14]; // 4-Header, 12-Msg bytes
static unsigned char chan_data[14];

// ECAN variables
static unsigned long id; // Holds CAN msgID
static unsigned char data[8]; // Holds CAN data bytes
static unsigned char dataLen; // Holds number of CAN data bytes
static ECAN_RX_MSG_FLAGS flags; // Holds information about received message

/**
 * Interrupts
 */

#pragma code high_vector = 0x08
void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code
#pragma interrupt high_isr

/**
 * void high_isr(void)
 * Description: Function to service interrupts
 * Conditions: Timer0 module must be setup along with the oscillator being used
 *             ECAN must be configured
 * Input: none
 * Return Values: none
 * Side Effects: Reloads the timer0 registers and increments millis
 *               Resets the interrupt flags after servicing them
 */
void high_isr(void) {
    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(0x85); // Load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // Check for recieved CAN message
    if(PIR5bits.RXB1IF) {
        PIR5bits.RXB1IF = 0; // Reset the flag

        // Get data from receive buffer
        ECANReceiveMessage(&id, data, &dataLen, &flags);
        bufferData(); // Put data in an array
    }

    if(millis % DATA_PER == 0) {
        send_msg();
    }
}

void main(void) {

    /**
     * Variable Declarations
     */

    int index;
    for(index = 0; index < 14; index++) {
        chan_data[index] = 0;
    }

    millis = 0; // Clear milliseconds count

    TRISCbits.TRISC6 = OUTPUT; // Programmable termination
    TERM_LAT = 0; // Not terminating

    init_unused_pins();

    /**
     * Peripheral Initialization
     */

    init_oscillator();

    ANCON0 = 0x00; // Default all pins to digital
    ANCON1 = 0x00; // Default all pins to digital

    init_timer0();

    // USART Setup
    Open2USART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 1708);
    baud2USART(BAUD_16_BIT_RATE & BAUD_AUTO_OFF & BAUD_IDLE_CLK_LOW & BAUD_WAKEUP_OFF);
    BAUDCON2bits.RXDTP = 0; // Do not invert the recieved bits (0 to not invert)
    BAUDCON2bits.TXCKP = 0; // Do not invert the transmitted bits

    // Setup ECAN
    ECANInitialize();

    // Interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    /**
     * Main loop
     */
    while(1);
}

/**
 * Local Functions
 */

/**
 * void write(BYTE data)
 * Description: Function to send out a byte on the USART module
 * Conditions: USART must be configured
 * Input: data - data to transmit
 * Return Values: none
 * Side Effects: none
 */
void write(BYTE data) {
    while(Busy2USART());
    Write2USART(data);
}

/**
 * void send_msg(BYTE type)
 * Description: Function to collect data for the message before sending out to the XBee
 * Conditions: none
 * Input: type - whether we're sending out data or addresses
 * Return Values: none
 * Side Effects: msg is written to. Message count is incremented.
 */
void send_msg() {
    unsigned char i;

    // Put message header in
    msg[0] = 0x80;
    msg[1] = 0x81;
    msg[2] = 0x82;
    msg[3] = 0x83;

    // Add data now
    for(i = 0; i < 14; i++) {
        msg[i + 4] = chan_data[i];
    }

    /**
     * Transmit the message now
     *
     * 4 - Header
     * 12 - Msg bytes
     */
    send(4 + 14);
}

/**
 * void send(unsigned char num)
 *
 * Description: Function to send a message byte by byte to the XBee
 * Conditions: USART must be configured
 * Input: num - the number of bytes to transmit to the XBee
 * Return Values: nono
 * Side Effects: none
 */
void send(unsigned char num) {
    unsigned char i;
    // Send each byte of the message till we write the prescribed number of bytes
    for(i = 0; i < num; i++) {
        write(msg[i]);
    }
}

/**
 * void bufferData()
 *
 * Description: Function to take data from CAN messages and put into chan_data
 * Conditions: none
 * Input: none
 * Return Values: none
 * Side Effects: Chan_data is written to.
 */
void bufferData() {
    // Collect the engine sensor data and place in the chan_data array
    if(id == MOTEC_ID) {
        chan_data[RPM] = data[RPM_BYTE];
        chan_data[RPM + 1] = data[RPM_BYTE + 1];
        chan_data[OIL_P] = data[OIL_P_BYTE];
        chan_data[OIL_P + 1] = data[OIL_P_BYTE + 1];
        chan_data[OIL_T] = data[OIL_T_BYTE];
        chan_data[OIL_T + 1] = data[OIL_T_BYTE + 1];
    } else if(id == MOTEC_ID + 1) {
        chan_data[VOLTAGE] = data[VOLTAGE_BYTE];
        chan_data[VOLTAGE + 1] = data[VOLTAGE_BYTE + 1];
        chan_data[ENGINE_T] = data[ENGINE_T_BYTE];
        chan_data[ENGINE_T + 1] = data[ENGINE_T_BYTE + 1];
        chan_data[LAMBDA] = data[LAMBDA_BYTE];
        chan_data[LAMBDA + 1] = data[LAMBDA_BYTE + 1];
    } else if(id == MOTEC_ID + 4) {
        chan_data[SPEED] = data[GDN_SPD_BYTE];
        chan_data[SPEED + 1] = data[GDN_SPD_BYTE + 1];
    }
}
