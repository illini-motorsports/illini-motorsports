/******************************************************************************
 *
 *                  Telemetry Node C Main Code
 *
 ******************************************************************************
 * FileName:        Telem.c
 * Dependencies:    p18F46K80.h,
 *                  Telem.h,
 *                  timers.h,
 *                  usart.h,
 *                  ECAN.h,
 *                  crc.h
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Version:         1.00
 * Author:          George Schwieters
 ******************************************************************************
 * Revision History
 *
 *
 *******************************************************************************/

/***********************************************/
/*  Header Files                               */
/***********************************************/

#include "p18F46K80.h"
#include "Telem.h"
#include "timers.h"
#include "usart.h"
#include "ECAN.h"
#include "crc.h"


/***********************************************/
/* PIC18F46K80 Configuration Bits Settings     */
/***********************************************/

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


/***********************************************/
/* Global Variable Declarations                */
/***********************************************/

volatile int millis; // holds timer0 rollover count
volatile int lastMsgTime = 0; //time of the last message sent by the CAN

unsigned long msg_counter;
unsigned long crc32;
BYTE msg[4 /* header */ + NUM_MSG * 2 /* msg bytes */ + 4 /* msg counter */ + 1 /* message type */ + 4 /* CRC32 */];
BYTE chan_data[NUM_MSG * 2];
BYTE chan_addr[NUM_MSG];

// ECAN variables
unsigned long id; // holds CAN msgID
BYTE data[8]; // holds CAN data bytes
BYTE dataLen; // holds number of CAN data bytes
ECAN_RX_MSG_FLAGS flags; // holds information about recieved message


/***********************************************/
/* Interrupts                                  */
/***********************************************/

/*********************************************************************************
  Interrupt Function:
    void high_isr(void)
  Summary:
    Function to service interrupts
  Conditions:
    Timer0 module must be setup along with the oscillator being used
    ECAN must be configured
  Input:
    none
  Return Values:
    none
  Side Effects:
    Reloads the timer0 registers and increments millis
    Resets the interrupt flags after servicing them
  Description:

 *********************************************************************************/
#pragma code high_vector = 0x08

void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr

void high_isr(void) {

    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        WriteTimer0(0x85); // Load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }
    

    // check for recieved CAN message
    if(PIR5bits.RXB1IF) {
        lastMsgTime = millis; //set time of CAN message to current time
        PIR5bits.RXB1IF = FALSE; // reset the flag
        // get data from recieve buffer
        ECANReceiveMessage(&id, data, &dataLen, &flags);
        bufferData(); // put data in an array
    }

    //if more than 500 ms have passed then send error message
    if(millis - lastMsgTime > 500) {
        if(millis % ERROR_PER == 0)
            send_msg(ERROR,0);
    }
    else
    {
        if(millis % ADDR_PER == 0)
            send_msg(ADDR,-1);
        if(millis % DATA_PER == 0)
            send_msg(DATA,-1);
    }

    return;
}


/***********************************************/
/*  Main Program                               */

/***********************************************/

void main(void) {

    /*************************
     * Variable Declarations *
     *************************/

    unsigned long test;
    int index;
    char test_msg[10] = "123456789";
    msg_counter = 0;

    // setup channel identifiers
    chan_addr[OIL_T / 2] = OT;
    chan_addr[ENGINE_T / 2] = ET;
    chan_addr[VOLTAGE / 2] = VOLT;
    chan_addr[OIL_P / 2] = OP;
    chan_addr[SPEED / 2] = SPD;
    chan_addr[RPM / 2] = TACH;

    
    for(index = 0; index < NUM_MSG * 2; index++)
    {
        chan_data[index] = 0;
    }
    /*************************
     * Oscillator Set-Up     *
     *************************/

#ifdef INTERNAL
    // OSCTUNE
    OSCTUNEbits.INTSRC = 0; // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
    OSCTUNEbits.PLLEN = 1; // Frequency Multiplier PLL Select (1 to enable)
    OSCTUNEbits.TUN5 = 0; // Fast RC Oscillator Frequency Tuning (seems to be 2's comp encoding)
    OSCTUNEbits.TUN4 = 0; // 011111 = max
    OSCTUNEbits.TUN3 = 0; // ... 000001
    OSCTUNEbits.TUN2 = 0; // 000000 = center (running at calibrated frequency)
    OSCTUNEbits.TUN1 = 0; // 111111 ...
    OSCTUNEbits.TUN0 = 0; // 100000

    // OSCCCON
    OSCCONbits.IDLEN = 1; // Idle Enable Bit (1 to enter idle mode after SLEEP instruction else sleep mode is entered)
    OSCCONbits.IRCF2 = 1; // Internal Oscillator Frequency Select Bits
    OSCCONbits.IRCF1 = 1; // When using HF, settings are:
    OSCCONbits.IRCF0 = 1; // 111 - 16 MHz, 110 - 8MHz (default), 101 - 4MHz, 100 - 2 MHz, 011 - 1 MHz
    OSCCONbits.SCS1 = 0;
    OSCCONbits.SCS0 = 0;

    // OSCCON2
    OSCCON2bits.MFIOSEL = 0;

    while(!OSCCONbits.HFIOFS); // Wait for stable clock

#else
    // OSCTUNE
    OSCTUNEbits.INTSRC = 0; // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
    OSCTUNEbits.PLLEN = 1; // Frequency Multiplier PLL Select (1 to enable)

    // OSCCCON
    OSCCONbits.SCS1 = 0; // Select configuration chosen oscillator
    OSCCONbits.SCS0 = 0; // SCS = 00

    // OSCCON2
    OSCCON2bits.MFIOSEL = 0;

    while(!OSCCONbits.OSTS); // Wait for stable external clock
#endif

    /*************************
     * Peripherals Setup     *
     *************************/

    ANCON0 = 0x00; // Default all pins to digital
    ANCON1 = 0x00; // Default all pins to digital

    // Turn on and configure the TIMER1 oscillator
    OpenTimer0(TIMER_INT_ON & T0_8BIT & T0_SOURCE_INT & T0_PS_1_128);
    WriteTimer0(0x82); // Load timer register
    millis = 0; // Clear milliseconds count
    INTCONbits.TMR0IE = 1; // Turn on timer0 interupts

    Open2USART(USART_TX_INT_OFF & USART_RX_INT_ON & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 1708);
    baud2USART(BAUD_16_BIT_RATE & BAUD_AUTO_OFF & BAUD_IDLE_CLK_LOW & BAUD_WAKEUP_OFF);
    BAUDCON2bits.RXDTP = 0; // do not invert the recieved bits (0 to not invert)
    BAUDCON2bits.TXCKP = 0; // do not invert the transmitted bits

    TRISCbits.TRISC6 = OUTPUT; // programmable termination
    TERM_LAT = TRUE;

    ECANInitialize(); // setup ECAN

    // interrupts setup
    INTCONbits.GIE = 1; // Global Interrupt Enable (1 enables)
    INTCONbits.PEIE = 1; // Peripheral Interrupt Enable (1 enables)
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)

    /***************end setup; begin main loop************************************/

    while(1) {
        test = crcFast((unsigned char *) test_msg, 9);
    }

    return;
}

/***********************************************/
/* Functions                                   */
/***********************************************/

/*********************************************************************************
  Function:
    void write(BYTE data)
  Summary:
    Function to send out a byte on the USART module
  Conditions:
    USART must be configured
  Input:
    data - data to transmit
  Return Values:
    none
  Side Effects:
    none
  Description:

 *********************************************************************************/
void write(BYTE data) {

    while(Busy2USART());

    Write2USART(data);

    return;
}

/*********************************************************************************
  Function:
    void send_msg(BYTE type)
  Summary:
    Function to collect data for the message before sending out to the XBee
  Conditions:
    none
  Input:
    type - whether we're sending out data or addresses
  Return Values:
    none
  Side Effects:
    msg is written to
    message count is incremented
  Description:

 *********************************************************************************/

void send_msg(BYTE type,unsigned int errorType) {

    BYTE i;

    // put message header in
    msg[0] = 0x80;
    msg[1] = 0x81;
    msg[2] = 0x82;
    if(type == DATA)
        msg[3] = NUM_MSG * 2;
    else if(type == ADDR)
        msg[3] = NUM_MSG;
    else if(type == ERROR)
        msg[3] = 1;
    // send message bytes
    if(type == DATA) {
        // add data now
        for(i = 0; i < msg[3]; i++) {
            msg[i + 4] = chan_data[i];
        }
    } else if(type == ADDR) {
        // add addresses now
        for(i = 0; i < msg[3]; i++) {
            msg[i + 4] = chan_addr[i];
        }
    } else if(type == ERROR) {
        if(errorType == SIGNAL)
        {
            for(i = 0; i < msg[3]; i++)
                msg[i + 4] = 0x00;
        }
        else if(errorType == CAN)
        {
            for(i = 0; i < msg[3]; i++)
                msg[i + 4] = 0xFF;
        }
    }
    // add message counter bytes
    msg[4 + msg[3] + 0] = ((BYTE*) & msg_counter)[3];
    msg[4 + msg[3] + 1] = ((BYTE*) & msg_counter)[2];
    msg[4 + msg[3] + 2] = ((BYTE*) & msg_counter)[1];
    msg[4 + msg[3] + 3] = ((BYTE*) & msg_counter)[0];
    // add message type
    msg[4 + msg[3] + 4] = type;
    // calculate the CRC32 and append to the end of the message
    crc32 = crcFast(msg, 4 + msg[3] + 4 + 1);
    msg[4 + msg[3] + 5 + 0] = ((BYTE*) & crc32)[3];
    msg[4 + msg[3] + 5 + 1] = ((BYTE*) & crc32)[2];
    msg[4 + msg[3] + 5 + 2] = ((BYTE*) & crc32)[1];
    msg[4 + msg[3] + 5 + 3] = ((BYTE*) & crc32)[0];
    // transmit the message now
    send(4 /* header */ + msg[3] /* msg bytes */ + 4 /* msg counter */ + 1 /* message type */ + 4 /* CRC32 */);

    msg_counter++;

    return;
}

/*********************************************************************************
  Function:
    void send(BYTE num)
  Summary:
    Function to send a message byte by byte to the XBee
  Conditions:
    USART must be configured
  Input:
    num - the number of bytes to transmit to the XBee
  Return Values:
    nono
  Side Effects:
    none
  Description:

 *********************************************************************************/

void send(BYTE num) {

    BYTE i;
    // send each byte of the message till we write the prescribed number of bytes
    for(i = 0; i < num; i++) {
        write(msg[i]);
    }

    return;
}

/*********************************************************************************
  Function:
    void bufferData()
  Summary:
    Function to take data from CAN messages and put into chan_data
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    chan_data is written to
  Description:

 *********************************************************************************/

void bufferData() {

    // collect the engine sensor data and place in the chan_data array
    if(id == ECU_ID_0) {
        chan_data[RPM] = data[RPM_BYTE];
        chan_data[RPM + 1] = data[RPM_BYTE + 1];
        chan_data[OIL_P] = data[OIL_P_BYTE];
        chan_data[OIL_P + 1] = data[OIL_P_BYTE + 1];
        chan_data[OIL_T] = data[OIL_T_BYTE];
        chan_data[OIL_T + 1] = data[OIL_T_BYTE + 1];
    } else if(id == ECU_ID_1) {
        chan_data[VOLTAGE] = data[VOLTAGE_BYTE];
        chan_data[VOLTAGE + 1] = data[VOLTAGE_BYTE + 1];
        chan_data[ENGINE_T] = data[ENGINE_T_BYTE];
        chan_data[ENGINE_T + 1] = data[ENGINE_T_BYTE + 1];
    } else if(id == ECU_ID_4) {
        chan_data[SPEED] = data[GDN_SPD_BYTE];
        chan_data[SPEED + 1] = data[GDN_SPD_BYTE + 1];
    }

    return;
}
