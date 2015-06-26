/******************************************************************************
 *
 *              MDD File I/O Main Code
 *
 ******************************************************************************
 * FileName:        DAQ_Demo.c
 * Dependencies:    FSIO.h, SD-SPI.h, ECAN_logging_config.h, DAQ.h, p18F46K80.h,
                    timers.h and capture.h
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Version:         demo

*******************************************************************************
    USER REVISON HISTORY

//
// 03/08/13 - intial release with various compilier optimizations (consts, statics etc.)
//          and optimized coding i.e. unsigned whenever possible and 8-bit variables
//          as much as possible etc. Also added rough documentation.
//
//

*******************************************************************************/


/***********************************************/
/*  Header Files                               */
/***********************************************/

#include "FSIO.h"
#include "SD-SPI.h"
#include "ECAN.h"
#include "ECAN_logging_config.h"
#include "p18F46K80.h"
#include "timers.h"
#include "capture.h"
#include "DAQ.h"


/***********************************************/
/*  PIC18F46K80 Configuration Bits Settings    */
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
/*  Global Variable Declarations               */
/***********************************************/

// Data buffers A and B for writing
#pragma udata large_udataA = 0x700      // this specifies the location of the buffers in memory
volatile BYTE WriteBufferA[BUFF_SIZE];  // create buffer using defined buffer size
#pragma udata large_udataB = 0x900
volatile BYTE WriteBufferB[BUFF_SIZE];
#pragma udata

// volatile indicates that the value changes within interrupts and is read outside
// of the ISR so cached values of the variable cannot be used reliably
volatile MAIN_BITS MAIN;            // struct that holds bits used for flags and other stuff
volatile BUFF_HOLDER Buff;          // struct used for holding the buffer pointers
static unsigned int timestamp[4];           // holds CCP2 timestamps
static unsigned int dropped;                // keep track of number of dropped messages
static unsigned int seconds;                // holds timer1 rollover count


/***********************************************/
/*  Interrupts                                 */
/***********************************************/

/*********************************************************************************
  Interrupt Function:
    void high_isr(void)
  Summary:
    Service all interrupt flags: timer1 rollover, FIFO WM and CCP2 trigger
  Conditions:
    * Timer1 module must be setup along with the oscillator being used
    * ECAN must be configured
    * CCP2 must be configure for CAN capture functionality
  Input:
    none
  Return Values:
    none
  Side Effects:
    Resets the interrupt flags after servicing them
    Reloads the timer1 registers, sets pending flag and increments seconds when rollover occurs
    Writes timestamps
    Clears timestamp index and decrements seconds variable when servicing the FIFO buffer
    Swaps buffers and buffer lengths when swap flag is set
  Description:

  *********************************************************************************/
#pragma code high_vector = 0x08
void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

#pragma interrupt high_isr
void high_isr (void) {

    // check for incoming message to give timestamp
    if(PIR3bits.CCP2IF) {
        PIR3bits.CCP2IF = 0;
        // read CAN capture register and place in timestamp array
        // keeping track of how many are in the array
        timestamp[MAIN.MsgNum++] = ReadCapture2();
    }

    // service FIFO RX buffers
    if (PIR5bits.FIFOWMIF) {
        PIR5bits.FIFOWMIF = 0;
        MAIN.MsgNum = 0;

        // check if the seconds variable has been incremented
        if (MAIN.Pending) {
            // decrement seconds for now; during data processing later
            // it will be determined when to use the current value
            seconds--;
        }

        // check swap flag
        if (MAIN.Swap) {

            MAIN.BufferALen = 0;            // reset buffer A,
            MAIN.BufferAFull = FALSE;       // clear full and
            MAIN.Swap = FALSE;              // swap flag

            swap_len();
            swap_buff();
        }

        // process data in CAN FIFO
        service_FIFO();
    }

    // check for timer1 rollover
    if (PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;
        // load timer value such that the most signifcant bit is set so it
        // takes exactly one second for a 32.768kHz crystal to trigger a rollover interrupt
        WriteTimer1(0x8000);
        seconds++;
        // indicate that seconds was just incremented for correct timestamp creation during data processing
        MAIN.Pending = TRUE;
    }

    return;
}

/***********************************************/
/*  Main Program                               */
/***********************************************/

void main (void) {

    /*************************
     * Varaible Declarations *
     *************************/
    BUFF_HOLDER * Buff_p = &Buff;   // pointer to buffer stuct
    FSFILE * pointer;               // pointer to open file
    char fname[13] = "0000.CSV";    // holds name of file
    const char write = 'w';         // for opening file (must use variable for passing value in PIC18 when not using pgm function)
    SearchRec rec;                  // holds search parameters and found file info
    const unsigned char attributes  // holds search parameters
        = ATTR_ARCHIVE
        | ATTR_READ_ONLY
        | ATTR_HIDDEN;


    /*********************
     * Oscillator Set-Up *
     *********************/
#ifdef INTERNAL
    // OSCTUNE
    OSCTUNEbits.INTSRC = 0;     // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
    OSCTUNEbits.PLLEN = 1;      // Frequency Multiplier PLL Select (1 to enable)
    OSCTUNEbits.TUN5 = 0;       // Fast RC Oscillator Frequency Tuning (seems to be 2's comp encoding)
    OSCTUNEbits.TUN4 = 0;       // 011111 = max
    OSCTUNEbits.TUN3 = 0;       // ... 000001
    OSCTUNEbits.TUN2 = 0;       // 000000 = center (running at calibrated frequency)
    OSCTUNEbits.TUN1 = 0;       // 111111 ...
    OSCTUNEbits.TUN0 = 0;       // 100000

    // OSCCCON
    OSCCONbits.IDLEN = 1;       // Idle Enable Bit (1 to enter idle mode after SLEEP instruction else sleep mode is entered)
    OSCCONbits.IRCF2 = 1;       // Internal Oscillator Frequency Select Bits
    OSCCONbits.IRCF1 = 1;       // When using HF, settings are:
    OSCCONbits.IRCF0 = 1;       // 111 - 16 MHz, 110 - 8MHz (default), 101 - 4MHz, 100 - 2 MHz, 011 - 1 MHz
    OSCCONbits.SCS1 = 0;
    OSCCONbits.SCS0 = 0;

    // OSCCON2
    OSCCON2bits.MFIOSEL = 0;

    while(!OSCCONbits.HFIOFS);  // wait for stable clock

#else
    // OSCTUNE
    OSCTUNEbits.INTSRC = 0;     // Internal Oscillator Low-Frequency Source Select (1 for 31.25 kHz from 16MHz/512 or 0 for internal 31kHz)
    OSCTUNEbits.PLLEN = 1;      // Frequency Multiplier PLL Select (1 to enable)

    // OSCCCON
    OSCCONbits.SCS1 = 0;        // select configuration chosen oscillator
    OSCCONbits.SCS0 = 0;        // SCS = 00

    // OSCCON2
    OSCCON2bits.MFIOSEL = 0;

    while(!OSCCONbits.OSTS);    // wait for stable external clock
#endif

    /*********************
     * Pin Configuration *
     *********************/
    ANCON0 = 0x00;              // default all pins to digital
    ANCON1 = 0x00;              // default all pins to digital

    SD_CS_TRIS = OUTPUT;        // set card select pin to output
    SD_CS = 1;                  // card deselected

    TRISDbits.TRISD1 = INPUT;   // debugging logging control

    init_unused_pins();         // assert values to unused pins

    /***************************
     * Interrupt/Timer Set-Up  *
     ***************************/
    // turn on and configure the TIMER1 oscillator
    OpenTimer1(T1_16BIT_RW & T1_SOURCE_PINOSC & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_ON, 0x00);
    seconds = 0;            // clear seconds
    WriteTimer1(0x8000);    // load timer registers

    // turn on and configure capture module
    OpenCapture2(CAP_EVERY_FALL_EDGE & CAPTURE_INT_ON);

    // RCON
    RCONbits.IPEN = 0;          // Interrupt Priority Enable (1 enables)

    /****************
     * Libary Setup *
     ****************/
    while(!MDD_MediaDetect());  // wait for card presence
    while(!FSInit());           // setup file system library

    // initialize stuff but don't allow data collection yet
    // since file creation can take a while
    // and buffers will just fill up immediately otherwise
    Buff.BufferA = WriteBufferA;
    Buff.BufferB = WriteBufferB;
    MAIN.BufferALen = 0x200;
    MAIN.BufferBLen = 0x200;
    MAIN.Swap = FALSE;
    MAIN.BufferAFull = TRUE;
    MAIN.MsgNum = 0;            // holds CCP2 values

    ECANInitialize();           // setup ECAN module

    // INTCON
    INTCONbits.GIE = 1;         // Global Interrupt Enable (1 enables)
    INTCONbits.PEIE = 1;        // Peripheral Interrupt Enable (1 enables)

    /***************end initialization; begin program execution***************/


    // data aq loop
    while(1) {
        // check if we want to start data logging
        if (PORTDbits.RD1) {
            // file name loop
            while(1) {
                // look for file with proposed name
                if(FindFirst(fname, attributes, &rec))
                    if(FSerror() == CE_FILE_NOT_FOUND)  // check type of error was not finding the file
                        break;                          // exit loop, file name is unique
                    else
                        funct_error();

                // change file name and retest
                if (fname[3] == '9') {
                    fname[3] = '0';     // reset first number
                    fname[2]++;         // incement other number
                }
                else
                    fname[3]++;         // increment file number
            }

            // create csv data file
            pointer = FSfopen(fname, &write);
            if(pointer == NULL)
                funct_error();

            // setup buffers and flags to begin data collection
            MAIN.Pending = FALSE;
            MAIN.BufferALen = 0x000;
            MAIN.BufferBLen = 0x000;
            MAIN.BufferAFull = FALSE;
            dropped = 0;

            // logging loop
            while(1) {
                // write buffer A to file
                if (MAIN.BufferAFull) {
                    if(FSfwrite(Buff_p, pointer, (MAIN_BITS *) &MAIN) != BUFF_SIZE)
                        funct_error();
                }

                // check if we should stop logging
                if(!PORTDbits.RD1) {
                    // close csv data file
                    if(FSfclose(pointer))
                        funct_error();

                    break;      // leave logging loop
                }
            }// logging loop
        } // data acq start control
    }// data acq loop

    return;
}


/***********************************************/
/*  User Functions                             */
/***********************************************/

/******************************************************************************
  Function:
    void funct_error(void)
  Summary:
    Place to stay when error occurs
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    none
  Description:

  ****************************************************************************/
void funct_error(void) {
    while(1);       // stay here
    return;
}

/******************************************************************************
  Function:
    void init_unused_pins(void)
  Summary:
    Function to assert unused pins to eliminate sources of noise and reduce MCU
    power draw.
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    Certain pins will be set low and configured as outputs
  Description:

  ****************************************************************************/
void init_unused_pins(void) {

    // first configure to outputs
    TRISAbits.TRISA0 = OUTPUT;
    TRISAbits.TRISA1 = OUTPUT;
    TRISAbits.TRISA2 = OUTPUT;
    TRISAbits.TRISA3 = OUTPUT;
//  TRISAbits.TRISA5 = OUTPUT; CS'
//  TRISAbits.TRISA6 = OUTPUT; OSC2
//  TRISAbits.TRISA7 = OUTPUT; OSC1

    TRISBbits.TRISB0 = OUTPUT;
    TRISBbits.TRISB1 = OUTPUT;
//  TRISBbits.TRISB2 = OUTPUT; CANTX
//  TRISBbits.TRISB3 = OUTPUT; CANRX
    TRISBbits.TRISB4 = OUTPUT;
    TRISBbits.TRISB5 = OUTPUT;
    TRISBbits.TRISB6 = OUTPUT;
    TRISBbits.TRISB7 = OUTPUT;

//  TRISCbits.TRISC0 = OUTPUT; SOSC0
//  TRISCbits.TRISC1 = OUTPUT; SOSC1
    TRISCbits.TRISC2 = OUTPUT;
//  TRISCbits.TRISC3 = OUTPUT; SCK
//  TRISCbits.TRISC4 = OUTPUT; SDI
//  TRISCbits.TRISC5 = OUTPUT; SDO
    TRISCbits.TRISC6 = OUTPUT;
    TRISCbits.TRISC7 = OUTPUT;

    TRISDbits.TRISD0 = OUTPUT;
//  TRISDbits.TRISD1 = OUTPUT; debugging logging control
    TRISDbits.TRISD2 = OUTPUT;
    TRISDbits.TRISD3 = OUTPUT;
    TRISDbits.TRISD4 = OUTPUT;
    TRISDbits.TRISD5 = OUTPUT;
    TRISDbits.TRISD6 = OUTPUT;
    TRISDbits.TRISD7 = OUTPUT;

    TRISEbits.TRISE0 = OUTPUT;
    TRISEbits.TRISE1 = OUTPUT;
    TRISEbits.TRISE2 = OUTPUT;
//  TRISEbits.TRISE3 = OUTPUT; MCLR

    // then set pins low
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
//  LATAbits.LATA5 = 0; CS'
//  LATAbits.LATA6 = 0; OSC2
//  LATAbits.LATA7 = 0; OSC1

    LATBbits.LATB0 = 0;
    LATBbits.LATB1 = 0;
//  LATBbits.LATB2 = 0; CANTX
//  LATBbits.LATB3 = 0; CANRX
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
    LATBbits.LATB6 = 0;
    LATBbits.LATB7 = 0;

//  LATCbits.LATC0 = 0; SOSC0
//  LATCbits.LATC1 = 0; SOSC1
    LATCbits.LATC2 = 0;
//  LATCbits.LATC3 = 0; SCK
//  LATCbits.LATC4 = 0; SDI
//  LATCbits.LATC5 = 0; SDO
    LATCbits.LATC6 = 0;
    LATCbits.LATC7 = 0;

    LATDbits.LATD0 = 0;
//  LATDbits.LATD1 = 0; debugging logging control
    LATDbits.LATD2 = 0;
    LATDbits.LATD3 = 0;
    LATDbits.LATD4 = 0;
    LATDbits.LATD5 = 0;
    LATDbits.LATD6 = 0;
    LATDbits.LATD7 = 0;

    LATEbits.LATE0 = 0;
    LATEbits.LATE1 = 0;
    LATEbits.LATE2 = 0;
//  LATEbits.LATE3 = 0; MCLR

    return;
}

/******************************************************************************
  Function:
    void service_FIFO(void)
  Summary:
    Function to clear the FIFO buffer of recieved messages and create a message array
    to be written.
  Conditions:
    ECAN must be configured correctly
  Input:
    none
  Return Values:
    none
  Side Effects:
    clears out the recieve buffers
  Description:

  ****************************************************************************/
void service_FIFO(void) {

    static unsigned char i;     // for loop counters
    static unsigned char j;
    static BYTE msg[14];        // holds entire CAN message including timstamp
    unsigned long id;           // holds CAN msgID
    BYTE data[8];               // holds CAN data bytes
    BYTE dataLen;               // holds number of CAN data bytes
    ECAN_RX_MSG_FLAGS flags;    // holds information about recieved message

    // loop through messages in CAN buffers
    for (i = 0; i < MSGS_READ; i++) {
        // get message from RX buffer
        ECANReceiveMessage(&id, data, &dataLen, &flags);

        // collect message data before sending to buffer
        for (j = 0; j < dataLen + MSG_ID_LEN + TIMESTAMP_LEN; j++) {
            // get message ID
            if (j < MSG_ID_LEN) {
                msg[j] = ((BYTE *) (&id))[j];
            }
            // get data bytes
            else if (j < MSG_ID_LEN + dataLen) {
                msg[j] = data[j - MSG_ID_LEN];
            }
            // get CCP2 timer1 value that forms 2 LSBs of the timestamp
            else if (j < MSG_ID_LEN + dataLen + (TIMESTAMP_LEN / 2)) {
                msg[j] = ((BYTE*) timestamp)[j - MSG_ID_LEN - dataLen + i * 2];
            }
            // get seconds value that forms 2 MSBs of the timsetamp
            else if (j < MSG_ID_LEN + dataLen + TIMESTAMP_LEN) {
                // when seconds has just been incremented check for a set most signifcant bit of the
                // captured timer1 value that would indicate the current seconds value should be
                // decremented since the rollover happened after the timer value was captured for this message
                if (MAIN.Pending && (timestamp[i] & 0x4000)) {
                    msg[j] = ((BYTE *) &seconds)[j - MSG_ID_LEN - dataLen - (TIMESTAMP_LEN / 2)];
                }
                // otherwise once we find a message without the most signifacant bit set
                // we can increment the seconds value to the current value and clear the pending flag
                else {
                    if (MAIN.Pending) {
                        seconds++;
                        MAIN.Pending = FALSE;
                    }
                    msg[j] = ((BYTE *) &seconds)[j - MSG_ID_LEN - dataLen - (TIMESTAMP_LEN / 2)];
                }
            }
        }

        // should be cleared at this point but just in case
        MAIN.Pending = FALSE;

        // send messgage to buffer
        append_write_buffer(msg, dataLen + MSG_ID_LEN + TIMESTAMP_LEN);
    }

    return;
}


/******************************************************************************
  Function:
    void append_write_buffer(static const BYTE * temp, static unsigned char applen)
  Summary:
    Function to control how data is appended to the user data buffers
  Conditions:
    none
  Input:
    temp - data array that holds one whole CAN message with its timestamp
    applen - the length of the data array
  Return Values:
    none
  Side Effects:
    Either carries out modifying buffer A, partially to both A and B, just B or neither
    which increments the variable dropped. When A fills up the flag to write is raised.
  Description:
    This function controls how data is written using buffer A as the writing buffer and
    B as a temp cache for when we pass the data in A off to the gDatabuffer that the MDD I/O
    library functions use. When B fills up we are still writing data
    and all the buffers are filled meaning further messages will be dropped.

  ****************************************************************************/
void append_write_buffer(static const BYTE * temp, static unsigned char applen) {

    static unsigned char offset = 0;
    static unsigned char holder;
    static BOOL WRITTEN = FALSE;

    // message is dropped
    if (applen > 2 * BUFF_SIZE - MAIN.BufferALen - MAIN.BufferBLen) {
        dropped++;
        return;
    }

    // try writing to buffer A first
    if (!MAIN.BufferAFull) {
        // room for all of data to write
        if (BUFF_SIZE - MAIN.BufferALen > applen) {
            WRITTEN = TRUE;
        }
        // exactly enough room for the data
        else if (BUFF_SIZE - MAIN.BufferALen == applen) {
            MAIN.BufferAFull = TRUE;
            WRITTEN = TRUE;
        }
        // not enough room for all the data
        else {
            MAIN.BufferAFull = TRUE;
            // recalculate writing parameters for partial write
            offset = applen - (BUFF_SIZE - MAIN.BufferALen);
        }
        // add message to buffer
        buff_cat(Buff.BufferA, temp, &(MAIN.BufferALen), applen - offset, 0);
    }

    // write to buffer B if couldn't write any or all data to buffer A
    if (WRITTEN == FALSE) {
        // only use offset if there has been a partial write to buffer A
        if (offset != 0) {
            holder = applen;
            applen = offset;
            offset = holder - offset;
        }
        // add message to buffer
        buff_cat(Buff.BufferB, temp, &(MAIN.BufferBLen), applen, offset);
    }

    return;
}

/******************************************************************************
  Function:
    void buff_cat(static BYTE *WriteBuffer, static const BYTE *writeData, static unsigned int *bufflen,
                 static const unsigned char applen, static const unsigned char offset)
  Summary:
    Function write data to a user data buffer
  Conditions:
    none
  Input:
    WriteBuffer - pointer to the buffer we will write to
    writedata - pointer to the data array we will write
    bufflen - pointer to the variable holding the buffer length of the buffer we are writing to
    applen - the length of the data to be written
    offset - the index offset for the data to write
  Return Values:
    none
  Side Effects:
    none
  Description:

  ****************************************************************************/
void buff_cat(static BYTE *WriteBuffer, static const BYTE *writeData, static unsigned int *bufflen, static const unsigned char applen, static const unsigned char offset) {

    static unsigned char i;

    // increment through the data bytes sending them to the write buffer
    for(i = 0; i < applen; i++) {
        WriteBuffer[*bufflen + i] = writeData[i + offset];
    }

    // increment the data length count for the writing buffer
    *bufflen += (unsigned int) applen;

    return;
}

/******************************************************************************
  Function:
    void swap_len(void)
  Summary:
    Function to swap buffer length values
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    the values for buffer length will be modified
  Description:

  ****************************************************************************/
void swap_len(void) {

    const unsigned temp = MAIN.BufferALen;

    // swap lengths
    MAIN.BufferALen = MAIN.BufferBLen;
    MAIN.BufferBLen = temp;

    return;
}

/******************************************************************************
  Function:
    void swap_buff(void)
  Summary:
    swaps the pointers to the user data buffers
  Conditions:
    none
  Input:
    none
  Return Values:
    none
  Side Effects:
    The pointers in the Buff struct are modified
  Description:

  ****************************************************************************/
void swap_buff(void) {

    const BYTE *temp = Buff.BufferA;

    // swap pointers
    Buff.BufferA = Buff.BufferB;
    Buff.BufferB = temp;

    return;
}
