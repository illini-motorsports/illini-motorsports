/*
 * Analog Node Main File
 *
 * File Name:       CANHub.c
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18
 * Version:         2.00
 * Author:          George Schwieters
 * Created:         2012-2013
 */

#include "ECAN.h"
#include "CANHub.h"
#include "FSAE.h"


/*
 *  PIC18F46K80 Configuration Bits
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
#pragma config PWRTEN = OFF     // Power Up Timer (Disabled)
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


/*
 * Global Variables
 */

static volatile int millis; // millisecond count

#ifdef FRONT
static volatile unsigned char RADIO_SW;
// ECAN variables
static unsigned long id; // holds CAN msgID
static unsigned char data[8]; // holds CAN data bytes
static unsigned char dataLen; // holds number of CAN data bytes
static ECAN_RX_MSG_FLAGS flags; // holds information about received message
#endif


/*
 * Interrupts
 */

#pragma code high_vector = 0x08

void high_vector(void) {
    _asm goto high_isr _endasm
}
#pragma code

/*
 *  void high_isr(void)
 *
 *  Description:    This interrupt will service all high priority interrupts. This
 *                  section should be as short as possible.
 *  Input(s): none
 *  Return Value(s): none
 *  Side Effects:   This will modify INTCON, TMR0L & PIR5. Also it modifies the ECAN
 *                  global variables along with the millis, and RADIO_SW variables.
 */
#pragma interrupt high_isr

void high_isr(void) {

    unsigned char data[8]; // holds CAN data bytes

    // check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // load timer registers (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

#ifdef FRONT
    // check for received CAN message
    if(PIR5bits.RXB1IF) {
        // reset the flag
        PIR5bits.RXB1IF = FALSE;
        ECANReceiveMessage(&id, data, &dataLen, &flags);
        if(id == RADIO_SW_ID) {
            if(data[0] == RADIO_SW_BYTE0_ID)
                RADIO_SW = data[RADIO_SW_BYTE];
        }
    }
#endif

#if (FAST_NUM != 0)
    // wait until millisecond count is a multiple of the sample parameter for
    // high frequency sampled sensors
    if(!(millis % FAST_SAMPLE)) {

        // sample suspension pots
        sample(data, SUS_L_BYTE, SUS_L);
        sample(data, SUS_R_BYTE, SUS_R);

        // send the sampled values out on CAN
        ECANSendMessage(FAST_ID, data, FAST_NUM * 2, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_3);
    }
#endif

#if (SLOW_NUM != 0)
    // wait until millisecond count is a multiple of the sample parameter for
    // low frequency sampled sensors
    if(!(millis % SLOW_SAMPLE)) {

    #ifdef FRONT
        // sample the sensors
        sample(data, BRAKE_R_P_BYTE, BRAKE_R_P);
        sample(data, BRAKE_F_P_BYTE, BRAKE_F_P);
    #endif
        // send the sampled values out on CAN
        ECANSendMessage(SLOW_ID, data, SLOW_NUM * 2, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_2);
    }
#endif

    return;
}

void main(void) {

    /*
     * Variable Declarations
     */
#ifdef MOTEC_RESEND
    unsigned long id; // holds CAN msgID
    unsigned char data_r[8]; // holds CAN data bytes
    unsigned char dataLen; // holds number of CAN data bytes
    ECAN_RX_MSG_FLAGS flags; // holds information about received message
    FLAGS Recieved;
    unsigned char msg[8];
    unsigned int adl_tmr;
#endif

    init_unused_pins(); // assert values to unused pins

    /*
     * Variable Initialization
     */

    TRISCbits.TRISC6 = OUTPUT; // programmable termination
    TERM_LAT = TRUE;

#ifdef MOTEC_RESEND
    // initialize ADL message number
    msg[0] = 0x02;
    msg[1] = 0x00;
    adl_tmr = 0;
#endif
#ifdef FRONT
    RADIO_SW = FALSE;
    RADIO_TRIS_1 = OUTPUT;
    RADIO_TRIS_0 = OUTPUT;
#endif

    /*
     * Peripheral Initialization
     */

    // can use internal or external
    init_oscillator();

    // setup milliseconds interrupt
    init_timer0();

    // turn on and configure the A/D converter module
    init_ADC();

#ifdef FRONT
    ANCON0 = 0b00001111; // AN0 - AN3 are analog
    ANCON1 = 0b00000000; // rest are digital
    TRISAbits.TRISA0 = INPUT; // AN0
    TRISAbits.TRISA1 = INPUT; // AN1
    TRISAbits.TRISA2 = INPUT; // AN2
    TRISAbits.TRISA3 = INPUT; // AN3
    TRISAbits.TRISA5 = OUTPUT; // AN4
    LATAbits.LATA5 = 0;
    TRISEbits.TRISE0 = OUTPUT; // AN5
    LATEbits.LATE0 = 0;
    TRISEbits.TRISE1 = OUTPUT; // AN6
    LATEbits.LATE1 = 0;
    TRISEbits.TRISE2 = OUTPUT; // AN7
    LATEbits.LATE2 = 0;
    TRISBbits.TRISB1 = OUTPUT; // AN8
    LATBbits.LATB1 = 0;
    TRISBbits.TRISB4 = OUTPUT; // AN9
    LATBbits.LATB4 = 0;
    TRISBbits.TRISB0 = OUTPUT; // AN10
    LATBbits.LATB0 = 0;

#elif REAR
    ANCON0 = 0b00000110; // AN1 - AN2 are analog
    ANCON1 = 0b00000000; // rest are digital
    TRISAbits.TRISA0 = OUTPUT; // AN0
    LATAbits.LATA0 = 0;
    TRISAbits.TRISA1 = INPUT; // AN1
    TRISAbits.TRISA2 = INPUT; // AN2
    TRISAbits.TRISA3 = OUTPUT; // AN3
    LATAbits.LATA3 = 0;
    TRISAbits.TRISA5 = OUTPUT; // AN4
    LATAbits.LATA5 = 0;
    TRISEbits.TRISE0 = OUTPUT; // AN5
    LATEbits.LATE0 = 0;
    TRISEbits.TRISE1 = OUTPUT; // AN6
    LATEbits.LATE1 = 0;
    TRISEbits.TRISE2 = OUTPUT; // AN7
    LATEbits.LATE2 = 0;
    TRISBbits.TRISB1 = OUTPUT; // AN8
    LATBbits.LATB1 = 0;
    TRISBbits.TRISB4 = OUTPUT; // AN9
    LATBbits.LATB4 = 0;
    TRISBbits.TRISB0 = OUTPUT; // AN10
    LATBbits.LATB0 = 0;
#endif

    ECANInitialize(); // setup ECAN

    // interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    /***************end setup; begin main loop************************************/

    // all A/D operations are dealt with in the ISR
    // that's triggered by the 1 ms rollover timer
    while(1) {
#ifdef MOTEC_RESEND
        // poll for an accelerometer message (other messages are filtered out)
        while(!ECANReceiveMessage(&id, data_r, &dataLen, &flags));

        // check which accelerometer message was received
        if(id == Y_ID && !Recieved.Y_accel) {
            // process CAN bus data and put in ADL format
            process_resend(data_r, msg, Y_BYTE, Y_OFFSET, ADL7_BYTE, INTEL);
            Recieved.Y_accel = TRUE;
        } else if(id == X_ID && !Recieved.X_accel) {
            // process CAN bus data and put in ADL format
            process_resend(data_r, msg, X_BYTE, X_OFFSET, ADL8_BYTE, INTEL);
            Recieved.X_accel = TRUE;
        }

        // resend out data
        if(Recieved.X_accel && Recieved.Y_accel) {
            if(millis - adl_tmr > ADL_SAMPLE) {
                adl_tmr = millis;
                ECANSendMessage(ADL_ID, msg, ADL_DLC, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            }
            Recieved.X_accel = FALSE;
            Recieved.Y_accel = FALSE;
        }
#endif
#ifdef FRONT
        if(RADIO_SW) {
            RADIO_TRIS_0 = OUTPUT;
            RADIO_TRIS_1 = OUTPUT;
            RADIO_LAT_0 = FALSE;
            RADIO_LAT_1 = FALSE;
        } else {
            RADIO_TRIS_0 = INPUT;
            RADIO_TRIS_1 = INPUT;
        }
#endif
    }

    return;
}

/*
 *  Local Functions
 */

/*
 *  void sample(unsigned char *data, const unsigned char byte, const unsigned char ch)
 *
 *  Description:    This reads the analog voltage of a pin and then puts the value into the
 *                  data array that will be transmitted over CAN.
 *  Input(s):   data - pointer to array of data bytes
 *              ch - which pin to sample
 *              byte - where to write the data in the passed array
 *  Return Value(s): none
 *  Side Effects: This modifies the memory pointed to by data.
 */
void sample(unsigned char *data, const unsigned char byte, const unsigned char ch) {

    SelChanConvADC(ch); // configure which pin you want to read and start A/D converter

    while(BusyADC()); // wait for complete conversion

    // put result in data array in accordance with specified byte location
    ((unsigned int *) data)[byte / 2] = (unsigned int) ReadADC();

    return;
}

/*
 *  void sample(unsigned char *data, const unsigned char byte, const unsigned char ch)
 *
 *  Description:    This takes in incoming data and reformats it to be read by Motec
 *                  as an ADL CAN message.
 *  Input(s):   data - the data bytes from the received CAN message
 *              msg - the data bytes to be transmitted
 *              byte - the location of the received data within the received message
 *              offset - the offset to apply to the incoming data before getting transmitted
 *              ADL_ch - the ADL channel that we want the transmitted data to placed in
 *              order - the byte order of the incoming data (either INTEL or MOTOROLA)
 *  Return Value(s): none
 *  Side Effects: This modifies the memory pointed to by msg.
 */
void process_resend(const unsigned char *data, unsigned char *msg,
        const unsigned char byte, const int offset, const unsigned char ADL_ch,
        const unsigned char order) {

    unsigned char temp;

    // check which byte order we are dealing with
    if(order == INTEL) {
        // LSB byte comes first
        ((int *) msg)[ADL_ch / 2] = data[byte + 1] * 256 + data[byte] - offset;
        // swap bytes to get into Motorola order
        temp = msg[ADL_ch];
        msg[ADL_ch] = msg[ADL_ch + 1];
        msg[ADL_ch + 1] = temp;
    } else {
        // MSB byte comes first
        ((int *) msg)[ADL_ch / 2] = data[byte + 1] + data[byte] * 256 - offset;
        // swap bytes to get into Motorola order
        temp = msg[ADL_ch];
        msg[ADL_ch] = msg[ADL_ch + 1];
        msg[ADL_ch + 1] = temp;
    }

    return;
}
