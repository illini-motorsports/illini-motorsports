
/*
 *                  PDM Node Main File
 *
 * File Name:       PDM.c
 * Processor:       PIC18F46K80
 * Complier:        Microchip C18
 * Version:         1.00
 * Author:          George Schwieters
 * Created:         2013-2014
 */

#include "ECAN.h"
#include "PDM.h"
#include "FSAE.h"


/*
 * PIC18F46K80 Configuration Bits
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

// timing variables
static volatile int millis; // holds timer0 rollover count
static int P_tmr[NUM_LOADS - NON_INDUCTIVE];
static int err_tmr[NUM_LOADS];
static int PRIME_tmr;
static volatile unsigned int FAN_SW; // holds state of fan switch on steering wheel
static volatile int water_temp, oil_temp, oil_press, rpm;

// ECAN variables
static unsigned long id; // holds CAN msgID
static unsigned char data[8]; // holds CAN data bytes
static unsigned char dataLen; // holds number of CAN data bytes
static ECAN_RX_MSG_FLAGS flags; // holds information about recieved message

// check header for correct order of constants; it coincides with the array
// index order that is established with defines
static const unsigned int P_tmr_const[5] = {
    500 /*IGN*/, 500 /*FUEL*/, 500 /*Water*/,
    500 /*Starter*/, 500 /*Fan*/
};
static const unsigned char ch_num[NUM_LOADS + 2] = {
    IGN_ch, FUEL_ch, WATER_ch, START_ch, FAN_ch, PCB_ch,
    AUX_ch, ECU_ch, START_ch_2, START_ch_3
};
// current multiplier / resistor value (the switch turns off at 4.5 V on the feedback pin)
static const unsigned int current_ratio[NUM_LOADS] = {
    14 /*IGN*/, 14 /*FUEL*/,
    14 /*Water*/, 23 /*Starter0*/,
    14 /*Fan*/, 14 /*PCB*/, 8 /*AUX*/,
    14 /*ECU*/
};
static const unsigned int current_P_ratio[NUM_LOADS] = {
    28 /*IGN*/, 28 /*FUEL*/,
    28 /*Water*/, 47 /*Starter0*/,
    28 /*Fan*/, 0 /*PCB*/, 0 /*AUX*/,
    0 /*ECU*/
};


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
 *  Reaturn Value(s): none
 *  Side Effects:   This will modify INTCON, TMR0L & PIR5. Also it modiflies the ECAN
 *                  global variables along with the millis, oil_temp, water_temp,
 *                  oil_press, rpm, and FAN_SW variables.
 */

#pragma interrupt high_isr

void high_isr(void) {

    // check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // check for recieved CAN message
    if(PIR5bits.RXB1IF) {
        // reset the flag
        PIR5bits.RXB1IF = FALSE;
        ECANReceiveMessage(&id, data, &dataLen, &flags);
        if(id == ENGINE_TEMP_ID) {
            ((unsigned char*) &water_temp)[0] = data[ENGINE_TEMP_BYTE + 1];
            ((unsigned char*) &water_temp)[1] = data[ENGINE_TEMP_BYTE];
        }
        if(id == OIL_TEMP_ID) {
            ((unsigned char*) &oil_temp)[0] = data[OIL_TEMP_BYTE + 1];
            ((unsigned char*) &oil_temp)[1] = data[OIL_TEMP_BYTE];
        }
        if(id == OIL_PRESS_ID) {
            ((unsigned char*) &oil_press)[0] = data[OIL_PRESS_BYTE + 1];
            ((unsigned char*) &oil_press)[1] = data[OIL_PRESS_BYTE];
        }
        if(id == FAN_SW_ID) {
            if(data[0] == FAN_SW_ADL_ID)
                FAN_SW = data[FAN_SW_BYTE];
        }
        if(id == RPM_ID) {
            ((unsigned char*) &rpm)[0] = data[RPM_BYTE + 1];
            ((unsigned char*) &rpm)[1] = data[RPM_BYTE];
        }
    }

    return;
}

void main(void) {

    /*
     * Variable Declarations
     */

#ifdef PARANOID_MODE
    int oil_press_tmr = 0;
    int oil_temp_tmr = 0;
    int water_temp_tmr = 0;
#endif
    int CAN_tmr = 0;
    Error_Status STATUS;
    unsigned char AUTO_FAN;
    unsigned char PRIME = TRUE;
    unsigned char ON = FALSE;
    unsigned char i;
    unsigned char err_count[NUM_LOADS];
    unsigned int current[NUM_LOADS + 2];
    unsigned int current_P[NUM_LOADS + 2];
    unsigned int CAN_current[NUM_LOADS + 2];

    //init_unused_pins();       // there are no unused pins!!!

    /*
     * Variable Initialization
     */

    // clear error count for all laods
    for(i = 0; i < NUM_LOADS; i++) {
        err_count[i] = 0;
        err_tmr[i] = 0;
        if(i < NUM_LOADS - NON_INDUCTIVE) P_tmr[i] = 0;
    }

    // clear variables
    PRIME_tmr = 0;
    STATUS.bits = 0;
    rpm = 0;
    water_temp = 0;
    oil_temp = 0;
    oil_press = 0;
    FAN_SW = FALSE;

    /*
     * Peripheral Initialization
     */

    // can use internal or external
    init_oscillator();

    // setup millisecond interrupt
    init_timer0();

    // turn on and configure the A/D converter module
    init_ADC();

    ANCON0 = 0b11111111; // AN0 - 9 are analog
    ANCON1 = 0b00000011; // rest are digital
    TRISAbits.TRISA0 = INPUT; // AN0
    TRISAbits.TRISA1 = INPUT; // AN1
    TRISAbits.TRISA2 = INPUT; // AN2
    TRISAbits.TRISA3 = INPUT; // AN3
    TRISAbits.TRISA5 = INPUT; // AN4
    TRISEbits.TRISE0 = INPUT; // AN5
    TRISEbits.TRISE1 = INPUT; // AN6
    TRISEbits.TRISE2 = INPUT; // AN7
    TRISBbits.TRISB1 = INPUT; // AN8
    TRISBbits.TRISB4 = INPUT; // AN9

    // configure port I/O
    TRISBbits.TRISB5 = INPUT; // Stater switch
    TRISBbits.TRISB0 = INPUT; // On switch

    TRISDbits.TRISD0 = OUTPUT; // Ignition relay input
    TRISDbits.TRISD1 = OUTPUT; // Ignition Peak
    TRISDbits.TRISD2 = OUTPUT; // Fuel relay input
    TRISDbits.TRISD3 = OUTPUT; // Fuel Peak
    TRISDbits.TRISD4 = OUTPUT; // Fan relay input
    TRISDbits.TRISD5 = OUTPUT; // Fan Peak
    TRISDbits.TRISD6 = OUTPUT; // Starter relay input
    TRISDbits.TRISD7 = OUTPUT; // Starter Peak
    TRISCbits.TRISC6 = OUTPUT; // Water relay input
    TRISCbits.TRISC7 = OUTPUT; // Water peak

    IGN_LAT = PWR_OFF;
    IGN_P_LAT = PWR_OFF;
    FUEL_LAT = PWR_OFF;
    FUEL_P_LAT = PWR_OFF;
    FAN_LAT = PWR_OFF;
    FAN_P_LAT = PWR_OFF;
    WATER_LAT = PWR_OFF;
    WATER_P_LAT = PWR_OFF;
    START_LAT = PWR_OFF;
    START_P_LAT = PWR_OFF;

    TRISCbits.TRISC2 = OUTPUT; // AUX relay input
    TRISCbits.TRISC3 = OUTPUT; // ECU relay input
    TRISCbits.TRISC4 = OUTPUT; // PCB relay input

    AUX_LAT = PWR_ON;
    ECU_LAT = PWR_ON;
    PCB_LAT = PWR_ON;

    TRISCbits.TRISC5 = OUTPUT; // relay input
    TERM_LAT = PWR_OFF;

    ECANInitialize(); // setup ECAN

    // interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    /***************end setup; begin main loop************************************/

    while(1) {

#ifdef PARANOID_MODE
        // stay here during extreme conditions caused by oil pressure, oil temperature or water temperature
        if(preventEngineBlowup(&oil_press_tmr, &oil_temp_tmr, &water_temp_tmr) == TRUE) {
            // turn off the car
            FUEL_LAT = PWR_OFF;
            IGN_LAT = PWR_OFF;
            START_LAT = PWR_OFF;
            while(ON_SW);
        }
#endif

        // check if we should automatically turn on the fan or not
        checkWaterTemp(&AUTO_FAN);

        // check if car is on
        if(rpm > ON_THRESHOLD)
            ON = TRUE;
        else
            ON = FALSE;

        // determine how long the fuel pump should be left on
        if(!ON_SW)
            PRIME = TRUE;
        else if(millis - PRIME_tmr > PRIME_WAIT && FUEL_PORT)
            PRIME = FALSE;

        // turn on loads
        // check if already on and if so do nothing
        // also check if we are in an overcurrent condition in which case do nothing
        if(ON_SW & !IGN_PORT & !STATUS.Ignition) {
            IGN_P_LAT = PWR_ON;
            IGN_LAT = PWR_ON;
            P_tmr[IGN_val] = millis;
        }
        if(((ON_SW & PRIME) | ON | START_PORT) & !FUEL_PORT & !STATUS.Fuel) {
            FUEL_P_LAT = PWR_ON;
            FUEL_LAT = PWR_ON;
            P_tmr[FUEL_val] = millis;
            PRIME_tmr = millis;
        }
        if((FAN_SW | AUTO_FAN | ON) & !WATER_PORT & !STATUS.Water & !START_PORT) {
            WATER_P_LAT = PWR_ON;
            WATER_LAT = PWR_ON;
            P_tmr[WATER_val] = millis;
        }
        if(!START_SW & !START_PORT & !STATUS.Starter) {
            START_P_LAT = PWR_ON;
            START_LAT = PWR_ON;
            P_tmr[START_val] = millis;
        }
        if((FAN_SW | AUTO_FAN) & !FAN_PORT & !STATUS.Fan & !START_PORT) {
            FAN_P_LAT = PWR_ON;
            FAN_LAT = PWR_ON;
            P_tmr[FAN_val] = millis;
        }


        // turn off loads
        // check if the load is already off and if so do nothing
        // also turn off any overcurrent error state that may be enabled
        if(!ON_SW & (IGN_PORT | STATUS.Ignition)) {
            IGN_LAT = PWR_OFF;
            STATUS.Ignition = PWR_OFF;
            err_count[IGN_val] = 0;
        }
        if((!ON_SW | (!PRIME & !ON & !START_PORT)) & (FUEL_PORT | STATUS.Fuel)) {
            FUEL_LAT = PWR_OFF;
            STATUS.Fuel = PWR_OFF;
            err_count[FUEL_val] = 0;
        }
        if((!ON & !AUTO_FAN & !FAN_SW & (WATER_PORT | STATUS.Water)) | START_PORT) {
            WATER_LAT = PWR_OFF;
            STATUS.Water = PWR_OFF;
            err_count[WATER_val] = 0;
        }
        if(START_SW & (START_PORT | STATUS.Starter)) {
            START_LAT = PWR_OFF;
            STATUS.Starter = PWR_OFF;
            err_count[START_val] = 0;
        }
        if((!FAN_SW & !AUTO_FAN & (FAN_PORT | STATUS.Fan)) | START_PORT) {
            FAN_LAT = PWR_OFF;
            STATUS.Fan = PWR_OFF;
            err_count[FAN_val] = 0;
        }


        // check peak control timers
        // if enough time has passed then change the current limit to steady state
        for(i = 0; i < NUM_LOADS - NON_INDUCTIVE; i++) {
            if(millis - P_tmr[i] > P_tmr_const[i]) {
                switch(i) {
                    case FUEL_val:
                        if(FUEL_P_PORT)
                            FUEL_P_LAT = PWR_OFF;
                        break;
                    case IGN_val:
                        if(IGN_P_PORT)
                            IGN_P_LAT = PWR_OFF;
                        break;
                    case WATER_val:
                        if(WATER_P_PORT)
                            WATER_P_LAT = PWR_OFF;
                        break;
                    case START_val:
                        if(START_P_PORT)
                            START_P_LAT = PWR_OFF;
                        break;
                    case FAN_val:
                        if(FAN_P_PORT)
                            FAN_P_LAT = PWR_OFF;
                        break;
                }
            }
        }

#ifdef OVERCURRENT_HANDLING
        // check if enough time has passed for the error states
        // if enough time has passed then bring the relay input high again
        // as the number of error conditions increases so does the wait time
        for(i = 0; i < NUM_LOADS; i++) {
            if((millis - err_tmr[i] > ERROR_WAIT) && (err_count[i] < ERROR_LIMIT)) {
                if(STATUS.bits) {
                    // determine which load is to blame
                    switch(i) {
                        case FUEL_val:
                            if(STATUS.Fuel & !FUEL_PORT) {
                                FUEL_LAT = PWR_ON;
                                FUEL_P_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case IGN_val:
                            if(STATUS.Ignition & !IGN_PORT) {
                                IGN_LAT = PWR_ON;
                                IGN_P_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case WATER_val:
                            if(STATUS.Water & !WATER_PORT) {
                                WATER_LAT = PWR_ON;
                                WATER_P_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case START_val:
                            if(STATUS.Starter & !START_PORT) {
                                START_LAT = PWR_ON;
                                START_P_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case FAN_val:
                            if(STATUS.Fan & !FAN_PORT) {
                                FAN_LAT = PWR_ON;
                                FAN_P_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case AUX_val:
                            if(STATUS.AUX & !AUX_PORT) {
                                AUX_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case ECU_val:
                            if(STATUS.ECU & !ECU_PORT) {
                                ECU_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                        case PCB_val:
                            if(STATUS.PCB & !PCB_PORT) {
                                PCB_LAT = PWR_ON;
                                err_count[i]++;
                            }
                            break;
                    }
                }
            }
        }
#endif

        // sample the current of the loads
        for(i = 0; i < NUM_LOADS + 2; i++)
            sample((int *) current, i, ch_num[i]);

        // put total current value of starter in one location
        current[START_val] = current[START_val] + current[START_val_2] + current[START_val_3];

        // scale input voltage to get current value
        for(i = 0; i < NUM_LOADS; i++)
            current[i] = (unsigned long) current[i] * (unsigned long) current_ratio[i] * 5;
        for(i = 0; i < NUM_LOADS; i++)
            current_P[i] = (unsigned long) current[i] * (unsigned long) current_P_ratio[i] * 5;

#ifdef OVERCURRENT_HANDLING
        // determine if there is an overcurrent condition
        for(i = 0; i < NUM_LOADS; i++) {
            if(current[i] < MIN_MA_OVERCURRENT * 10) {
                switch(i) {
                    case FUEL_val:
                        if(FUEL_PORT) {
                            FUEL_LAT = PWR_OFF;
                            err_tmr[FUEL_val] = millis;
                            STATUS.Fuel = 1;
                        }
                        break;
                    case IGN_val:
                        if(IGN_PORT) {
                            IGN_LAT = PWR_OFF;
                            err_tmr[IGN_val] = millis;
                            STATUS.Ignition = 1;
                        }
                        break;
                    case WATER_val:
                        if(WATER_PORT) {
                            WATER_LAT = PWR_OFF;
                            err_tmr[WATER_val] = millis;
                            STATUS.Water = 1;
                        }
                        break;
                    case START_val:
                        if(START_PORT) {
                            START_LAT = PWR_OFF;
                            err_tmr[START_val] = millis;
                            STATUS.Starter = 1;
                        }
                        break;
                    case FAN_val:
                        if(FAN_PORT) {
                            FAN_LAT = PWR_OFF;
                            err_tmr[FAN_val] = millis;
                            STATUS.Fan = 1;
                        }
                        break;
                    case AUX_val:
                        if(AUX_PORT) {
                            AUX_LAT = PWR_OFF;
                            err_tmr[AUX_val] = millis;
                            STATUS.AUX = 1;
                        }
                        break;
                    case ECU_val:
                        if(ECU_PORT) {
                            ECU_LAT = PWR_OFF;
                            err_tmr[ECU_val] = millis;
                            STATUS.ECU = 1;
                        }
                        break;
                    case PCB_val:
                        if(PCB_PORT) {
                            PCB_LAT = PWR_OFF;
                            err_tmr[PCB_val] = millis;
                            STATUS.PCB = 1;
                        }
                        break;
                }
            }
        }
#endif

        // get the correct current data depeding on which state we are in with respect to peak control
        for(i = 0; i < NUM_LOADS - NON_INDUCTIVE; i++) {
            switch(i) {
                case FUEL_val:
                    if(FUEL_P_PORT) CAN_current[i] = current_P[i];
                    else CAN_current[i] = current[i];
                    break;
                case IGN_val:
                    if(IGN_P_PORT) CAN_current[i] = current_P[i];
                    else CAN_current[i] = current[i];
                    break;
                case WATER_val:
                    if(WATER_P_PORT) CAN_current[i] = current_P[i];
                    else CAN_current[i] = current[i];
                    break;
                case START_val:
                    if(START_P_PORT) CAN_current[i] = current_P[i];
                    else CAN_current[i] = current[i];
                    break;
                case FAN_val:
                    if(FAN_P_PORT) CAN_current[i] = current_P[i];
                    else CAN_current[i] = current[i];
                    break;
            }
        }

        // get current data for non inductive loads
        CAN_current[PCB_val] = current[PCB_val];
        CAN_current[AUX_val] = current[AUX_val];
        CAN_current[ECU_val] = current[ECU_val];

        // send out the current data
        if(millis - CAN_tmr > CAN_PER) {
            CAN_tmr = millis;
            ECANSendMessage(PDM_ID, (BYTE *) CAN_current, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 1, ((BYTE *) CAN_current) + 8, 8, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 2, ((BYTE *) CAN_current) + 16, 2, ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
        }

    } // end main while loop

    return;
}


/*
 *  Local Functions
 */

/*
 *  void sample(int *data, const unsigned char index, const unsigned char ch)
 *
 *  Description:    Function to read the analog voltage of a pin and then put the value into the
 *                  data array that will be transmited over CAN.
 *  Input(s):   *data - pointer to array of data bytes
 *              ch - which pin to sample
 *              index - where to write the data in the passed array
 *  Return Value(s): none
 *  Side Effects: This will modify what data points to.
 */
void sample(int *data, const unsigned char index, const unsigned char ch) {

    SelChanConvADC(ch); // configure which pin you want to read and start A/D converter

    while(BusyADC()); // wait for complete conversion

    // put result in data array in accordance with specified byte location
    data[index] = ReadADC();

    return;
}

/*
 *  void preventEngineBlowup(int* oil_press_tmr, int* oil_temp_tmr, int* water_temp_tmr)
 *
 *  Description:    This function checks on critical engine sensors
 *                  to determine if we should kill the engine.
 *  Input(s):   oil_press_tmr - timer for how long oil pressure can be in an error state
 *              oil_temp_tmr - timer for how long oil temperature can be in an error state
 *              water_temp_tmr - timer for how long water temperature can be in an error state
 *  Return Value(s): Boolean for whether or not we are fucked
 *  Side Effects: none
 */
unsigned char preventEngineBlowup(int* oil_press_tmr, int* oil_temp_tmr, int* water_temp_tmr) {

    // check oil temperature to see if we're too hot
    if(oil_temp > OT_THRESHOLD) {
        return TRUE;
    }
    // check oil pressure for too low of pressure
    if(rpm > RPM_THRESHOLD_H) {
        if(oil_press < OP_THRESHOLD_H) {
            return TRUE;
        }
    } else if(rpm > RPM_THRESHOLD_L) {
        if(oil_press < OP_THRESHOLD_L) {
            return TRUE;
        }
    }
    // check engine temperature to see if we're too hot
    if(water_temp > ET_THRESHOLD) {
        return TRUE;
    }

    // nothing is wrong
    return FALSE;
}

/*
 *  void checkWaterTemp(unsigned char * FAN_AUTO)
 *
 *  Description:    This function checks if we should turn on the fan or not.
 *  Input(s):   FAN_AUTO - pointer to a flag that controls turning on the fan automatically
 *  Return Value(s): none
 *  Side Effects: This modifies the flag pointed to by FAN_AUTO.
 */
void checkWaterTemp(unsigned char * FAN_AUTO) {
    if(*FAN_AUTO) {
        if(water_temp < FAN_THRESHOLD_L)
            *FAN_AUTO = FALSE;
    } else {
        if(water_temp > FAN_THRESHOLD_H)
            *FAN_AUTO = TRUE;
    }
    return;
}
