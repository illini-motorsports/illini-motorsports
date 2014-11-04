/*
 * PDM Node Main File
 *
 * File Name:       PDM.c
 * Processor:       PIC18F46K80
 * Compiler:        Microchip C18
 * Version:         2.00
 * Author:          George Schwieters
 * Author:          Andrew Mass
 * Created:         2013-2014
 */

#include "ECAN.h"
#include "FSAE.h"
#include "PDM.h"

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

// Timing variables
static volatile unsigned long millis; // Holds timer0 rollover count
static volatile unsigned long CAN_tmr;

// Car data variables
static volatile unsigned int FAN_SW; // Holds state of fan switch on steering wheel
static volatile signed int engine_temp, oil_temp, oil_press, rpm, voltage;

// Millisecond value for when each variable was last updated
static volatile unsigned long FAN_SW_tmr, engine_temp_tmr, oil_temp_tmr,
oil_press_tmr, rpm_tmr, voltage_tmr;

// ECAN variables
static unsigned long id; // holds CAN msgID
static unsigned char data[8]; // holds CAN data bytes
static unsigned char dataLen; // holds number of CAN data bytes
static ECAN_RX_MSG_FLAGS flags; // holds information about recieved message

/*
 * Check header for correct order of constants; it coincides with the array
 * index order that is established with defines
 */
static const unsigned int peak_tmr_const[5] = {
    500 /*IGN*/, 500 /*FUEL*/, 500 /*Water*/,
    500 /*Starter*/, 500 /*Fan*/
};

static const unsigned char ch_num[NUM_LOADS + 2] = {
    IGN_ch, FUEL_ch, WATER_ch, START_ch, FAN_ch, PCB_ch,
    AUX_ch, ECU_ch, START_ch_2, START_ch_3
};

// Current multiplier / resistor value (the switch turns off at 4.5 V on the feedback pin)
static const unsigned int current_ratio[NUM_LOADS] = {
    14 /*IGN*/, 14 /*FUEL*/,
    14 /*Water*/, 23 /*Starter0*/,
    14 /*Fan*/, 14 /*PCB*/, 8 /*AUX*/,
    14 /*ECU*/
};

static const unsigned int current_peak_ratio[NUM_LOADS] = {
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
 *  Description: This interrupt will service all high priority interrupts. This
 *               section should be as short as possible.
 *  Input(s): none
 *  Reaturn Value(s): none
 *  Side Effects: This will modify INTCON, TMR0L & PIR5. Also it modiflies the ECAN
 *                global variables along with the millis, oil_temp, water_temp,
 *                oil_press, rpm, voltage, and FAN_SW variables as well as
 *                each variable's respective timer.
 */
#pragma interrupt high_isr

void high_isr(void) {

    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // Load timer rgisters (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // Check for recieved CAN message
    if(PIR5bits.RXB1IF) {
        // Reset the flag
        PIR5bits.RXB1IF = 0;

        // Get data from receive buffer
        ECANReceiveMessage(&id, data, &dataLen, &flags);

        CAN_tmr = millis;

        switch(id) {
            case ENGINE_TEMP_ID: // VOLTAGE_ID
                ((unsigned char*) &engine_temp)[0] = data[ENGINE_TEMP_BYTE + 1];
                ((unsigned char*) &engine_temp)[1] = data[ENGINE_TEMP_BYTE];
                engine_temp_tmr = millis;

                ((unsigned char*) &voltage)[0] = data[VOLTAGE_BYTE + 1];
                ((unsigned char*) &voltage)[1] = data[VOLTAGE_BYTE];
                voltage_tmr = millis;
                break;
            case OIL_TEMP_ID: // OIL_PRESS_ID, RPM_ID
                ((unsigned char*) &oil_temp)[0] = data[OIL_TEMP_BYTE + 1];
                ((unsigned char*) &oil_temp)[1] = data[OIL_TEMP_BYTE];
                oil_temp_tmr = millis;

                ((unsigned char*) &oil_press)[0] = data[OIL_PRESS_BYTE + 1];
                ((unsigned char*) &oil_press)[1] = data[OIL_PRESS_BYTE];
                oil_press_tmr = millis;

                ((unsigned char*) &rpm)[0] = data[RPM_BYTE + 1];
                ((unsigned char*) &rpm)[1] = data[RPM_BYTE];
                rpm_tmr = millis;
                break;
            case FAN_SW_ID:
                if(data[0] == FAN_SW_ADL_ID) {
                    FAN_SW = data[FAN_SW_BYTE];
                    FAN_SW_tmr = millis;
                }
                break;
        }
    }
}

void main(void) {

    // init_unused_pins();       // There are no unused pins!!!

    /*
     * Variable Declarations and Initialization
     */

    Error_Status STATUS;

    unsigned char AUTO_FAN = 0;
    unsigned char UNDER_VOLTAGE, OVER_TEMP = 0;
    unsigned char PRIME = 1;
    unsigned char ON = 0;

    unsigned char i;

    unsigned int current[NUM_LOADS + 2];

    unsigned long peak_tmr[NUM_LOADS - NON_INDUCTIVE];
    unsigned long PRIME_tmr = 0;

    unsigned char voltage_crit_pending, oil_press_crit_pending,
            engine_temp_crit_pending, oil_temp_crit_pending = 0;
    unsigned long voltage_crit_tmr, oil_press_crit_tmr,
            engine_temp_crit_tmr, oil_temp_crit_tmr = 0;

    // Clear error count and peak timers for all loads
    for(i = 0; i < NUM_LOADS + 2; i++) {
        current[i] = 0;
        if(i < NUM_LOADS - NON_INDUCTIVE) {
            peak_tmr[i] = 0;
        }
    }

    // Clear variables
    millis = 0;
    CAN_tmr = 0;

    STATUS.bits = 0;

    FAN_SW = 0;
    engine_temp = 0;
    oil_temp = 0;
    oil_press = 0;
    rpm = 0;
    voltage = 0;

    FAN_SW_tmr = 0;
    engine_temp_tmr = 0;
    oil_temp_tmr = 0;
    oil_press_tmr = 0;
    rpm_tmr = 0;
    voltage_tmr = 0;

    /*
     * Peripheral Initialization
     */

    // Can use internal or external
    init_oscillator();

    // Setup millisecond interrupt
    init_timer0();

    // Turn on and configure the A/D converter module
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

    // Configure port I/O
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

    TRISCbits.TRISC5 = OUTPUT; // Relay input
    TERM_LAT = PWR_OFF; // Not terminating

    ECANInitialize(); // Setup ECAN

    // Interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    /*
     * Main Loop
     */

    while(1) {

        /*
         * New
         * ===
         ** Set ON
         ** Set AUTO_FAN
         ** Set PRIME
         ** Check out-of-range values. This could cause some conditions to be
         **    overridden(rode?, rided?) or the car to be killed.
         * Power on or off loads depending on external conditions. If powering
         *     on, start peak timers for inductive loads. If powering off,
         *     clear error count.
         * If enough time has passed, switch to transient current limit for
         *     inductive loads.
         * Sample current
         * Scale current
         * Send out current data on CAN
         *
         * todo
         * ====
         * Check whether CAN values are "stale"
         */

        // Check if car's engine is on
        ON = rpm > RPM_ON_THRESHOLD;

        //Set whether to automatically run the fan based on water temperature
        if(engine_temp < FAN_THRESHOLD_L) {
            AUTO_FAN = 0;
        } else if(engine_temp > FAN_THRESHOLD_H) {
            AUTO_FAN = 1;
        }

        //TODO: Understand this code
        // Determine how long the fuel pump should be left on
        if(!ON_SW_PORT) {
            PRIME = 1;
        } else if(millis - PRIME_tmr > PRIME_WAIT && FUEL_PORT) {
            PRIME = 0;
        }

        /*
         * If the latest CAN message was received more than CRIT_WAIT
         * milliseconds ago, kill the car.
         */
        if(millis - CAN_tmr > CRIT_WAIT_CAN) {
            //TODO: Kill the car
        }

        /*
         * If the voltage is critically low, wait for a short amount of time to
         * see if it returns to a safe value. If it doesn't, kill the car.
         */
        if(voltage_tmr > 0 && voltage < VOLTAGE_CRIT) {
            if(!voltage_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                voltage_crit_tmr = millis;
            }
            voltage_crit_pending = 1;

            if(millis - voltage_crit_tmr > CRIT_WAIT) {
                //TODO: Kill the car
            }
        } else {
            voltage_crit_pending = 0;
        }

        /*
         * If the oil pressure is critically low, wait for a short amount of
         * time to see if it returns to a safe value. If it doesn't, kill the car
         *
         * Note: the critical value for oil pressure depends on engine rpm
         */
        if(oil_press_tmr > 0 && rpm_tmr > 0 && (
                (rpm > RPM_THRESHOLD_H && oil_press < OIL_PRESS_CRIT_H) ||
                (rpm > RPM_THRESHOLD_L && oil_press < OIL_PRESS_CRIT_L))) {
            if(!oil_press_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                oil_press_crit_tmr = millis;
            }
            oil_press_crit_pending = 1;

            if(millis - oil_press_crit_tmr > CRIT_WAIT) {
                //TODO: Kill the car
            }
        } else {
            oil_press_crit_pending = 0;
        }

        /*
         * If the engine temperature is critically low, wait for a short amount
         * of time to see if it returns to a safe value. If it doesn't, kill the car
         */
        if(engine_temp_tmr > 0 && engine_temp > ENGINE_TEMP_CRIT) {
            if(!engine_temp_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                engine_temp_crit_tmr = millis;
            }
            engine_temp_crit_pending = 1;

            if(millis - engine_temp_crit_tmr > CRIT_WAIT) {
                //TODO: Kill the car
            }
        } else {
            engine_temp_crit_pending = 0;
        }

        /*
         * If the oil temperature is critically low, wait for a short amount of
         * time to see if it returns to a safe value. If it doesn't, kill the car
         */
        if(oil_temp_tmr > 0 && oil_temp > OIL_TEMP_CRIT) {
            if(!oil_temp_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                oil_temp_crit_tmr = millis;
            }
            oil_temp_crit_pending = 1;

            if(millis - oil_temp_crit_tmr > CRIT_WAIT) {
                //TODO: Kill the car
            }
        } else {
            oil_temp_crit_pending = 0;
        }

        // Determine if the battery voltage is too low
        UNDER_VOLTAGE = (voltage_tmr > 0 && voltage > VOLTAGE_WARN);

        // Determine if the engine or oil is too hot
        OVER_TEMP = (engine_temp_tmr > 0 && engine_temp > ENGINE_TEMP_WARN) ||
                (oil_temp_tmr > 0 && oil_temp_tmr > OIL_TEMP_WARN);

        /*
         *
        // turn on loads
        // check if already on and if so do nothing
        // also check if we are in an overcurrent condition in which case do nothing
        if(ON_SW_PORT & !IGN_PORT & !STATUS.Ignition) {
            IGN_P_LAT = PWR_ON;
            IGN_LAT = PWR_ON;
            peak_tmr[IGN_val] = millis;
        }
        if(((ON_SW_PORT & PRIME) | ON | START_PORT) & !FUEL_PORT & !STATUS.Fuel) {
            FUEL_P_LAT = PWR_ON;
            FUEL_LAT = PWR_ON;
            peak_tmr[FUEL_val] = millis;
            PRIME_tmr = millis;
        }
        if((FAN_SW | AUTO_FAN | ON) & !WATER_PORT & !STATUS.Water & !START_PORT) {
            WATER_P_LAT = PWR_ON;
            WATER_LAT = PWR_ON;
            peak_tmr[WATER_val] = millis;
        }
        if(!START_SW_PORT & !START_PORT & !STATUS.Starter) {
            START_P_LAT = PWR_ON;
            START_LAT = PWR_ON;
            peak_tmr[START_val] = millis;
        }
        if((FAN_SW | AUTO_FAN) & !FAN_PORT & !STATUS.Fan & !START_PORT) {
            FAN_P_LAT = PWR_ON;
            FAN_LAT = PWR_ON;
            peak_tmr[FAN_val] = millis;
        }

        // turn off loads
        // check if the load is already off and if so do nothing
        // also turn off any overcurrent error state that may be enabled
        if(!ON_SW_PORT & (IGN_PORT | STATUS.Ignition)) {
            IGN_LAT = PWR_OFF;
            STATUS.Ignition = PWR_OFF;
            err_count[IGN_val] = 0;
        }
        if((!ON_SW_PORT | (!PRIME & !ON & !START_PORT)) & (FUEL_PORT | STATUS.Fuel)) {
            FUEL_LAT = PWR_OFF;
            STATUS.Fuel = PWR_OFF;
            err_count[FUEL_val] = 0;
        }
        if((!ON & !AUTO_FAN & !FAN_SW & (WATER_PORT | STATUS.Water)) | START_PORT) {
            WATER_LAT = PWR_OFF;
            STATUS.Water = PWR_OFF;
            err_count[WATER_val] = 0;
        }
        if(START_SW_PORT & (START_PORT | STATUS.Starter)) {
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
            if(millis - peak_tmr[i] > peak_tmr_const[i]) {
                switch(i) {
                    case FUEL_val:
                        if(FUEL_P_PORT) {
                            FUEL_P_LAT = PWR_OFF;
                        }
                        break;
                    case IGN_val:
                        if(IGN_P_PORT) {
                            IGN_P_LAT = PWR_OFF;
                        }
                        break;
                    case WATER_val:
                        if(WATER_P_PORT) {
                            WATER_P_LAT = PWR_OFF;
                        }
                        break;
                    case START_val:
                        if(START_P_PORT) {
                            START_P_LAT = PWR_OFF;
                        }
                        break;
                    case FAN_val:
                        if(FAN_P_PORT) {
                            FAN_P_LAT = PWR_OFF;
                        }
                        break;
                }
            }
        }

        // Sample the current of the loads
        for(i = 0; i < NUM_LOADS + 2; i++) {
            sample((int *) current, i, ch_num[i]);
        }

        // Put total current value of starter in one location
        current[START_val] = current[START_val] + current[START_val_2] + current[START_val_3];

        // Scale input voltage to get current value
        for(i = 0; i < NUM_LOADS; i++) {
            unsigned char peak = 0;
            switch(i) {
                case FUEL_val:
                    peak = FUEL_P_PORT;
                    break;
                case IGN_val:
                    peak = IGN_P_PORT;
                    break;
                case WATER_val:
                    peak = WATER_P_PORT;
                    break;
                case START_val:
                    peak = START_P_PORT;
                    break;
                case FAN_val:
                    peak = FAN_P_PORT;
                    break;
            }

            current[i] = peak ? (unsigned long) current[i] * (unsigned long) current_peak_ratio[i] * 5 :
                    (unsigned long) current[i] * (unsigned long) current_ratio[i] * 5;
        }

        // Send out the current data
        if(millis - CAN_tmr > CAN_PERIOD) {
            CAN_tmr = millis;
            ECANSendMessage(PDM_ID, (unsigned char *) current, 8,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 1, ((unsigned char *) current) + 8, 8,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 2, ((unsigned char *) current) + 16, 2,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
        }
         */
    }
}

/*
 *  Local Functions
 */

/*
 *  void sample(int *data, const unsigned char index, const unsigned char ch)
 *
 *  Description: Function to read the analog voltage of a pin and then put the value into the
 *               data array that will be transmited over CAN.
 *  Input(s): data - pointer to array of data bytes
 *            ch - which pin to sample
 *            index - where to write the data in the passed array
 *  Return Value(s): none
 *  Side Effects: This will modify what data points to.
 */
void sample(int *data, const unsigned char index, const unsigned char ch) {

    SelChanConvADC(ch); // configure which pin you want to read and start A/D converter

    while(BusyADC()); // wait for complete conversion

    // put result in data array in accordance with specified byte location
    data[index] = ReadADC();
}
