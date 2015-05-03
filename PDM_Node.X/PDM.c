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

//TODO: Replace (FAN_SW || AUTO_FAN || OVER_TEMP) with (FAN_PORT)?

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

// Car data variables
static volatile unsigned char FAN_SW; // Holds state of fan switch on steering wheel
static volatile signed int engine_temp, oil_temp, oil_press, rpm, voltage;

// Millisecond value for when each variable was last updated
static volatile unsigned long FAN_SW_tmr, engine_temp_tmr, oil_temp_tmr,
oil_press_tmr, rpm_tmr, voltage_tmr, CAN_recv_tmr;

// ECAN variables
static unsigned long id; // Holds CAN msgID
static unsigned char data[8]; // Holds CAN data bytes
static unsigned char dataLen; // Holds number of CAN data bytes
static ECAN_RX_MSG_FLAGS flags; // Holds information about received message

unsigned char load_states[NUM_LOADS];

static const unsigned char ch_num[NUM_LOADS + 2] = {
    ECU_ch, FUEL_ch, WATER_ch, START_ch, FAN_ch, PCB_ch,
    AUX_ch, START_ch_2, START_ch_3
};

/*
 * Current multiplier / resistor value
 *
 * The switch turns off at 4.5 V on the feedback pin
 */

// Notes:
// (numerator of ratio) / { [(MOSFET current ratio on fb pin) / (resistance on fb pin)] * 1/(order of magnitude of result) * [(voltage range) / (max value of A/D converter)] }
// 10000 / ((2800 / 900) * 1000 * (5 / 2^12))
// gives milliamps
// 10000 / ((8800 / 1100) * 100 * (5 / 2^12))
// gives centiamps (for starter)
//
//Fan
//peak - 500
//1100
//
//Water
//peak - 500
//1100
//
//Fuel
//peak - 800
//600
//
//ECU
//peak - 500
//1500
//
//Aux
//1500
//
//PCB
//1500
//
//Starter
//peak - 500
//1100
static const unsigned long current_ratio[NUM_LOADS] = {
    2318 /*ECU*/, 1755 /*FUEL*/, 3218 /*Water*/, 10240 /*Starter0*/,
    1700 /*Fan*/, 5266 /*PCB*/, 5266 /*AUX*/
};

static const unsigned long current_peak_ratio[NUM_LOADS] = {
    579 /*ECU*/, 1003 /*FUEL*/, 1005 /*Water*/, 3200 /*Starter0*/,
    530 /*Fan*/, 0 /*PCB*/, 0 /*AUX*/
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
 *  Return Value(s): none
 *  Side Effects: This will modify INTCON, TMR0L & PIR5. Also it modifies the ECAN
 *                global variables along with the millis, oil_temp, water_temp,
 *                oil_press, rpm, voltage, and FAN_SW variables as well as
 *                each variable's respective timer.
 */
#pragma interrupt high_isr

void high_isr(void) {
    // Check for timer0 rollover indicating a millisecond has passed
    if(INTCONbits.TMR0IF) {
        INTCONbits.TMR0IF = 0;
        TMR0L = TMR0_RELOAD; // Load timer registers (0xFF (max val) - 0x7D (125) = 0x82)
        millis++;
    }

    // Check for received CAN message
    if(PIR5bits.RXB1IF) {
        // Reset the flag
        PIR5bits.RXB1IF = 0;

        // Get data from receive buffer
        ECANReceiveMessage(&id, data, &dataLen, &flags);

        CAN_recv_tmr = millis;

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

    /*
     * Variable Declarations and Initialization
     */

    unsigned char ON = 0;
    unsigned char PRIME = 1;
    unsigned char OVER_TEMP = 0;

    unsigned char i = 0;

    unsigned int current[NUM_LOADS + 2];
    unsigned long current_calc;

    unsigned long PRIME_tmr = 0;
    unsigned long CAN_send_tmr = 0;

#ifdef MAX_START
    unsigned long START_tmr = 0;
#endif

    unsigned long ECU_peak_tmr = 0;
    unsigned long FUEL_peak_tmr = 0;
    unsigned long WATER_peak_tmr = 0;
    unsigned long START_peak_tmr = 0;
    unsigned long FAN_peak_tmr = 0;

#ifdef CRIT_KILL
    unsigned long voltage_crit_tmr = 0;
    unsigned long oil_press_crit_tmr = 0;
    unsigned long engine_temp_crit_tmr = 0;
    unsigned long oil_temp_crit_tmr = 0;

    unsigned char voltage_crit_pending = 0;
    unsigned char oil_press_crit_pending = 0;
    unsigned char engine_temp_crit_pending = 0;
    unsigned char oil_temp_crit_pending = 0;
#endif


    // Clear error count and peak timers for all loads
    for(i = 0; i < NUM_LOADS + 2; i++) {
        current[i] = 0;
    }

    // Clear variables
    millis = 0;

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
    CAN_recv_tmr = 0;

    /*
     * Peripheral Initialization
     */

    init_unused_pins();

    // Can use internal or external
    init_oscillator();

    // Setup millisecond interrupt
    init_timer0();

    // Turn on and configure the A/D converter module
    init_ADC();

    ANCON0 = 0b11111011; // AN0, AN1, and AN3 - 9 are analog
    ANCON1 = 0b00000011; // rest are digital
    TRISAbits.TRISA0 = INPUT; // AN0
    TRISAbits.TRISA1 = INPUT; // AN1
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

    TRISDbits.TRISD0 = OUTPUT; // ECU MOSFET input
    TRISDbits.TRISD1 = OUTPUT; // ECU peak
    TRISDbits.TRISD2 = OUTPUT; // Fuel MOSFET input
    TRISDbits.TRISD3 = OUTPUT; // Fuel Peak
    TRISDbits.TRISD4 = OUTPUT; // Fan MOSFET input
    TRISDbits.TRISD5 = OUTPUT; // Fan Peak
    TRISDbits.TRISD6 = OUTPUT; // Starter MOSFET input
    TRISDbits.TRISD7 = OUTPUT; // Starter Peak
    TRISCbits.TRISC6 = OUTPUT; // Water MOSFET input
    TRISCbits.TRISC7 = OUTPUT; // Water peak

    // Turn off some inductive loads
    FUEL_LAT = PWR_OFF;
    FUEL_P_LAT = PWR_OFF;
    load_states[FUEL_val] = 0;

    FAN_LAT = PWR_OFF;
    FAN_P_LAT = PWR_OFF;
    load_states[FAN_val] = 0;

    WATER_LAT = PWR_OFF;
    WATER_P_LAT = PWR_OFF;
    load_states[WATER_val] = 0;

    START_LAT = PWR_OFF;
    START_P_LAT = PWR_OFF;
    load_states[START_val] = 0;

    TRISCbits.TRISC2 = OUTPUT; // AUX MOSFET input
    TRISCbits.TRISC4 = OUTPUT; // PCB MOSFET input

    // Turn on non-inductive loads
    AUX_LAT = PWR_ON;
    load_states[AUX_val] = 1;
    PCB_LAT = PWR_ON;
    load_states[PCB_val] = 1;

    TRISCbits.TRISC5 = OUTPUT; // Relay output
    TERM_LAT = PWR_ON; // Not terminating

    ECANInitialize(); // Setup ECAN

    // Interrupts setup
    RCONbits.IPEN = 0; // Interrupt Priority Enable (1 enables)
    STI();

    // Turn on ECU
    ECU_P_LAT = PWR_ON;
    ECU_LAT = PWR_ON;
    load_states[ECU_val] = 1;
    ECU_peak_tmr = millis;

    /*
     * Main Loop
     */

    while(1) {
        // Check if car's engine is on
        CLI();
        ON = rpm > RPM_ON_THRESHOLD;
        STI();

        // Determine how long the fuel pump should be left on
        CLI();
        if(ON_SW_PORT) {
            PRIME = 1;
        } else if(millis - PRIME_tmr > PRIME_WAIT && FUEL_PORT) {
            PRIME = 0;
        }
        STI();

#ifdef CRIT_KILL
        /*
         * If the voltage is critically low, wait for a short amount of time to
         * see if it returns to a safe value. If it doesn't, kill the car.
         */
        CLI();
        if(voltage_tmr > 0 && voltage < VOLTAGE_CRIT) {
            if(!voltage_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                voltage_crit_tmr = millis;
            }
            voltage_crit_pending = 1;

            if(millis - voltage_crit_tmr > CRIT_WAIT_VOLTAGE) {
                AUX_LAT = PWR_OFF;
                load_states[AUX_val] = 0;
                PCB_LAT = PWR_OFF;
                load_states[PCB_val] = 0;
                ECU_LAT = PWR_OFF;
                load_states[ECU_val] = 0;
                killCar();
            }
        } else {
            voltage_crit_pending = 0;
        }
        STI();

        /*
         * If the oil pressure is critically low, wait for a short amount of
         * time to see if it returns to a safe value. If it doesn't, kill the car
         *
         * Note: the critical value for oil pressure depends on engine rpm
         */
        CLI();
        if(oil_press_tmr > 0 && rpm_tmr > 0 && (
                (rpm > RPM_THRESHOLD_H && oil_press < OIL_PRESS_CRIT_H) ||
                (rpm > RPM_THRESHOLD_L && oil_press < OIL_PRESS_CRIT_L))) {
            if(!oil_press_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                oil_press_crit_tmr = millis;
            }
            oil_press_crit_pending = 1;

            if(millis - oil_press_crit_tmr > CRIT_WAIT_OIL_PRESS) {
                killCar();
            }
        } else {
            oil_press_crit_pending = 0;
        }
        STI();

        /*
         * If the engine temperature is critically low, wait for a short amount
         * of time to see if it returns to a safe value. If it doesn't, kill the car
         */
        CLI();
        if(engine_temp_tmr > 0 && engine_temp > ENGINE_TEMP_CRIT) {
            if(!engine_temp_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                engine_temp_crit_tmr = millis;
            }
            engine_temp_crit_pending = 1;

            if(millis - engine_temp_crit_tmr > CRIT_WAIT_TEMP) {
                killCar();
            }
        } else {
            engine_temp_crit_pending = 0;
        }
        STI();

        /*
         * If the oil temperature is critically low, wait for a short amount of
         * time to see if it returns to a safe value. If it doesn't, kill the car
         */
        CLI();
        if(oil_temp_tmr > 0 && oil_temp > OIL_TEMP_CRIT) {
            if(!oil_temp_crit_pending) {
                // Only set the timer if this is the first time the error was observed
                oil_temp_crit_tmr = millis;
            }
            oil_temp_crit_pending = 1;

            if(millis - oil_temp_crit_tmr > CRIT_WAIT_TEMP) {
                killCar();
            }
        } else {
            oil_temp_crit_pending = 0;
        }
        STI();
#endif

        // Engine temperature triggered fan control
        CLI();
        if(OVER_TEMP) {
            if(engine_temp < FAN_THRESHOLD_L)
                OVER_TEMP = 0;
        } else {
            if(engine_temp > FAN_THRESHOLD_H)
                OVER_TEMP = 1;
        }
        STI();

        /*
         * Toggle inductive loads.
         *
         * When turning on a load, turn on the peak latch as well and set the
         * peak timer. The peak latch will be disabled at a set time later.
         *
         * Only power on or power off a load if it is not already on or off
         * respectively, even if the conditions match.
         */
        CLI();
        if(millis - CAN_recv_tmr > BASIC_CONTROL_WAIT ||
                millis - engine_temp_tmr > BASIC_CONTROL_WAIT ||
                millis - voltage_tmr > BASIC_CONTROL_WAIT ||
                millis - oil_temp_tmr > BASIC_CONTROL_WAIT ||
                millis - oil_press_tmr > BASIC_CONTROL_WAIT ||
                millis - rpm_tmr > BASIC_CONTROL_WAIT) {
            /**
             * Perform basic load control.
             *
             * FUEL, WATER, and FAN will turn on when the ON_SW is in the on
             * position and turn off when the ON_SW is in the off position.
             * START and ECU will still be controlled normally as they do not
             * depend on CAN.
             */

            if(!ON_SW_PORT) {
                // FUEL
                if(!FUEL_PORT) {
                    FUEL_P_LAT = PWR_ON;
                    FUEL_LAT = PWR_ON;
                    load_states[FUEL_val] = 1;
                    FUEL_peak_tmr = millis;
                    PRIME_tmr = millis;
                }

                // WATER
                if(!WATER_PORT) {
                    WATER_P_LAT = PWR_ON;
                    WATER_LAT = PWR_ON;
                    load_states[WATER_val] = 1;
                    WATER_peak_tmr = millis;
                }

                // FAN
                if(!FAN_PORT) {
                    FAN_P_LAT = PWR_ON;
                    FAN_LAT = PWR_ON;
                    load_states[FAN_val] = 1;
                    FAN_peak_tmr = millis;
                }
            } else {
                // FUEL
                if(FUEL_PORT) {
                    FUEL_LAT = PWR_OFF;
                    load_states[FUEL_val] = 0;
                }

                // WATER
                if(WATER_PORT) {
                    WATER_LAT = PWR_OFF;
                    load_states[WATER_val] = 0;
                }

                // FAN
                if(FAN_PORT) {
                    FAN_LAT = PWR_OFF;
                    load_states[FAN_val] = 0;
                }
            }
        } else {
            /**
             * Perform regular load control.
             */

            // FUEL
            if(!ON_SW_PORT && (PRIME || ON || START_PORT)) {
                if(!FUEL_PORT) {
                    FUEL_P_LAT = PWR_ON;
                    FUEL_LAT = PWR_ON;
                    load_states[FUEL_val] = 1;
                    FUEL_peak_tmr = millis;
                    PRIME_tmr = millis;
                }
            } else if(ON_SW_PORT || (!PRIME && !ON && !START_PORT)) {
                if(FUEL_PORT) {
                    FUEL_LAT = PWR_OFF;
                    load_states[FUEL_val] = 0;
                }
            }

            // WATER
            if((FAN_SW || OVER_TEMP || ON) && !START_PORT) {
                if(!WATER_PORT) {
                    WATER_P_LAT = PWR_ON;
                    WATER_LAT = PWR_ON;
                    WATER_peak_tmr = millis;
                    load_states[WATER_val] = 1;
                }
            } else if((!ON && !FAN_SW && !OVER_TEMP) || START_PORT) {
                if(WATER_PORT) {
                    WATER_LAT = PWR_OFF;
                    load_states[WATER_val] = 0;
                }
            }

            // FAN
            if((FAN_SW || OVER_TEMP) && !START_PORT) {
                if(!FAN_PORT) {
                    FAN_P_LAT = PWR_ON;
                    FAN_LAT = PWR_ON;
                    FAN_peak_tmr = millis;
                    load_states[FAN_val] = 1;
                }
            } else if((!FAN_SW && !OVER_TEMP) || START_PORT) {
                if(FAN_PORT) {
                    FAN_LAT = PWR_OFF;
                    load_states[FAN_val] = 0;
                }
            }
        }
        STI();

        /*
         * Note: Because there is no "cooldown" timer, if the start switch
         * remains in the on position the MAX_START code will not prevent
         * cranking longer than START_WAIT milliseconds.
         */
        CLI();
#ifdef MAX_START
        // START
        if(!START_SW_PORT && (millis - START_tmr < START_WAIT)) {
            if(!START_PORT) {
                START_P_LAT = PWR_ON;
                START_LAT = PWR_ON;
                load_states[START_val] = 1;
                START_peak_tmr = millis;
                START_tmr = millis;
            }
        } else if(START_SW_PORT || (millis - START_tmr >= START_WAIT)) {
            if(START_SW_PORT) {
                // Reset START_tmr if the start switch is in the off position
                START_tmr = millis;
            }
            if(START_PORT) {
                START_LAT = PWR_OFF;
                load_states[START_val] = 0;
            }
        }
#else
        if(!START_SW_PORT) {
            if(!START_PORT) {
                START_P_LAT = PWR_ON;
                START_LAT = PWR_ON;
                load_states[START_val] = 1;
                START_peak_tmr = millis;
            }
        } else if(START_SW_PORT) {
            if(START_PORT) {
                START_LAT = PWR_OFF;
                load_states[START_val] = 0;
            }
        }
#endif
        STI();

        /*
         * Check peak control timers.
         *
         * If enough time has passed then change the current limit to steady state.
         */
        CLI();
        // ECU
        if(millis - ECU_peak_tmr > ECU_PEAK_WAIT && ECU_P_PORT) {
            ECU_P_LAT = PWR_OFF;
        }

        // FUEL
        if(millis - FUEL_peak_tmr > FUEL_PEAK_WAIT && FUEL_P_PORT) {
            FUEL_P_LAT = PWR_OFF;
        }

        // WATER
        if(millis - WATER_peak_tmr > WATER_PEAK_WAIT && WATER_P_PORT) {
            WATER_P_LAT = PWR_OFF;
        }

        // START
        if(millis - START_peak_tmr > START_PEAK_WAIT && START_P_PORT) {
            START_P_LAT = PWR_OFF;
        }

        // FAN
        if(millis - FAN_peak_tmr > FAN_PEAK_WAIT && FAN_P_PORT) {
            FAN_P_LAT = PWR_OFF;
        }
        STI();

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
                case ECU_val:
                    peak = ECU_P_PORT;
                    break;
                case FUEL_val:
                    peak = FUEL_P_PORT;
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

            // Use a different ratio if the load is currently in peak control
            current_calc = current[i];
            current_calc = peak ? (current_calc * 10000) / current_peak_ratio[i] :
                    (current_calc * 10000) / current_ratio[i];
            current[i] = current_calc;
        }

        // Send out the current data
        CLI();
        if(millis - CAN_send_tmr > CAN_PERIOD) {
            CAN_send_tmr = millis;
            ECANSendMessage(PDM_ID, (unsigned char *) current, 8,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 1, ((unsigned char *) current) + 8, 8,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
            ECANSendMessage(PDM_ID + 2, load_states, 8,
                    ECAN_TX_STD_FRAME | ECAN_TX_NO_RTR_FRAME | ECAN_TX_PRIORITY_1);
        }
        STI();
    }
}

/*
 *  Local Functions
 */

/*
 * void killCar()
 *
 * Description: Kills the car and blocks until the ON switch is cycled. Then
 *              the microcontroller is software reset.
 * Inputs(s): none
 * Return Values(s): none
 * Side Effects: All inductive loads will be powered off, which will kill the
 *               engine. The device will be reset if the driver cycles the
 *               ON switch. All maskable interrupts will be disabled.
 */
void killCar() {
    // Disable interrupts
    CLI();

    // Shut off all inductive loads
    FUEL_LAT = PWR_OFF;
    load_states[FUEL_val] = 0;
    WATER_LAT = PWR_OFF;
    load_states[WATER_val] = 0;
    FAN_LAT = PWR_OFF;
    load_states[FAN_val] = 0;
    START_LAT = PWR_OFF;
    load_states[START_val] = 0;

    // If low voltage killed the car don't allow a restart
    if(voltage < VOLTAGE_CRIT) {
        while(1);
    }

    // Do nothing until the on switch is turned off
    while(!ON_SW_PORT);

    // Perform a MCLR reset of the device.
    _asm RESET _endasm
}

/*
 *  void sample(int *data, const unsigned char index, const unsigned char ch)
 *
 *  Description: Function to read the analog voltage of a pin and then put the value into the
 *               data array that will be transmitted over CAN.
 *  Input(s): data - pointer to array of data bytes
 *            ch - which pin to sample
 *            index - where to write the data in the passed array
 *  Return Value(s): none
 *  Side Effects: This will modify what data points to.
 */
void sample(int *data, const unsigned char index, const unsigned char ch) {
    // Configure which pin you want to read and start A/D converter
    SelChanConvADC(ch);

    // Wait for complete conversion
    while(BusyADC());

    // Put result in data array in accordance with specified byte location
    data[index] = ReadADC();
}
