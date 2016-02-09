/**
 * FSAE Library 32bit Config
 *
 * Processor: PIC32MZ2048EFM100
 * Compiler:  Microchip XC32
 * Author:    Andrew Mass
 * Created:   2015-2016
 */
#include "FSAE_config_32.h"

// PIC32MZ2048EFM100 Configuration Bit Settings

// DEVCP0
#pragma config CP = 0b1 // Code Protect (Protection is disabled)

// DEVCFG0
#pragma config EJTAGBEN = 0b1  // EJTAG Boot Enable (Normal EJTAG functionality)
#pragma config POSCBOOST = 0b0 // Primary Oscillator Boost Kick Start Enable (Normal start of the oscillator)
#pragma config POSCGAIN = 0b00 // Primary Oscillator Gain Control (1x gain setting)
#pragma config SOSCBOOST = 0b0 // Secondary Oscillator Boost Kick Start Enable (Normal start of the oscillator)
#pragma config SOSCGAIN = 0b00 // Secondary Oscillator Gain Control (1x gain setting)
#pragma config SMCLR = 0b1     // Soft Master Clear Enable (MCLR pin generates a normal system Reset)
#pragma config DBGPER = 0b111  // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 0,1,2 permission regions)
#pragma config FSLEEP = 0b1    // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config FECCCON = 0b11  // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config BOOTISA = 0b1   // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config TRCEN = 0b0     // Trace Enable (Trace features in the CPU are disabled)
#pragma config ICESEL = 0b11   // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config JTAGEN = 0b0    // JTAG Enable (JTAG is disabled)
#pragma config DEBUG = 0b00    // Background Debugger Enable (Debugger is enabled)

// DEVCFG1
#pragma config FDMTEN = 0b0     // Deadman Timer Enable (Deadman Timer is disbled)
#pragma config DMTCNT = 0b10111 // Deadman Timer Count Select Bits (2^31)
#pragma config FWDTWINSZ = 0b11 // Watchdog Timer Window Size (Window size is 25%)
#pragma config FWDTEN = 0b0     // Watchdog Timer Enable (WDT Disabled)
#pragma config WINDIS = 0b1     // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config WDTSPGM = 0b1    // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WDTPS = 0b10100  // Watchdog Timer Postscaler (1:1048576)
#pragma config FCKSM = 0b11     // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config OSCIOFNC = 0b1   // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config POSCMOD = 0b00   // Primary Oscillator Configuration (EC mode selected)
#pragma config IESO = 0b0       // Internal/External Switch Over (Disabled)
#pragma config FSOSCEN = 0b0    // Secondary Oscillator Enable (Disable SOSC)
#pragma config DMTINTV = 0b111  // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FNOSC = 0b001    // Oscillator Selection Bits (SPLL)

/**
 * SYSCLK == SPLL == ((FPLLICLK / FPLLIDIV) / FPLLODIV) * FPLLMULT ==
 * ((POSC / 3) / 2) * 50 == (24Mhz / 6) * 50 == 200Mhz
 */

// DEVCFG2
#pragma config UPLLFSEL = 0b1      // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)
#pragma config FPLLODIV = 0b001    // Default System PLL Output Divisor (PLL output divided by 2)
#pragma config FPLLMULT = 0b110001 // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLICLK = 0b0      // System PLL Input Clock Selection (POSC is input to the System PLL)
#pragma config FPLLRNG = 0b010     // System PLL Divided Input Clock Frequency Range (8-16Mhz)
#pragma config FPLLIDIV = 0b010    // System PLL Input Divider (Divide by 3)

// DEVCFG3
#pragma config FUSBIDIO = 0b0  // USB USBID Selection (Controlled by the port function)
#pragma config IOL1WAY = 0b0   // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = 0b0  // Peripheral Module Disable Configuration (Allow multiple reconfiguration)
#pragma config PGL1WAY = 0b0   // Permission Group Lock One Way Configuration (Allow multiple reconfiguration)
#pragma config FETHIO = 0b1    // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FMIIEN = 0b1    // Ethernet RMII/MII Enable (MII Enabled)
#pragma config USERID = 0xBEEF // 16-bit User Defined Value (0xBEEF)

/**
 * Unlock Sequence
 */
void unlock_config(void) {
  asm volatile("di"); // Disable interrupts
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
}

/**
 * Lock Sequence
 */
void lock_config(void) {
  SYSKEY = 0xDEADBEEF;
  asm volatile("ei"); // Enable interrupts
}

void init_general(void) {
  unlock_config();

  // PRECON
  PRECONbits.PFMSECEN = 0;  // Flash SEC Interrupt Enable (Do not generate an interrupt when the PFMSEC bit is set)
  PRECONbits.PREFEN = 0b11; // Predictive Prefetch Enable (Enable predictive prefetch for any address)
  PRECONbits.PFMWS = 0b010; // PFM Access Time Defined in Terms of SYSCLK Wait States (Two wait states)

  // CFGCON
  CFGCONbits.DMAPRI = 0;    // DMA Read and DMA Write Arbitration Priority to SRAM (DMA uses Least Recently Serviced Arbitration)
  CFGCONbits.CPUPRI = 0;    // CPU Arbitration Priority to SRAM When Servicing an Interrupt (CPU uses Least Recently Serviced Arbitration)
  CFGCONbits.ICACLK = 0;    // Input Capture Alternate Clock Selection (All Input Capture modules use Timer2/3 as their timebase clock)
  CFGCONbits.OCACLK = 0;    // Output Compare Alternate Clock Selection (All Output Compare modules use Timer2/3 as their timebase clock)
  CFGCONbits.IOLOCK = 1;    // Peripheral Pin Select Lock (Peripheral Pin Select is locked. Writes to PPS registers are not allowed)
  CFGCONbits.PMDLOCK = 1;   // Peripheral Module Disable (Peripheral module is locked. Writes to PMD registers are not allowed)
  CFGCONbits.PGLOCK = 1;    // Permission Group Lock (Permission Group registers are locked. Writes to PG registers are not allowed)
  CFGCONbits.USBSSEN = 1;   // USB Suspend Sleep Enable (USB PHY clock is shut down when Sleep mode is active)
  CFGCONbits.IOANCPN = 0;   // I/O Analog Charge Pump Enable (Charge pump disabled)
  CFGCONbits.ECCCON = 0b10; // Flash ECC Configuration (ECC and dynamic ECC are disabled (ECCCON<1:0> bits are locked))
  CFGCONbits.JTAGEN = 0;    // JTAG Port Enable (Disable the JTAG port)
  CFGCONbits.TROEN = 0;     // Trace Output Enable (Disable trace outputs and stop trace clock)
  CFGCONbits.TDOEN = 1;     // TDO Enable for 2-Wire JTAG (2-wire JTAG protocol uses TDO)

  // CFGPG
  CFGPGbits.CRYPTPG = 0; // Crypto Engine Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.FCPG = 0;    // Flash Control Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.SQI1PG = 0;  // SQI Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.ETHPG = 0;   // Ethernet Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.CAN2PG = 0;  // CAN2 Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.CAN1PG = 0;  // CAN1 Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.USBPG = 0;   // USB Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.DMAPG = 0;   // DMA Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.CPUPG = 0;   // CPU Permission Group (Initiator is assigned to Permission Group 0)

  // INTCON
  INTCONbits.MVEC = 1; // Multi Vector Configuration Bit (Configured for multi-vectored mode)

  // PRISS
  PRISSbits.PRI7SS = 7; // IPL 7 Shadow Set (Shadow Set 7)
  PRISSbits.PRI6SS = 6; // IPL 6 Shadow Set (Shadow Set 6)
  PRISSbits.PRI5SS = 5; // IPL 5 Shadow Set (Shadow Set 7)
  PRISSbits.PRI4SS = 4; // IPL 4 Shadow Set (Shadow Set 4)
  PRISSbits.PRI3SS = 3; // IPL 3 Shadow Set (Shadow Set 3)
  PRISSbits.PRI2SS = 2; // IPL 2 Shadow Set (Shadow Set 2)
  PRISSbits.PRI1SS = 1; // IPL 1 Shadow Set (Shadow Set 1)
  PRISSbits.SS0 = 0;    // Single Vector Shadow Set (Not presented with a shadow set)

  lock_config();
}

void init_peripheral_modules(void) {
  unlock_config();
  CFGCONbits.PMDLOCK = 0;

  /**
   * To disable a module, first set its ON bit (or equivalent) to 0 and then set
   * the appropriate PMD bit to 1 (disabled).
   */

  // ADC
  //TODO: ADC ON bit
  //PMD1bits.ADCMD = 1;

  // Comparator Voltage Reference
  CVRCONbits.ON = 0;
  PMD1bits.CVRMD = 1;

  // Comparator 1
  CM1CONbits.ON = 0;
  PMD2bits.CMP1MD = 1;

  // Comparator 2
  CM2CONbits.ON = 0;
  PMD2bits.CMP2MD = 1;

  // Input Capture 1
  IC1CONbits.ON = 0;
  PMD3bits.IC1MD = 1;

  // Input Capture 2
  IC2CONbits.ON = 0;
  PMD3bits.IC2MD = 1;

  // Input Capture 3
  IC3CONbits.ON = 0;
  PMD3bits.IC3MD = 1;

  // Input Capture 4
  IC4CONbits.ON = 0;
  PMD3bits.IC4MD = 1;

  // Input Capture 5
  IC5CONbits.ON = 0;
  PMD3bits.IC5MD = 1;

  // Input Capture 6
  IC6CONbits.ON = 0;
  PMD3bits.IC6MD = 1;

  // Input Capture 7
  IC7CONbits.ON = 0;
  PMD3bits.IC7MD = 1;

  // Input Capture 8
  IC8CONbits.ON = 0;
  PMD3bits.IC8MD = 1;

  // Input Capture 9
  IC9CONbits.ON = 0;
  PMD3bits.IC9MD = 1;

  // Output Compare 1
  OC1CONbits.ON = 0;
  PMD3bits.OC1MD = 1;

  // Output Compare 2
  OC2CONbits.ON = 0;
  PMD3bits.OC2MD = 1;

  // Output Compare 3
  OC3CONbits.ON = 0;
  PMD3bits.OC3MD = 1;

  // Output Compare 4
  OC4CONbits.ON = 0;
  PMD3bits.OC4MD = 1;

  // Output Compare 5
  OC5CONbits.ON = 0;
  PMD3bits.OC5MD = 1;

  // Output Compare 6
  OC6CONbits.ON = 0;
  PMD3bits.OC6MD = 1;

  // Output Compare 7
  OC7CONbits.ON = 0;
  PMD3bits.OC7MD = 1;

  // Output Compare 8
  OC8CONbits.ON = 0;
  PMD3bits.OC8MD = 1;

  // Output Compare 9
  OC9CONbits.ON = 0;
  PMD3bits.OC9MD = 1;

  // Timer1
  //T1CONbits.ON = 0;
  //PMD4bits.T1MD = 1;

  // Timer2
  //T2CONbits.ON = 0;
  //PMD4bits.T2MD = 1;

  // Timer3
  T3CONbits.ON = 0;
  PMD4bits.T3MD = 1;

  // Timer4
  T4CONbits.ON = 0;
  PMD4bits.T4MD = 1;

  // Timer5
  T5CONbits.ON = 0;
  PMD4bits.T5MD = 1;

  // Timer6
  T6CONbits.ON = 0;
  PMD4bits.T6MD = 1;

  // Timer7
  T7CONbits.ON = 0;
  PMD4bits.T7MD = 1;

  // Timer8
  T8CONbits.ON = 0;
  PMD4bits.T8MD = 1;

  // Timer9
  T9CONbits.ON = 0;
  PMD4bits.T9MD = 1;

  // UART1
  U1MODEbits.ON = 0;
  PMD5bits.U1MD = 1;

  // UART2
  U2MODEbits.ON = 0;
  PMD5bits.U2MD = 1;

  // UART3
  U3MODEbits.ON = 0;
  PMD5bits.U3MD = 1;

  // UART4
  U4MODEbits.ON = 0;
  PMD5bits.U4MD = 1;

  // UART5
  U5MODEbits.ON = 0;
  PMD5bits.U5MD = 1;

  // UART6
  U6MODEbits.ON = 0;
  PMD5bits.U6MD = 1;

  // SPI1
  //SPI1CONbits.ON = 0;
  //PMD5bits.SPI1MD = 1;

  // SPI2
  SPI2CONbits.ON = 0;
  PMD5bits.SPI2MD = 1;

  // SPI3
  SPI3CONbits.ON = 0;
  PMD5bits.SPI3MD = 1;

  // SPI4
  SPI4CONbits.ON = 0;
  PMD5bits.SPI4MD = 1;

  // SPI5
  SPI5CONbits.ON = 0;
  PMD5bits.SPI5MD = 1;

  // SPI6
  SPI6CONbits.ON = 0;
  PMD5bits.SPI6MD = 1;

  // I2C1
  I2C1CONbits.ON = 0;
  PMD5bits.I2C1MD = 1;

  // I2C3
  I2C3CONbits.ON = 0;
  PMD5bits.I2C3MD = 1;

  // I2C4
  I2C4CONbits.ON = 0;
  PMD5bits.I2C4MD = 1;

  // I2C5
  I2C5CONbits.ON = 0;
  PMD5bits.I2C5MD = 1;

  // USB
  //TODO: USB ON bit
  //PMD5bits.USBMD = 1;

  // CAN 1
  //C1CONbits.ON = 0;
  //PMD5bits.CAN1MD = 1;

  // CAN 2
  C2CONbits.ON = 0;
  PMD5bits.CAN2MD = 1;

  // RTCC
  RTCCONbits.ON = 0;
  PMD6bits.RTCCMD = 1;

  /**
   * Note: Reference clock outputs are not disabled due to an error condition
   * noted in the revision A1 errata.
   */

  // Reference Clock Output 1
  //REFO1CONbits.ON = 0;
  //PMD6bits.REFO1MD = 1;

  // Reference Clock Output 2
  //REFO2CONbits.ON = 0;
  //PMD6bits.REFO2MD = 1;

  // Reference Clock Output 3
  //REFO3CONbits.ON = 0;
  //PMD6bits.REFO3MD = 1;

  // Reference Clock Output 4
  //REFO4CONbits.ON = 0;
  //PMD6bits.REFO4MD = 1;

  // PMP
  PMCONbits.ON = 0;
  PMD6bits.PMPMD = 1;

  // SQI 1
  SQI1CFGbits.SQIEN = 0;
  PMD6bits.SQI1MD = 1;

  // Ethernet
  ETHCON1bits.ON = 0;
  PMD6bits.ETHMD = 1;

  // DMA
  DMACONbits.ON = 0;
  PMD7bits.DMAMD = 1;

  // Random Number Generator
  RNGCONbits.PRNGEN = 0;
  RNGCONbits.TRNGEN = 0;
  PMD7bits.RNGMD = 1;

  // Crypto
  CECONbits.DMAEN = 0;
  PMD7bits.CRYPTMD = 1;

  CFGCONbits.PMDLOCK = 1;
  lock_config();
}

void init_gpio_pins(void) {
  // Set to outputs
  TRISAbits.TRISA0 = OUTPUT;
  TRISAbits.TRISA1 = OUTPUT;
  TRISAbits.TRISA2 = OUTPUT;
  TRISAbits.TRISA3 = OUTPUT;
  TRISAbits.TRISA4 = OUTPUT;
  TRISAbits.TRISA5 = OUTPUT;
  TRISAbits.TRISA6 = OUTPUT;
  TRISAbits.TRISA7 = OUTPUT;
  TRISAbits.TRISA9 = OUTPUT;
  TRISAbits.TRISA10 = OUTPUT;
  TRISAbits.TRISA14 = OUTPUT;
  TRISAbits.TRISA15 = OUTPUT;

  // TRISB
  TRISBbits.TRISB0 = OUTPUT;
  TRISBbits.TRISB1 = OUTPUT;
  TRISBbits.TRISB2 = OUTPUT;
  TRISBbits.TRISB3 = OUTPUT;
  TRISBbits.TRISB4 = OUTPUT;
  TRISBbits.TRISB5 = OUTPUT;
  TRISBbits.TRISB6 = OUTPUT;
  TRISBbits.TRISB7 = OUTPUT;
  TRISBbits.TRISB8 = OUTPUT;
  TRISBbits.TRISB9 = OUTPUT;
  TRISBbits.TRISB10 = OUTPUT;
  TRISBbits.TRISB11 = OUTPUT;
  TRISBbits.TRISB12 = OUTPUT;
  TRISBbits.TRISB13 = OUTPUT;
  TRISBbits.TRISB14 = OUTPUT;
  TRISBbits.TRISB15 = OUTPUT;

  // TRISC
  TRISCbits.TRISC1 = OUTPUT;
  TRISCbits.TRISC2 = OUTPUT;
  TRISCbits.TRISC3 = OUTPUT;
  TRISCbits.TRISC4 = OUTPUT;
  TRISCbits.TRISC12 = OUTPUT;
  TRISCbits.TRISC13 = OUTPUT;
  TRISCbits.TRISC14 = OUTPUT;
  TRISCbits.TRISC15 = OUTPUT;

  // TRISD
  TRISDbits.TRISD0 = OUTPUT;
  TRISDbits.TRISD1 = OUTPUT;
  TRISDbits.TRISD2 = OUTPUT;
  TRISDbits.TRISD3 = OUTPUT;
  TRISDbits.TRISD4 = OUTPUT;
  TRISDbits.TRISD5 = OUTPUT;
  TRISDbits.TRISD9 = OUTPUT;
  TRISDbits.TRISD10 = OUTPUT;
  TRISDbits.TRISD11 = OUTPUT;
  TRISDbits.TRISD12 = OUTPUT;
  TRISDbits.TRISD13 = OUTPUT;
  TRISDbits.TRISD14 = OUTPUT;
  TRISDbits.TRISD15 = OUTPUT;

  // TRISE
  TRISEbits.TRISE0 = OUTPUT;
  TRISEbits.TRISE1 = OUTPUT;
  TRISEbits.TRISE2 = OUTPUT;
  TRISEbits.TRISE3 = OUTPUT;
  TRISEbits.TRISE4 = OUTPUT;
  TRISEbits.TRISE5 = OUTPUT;
  TRISEbits.TRISE6 = OUTPUT;
  TRISEbits.TRISE7 = OUTPUT;
  TRISEbits.TRISE8 = OUTPUT;
  TRISEbits.TRISE9 = OUTPUT;

  // TRISF
  TRISFbits.TRISF0 = OUTPUT;
  TRISFbits.TRISF1 = OUTPUT;
  TRISFbits.TRISF2 = OUTPUT;
  TRISFbits.TRISF3 = OUTPUT;
  TRISFbits.TRISF4 = OUTPUT;
  TRISFbits.TRISF5 = OUTPUT;
  TRISFbits.TRISF8 = OUTPUT;
  TRISFbits.TRISF12 = OUTPUT;
  TRISFbits.TRISF13 = OUTPUT;

  // TRISG
  TRISGbits.TRISG0 = OUTPUT;
  TRISGbits.TRISG1 = OUTPUT;
  TRISGbits.TRISG6 = OUTPUT;
  TRISGbits.TRISG7 = OUTPUT;
  TRISGbits.TRISG8 = OUTPUT;
  TRISGbits.TRISG9 = OUTPUT;
  TRISGbits.TRISG12 = OUTPUT;
  TRISGbits.TRISG13 = OUTPUT;
  TRISGbits.TRISG14 = OUTPUT;
  TRISGbits.TRISG15 = OUTPUT;

  // Drive to logic level low

  // LATA
  LATAbits.LATA0 = 0;
  LATAbits.LATA1 = 0;
  LATAbits.LATA2 = 0;
  LATAbits.LATA3 = 0;
  LATAbits.LATA4 = 0;
  LATAbits.LATA5 = 0;
  LATAbits.LATA6 = 0;
  LATAbits.LATA7 = 0;
  LATAbits.LATA9 = 0;
  LATAbits.LATA10 = 0;
  LATAbits.LATA14 = 0;
  LATAbits.LATA15 = 0;

  // LATB
  LATBbits.LATB0 = 0;
  LATBbits.LATB1 = 0;
  LATBbits.LATB2 = 0;
  LATBbits.LATB3 = 0;
  LATBbits.LATB4 = 0;
  LATBbits.LATB5 = 0;
  LATBbits.LATB6 = 0;
  LATBbits.LATB7 = 0;
  LATBbits.LATB8 = 0;
  LATBbits.LATB9 = 0;
  LATBbits.LATB10 = 0;
  LATBbits.LATB11 = 0;
  LATBbits.LATB12 = 0;
  LATBbits.LATB13 = 0;
  LATBbits.LATB14 = 0;
  LATBbits.LATB15 = 0;

  // LATC
  LATCbits.LATC1 = 0;
  LATCbits.LATC2 = 0;
  LATCbits.LATC3 = 0;
  LATCbits.LATC4 = 0;
  LATCbits.LATC12 = 0;
  LATCbits.LATC13 = 0;
  LATCbits.LATC14 = 0;
  LATCbits.LATC15 = 0;

  // LATD
  LATDbits.LATD0 = 0;
  LATDbits.LATD1 = 0;
  LATDbits.LATD2 = 0;
  LATDbits.LATD3 = 0;
  LATDbits.LATD4 = 0;
  LATDbits.LATD5 = 0;
  LATDbits.LATD9 = 0;
  LATDbits.LATD10 = 0;
  LATDbits.LATD11 = 0;
  LATDbits.LATD12 = 0;
  LATDbits.LATD13 = 0;
  LATDbits.LATD14 = 0;
  LATDbits.LATD15 = 0;

  // LATE
  LATEbits.LATE0 = 0;
  LATEbits.LATE1 = 0;
  LATEbits.LATE2 = 0;
  LATEbits.LATE3 = 0;
  LATEbits.LATE4 = 0;
  LATEbits.LATE5 = 0;
  LATEbits.LATE6 = 0;
  LATEbits.LATE7 = 0;
  LATEbits.LATE8 = 0;
  LATEbits.LATE9 = 0;

  // LATF
  LATFbits.LATF0 = 0;
  LATFbits.LATF1 = 0;
  LATFbits.LATF2 = 0;
  LATFbits.LATF3 = 0;
  LATFbits.LATF4 = 0;
  LATFbits.LATF5 = 0;
  LATFbits.LATF8 = 0;
  LATFbits.LATF12 = 0;
  LATFbits.LATF13 = 0;

  // LATG
  LATGbits.LATG0 = 0;
  LATGbits.LATG1 = 0;
  LATGbits.LATG6 = 0;
  LATGbits.LATG7 = 0;
  LATGbits.LATG8 = 0;
  LATGbits.LATG9 = 0;
  LATGbits.LATG12 = 0;
  LATGbits.LATG13 = 0;
  LATGbits.LATG14 = 0;
  LATGbits.LATG15 = 0;
}

void init_oscillator(void) {
  unlock_config();

  // OSCCON
  OSCCONbits.FRCDIV = 0b000; // Internal Fast RC (FRC) Oscillator Clock Divider (FRC divided by 1)
  OSCCONbits.DRMEN = 0;      // Dream Mode Enable (Dream mode is disabled)
  OSCCONbits.SLP2SPD = 0;    // Sleep 2-speed Startup Control (Use the selected clock directly)
  OSCCONbits.CLKLOCK = 0;    // Clock Selection Lock Enable (Clock and PLL selections are not locked and may be modified)
  OSCCONbits.SLPEN = 0;      // Sleep Mode Enable (Device will enter Idle mode when a WAIT instruction is executed)
  OSCCONbits.SOSCEN = 0;     // Secondary Oscillator (SOSC) Enable (Disable Secondary Oscillator)

  // OSCTUN
  OSCTUNbits.TUN = 0b00000; // FRC Oscillator Tuning (Center frequency. Oscillator runs at calibrated frequency (8 MHz))

  // PB1DIV
  while(!PB1DIVbits.PBDIVRDY);
  PB1DIVbits.PBDIV = 0b0000001; // Peripheral Bus 1 Clock Divisor Control (PBCLK1 is SYSCLK divided by 2)

  // PB2DIV
  PB2DIVbits.ON = 1;            // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
  while(!PB2DIVbits.PBDIVRDY);
  PB2DIVbits.PBDIV = 0b0000001; // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 2)

  // PB3DIV
  PB3DIVbits.ON = 1;            // Peripheral Bus 3 Output Clock Enable (Output clock is enabled)
  while(!PB3DIVbits.PBDIVRDY);
  PB3DIVbits.PBDIV = 0b0110001; // Peripheral Bus 3 Clock Divisor Control (PBCLK3 is SYSCLK divided by 50)

  // PB4DIV
  PB4DIVbits.ON = 1;            // Peripheral Bus 4 Output Clock Enable (Output clock is enabled)
  while(!PB4DIVbits.PBDIVRDY);
  PB4DIVbits.PBDIV = 0b0000001; // Peripheral Bus 4 Clock Divisor Control (PBCLK4 is SYSCLK divided by 2)

  // PB5DIV
  PB5DIVbits.ON = 1;            // Peripheral Bus 5 Output Clock Enable (Output clock is enabled)
  while(!PB5DIVbits.PBDIVRDY);
  PB5DIVbits.PBDIV = 0b0000001; // Peripheral Bus 5 Clock Divisor Control (PBCLK5 is SYSCLK divided by 2)

  // PB7DIV
  PB7DIVbits.ON = 1;            // Peripheral Bus 7 Output Clock Enable (Output clock is enabled)
  while(!PB7DIVbits.PBDIVRDY);
  PB7DIVbits.PBDIV = 0b0000000; // Peripheral Bus 7 Clock Divisor Control (PBCLK7 is SYSCLK divided by 1)

  // PB8DIV
  PB8DIVbits.ON = 1;            // Peripheral Bus 8 Output Clock Enable (Output clock is enabled)
  while(!PB8DIVbits.PBDIVRDY);
  PB8DIVbits.PBDIV = 0b0000001; // Peripheral Bus 8 Clock Divisor Control (PBCLK8 is SYSCLK divided by 2)

#ifdef REFCLKO
  /**
   * REFO1CLK == (PBCLK1 / (2 * (RODIV + (ROTRIM / 512)))) ==
   * (100Mhz / (2 * (2 + (256/512)))) == (100Mhz / 5) == 20Mhz
   *
   * In other words, REFO1CLK is set up to output at (SYSCLK / 10).
   */

  // Initialize REFCLKO1 PPS pin
  CFGCONbits.IOLOCK = 0;
  TRISFbits.TRISF0 = OUTPUT;
  RPF0R = 0b1111; // Assign REFCLKO1 to RF0
  CFGCONbits.IOLOCK = 1;

  // REFO1CON
  REFO1CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO1CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)

  REFO1CONbits.ROSEL = 0b0001;            // Reference Clock Source Select (PBCLK1)
  REFO1CONbits.SIDL = 1;                  // Peripheral Stop in Idle Mode (Discontinue module operation when device enters Idle mode)
  REFO1CONbits.OE = 1;                    // Reference Clock Output Enable (Reference clock is driven out on REFCLKO1 pin)

  REFO1CONbits.DIVSWEN = 1;               // Divider Switch Enable (Divider switch is in progress)
  REFO1CONbits.RODIV = 0b000000000000010; // Reference Clock Divider (Divide by 2)
  REFO1CONbits.DIVSWEN = 0;               // Divider Switch Enable (Divider switch is complete)

  // REFO1TRIM
  REFO1TRIMbits.ROTRIM = 0b100000000;     // Reference Oscillator Trim (256/512 divisor added to RODIV value)

  // Enable REFCLKO1
  REFO1CONbits.ACTIVE = 1;                // Reference Clock Request Status (Reference clock request is active)
  REFO1CONbits.ON = 1;                    // Output Enable (Reference Oscillator Module enabled)
#else
  // REF01CON
  REFO1CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO1CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO1CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO2 pin)
#endif

  // REF02CON
  REFO2CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO2CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO2CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO2 pin)

  // REF03CON
  REFO3CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO3CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO3CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO3 pin)

  // REF04CON
  REFO4CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO4CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO4CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO4 pin)

  lock_config();
}

void init_timer1(void) {
  unlock_config();

  // Disable TMR1
  T1CONbits.ON = 0; // Timer On (Timer is disabled)

  // T1CON
  T1CONbits.TCS = 0;      // Timer Clock Source Select (Internal peripheral clock)
  T1CONbits.SIDL = 0;     // Stop in Idle Mode (Continue operation even in Idle mode)
  T1CONbits.TWDIS = 1;    // Asynchronous Timer Write Disable (Writes to TMR1 are ignored until pending write operation completes)
  T1CONbits.TGATE = 0;    // Timer Gated Time Accumulation Enable (Gated time accumulation is disabled)
  T1CONbits.TCKPS = 0b10; // Timer Input Clock Prescale Select (1:64 prescale value)

  // TMR1
  TMR1 = 0; // TMR1 Count Register (0)

  /**
   * The clock source is PBCLK3, which is configured to run at SYSCLOCK / 50.
   * Currently, this gives a speed of 4Mhz. TMR1 uses a 1:64 prescale, meaning
   * 1 second should be equal to 4000000 / 64 == 62500 TMR1 cycles.
   */

  // PR1
  PR1 = 0xF424; // PR1 Period Register (62500)

  // Set up TMR1 Interrupt
  IFS0bits.T1IF = 0; // TMR1 Interrupt Flag Status (No interrupt request has occured)
  IPC1bits.T1IP = 5; // TMR1 Interrupt Priority (Interrupt priority is 5)
  IPC1bits.T1IS = 3; // TMR1 Interrupt Subpriority (Interrupt subpriority is 3)
  IEC0bits.T1IE = 1; // TMR1 Interrupt Enable Control (Interrupt is enabled)

  // Enable TMR1
  T1CONbits.ON = 1; // Timer On (Timer is enabled)

  lock_config();
}

/**
 * void init_timer2(void)
 *
 * Initializes Timer 2, which is configured to generate an interrupt every 1 ms
 */
void init_timer2(void) {
  unlock_config();

  // Disable TMR2
  T2CONbits.ON = 0; // Timer On (Timer is disabled)

  // T2CON
  T2CONbits.TCS = 0;       // Timer Clock Source Select (Internal peripheral clock)
  T2CONbits.SIDL = 0;      // Stop in Idle Mode (Continue operation even in Idle mode)
  T2CONbits.TGATE = 0;     // Timer Gated Time Accumulation Enable (Gated time accumulation is disabled)
  T2CONbits.TCKPS = 0b010; // Timer Input Clock Prescale Select (1:4 prescale value)

  // TMR2
  TMR2 = 0; // TMR2 Count Register (0)

  /**
   * The clock source is PBCLK3, which is configured to run at SYSCLOCK / 50.
   * Currently, this gives a speed of 4Mhz. TMR2 uses a 1:4 prescale, meaning
   * 1 millisecond should be equal to 4000 / 4  == 1000 TMR2 cycles.
   */

  // PR2
  PR2 = 0x3E8; // PR2 Period Register (1000)

  // Set up TMR2 Interrupt
  IFS0bits.T2IF = 0; // TMR2 Interrupt Flag Status (No interrupt request has occured)
  IPC2bits.T2IP = 6; // TMR2 Interrupt Priority (Interrupt priority is 6)
  IPC2bits.T2IS = 3; // TMR2 Interrupt Subpriority (Interrupt subpriority is 3)
  IEC0bits.T2IE = 1; // TMR2 Interrupt Enable Control (Interrupt is enabled)

  // Enable TMR2
  T2CONbits.ON = 1; // Timer On (Timer is enabled)

  lock_config();
}

void init_spi() {
  unlock_config();

  // Initialize SDI1/SDO1 PPS pins
  CFGCONbits.IOLOCK = 0;
  TRISBbits.TRISB9 = INPUT;
  SDI1Rbits.SDI1R = 0b0101; // RPB9
  TRISBbits.TRISB10 = OUTPUT;
  RPB10Rbits.RPB10R = 0b0101; // SDO1
  CFGCONbits.IOLOCK = 1;

  // Disable interrupts
  IEC3bits.SPI1EIE = 0;
  IEC3bits.SPI1RXIE = 0;
  IEC3bits.SPI1TXIE = 0;

  // Disable SPI1 module
  SPI1CONbits.ON = 0;

  // Clear receive buffer
  SPI1BUF = 0;

  // Use standard buffer mode
  SPI1CONbits.ENHBUF = 0;

  /**
   * F_SCK = F_PBCLK2 / (2 * (SPI1BRG + 1))
   * F_SCK = 100Mhz / (2 * (4 + 1))
   * F_SCK = 10Mhz
   */

  // Set the baud rate (see above equation)
  SPI1BRG = 4;

  SPI1STATbits.SPIROV = 0;

  SPI1CONbits.MCLKSEL = 0; // Master Clock Enable bit (PBCLK2 is used by the Baud Rate Generator)
  SPI1CONbits.SIDL = 0;    // Stop in Idle Mode bit (Continue operation in Idle mode)
  SPI1CONbits.MODE32 = 0;  // 32/16-Bit Communication Select bits (16-bit)
  SPI1CONbits.MODE16 = 1;  // 32/16-Bit Communication Select bits (16-bit)
  SPI1CONbits.MSTEN = 1;   // Master Mode Enable bit (Master mode)
  SPI1CONbits.CKE = 1;     // SPI Clock Edge Select bit (Serial output data changes on transition from active clock state to idle clock state)

  // Enable SPI1 module
  SPI1CONbits.ON = 1;

  lock_config();
}


/**
 * void init_termination(void)
 *
 * Sets up programmable CAN termination based on user defines.
 */
void init_termination(void) {
  // Initialize pin
  TERM_TRIS = OUTPUT;

  // Set termination based on value defined in specific node's header
  TERM_LAT = TERMINATING;
}
