/**
 * Logger Config
 *
 * Processor:   PIC32MZ2048ECM064
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2015-2016
 */
#include "Logger_config.h"

// PIC32MZ2048ECM064 Configuration Bit Settings

// DEVCFG3
#pragma config FUSBIDIO = ON   // USB USBID Selection (Controlled by the USB Module)
#pragma config IOL1WAY = OFF   // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config PMDL1WAY = OFF  // Peripheral Module Disable Configuration (Allow multiple reconfiguration)
#pragma config PGL1WAY = OFF   // Permission Group Lock One Way Configuration (Allow multiple reconfiguration)
#pragma config FETHIO = ON     // Ethernet I/O Pin Select (Default Ethernet I/O)
#pragma config FMIIEN = ON     // Ethernet RMII/MII Enable (MII Enabled)
#pragma config USERID = 0xDEAD // 16-bit User Defined Value (0xDEAD)

// DEVCFG2
#pragma config UPLLEN = OFF          // (May not exist) USB PLL Enable (USB PLL is disabled)
#pragma config UPLLFSEL = FREQ_24MHZ // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)
#pragma config FPLLODIV = 0b001      // Default System PLL Output Divisor (PLL output divided by 2)
#pragma config FPLLMULT = MUL_50     // System PLL Multiplier (PLL Multiply by 50)
#pragma config FPLLICLK = PLL_FRC    // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLRNG = 0b001       // System PLL Divided Input Clock Frequency Range (5-10Mhz)
#pragma config FPLLIDIV = 0b000      // System PLL Input Divider (Divide by 1)

/**
 * SYSCLK == SPLL == ((FPLLICLK / FPLLIDIV) / FPLLODIV) * FPLLMULT ==
 * ((FRC / 1) / 2) * 50 == (8Mhz / 2) * 50 == 200Mhz
 *
 * NOTE: FPLLIDIV seems to do absolutely nothing, no matter what it is set at.
 */

// DEVCFG1
#pragma config FDMTEN = OFF          // Deadman Timer Enable (Deadman Timer is disbled)
#pragma config DMTCNT = 0b10111      // Deadman Timer Count Select Bits (2^31)
#pragma config FWDTWINSZ = WINSZ_25  // Watchdog Timer Window Size (Window size is 25%)
#pragma config FWDTEN = OFF          // Watchdog Timer Enable (WDT Disabled)
#pragma config WINDIS = NORMAL       // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config WDTSPGM = STOP        // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WDTPS = PS1048576     // Watchdog Timer Postscaler (1:1048576)
#pragma config FCKSM = CSECME        // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config OSCIOFNC = ON         // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config POSCMOD = 0b11        // Primary Oscillator Configuration (POSC disabled)
#pragma config IESO = OFF            // Internal/External Switch Over (Disabled)
#pragma config FSOSCEN = OFF         // Secondary Oscillator Enable (Disable SOSC)
#pragma config DMTINTV = WIN_127_128 // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FNOSC = 0b001         // Oscillator Selection Bits (SPLL)

// DEVCFG0
#pragma config EJTAGBEN = NORMAL      // EJTAG Boot (Normal EJTAG functionality)
#pragma config DBGPER = ALLOW_PG2     // Debug Mode CPU Access Permission (Allow CPU access to Permission Group 2 permission regions)
#pragma config FSLEEP = OFF           // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config FECCCON = OFF_UNLOCKED // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config BOOTISA = MIPS32       // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config TRCEN = OFF            // Trace Enable (Trace features in the CPU are disabled)
#pragma config ICESEL = ICS_PGx1      // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config JTAGEN = OFF           // JTAG Enable (JTAG Port Enabled)
#pragma config DEBUG = ON             // Background Debugger Enable (Debugger is disabled)

// DEVCP0
#pragma config CP = OFF // Code Protect (Protection Disabled)

/**
 * We have to allocate memory in RAM for the CAN1 modules's FIFOs. There can be
 * up to 32 FIFOs, and each FIFO can contain up to 32 message buffers for a
 * total of 1024 message buffers. Each message buffer takes up 4 words
 * (16 bytes) 2 for the CAN message data and 2 for a timestamp. Note that on a
 * PIC32 an int is 1 word (4 bytes).
 *
 * Here, we allocate 256 words, enough for 32 message buffers in two FIFOs.
 */
unsigned int CAN_FIFO_Buffers[256];

// Unlock Sequence
void unlock_config(void) {
  asm volatile("di"); // Disable interrupts
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;
}

// Lock Sequence
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
  //CFGPGbits.CAN1PG = 0;  // CAN1 Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.USBPG = 0;   // USB Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.DMAPG = 0;   // DMA Module Permission Group (Initiator is assigned to Permission Group 0)
  CFGPGbits.CPUPG = 0;   // CPU Permission Group (Initiator is assigned to Permission Group 0)

  lock_config();
}

void init_peripheral_modules(void) {
  unlock_config();
  CFGCONbits.PMDLOCK = 0;

  /**
   * To disable a module, first set its ON bit (or equivalent) to 0 and then set
   * the appropriate PMD bit to 1 (disabled).
   */

  // ADC 1
  AD1CON1bits.ADCEN = 0;
  PMD1bits.AD1MD = 1;

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
  T2CONbits.ON = 0;
  PMD4bits.T2MD = 1;

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
  SPI1CONbits.ON = 0;
  PMD5bits.SPI1MD = 1;

  // SPI2
  SPI2CONbits.ON = 0;
  PMD5bits.SPI2MD = 1;

  // SPI3
  SPI3CONbits.ON = 0;
  PMD5bits.SPI3MD = 1;

  // SPI4
  SPI4CONbits.ON = 0;
  PMD5bits.SPI4MD = 1;

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
  //TODO: USB ON bit?
  PMD5bits.USBMD = 1;

  // CAN 1
  //C1CONbits.ON = 0;
  //PMD5bits.CAN1MD = 1;

  // CAN 2
  C2CONbits.ON = 0;
  PMD5bits.CAN2MD = 1;

  // RTCC
  RTCCONbits.ON = 0;
  PMD6bits.RTCCMD = 1;

  // Reference Clock Output 1
  //REFO1CONbits.ON = 0;
  //PMD6bits.REFO1MD = 1;

  // Reference Clock Output 2
  REFO2CONbits.ON = 0;
  PMD6bits.REFO2MD = 1;

  // Reference Clock Output 3
  REFO3CONbits.ON = 0;
  PMD6bits.REFO3MD = 1;

  // Reference Clock Output 4
  REFO4CONbits.ON = 0;
  PMD6bits.REFO4MD = 1;

  // PMP
  PMCONbits.ON = 0;
  PMD6bits.PMPMD = 1;

  // SQI 1
  //TODO: SQI ON bit?
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

void init_unused_pins(void) {
  // Set to outputs

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

  // TRISE
  TRISEbits.TRISE0 = OUTPUT;
  TRISEbits.TRISE1 = OUTPUT;
  TRISEbits.TRISE2 = OUTPUT;
  TRISEbits.TRISE3 = OUTPUT;
  TRISEbits.TRISE4 = OUTPUT;
  TRISEbits.TRISE5 = OUTPUT;
  TRISEbits.TRISE6 = OUTPUT;
  TRISEbits.TRISE7 = OUTPUT;

  // TRISF
  TRISFbits.TRISF0 = OUTPUT;
  TRISFbits.TRISF1 = OUTPUT;
  TRISFbits.TRISF3 = OUTPUT;
  TRISFbits.TRISF4 = OUTPUT;
  TRISFbits.TRISF5 = OUTPUT;

  // Drive to logic level low

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

  // LATE
  LATEbits.LATE0 = 0;
  LATEbits.LATE1 = 0;
  LATEbits.LATE2 = 0;
  LATEbits.LATE3 = 0;
  LATEbits.LATE4 = 0;
  LATEbits.LATE5 = 0;
  LATEbits.LATE6 = 0;
  LATEbits.LATE7 = 0;

  // LATF
  LATFbits.LATF0 = 0;
  LATFbits.LATF1 = 0;
  LATFbits.LATF3 = 0;
  LATFbits.LATF4 = 0;
  LATFbits.LATF5 = 0;
}

void init_oscillator(void) {
  unlock_config();

  // OSCCON
  OSCCONbits.FRCDIV = 0b000; // Internal Fast RC (FRC) Oscillator Clock Divider (FRC divided by 1)
  OSCCONbits.DRMEN = 0;      // Dream Mode Enable (Dream mode is disabled)
  OSCCONbits.CLKLOCK = 0;    // Clock Selection Lock Enable (Clock and PLL selections are not locked and may be modified)
  OSCCONbits.SLPEN = 0;      // Sleep Mode Enable (Device will enter Idle mode when a WAIT instruction is executed)

  // OSCTUN
  OSCTUNbits.TUN = 0b00000; // FRC Oscillator Tuning (Center frequency. Oscillator runs at calibrated frequency (8 MHz))

  // PB1DIV
  while(!PB1DIVbits.PBDIVRDY);
  PB1DIVbits.PBDIV = 0b0000001; // Peripheral Bus 1 Clock Divisor Control (PBCLK1 is SYSCLK divided by 2)

  // PB2DIV
  PB2DIVbits.ON = 0;            // Peripheral Bus 2 Output Clock Enable (Output clock is disabled)
  PB2DIVbits.PBDIV = 0b0000001; // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 2)

  // PB3DIV
  PB3DIVbits.ON = 1;            // Peripheral Bus 3 Output Clock Enable (Output clock is enabled)
  while(!PB3DIVbits.PBDIVRDY);
  PB3DIVbits.PBDIV = 0b0110010; // Peripheral Bus 3 Clock Divisor Control (PBCLK3 is SYSCLK divided by 50)

  // PB4DIV
  PB4DIVbits.ON = 1;            // Peripheral Bus 4 Output Clock Enable (Output clock is enabled)
  while(!PB4DIVbits.PBDIVRDY);
  PB4DIVbits.PBDIV = 0b0000001; // Peripheral Bus 4 Clock Divisor Control (PBCLK4 is SYSCLK divided by 2)

  // PB5DIV
  PB5DIVbits.ON = 1;            // Peripheral Bus 5 Output Clock Enable (Output clock is enabled)
  PB5DIVbits.PBDIV = 0b0000001; // Peripheral Bus 5 Clock Divisor Control (PBCLK5 is SYSCLK divided by 2)

  // PB7DIV
  PB7DIVbits.ON = 1;            // Peripheral Bus 7 Output Clock Enable (Output clock is disabled)
  while(!PB7DIVbits.PBDIVRDY);
  PB7DIVbits.PBDIV = 0b0000000; // Peripheral Bus 7 Clock Divisor Control (PBCLK7 is SYSCLK divided by 1)

  // PB8DIV
  PB8DIVbits.ON = 0;            // Peripheral Bus 8 Output Clock Enable (Output clock is disabled)
  PB8DIVbits.PBDIV = 0b0000001; // Peripheral Bus 8 Clock Divisor Control (PBCLK8 is SYSCLK divided by 2)

  /**
   * REFO1CLK == (PBCLK1 / (2 * (RODIV + (ROTRIM / 512)))) ==
   * (100Mhz / (2 * (2 + (256/512)))) == (100Mhz / 5) == 20Mhz
   *
   * In other words, REFO1CLK is set up to output at (SYSCLK / 10).
   */

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

  REFO1CONbits.ACTIVE = 1;                // Reference Clock Request Status (Reference clock request is active)
  REFO1CONbits.ON = 1;                    // Output Enable (Reference Oscillator Module enabled)

  // REF02CON
  REFO2CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO2CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO2CONbits.RODIV = 0b000000000000000; // Reference Clock Divider (Output clock is same frequency as source clock (no divider))
  REFO2CONbits.SIDL = 1;                  // Peripheral Stop in Idle Mode (Discontinue module operation when device enters Idle mode)
  REFO2CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO2 pin)
  REFO2CONbits.ROSEL = 0b0000;            // Reference Clock Source Select (SYSCLK)
  REFO2CONbits.DIVSWEN = 0;               // Divider Switch Enable (Divider switch is complete)

  // REF02TRIM
  REFO2TRIMbits.ROTRIM = 0b000000000; // Reference Oscillator Trim (0/512 divisor added to RODIV value)

  // REF03CON
  REFO3CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO3CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO3CONbits.RODIV = 0b000000000000000; // Reference Clock Divider (Output clock is same frequency as source clock (no divider))
  REFO3CONbits.SIDL = 1;                  // Peripheral Stop in Idle Mode (Discontinue module operation when device enters Idle mode)
  REFO3CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO3 pin)
  REFO3CONbits.ROSEL = 0b0000;            // Reference Clock Source Select (SYSCLK)
  REFO3CONbits.DIVSWEN = 0;               // Divider Switch Enable (Divider switch is complete)

  // REF03TRIM
  REFO3TRIMbits.ROTRIM = 0b000000000; // Reference Oscillator Trim (0/512 divisor added to RODIV value)

  // REF04CON
  REFO4CONbits.ACTIVE = 0;                // Reference Clock Request Status (Reference clock request is not active)
  REFO4CONbits.ON = 0;                    // Output Enable (Reference Oscillator Module disabled)
  REFO4CONbits.RODIV = 0b000000000000000; // Reference Clock Divider (Output clock is same frequency as source clock (no divider))
  REFO4CONbits.SIDL = 1;                  // Peripheral Stop in Idle Mode (Discontinue module operation when device enters Idle mode)
  REFO4CONbits.OE = 0;                    // Reference Clock Output Enable (Reference clock is not driven out on REFCLKO4 pin)
  REFO4CONbits.ROSEL = 0b0000;            // Reference Clock Source Select (SYSCLK)
  REFO4CONbits.DIVSWEN = 0;               // Divider Switch Enable (Divider switch is complete)

  // REF04TRIM
  REFO4TRIMbits.ROTRIM = 0b000000000; // Reference Oscillator Trim (0/512 divisor added to RODIV value)

  lock_config();
}

void init_can(void) {
  unlock_config();
  CFGCONbits.IOLOCK = 0;

  // Set PPS pins for C1TX/C1RX
  RPD2R = 0b1111; // RPD2 Peripheral Pin Select (C1TX)
  C1RXR = 0b0000; // C1RX Peripheral Pin Select (RPD3)

  // Set port direction for C1TX/C1RX
  TRISDbits.TRISD2 = OUTPUT;
  TRISDbits.TRISD3 = INPUT;

  C1CONbits.ON = 1; // CAN On (Enabled)

  // C1CON
  C1CONbits.CANCAP = 1; // CAN Message Receive Timestamp Timer Capture Enable (Enabled)
  C1CONbits.SIDL = 0;   // CAN Stop in Idle (CAN continues operation when system enters idle mode)

  /**
   * Configure CAN1 to run at 1Mpbs baud rate
   *
   * Ntq = 10 (1 + 4 + 3 + 2)
   * Ftq = Ntq * Fbaud = 10 * 1Mbps = 10Mhz
   * BRP = (Fsys / (2 * Ftq)) - 1 = (200Mhz / 20Mhz) - 1 = 9
   */

  // C1CFG
  C1CFGbits.BRP = 9;      // Baud Rate Prescaler (See above equation)
  C1CFGbits.SEG2PHTS = 1; // Phase Segment 2 Time Select (Freely programmable)
  C1CFGbits.SEG2PH = 2;   // Phase Buffer Segment 2 (Length is 3 * Tq)
  C1CFGbits.SEG1PH = 2;   // Phase Buffer Segment 1 (Length is 3 * Tq)
  C1CFGbits.PRSEG = 2;    // Propagation Time Segment (Length is 3 * Tq)
  C1CFGbits.SAM = 1;      // Sample of the CAN Bus Line (Bus line is sampled three times at the sample point)
  C1CFGbits.SJW = 2;      // Synchronization Jump Width (Length is 3 * Tq)
  C1CFGbits.WAKFIL = 0;   // CAN Bus Line Filter Enable (CAN bus line filter is not used for wake-up)

  /**
   * Set the base address of the CAN1 FIFOs as physical address of the memory
   * that we previously allocated.
   */
  C1FIFOBA = KVA_TO_PA(CAN_FIFO_Buffers);

  // CAN1 FIFO 0
  C1FIFOCON0bits.TXEN = 0;        // TX/RX Buffer Selection (Receive FIFO)
  C1FIFOCON0bits.FSIZE = 0b11111; // FIFO Size bits (32 messages deep)
  C1FIFOCON0bits.DONLY = 0;       // Store Message Data Only (Full message is stored, including identifier)
  C1FIFOINT0bits.RXHALFIE = 0;    // FIFO Half Full Interrupt Enable (Disabled)
  C1FIFOINT0bits.RXFULLIE = 0;    // FIFO Full Interrupt Enable (Disabled)
  C1FIFOINT0bits.RXNEMPTYIE = 0;  // FIFO Not Empty Interrupt Enable (Disabled)
  C1FIFOINT0bits.RXOVFLIE = 0;    // FIFO Overflow Interrupt Enable (Disabled)

  // CAN1 FIFO 1
  C1FIFOCON1bits.TXEN = 1;        // TX/RX Buffer Selection (Transmit FIFO)
  C1FIFOCON1bits.FSIZE = 0b11111; // FIFO Size bits (32 messages deep)

  // CAN1 Mask 0
  C1RXF0bits.EXID = 0;    //
  C1RXF0bits.SID = 0x200; //
  C1RXF0bits.EID = 0;     //

  C1RXM0bits.MIDE = 1;    //
  C1RXM0bits.SID = 0x7F8; //

  // CAN1 Filter 0
  C1FLTCON0bits.FLTEN0 = 0; //
  C1FLTCON0bits.MSEL0 = 0;  //
  C1FLTCON0bits.FSEL0 = 0;  //
  C1FLTCON0bits.FLTEN0 = 1; //

  // Set up CAN1 Interrupt
  IFS4bits.CAN1IF = 0;  // CAN1 Interrupt Flag Status (No interrupt request has occurred)
  IPC37bits.CAN1IP = 6; // CAN1 Interrupt Priority (Interrupt priority is 6)
  IPC37bits.CAN1IS = 3; // CAN1 Interrupt Subpriority (Interrupt subpriority is 3)
  IEC4bits.CAN1IE = 0;  // CAN1 Interrupt Enable Control (Interrupt is disabled)

  C1CONbits.REQOP = 0b000; // Request Operation Mode (Set Normal Operation mode)
  while(C1CONbits.OPMOD != 0b000); // Wait for the module to finish

  CFGCONbits.IOLOCK = 1;
  lock_config();
}

void init_timer1(void) {
  unlock_config();

  // Disable TMR1
  T1CONbits.ON = 1; // Timer On (Timer is disabled)

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
  IPC1bits.T1IP = 7; // TMR1 Interrupt Priority (Interrupt priority is 7)
  IPC1bits.T1IS = 3; // TMR1 Interrupt Subpriority (Interrupt subpriority is 3)
  IEC0bits.T1IE = 1; // TMR1 Interrupt Enable Control (Interrupt is enabled)

  //asm volatile("ei"); // Enable all interrupts

  // Enable TMR1
  T1CONbits.ON = 1; // Timer On (Timer is enabled)

  lock_config();
}
