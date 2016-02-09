/*
 * File: Wheel.c
 * Author: Jake Leonard
 * Comments: Main code that controls the wheel module
 */
#include "Wheel.h"

volatile uint32_t seconds = 0;
volatile uint32_t millis = 0;
volatile uint32_t delay_millis = 0;
volatile uint8_t mode = 0;
uint8_t momentaries[4] = {0};
uint8_t toggles[4] = {0};

//	Exception handler:

static enum {
    EXCEP_IRQ = 0, // interrupt
    EXCEP_AdEL = 4, // address error exception (load or ifetch)
    EXCEP_AdES, // address error exception (store)
    EXCEP_IBE, // bus error (ifetch)
    EXCEP_DBE, // bus error (load/store)
    EXCEP_Sys, // syscall
    EXCEP_Bp, // breakpoint
    EXCEP_RI, // reserved instruction
    EXCEP_CpU, // coprocessor unusable
    EXCEP_Overflow, // arithmetic overflow
    EXCEP_Trap, // trap (possible divide by zero)
    EXCEP_IS1 = 16, // implementation specfic 1
    EXCEP_CEU, // CorExtend Unuseable
    EXCEP_C2E // coprocessor 2
} _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

// this function overrides the normal _weak_ generic handler

void _general_exception_handler(void) {
    asm volatile("mfc0 %0,$13" : "=r" (_excep_code));
    asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

    _excep_code = (_excep_code & 0x0000007C) >> 2;
    while (1) {
        // Examine _excep_code to identify the type of exception
        // Examine _excep_addr to find the address that caused the exception
        Nop();
        Nop();
        Nop();
    }
}//	End of exception handler

void main(void) {
    init_general(); // Set general runtime configuration bits
    init_gpio_pins(); // Set all I/O pins to low outputs
    //init_peripheral_modules(); // Disable unused peripheral modules
    init_oscillator(); // Initialize oscillator configuration bits
    init_timer1(); // Initialize Second interrupts
    init_timer2(); // Initialize Millisecond interrupts
    init_spi(); // Initialize SPI interface
    init_termination(TERMINATING); // Initialize Programmable Termination
    //init_can(); // Initialize CAN

    asm volatile ("ei"); // Enable interrupts

    // Set up indicators
    PIC_FN_TRIS = OUTPUT;
    PIC_MODE_TRIS = OUTPUT;
    PIC_FN_LAT = 1;
    PIC_MODE_LAT = 0;
    LCD_RST_TRIS = OUTPUT;
    LCD_RST_LAT = 1;
    LCD_BACKLITE_TRIS = OUTPUT;
    //LCD_BACKLITE_LAT = 0;

    // LCD Chip Select
    LCD_CS_TRIS = OUTPUT;
    LCD_CS_LAT = 0;

    // Set switches to input
    SW_MOMENT1_TRIS = INPUT;
    SW_MOMENT2_TRIS = INPUT;
    SW_MOMENT3_TRIS = INPUT;
    SW_MOMENT4_TRIS = INPUT;
    SW_TOG1_TRIS = INPUT;
    SW_TOG2_TRIS = INPUT;
    SW_TOG3_TRIS = INPUT;
    SW_TOG4_TRIS = INPUT;

    // Start LCD
    LCD_Reset();
    LCD_Init();
    while (1) {
        update_sw_values();
        delay(100);
        PIC_FN_LAT = 1;
        PIC_MODE_LAT = 0;
        delay(100);
        PIC_FN_LAT = 0;
        PIC_MODE_LAT = 1;
    }
}

void update_sw_values() {
    momentaries[0] = SW_MOMENT1_PORT;
    momentaries[1] = SW_MOMENT2_PORT;
    momentaries[2] = SW_MOMENT3_PORT;
    momentaries[3] = SW_MOMENT4_PORT;
    toggles[0] = SW_TOG1_PORT;
    toggles[1] = SW_TOG2_PORT;
    toggles[2] = SW_TOG3_PORT;
    toggles[3] = SW_TOG4_PORT;
}

// Uses the millis counter to create a precise delay

void delay(uint32_t mil) {
    if (mil == 1) mil++; //Might be entering on the end of the millisecond
    CLI(); //Turn off interrupts for any interaction with millis
    register uint32_t delay_stop = millis + mil;
    STI(); // Turn it back on again
    Nop();
    CLI();
    while (delay_stop > millis) {
        STI();
        Nop(); //Keep interrupts on and wait for stuff to happen
        Nop();
        Nop();
        CLI();
    }
    STI();
}

// Called every second

void __attribute__((vector(_TIMER_1_VECTOR), interrupt(IPL7SRS))) timer1_inthnd(void) {
    seconds++;
    //PIC_FN_LAT = PIC_FN_LAT ? 0 : 1; // Toggle the LED
    //PIC_MODE_LAT = PIC_MODE_LAT ? 0 : 1; // Toggle the LED
    //LCD_BACKLITE_LAT = LCD_BACKLITE_LAT ? 0 : 1; // Toggle the LED
    IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag
}

// Called every millisecond

void __attribute__((vector(_TIMER_2_VECTOR), interrupt(IPL7SRS))) timer2_inthnd(void) {
    millis++;
    IFS0bits.T2IF = 0; // Clear TMR2 Interrupt Flag
}