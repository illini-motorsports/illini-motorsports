/**
 * GCM Header
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Andrew Mass
 * Created:     2016-2017
 */
#ifndef GCM_H
#define GCM_H

#include <sys/types.h>
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/CAN.h"
#include "../FSAE.X/errno.h"

#define abs(n) ( ((n) >= 0) ? (n) : -(n) )

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Thresholds
#define RPM_ON_THRESHOLD 600.0 // rpm
#define MAX_RETRY        2
#define GEAR_VOLT_RIPPLE 0.1   // V

// Definitions for error conditions
//TODO: Decide on these values
#define LOW_VOLT      10.0  // V
#define LOWER_VOLT    8.0   // V
#define RPM_NEUT_LOCK 6000  // rpm
#define RPM_UNDERREV  750   // rpm
#define RPM_OVERREV   14200 // rpm

// Timing constants (ms)
#define DEBOUNCE_WAIT    10
#define TEMP_SAMP_INTV   333
#define SENSOR_SAMP_INTV 5

#define DIAG_MSG_SEND    500
#define STATE_MSG_SEND   250

#define LOCKOUT_DUR    50
#define MAX_SHIFT_DUR  150
#define RELAX_WAIT     10  //TODO: Tune this value
#define PWR_CUT_WAIT   10  //TODO: Tune this value
#define UP_SHIFT_DUR   150 //TODO: Tune this value
#define DN_SHIFT_DUR   125 //TODO: Tune this value
#define NT_SHIFT_DUR   50  //TODO: Tune this value

#define CAN_STATE_WAIT 1000
#define CUT_RETRY_WAIT 2

// Raw (bouncy) switch state definitions
#define SHIFT_UP_RAW (!SHIFT_UP_PORT)
#define SHIFT_DN_RAW (!SHIFT_DN_PORT)
#define SHIFT_NT_RAW (!SHIFT_NT_PORT)

// Debounced switch state definitions (use these for everything)
#define SHIFT_UP_SW (((switch_debounced >> 2) & 0x1))
#define SHIFT_DN_SW (((switch_debounced >> 1) & 0x1))
#define SHIFT_NT_SW (((switch_debounced >> 0) & 0x1))

// Definitions for gear position variable
#define GEAR_NEUT 0
#define GEAR_FAIL 7

// Definitions for shift enum
#define SHIFT_ENUM_UP 0
#define SHIFT_ENUM_DN 1
#define SHIFT_ENUM_NT 2

// Miscellaneous definitions
#define PWR_CUT_SPOOF 		0xE803 // Value for "spoofed" gear shift force sensor
#define ACT_ON  				  0
#define ACT_OFF 				  1
#define NOT_SHIFTING 			0
#define SHIFTING     			1
#define CUT_END    				0
#define CUT_START  				1
#define CUT_RESEND 				2
#define MAX_AUTO_GEAR 	  6
#define LAUNCH_WS_DIFF    1.5
#define LAUNCH_FRONT_WS		10

// Gear ratio of standard Yamaha R6 YZF 08 transmission
const double gear_ratio[7] = {
  1.0,
  2.583,
  2.000,
  1.667,
  1.444,
  1.286,
  1.150
};

// Optiomal Shift RPM's for Yamaha R6 transmission
const uint16_t shift_rpm[6] = {
	12444,
	11950,
	12155,
	12100,
	11150,
	20000
};

// Pin definitions
// neut = a5
// up = e5
// down = g15
#define SHIFT_UP_TRIS  TRISGbits.TRISG15
#define SHIFT_UP_ANSEL ANSELGbits.ANSG15
#define SHIFT_UP_PORT  PORTGbits.RG15
#define SHIFT_DN_TRIS  TRISAbits.TRISA5
#define SHIFT_DN_ANSEL ANSELAbits.ANSA5
#define SHIFT_DN_PORT  PORTAbits.RA5
#define SHIFT_NT_TRIS  TRISEbits.TRISE5
#define SHIFT_NT_ANSEL ANSELEbits.ANSE5
#define SHIFT_NT_PORT  PORTEbits.RE5

#define ACT_UP_TRIS TRISEbits.TRISE7
#define ACT_UP_LAT  LATEbits.LATE7
#define ACT_DN_TRIS TRISCbits.TRISC1
#define ACT_DN_LAT  LATCbits.LATC1

#define ADC_GEAR_TRIS  TRISGbits.TRISG6
#define ADC_GEAR_ANSEL ANSELGbits.ANSG6
#define ADC_GEAR_CSS   ADCCSS1bits.CSS14
#define ADC_GEAR_CHN   14

#define ADC_FORCE_TRIS  TRISEbits.TRISE8
#define ADC_FORCE_ANSEL ANSELEbits.ANSE8
#define ADC_FORCE_CSS   ADCCSS1bits.CSS25
#define ADC_FORCE_CHN   25

// Enum for current GCM mode
typedef enum _gcm_mode {NORMAL_MODE, AUTO_UPSHIFT_MODE} gcm_mode;

/**
 * Function definitions
 */

void main(void);

// ADC sample functions
void sample_temp(void);
void sample_sensors(uint8_t is_shifting);

// CAN functions
void process_CAN_msg(CAN_message msg);
void send_diag_can(void);
void send_state_can(uint8_t override);

// Logic functions
void process_auto_upshift(void);
void process_upshift_press(void);
void process_downshift_press(void);
uint8_t check_shift_conditions(uint8_t shift_enum);
void check_gcm_mode(void);
void do_shift(uint8_t shift_enum);
void do_shift_gear_fail(uint8_t shift_enum);

// Utility functions
void relax_wait(void);
void main_loop_misc(void);
void debounce_switches(void);
void send_power_cut(uint8_t is_start);
uint16_t get_threshold_rpm(uint8_t gear);
uint8_t is_in_launch(void);

#endif /* GCM_H */
