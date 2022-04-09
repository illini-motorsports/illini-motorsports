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

#include "../FSAE.X/CAN.h"
#include "../FSAE.X/FSAE_adc.h"
#include "../FSAE.X/FSAE_can.h"
#include "../FSAE.X/FSAE_config.h"
#include "../FSAE.X/errno.h"
#include <sys/types.h>

#define abs(n) (((n) >= 0) ? (n) : -(n))

// Misc state definitions
#define ENG_ON (eng_rpm > RPM_ON_THRESHOLD)

// Thresholds
#define RPM_ON_THRESHOLD 600.0 // rpm
#define MAX_RETRY 2
#define GEAR_VOLT_RIPPLE 0.3 // V

// Gear Voltages
// ECU ADC SEES DIFFERENT VOLTAGES FOR SOME REASON
// TUNE THESE CUTOFFS INDEPENDENTLY FOR BOTH DEVICES!
#define GEAR_VOLT_1 0.72
#define GEAR_VOLT_NEUT 1.05
#define GEAR_VOLT_2 1.45
#define GEAR_VOLT_3 2.20
#define GEAR_VOLT_4 2.97
#define GEAR_VOLT_5 3.74
#define GEAR_VOLT_6 4.54

// Definitions for error conditions
// TODO: Decide on these values
#define LOW_VOLT 10.0      // V
#define LOWER_VOLT 8.0     // V
#define RPM_NEUT_LOCK 6000 // rpm
#define RPM_UNDERREV 750   // rpm
#define RPM_OVERREV 14200  // rpm

// Timing constants (ms)
#define DEBOUNCE_WAIT 10
#define TEMP_SAMP_INTV 333
#define SENSOR_SAMP_INTV 5
#define IGNITION_CUT_CAN_SEND 1
#define DIAG_MSG_SEND 100
#define STATE_MSG_SEND 10
#define GEAR_STATUS_CAN_SEND 1
#define ECU_MSG_SEND 1
#define CUT_MSG_SEND 1
void send_ignition_cut();

#define LOCKOUT_DUR 25
#define MAX_SHIFT_DUR 300
#define RELAX_WAIT 10 // TODO: Tune this value
#define PWR_CUT_WAIT 10 // TODO: Tune this value
#define UP_SHIFT_DUR 150 // TODO: Tune this value
#define DN_SHIFT_DUR 125 // TODO: Tune this value
#define NT_SHIFT_DUR 50 // TODO: Tune this value
#define ABS_WS_ID 0x24A

#define CAN_STATE_WAIT 1000
#define CUT_RETRY_WAIT 2

// Raw (bouncy) switch state definitions
#define SHIFT_UP_RAW (!SHIFT_UP_PORT)
#define SHIFT_DN_RAW (!SHIFT_DN_PORT)
#define SHIFT_NT_RAW (!SHIFT_NT_PORT)

// Debounced switch state definitions (use these for everything)
#define SHIFT_UP_SW (((switch_debounced >> 2) & 0x1))
#define SHIFT_DN_SW (((switch_debounced >> 1) & 0x1))
#define SHIFT_NT_SW 0

// Definitions for gear position variable
#define GEAR_NEUT 0
#define GEAR_FAIL 7

// Definitions for shift enum
#define SHIFT_ENUM_UP 0
#define SHIFT_ENUM_DN 1
#define SHIFT_ENUM_NT 2

// Ignition cut flags
#define IGNITION_CUT_UPSHIFT 1
#define IGNITION_CUT_DOWNSHIFT 2
#define IGNITION_CUT_ALL 4
#define IGNITION_CUT_ENABLE 1
#define IGNITION_CUT_DISABLE 0

// Miscellaneous definitions
#define PWR_CUT_SPOOF 0xE803 // Value for "spoofed" gear shift force sensor
#define ACT_ON 0
#define ACT_OFF 1
#define NOT_SHIFTING 0
#define SHIFTING 1
#define CUT_END 0
#define CUT_START 1
#define CUT_RESEND 2
#define MAX_AUTO_GEAR 6
#define LAUNCH_WS_DIFF 1.2
#define LAUNCH_FRONT_WS 15

// Auto-upshifting thresholds
#define MAX_AUTO_UPSHIFT_ENGINE_RPM 4000
#define MAX_AUTO_UPSHIFT_BRAKE_PRESSURE 3
#define MAX_LAUNCH_SPEED 55

// Gear ratio of standard Yamaha R6 YZF 08 transmission
const double gear_ratio[7] = {1.0, 2.583, 2.000, 1.667, 1.444, 1.286, 1.150};

// Optiomal Shift RPM's for Yamaha R6 transmission
// 1-->2, 2-->3, 3-->4, 4-->5, 5-->6, 6-->Break
const uint16_t shift_rpm[6] = {12026, 11492, 11399, 11191, 11220, 20000};

const float shift_speed_min[6] = {60, 80, 100, 120, 160};

// Pin definitions
// neut = a5
// up = e5
// down = g15
//#define SHIFT_NT_TRIS  TRISEbits.TRISE5
//#define SHIFT_NT_ANSEL ANSELEbits.ANSE5
//#define SHIFT_NT_PORT  PORTEbits.RE5
//#define SHIFT_DN_TRIS  TRISGbits.TRISG15
//#define SHIFT_DN_ANSEL ANSELGbits.ANSG15
//#define SHIFT_DN_PORT  PORTGbits.RG15
//#define SHIFT_UP_TRIS  TRISAbits.TRISA5
//#define SHIFT_UP_ANSEL ANSELAbits.ANSA5
//#define SHIFT_UP_PORT  PORTAbits.RA5

#define SHIFT_NT_TRIS TRISEbits.TRISE5
#define SHIFT_NT_ANSEL ANSELEbits.ANSE5
#define SHIFT_NT_PORT PORTEbits.RE5
#define SHIFT_UP_TRIS TRISGbits.TRISG15
#define SHIFT_UP_ANSEL ANSELGbits.ANSG15
#define SHIFT_UP_PORT PORTGbits.RG15
#define SHIFT_DN_TRIS TRISAbits.TRISA5
#define SHIFT_DN_ANSEL ANSELAbits.ANSA5
#define SHIFT_DN_PORT PORTAbits.RA5

#define ACT_UP_TRIS TRISEbits.TRISE7
#define ACT_UP_LAT LATEbits.LATE7
#define ACT_DN_TRIS TRISCbits.TRISC1
#define ACT_DN_LAT LATCbits.LATC1

#define ADC_GEAR_TRIS TRISGbits.TRISG6
#define ADC_GEAR_ANSEL ANSELGbits.ANSG6
#define ADC_GEAR_CSS ADCCSS1bits.CSS14
#define ADC_GEAR_CHN 14

#define ADC_FORCE_TRIS TRISEbits.TRISE8
#define ADC_FORCE_ANSEL ANSELEbits.ANSE8
#define ADC_FORCE_CSS ADCCSS1bits.CSS25
#define ADC_FORCE_CHN 25

// Enum for current GCM mode
typedef enum _gcm_mode
{
    NORMAL_MODE,
    AUTO_UPSHIFT_MODE
} gcm_mode;

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
void send_lat_accel(void);
void send_state_can(uint8_t override);
void send_ignition_cut_status_can(uint8_t override);
void send_gear_status_can(uint8_t override);


// Logic functions
void process_auto_upshift(void);
void process_upshift_press(void);
void process_downshift_press(void);
uint8_t check_shift_conditions(uint8_t shift_enum);
void update_gcm_mode(void);
void do_shift(uint8_t shift_enum);
void do_shift_gear_fail(uint8_t shift_enum);

void set_ignition_cut(uint8_t type, uint8_t status);

// Utility functions
void relax_wait(void);
void main_loop_misc(void);
void debounce_switches(void);
void send_power_cut(uint8_t is_start);
uint16_t get_threshold_rpm(uint8_t gear);
float get_threshold_speed(uint8_t gear);
uint8_t is_in_launch(void);

#endif /* GCM_H */