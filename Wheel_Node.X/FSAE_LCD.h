/**
 * FSAE LCD Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */

#ifndef _FSAE_LCD_H    /* Guard against multiple inclusion */
#define _FSAE_LCD_H

#include <xc.h>
#include <sys/types.h>
#include "RA8875_driver.h"

// Define race screen constants
#define NUM_SCREENS           12
#define RACE_SCREEN           0
#define PDM_DRAW_SCREEN       1
#define PDM_CUT_SCREEN        2
#define MOTEC_SCREEN          3
#define END_RACE_SCREEN       4
#define CHASSIS_SCREEN        5
#define GENERAL_SCREEN        6
#define BRAKE_SCREEN          7
#define PDM_GRID_SCREEN       8
#define WHEELSPEED_SCREEN     9
#define THROTTLE_SCREEN       10
#define IMU_SCREEN            11

#define MIN_REFRESH           350

#define MIN_SUS_POS           5
#define MAX_SUS_POS           20

#define MIN_BRAKE_PRESS       0
#define MAX_BRAKE_PRESS       250

#define REV_SUB_1           	6000
#define REV_SUB_2           	5000
#define REV_SUB_3           	4000
#define REV_SUB_4           	3000
#define REV_SUB_5           	2500
#define REV_SUB_6           	2000
#define REV_SUB_7           	1500
#define REV_SUB_8           	1000
#define REV_SUB_9           	500

// PDM Dataitem Constants
#define PDM_DATAITEM_SIZE     93

#define UPTIME_IDX            0
#define PCB_TEMP_IDX          1
#define IC_TEMP_IDX           2
#define STR_ENABLITY_IDX      3
#define BVBAT_ENABLITY_IDX    4
#define AUX_ENABLITY_IDX      5
#define ECU_ENABLITY_IDX      6
#define WTR_ENABLITY_IDX      7
#define FAN_ENABLITY_IDX      8
#define PDLD_ENABLITY_IDX     9
#define PDLU_ENABLITY_IDX     10
#define ABS_ENABLITY_IDX      11
#define INJ_ENABLITY_IDX      12
#define IGN_ENABLITY_IDX      13
#define FUEL_ENABLITY_IDX     14
#define STR_PEAK_MODE_IDX     15
#define BVBAT_PEAK_MODE_IDX   16
#define AUX_PEAK_MODE_IDX     17
#define ECU_PEAK_MODE_IDX     18
#define WTR_PEAK_MODE_IDX     19
#define FAN_PEAK_MODE_IDX     20
#define PDLD_PEAK_MODE_IDX    21
#define PDLU_PEAK_MODE_IDX    22
#define ABS_PEAK_MODE_IDX     23
#define INJ_PEAK_MODE_IDX     24
#define IGN_PEAK_MODE_IDX     25
#define FUEL_PEAK_MODE_IDX    26
#define TOTAL_CURRENT_IDX     27
#define AUX2_SWITCH_IDX       28
#define AUX1_SWITCH_IDX       29
#define ABS_SWITCH_IDX        30
#define KILL_SWITCH_IDX       31
#define ACT_DN_SWITCH_IDX     32
#define ACT_UP_SWITCH_IDX     33
#define ON_SWITCH_IDX         34
#define STR_SWITCH_IDX        35
#define KILL_ENGINE_FLAG_IDX  36
#define KILL_CAR_FLAG_IDX     37
#define OVER_TEMP_FLAG_IDX    38
#define FUEL_PRIME_FLAG_IDX   39
#define VBAT_RAIL_IDX         40
#define V12_RAIL_IDX          41
#define V5_RAIL_IDX           42
#define V3V3_RAIL_IDX         43
#define FUEL_DRAW_IDX         44
#define IGN_DRAW_IDX          45
#define INJ_DRAW_IDX          46
#define ABS_DRAW_IDX          47
#define PDLU_DRAW_IDX         48
#define PDLD_DRAW_IDX         49
#define FAN_DRAW_IDX          50
#define WTR_DRAW_IDX          51
#define ECU_DRAW_IDX          52
#define AUX_DRAW_IDX          53
#define BVBAT_DRAW_IDX        54
#define STR_DRAW_IDX          55
#define FUEL_CUT_IDX          56
#define IGN_CUT_IDX           57
#define INJ_CUT_IDX           58
#define ABS_CUT_IDX           59
#define PDLU_CUT_IDX          60
#define PDLD_CUT_IDX          61
#define FAN_CUT_IDX           62
#define WTR_CUT_IDX           63
#define ECU_CUT_IDX           64
#define AUX_CUT_IDX           65
#define BVBAT_CUT_IDX         66
#define FUEL_CUT_P_IDX        67
#define FAN_CUT_P_IDX         68
#define WTR_CUT_P_IDX         69
#define ECU_CUT_P_IDX         80
#define FUEL_OC_COUNT_IDX     81
#define IGN_OC_COUNT_IDX      82
#define INJ_OC_COUNT_IDX      83
#define ABS_OC_COUNT_IDX      84
#define PDLU_OC_COUNT_IDX     85
#define PDLD_OC_COUNT_IDX     86
#define FAN_OC_COUNT_IDX      87
#define WTR_OC_COUNT_IDX      88
#define ECU_OC_COUNT_IDX      89
#define AUX_OC_COUNT_IDX      90
#define BVBAT_OC_COUNT_IDX    91
#define STR_OC_COUNT_IDX      92

//GCM DataItem Constants
#define GCM_DATAITEM_SIZE     13

#define GEAR_IDX              3
#define GEAR_VOLT_IDX         4
#define FORCE_IDX             5
#define PADDLE_UP_SW_IDX      6
#define PADDLE_DOWN_SW_IDX    7
#define NEUTRAL_SW_IDX        8
#define QUEUE_UP_IDX          9
#define QUEUE_DN_IDX          10
#define QUEUE_NT_IDX          11
#define MODE_IDX              12

//MOTEC DataItem Constants
#define MOTEC_DATAITEM_SIZE   30

#define ENG_RPM_IDX           0
#define THROTTLE_POS_IDX      1
#define LAMBDA_IDX            2
#define VOLT_ECU_IDX          3
#define ENG_TEMP_IDX          4
#define OIL_TEMP_IDX          5
#define MANIFOLD_TEMP_IDX     6
#define FUEL_TEMP_IDX         7
#define AMBIENT_PRES_IDX      8
#define OIL_PRES_IDX          9
#define MANIFOLD_PRES_IDX     10
#define FUEL_PRES_IDX         11
#define WHEELSPEED_FL_IDX     12
#define WHEELSPEED_FR_IDX     13
#define WHEELSPEED_RL_IDX     14
#define WHEELSPEED_RR_IDX     15
#define DRIVE_SPEED_IDX       16
#define GROUND_SPEED_IDX      17
#define GPS_SPEED_IDX         18
#define GPS_ALT_IDX           19
#define GPS_LAT_IDX           20
#define GPS_LONG_IDX          21
#define GPS_TIME_IDX          22
#define RUN_TIME_IDX          23
#define FUEL_USED_IDX         24
#define FUEL_INJ_DUTY_IDX     25
#define FUEL_TRIM_IDX         26
#define SHIFT_FORCE_IDX       27
#define AIR_TEMP_IDX          28
#define WHEELSPEED_AVG_IDX    29

//TireTemp DataItem Constants
#define TIRETEMP_DATAITEM_SIZE  20

#define FL_IDX                  0
#define FL0_IDX                 1
#define FL1_IDX                 2
#define FL2_IDX                 3
#define FL3_IDX                 4
#define FR_IDX                  5
#define FR0_IDX                 6
#define FR1_IDX                 7
#define FR2_IDX                 8
#define FR3_IDX                 9
#define RL_IDX                  10
#define RL0_IDX                 11
#define RL1_IDX                 12
#define RL2_IDX                 13
#define RL3_IDX                 14
#define RR_IDX                  15
#define RR0_IDX                 16
#define RR1_IDX                 17
#define RR2_IDX                 18
#define RR3_IDX                 19

#define SPM_DATAITEM_SIZE       78
                               
#define ANALOG_CHAN_0_IDX       3
#define ANALOG_CHAN_1_IDX       4
#define ANALOG_CHAN_2_IDX       5
#define ANALOG_CHAN_3_IDX       6
#define ANALOG_CHAN_4_IDX       7
#define ANALOG_CHAN_5_IDX       8
#define ANALOG_CHAN_6_IDX       9
#define ANALOG_CHAN_7_IDX       10
#define ANALOG_CHAN_8_IDX       11
#define ANALOG_CHAN_9_IDX       12
#define ANALOG_CHAN_10_IDX      13
#define ANALOG_CHAN_11_IDX      14
#define ANALOG_CHAN_12_IDX      15
#define ANALOG_CHAN_13_IDX      16
#define ANALOG_CHAN_14_IDX      17
#define ANALOG_CHAN_15_IDX      18
#define ANALOG_CHAN_16_IDX      19
#define ANALOG_CHAN_17_IDX      20
#define ANALOG_CHAN_18_IDX      21
#define ANALOG_CHAN_19_IDX      22
#define ANALOG_CHAN_20_IDX      23
#define ANALOG_CHAN_21_IDX      24
#define ANALOG_CHAN_22_IDX      25
#define ANALOG_CHAN_23_IDX      26
#define ANALOG_CHAN_24_IDX      27
#define ANALOG_CHAN_25_IDX      28
#define ANALOG_CHAN_26_IDX      29
#define ANALOG_CHAN_27_IDX      30
#define ANALOG_CHAN_28_IDX      31
#define ANALOG_CHAN_29_IDX      32
#define ANALOG_CHAN_30_IDX      33
#define ANALOG_CHAN_31_IDX      34
#define ANALOG_CHAN_32_IDX      35
#define ANALOG_CHAN_33_IDX      36
#define ANALOG_CHAN_34_IDX      37
#define ANALOG_CHAN_35_IDX      38
#define TCOUPLE_0_IDX           39
#define TCOUPLE_1_IDX           40
#define TCOUPLE_2_IDX           41
#define TCOUPLE_3_IDX           42
#define TCOUPLE_4_IDX           43
#define TCOUPLE_5_IDX           44
#define AVG_JUNCT_TEMP_IDX      45
#define TCOUPLE_0_FAULT_IDX     46
#define TCOUPLE_1_FAULT_IDX     47
#define TCOUPLE_2_FAULT_IDX     48
#define TCOUPLE_3_FAULT_IDX     49
#define TCOUPLE_4_FAULT_IDX     50
#define TCOUPLE_5_FAULT_IDX     51
#define DIGITAL_INPUT_0_IDX     52
#define DIGITAL_INPUT_1_IDX     53
#define DIGITAL_INPUT_2_IDX     54
#define DIGITAL_INPUT_3_IDX     55
#define DIGITAL_INPUT_4_IDX     56
#define DIGITAL_INPUT_5_IDX     57
#define DIGITAL_INPUT_6_IDX     58
#define DIGITAL_INPUT_7_IDX     59
#define DIGITAL_INPUT_8_IDX     60
#define DIGITAL_INPUT_9_IDX     61
#define DIGITAL_INPUT_10_IDX    62
#define DIGITAL_INPUT_11_IDX    63
#define DIGITAL_INPUT_12_IDX    64
#define DIGITAL_INPUT_13_IDX    65
#define DIGITAL_INPUT_14_IDX    66
#define DIGITAL_INPUT_15_IDX    67
#define FREQ_COUNT_0_IDX        68
#define FREQ_COUNT_1_IDX        69
#define FREQ_COUNT_2_IDX        70
#define PGA_0_SETTINGS_IDX      71
#define PGA_1_SETTINGS_IDX      72
#define PGA_2_SETTINGS_IDX      73
#define PGA_3_SETTINGS_IDX      74
#define FREQ_0_SETTINGS_IDX     75
#define FREQ_1_SETTINGS_IDX     76
#define FREQ_2_SETTINGS_IDX     77

#define WHEEL_DATAITEM_SIZE     16
#define ROTARY_0_IDX            3
#define ROTARY_1_IDX            4
#define ROTARY_2_IDX            5
#define TROTARY_0_IDX           6
#define TROTARY_1_IDX           7
#define MOM_AUX_IDX             8
#define MOM_ACK_IDX             9
#define MOM_RDO_IDX             10
#define MOM_NEU_IDX             11
#define SW_ND_IDX               12
#define SW_FAN_IDX              13
#define SW_WTR_IDX              14
#define SW_FUEL_IDX             15

//IMU dataItem constants
#define IMU_DATAITEM_SIZE       4
#define LATERAL_G_IDX           0
#define LONGITUDINAL_G_IDX      1
#define YAW_RATE_IDX            2
#define YAW_ACCEL_IDX           3

/*
 * Defines a data stream that is relevant to one or more screens
 *
 * value -    double value of stream
 * warnThreshold -  Value where data will enter a warning state
 * errThreshold -   Value where data will enter an error state
 * thresholdDir - 1 if max, 0 if min
 * refreshInterval -  Maximum refresh frequency
 * wholeDigits -  Number of whole digits to display
 * decDigits -    Number of decimal digits to display
 */
typedef struct packed {
  double value;
  double warnThreshold;
  double errThreshold;
  unsigned thresholdDir:1;
  unsigned warningState:1;
  unsigned blinkState:1;
  uint32_t refreshInterval;
  uint32_t refreshTime;
  uint8_t wholeDigits;
  uint8_t decDigits;
} dataItem;


/*
 * Defines the minimum amount of information a redrawItem function needs to work
 *
 * x -    X coordinate
 * y -    Y coordinate
 * size -   Size of item
 */

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t size;
} screenItemInfo;


/*
 * Defines an item that will be displayed on a specific screen
 *
 * currentValue -   Current value being displayed
 * data -     Pointer to corresponding dataItem
 * refreshTime -  Time that the value was previously refreshed
 * info -     Struct that contains necessary info for redrawing
 * redrawItem -   Redraw function pointer, called when the item is refreshed
 */
typedef struct {
  double currentValue;
  volatile dataItem * data;
  uint32_t refreshTime;
  screenItemInfo info;
  void (*redrawItem)(screenItemInfo *, volatile dataItem *, double);
} screenItem;

/*
 * Defines a screen
 *
 * items -  Array of screen Items that will be on that screen
 * len -  Length of screenItem array
 */
typedef struct {
  screenItem * items;
  uint8_t len;
} screen;

// Define all screen item arrays for each screen
screenItem raceScreenItems[10], pdmDrawItems[33], pdmGridItems[40], pdmCutItems[21], brakeItems[8], motecItems[30], endRaceItems[9], chassisItems[20], generalItems[7], wheelSpeedItems[2], autoUpItems[2], throttleItems[3], imuItems[3];

// Define all screen structs
screen raceScreen, pdmDrawScreen, pdmCutScreen, pdmGridScreen, brakeScreen, motecScreen, endRaceScreen, chassisScreen, generalScreen, wheelSpeedScreen, throttleScreen, imuScreen;

// Define master array of all screen structs
screen* allScreens[NUM_SCREENS];

uint8_t screenNumber, auxNumber;

volatile uint16_t backgroundColor, foregroundColor, warningColor, errorColor;

volatile dataItem pdmDataItems[PDM_DATAITEM_SIZE], gcmDataItems[GCM_DATAITEM_SIZE], motecDataItems[MOTEC_DATAITEM_SIZE], tireTempDataItems[TIRETEMP_DATAITEM_SIZE], spmDataItems[SPM_DATAITEM_SIZE], wheelDataItems[WHEEL_DATAITEM_SIZE], imuDataItems[IMU_DATAITEM_SIZE];

volatile dataItem *fanSw[2], *fuelSw[2], *wtrSw[2], *shiftLights[2], *gForce[2];

void initDataItems(void); // Writes default values to all data items
// Initializes an individual dataItem
void initDataItem(volatile dataItem* data, double warn, double err,
  uint32_t refresh, uint8_t whole, uint8_t dec);
void setDataItemDigits(volatile dataItem* data, uint8_t whole, uint8_t dec);
void initAllScreens(void); // Initializes all screenItems
void initScreen(uint8_t num); // Draws all non-dataItem data to start a screen
// Initializes an individual screenItem
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size,
  void (*redrawItem)(screenItemInfo *, volatile dataItem *, double),
  volatile dataItem* data);
// Toggles between a few different data items on one screen
void changeAUXType(uint8_t num);
void changeScreen(uint8_t num);
// Continuously runs and refreshes stuff
void refreshScreenItems(void);
void clearScreen(void);
void resetScreenItems(void); // Resets all the values
uint8_t initNightMode(uint8_t on); // Night mode stuff
void nightMode(uint8_t on);
uint8_t checkDataChange(volatile dataItem *data, double currentValue);

// Redraw Functions!
void redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawFanSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawFUELPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawWTRPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawGCMMode(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawTireTemp(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawSPBar(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawBrakeBar(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawRotary(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawShiftLightsRPM(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawKILLCluster(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawGforceGraph(screenItemInfo * item, volatile dataItem * data, double currentValue);


// Helper functions for colorful redraw functions
uint16_t tempColor(uint8_t temp);

#endif /* _FSAE_LCD_H */
