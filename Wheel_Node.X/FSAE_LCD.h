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
#define NUM_SCREENS           1
#define RACE_SCREEN           0

#define MIN_REFRESH           350
#define CAN_TIMEOUT           1000

// PDM Dataitem Constants
#define PDM_DATAITEM_SIZE     2

#define VBAT_RAIL_IDX         0
#define KILL_SWITCH_IDX       1

//GCM DataItem Constants
#define GCM_DATAITEM_SIZE     1

#define GEAR_IDX              0

//MOTEC DataItem Constants
#define MOTEC_DATAITEM_SIZE   6

#define ENG_RPM_IDX           0
#define THROTTLE_POS_IDX      1
#define LAMBDA_IDX            2
#define ENG_TEMP_IDX          3
#define OIL_TEMP_IDX          4
#define OIL_PRES_IDX          5

#define WHEEL_DATAITEM_SIZE   13

#define ROTARY_0_IDX          0
#define ROTARY_1_IDX          1
#define ROTARY_2_IDX          2
#define TROTARY_0_IDX         3
#define TROTARY_1_IDX         4
#define MOM_AUX_IDX           5
#define MOM_ACK_IDX           6
#define MOM_RDO_IDX           7
#define MOM_NEU_IDX           8
#define SW_ND_IDX             9
#define SW_FAN_IDX            10
#define SW_WTR_IDX            11
#define SW_FUEL_IDX           12

/*
 * Defines a data stream that is relevant to one or more screens
 *
 * value -          double value of stream
 * warnThreshold -  Value where data will enter a warning state
 * errThreshold -   Value where data will enter an error state
 * thresholdDir -   1 if max, 0 if min
 * warningState -   1 if crossed threshold, 0 if normal
 * refreshTime -    millis of last data refresh
 * wholeDigits -    Number of whole digits to display
 * decDigits -      Number of decimal digits to display
 */
typedef struct packed {
  double value;
  double warnThreshold;
  double errThreshold;
  unsigned thresholdDir:1;
  unsigned warningState:1;
  uint32_t refreshTime;
  uint8_t wholeDigits;
  uint8_t decDigits;
} dataItem;


/*
 * Defines the minimum amount of information a redrawItem function needs to work
 *
 * x -    X coordinate
 * y -    Y coordinate
 * size - Size of item
 */

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t size;
} screenItemInfo;

/*
 * Enables there to be multiple dataItems for a given screenItem
 *
 * data -         DataItem with relevant data
 * currentValue - Current value being displayed
 * next -         pointer to next screenItemNode (null if end of list)
 */
typedef struct _screenItemNode {
  volatile dataItem * data;
  double currentValue;
  struct _screenItemNode * next;
} screenItemNode;

/*
 * Defines an item that will be displayed on a specific screen
 *
 * refreshInterval -  Maximum refresh frequency
 * refreshTime -      Time that the value was previously refreshed
 * head -             Head of a linked list of screenItemNodes containing dataItem pointers
 * info -             Struct that contains necessary info for redrawing
 * redrawItem -       Redraw function pointer, called when the item is refreshed
 * label -            Label to be drawn underneath the screenItem
 * labelSize -        Size to draw the label: 0 - 3, -1 means the label will not be drawn
 */
typedef struct {
  uint32_t refreshInterval;
  uint32_t refreshTime;
  screenItemNode head;
  screenItemInfo info;
  void (*redrawItem)(screenItemInfo *, screenItemNode *);
  char * label;
  int8_t labelSize;
} screenItem;

/*
  Defines a screen
 *
 * items -  Array of screen Items that will be on that screen
 * len -    Length of screenItem array
 */
typedef struct {
  screenItem * items;
  uint8_t len;
} screen;

const uint16_t shiftLightSub[10] = {
  6000,
  5000,
  4000,
  3000,
  2500,
  2000,
  1500,
  1000,
  500,
  0
};

// Define all screen item arrays for each screen
screenItem raceScreenItems[7], shiftLightsItem;

screenItemNode shiftLightsGearPos, shiftLightsRPM;

// Define all screen structs
screen raceScreen;

// Define master array of all screen structs
screen* allScreens[NUM_SCREENS];

uint8_t screenNumber, warningCount;

volatile uint16_t backgroundColor, foregroundColor, foregroundColor2, warningColor, errorColor;

volatile dataItem pdmDataItems[PDM_DATAITEM_SIZE], gcmDataItems[GCM_DATAITEM_SIZE], motecDataItems[MOTEC_DATAITEM_SIZE], wheelDataItems[WHEEL_DATAITEM_SIZE];

void initDataItems(void); // Writes default values to all data items
// Initializes an individual dataItem
void initDataItem(volatile dataItem* data, double warn, double err, uint8_t whole, uint8_t dec);
void setDataItemDigits(volatile dataItem* data, uint8_t whole, uint8_t dec);
void initAllScreens(void); // Initializes all screenItems
void initScreen(uint8_t num); // Draws all non-dataItem data to start a screen
// Initializes an individual screenItem
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size,
  void (*redrawItem)(screenItemInfo *, screenItemNode *),
  volatile dataItem* data, uint32_t refresh, char * label, int8_t labelSize);
// Toggles between a few different data items on one screen
void changeScreen(uint8_t num);
// Continuously runs and refreshes stuff
void refreshScreenItems(void);
void clearScreen(void);
void resetScreenItems(void); // Resets all the values
uint8_t initNightMode(uint8_t on); // Night mode stuff
void nightMode(uint8_t on);
uint8_t checkDataChange(screenItemNode * head);
uint8_t getShiftLightsRevRange(uint16_t rpm, uint8_t gear);

// Redraw Functions!
void redrawDigit(screenItemInfo * item, screenItemNode * head);
void redrawGearPos(screenItemInfo * item, screenItemNode * head);
void redrawShiftLights(screenItemInfo * item, screenItemNode * head);

#endif /* _FSAE_LCD_H */
