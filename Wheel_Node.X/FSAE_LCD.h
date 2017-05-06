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
#define NUM_SCREENS 		8
#define RACE_SCREEN 		0
#define PDM_DRAW_SCREEN 	1
#define PDM_CUT_SCREEN  	2
#define MOTEC_SCREEN		3
#define END_RACE_SCREEN		4
#define CHASSIS_SCREEN		5
#define GENERAL_SCREEN		6
#define BRAKE_SCREEN		7
#define MIN_REFRESH		100

#define MIN_SUS_POS		5
#define MAX_SUS_POS		20

#define MIN_BRAKE_PRESS     	0
#define MAX_BRAKE_PRESS     	250

#define REV_RANGE_1 7000
#define REV_RANGE_2 8000
#define REV_RANGE_3 9000
#define REV_RANGE_4 10000
#define REV_RANGE_5 10500
#define REV_RANGE_6 11000
#define REV_RANGE_7 11500
#define REV_RANGE_8 12000
#define REV_RANGE_9 12500
#define REV_RANGE_REDLINE 13000

/*
 * Defines a data stream that is relevant to one or more screens
 *
 * value - 		double value of stream
 * warnThreshold - 	Value where data will enter a warning state
 * errThreshold - 	Value where data will enter an error state
 * refreshInterval -	Maximum refresh frequency
 * wholeDigits - 	Number of whole digits to display
 * decDigits - 		Number of decimal digits to display
 */
typedef struct {
	double value;
	double warnThreshold;
	double errThreshold;
	uint32_t refreshInterval;
	uint8_t wholeDigits;
	uint8_t decDigits;
} dataItem;


/*
 * Defines the minimum amount of information a redrawItem function needs to work
 *
 * x -		X coordinate
 * y - 		Y coordinate
 * size - 	Size of item
 */

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t size;
} screenItemInfo;


/*
 * Defines an item that will be displayed on a specific screen
 *
 * currentValue - 	Current value being displayed
 * data - 		Pointer to corresponding dataItem
 * refreshTime -	Time that the value was previously refreshed
 * info - 		Struct that contains necessary info for redrawing
 * redrawItem - 	Redraw function pointer, called when the item is refreshed
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
 * items -	Array of screen Items that will be on that screen
 * len - 	Length of screenItem array
 */
typedef struct {
	screenItem * items;
	uint8_t len;
} screen;

/*
typedef struct {
	char * errText;
	dataItem * item;
	uint8_t priority;
} errMsg;
*/

// Define all screen item arrays for each screen
screenItem raceScreenItems[10], pdmDrawItems[20], pdmCutItems[21], brakeItems[8], motecItems[30], endRaceItems[9], chassisItems[20], generalItems[7];
// Define all screen structs
screen raceScreen, pdmDrawScreen, pdmCutScreen, brakeScreen, motecScreen, endRaceScreen, chassisScreen, generalScreen;
// Define master array of all screen structs
screen* allScreens[8];

uint8_t screenNumber, auxNumber;

volatile uint16_t backgroundColor, foregroundColor, warningColor, errorColor;

// General Items
volatile dataItem * fanSw[2], * fuelSw[2], * wtrSw[2];

// Uptimes
volatile dataItem paddleUptime, loggerUptime, swUptime, pdmUptime;

// Motec Data Stream
volatile dataItem rpm, throtPos, oilPress, oilTemp, waterTemp, lambda, manifoldPress, batVoltage, wheelSpeedFL, wheelSpeedFR, wheelSpeedRL, wheelSpeedRR, gpsLat, gpsLong, groundSpeed, driveSpeed, gpsSpeed, manifoldTemp, ambientTemp, ambientPress, fuelTemp, fuelPress, lambda1, lambda2, lambda3, lambda4, lcEnablity, fuelConsum, gpsAltitude, gpsTime, runTime, fuelInjDuty, fuelTrim;

// Tire Temps
volatile dataItem ttFLA[4], ttFL, ttFRA[4], ttFR, ttRLA[4], ttRL, ttRRA[4], ttRR;

// Wheel Buttons
volatile dataItem rotary[3], tRotary[2], switches[4], momentaries[4];

// PDM
volatile dataItem pdmTemp, pdmICTemp, pdmCurrentDraw, pdmVBat, pdm12v, pdm5v5, pdm5v, pdm3v3, pdmIGNdraw, pdmIGNcut, pdmINJdraw, pdmINJcut, pdmFUELdraw, pdmFUELcut, pdmFUELPcut, pdmECUdraw, pdmECUcut, pdmECUPcut, pdmWTRdraw, pdmWTRcut, pdmWTRPcut, pdmFANdraw, pdmFANcut, pdmFANPcut, pdmAUXdraw, pdmAUXcut, pdmPDLUdraw, pdmPDLUcut, pdmPDLDdraw, pdmPDLDcut, pdm5v5draw, pdm5v5cut, pdmBATdraw, pdmBATcut, pdmSTRdraw, pdmSTRcut, pdmTOTdraw, pdmABSdraw, pdmABScut, pdmBVBATdraw, pdmBVBATcut;

// PDM Bitmaps
volatile dataItem STRenabl, BVBATenabl, PDLDenabl, PDLUenabl, AUXenabl, ABSenabl, FANenabl, WTRenabl, ECUenabl, FUELenabl, INJenabl, IGNenabl, STRpm, BVBATpm, B5V5pm, PDLDpm, PDLUpm, ABSpm, AUXpm, FANpm, WTRpm, ECUpm, FUELpm, INJpm, IGNpm, KILLpdmSw, ACT_DNpdmSw, ACT_UPpdmSw, ONpdmSw, STRpdmSw, AUX1pdmSw, AUX2pdmSw, ABSpdmSw, KillEngineFlag, KillCarFlag, OverTempFlag, FuelPrimeFlag;

volatile dataItem FUELOCCount, IGNOCCount, INJOCCount, ABSOCCount, PDLUOCCount, PDLDOCCount, FANOCCount, WTROCCount, ECUOCCount, AUXOCCount, BVBATOCCount, STROCCount;

// Rear Analog Hub
volatile dataItem susPosRR, susPosRL, engOutput, battCurrent, radInputTemp, radOutputTemp, swirlTemp, swirlPress;

// Front Analog Hub
volatile dataItem susPosFR, susPosFL, brakePressFront, brakePressRear, brakeMaxFront, brakeMinFront,brakeMaxRear, brakeMinRear, steeringAngle, accelPedalPos0, accelPedalPos1;

// Paddle Shifting
volatile dataItem paddleTemp, gearPos, neutQueue, upQueue, downQueue, gearVoltage;

// End Race Data
volatile dataItem endLogNum, endNumLaps, endFastLap, endTireTempFL, endTireTempFR, endTireTempRL, endTireTempRR, endAmbientTemp, endFuelConsum;

// Lap Time Ring Buffer
volatile dataItem lapTimeBuffer[20];
uint8_t lapTimeHead, numLaps;

void initDataItems(void); // Writes default values to all data items
// Initializes an individual dataItem
void initDataItem(volatile dataItem* data, double warn, double err,
	uint32_t refresh, uint8_t whole, uint8_t dec);
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
double getMinLap(void);
void endRace(void);
void displayNoErrors(void);

// Redraw Functions!
void redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawFanSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawFUELPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawWTRPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawTireTemp(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawSPBar(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawBrakeBar(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawRotary(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawShiftLightsRPM(screenItemInfo * item, volatile dataItem * data, double currentValue);
void redrawKILLCluster(screenItemInfo * item, volatile dataItem * data, double currentValue);

// Helper functions for colorful redraw functions
uint16_t tempColor(uint8_t temp);

#endif /* _FSAE_LCD_H */
