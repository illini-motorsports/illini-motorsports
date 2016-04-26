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

#define BACKGROUND_COLOR 	RA8875_WHITE
#define FOREGROUND_COLOR 	RA8875_BLACK
#define WARNING_COLOR 		BACKGROUND_COLOR
#define ERROR_COLOR 			BACKGROUND_COLOR
#define NUM_SCREENS 			6
#define RACE_SCREEN 			0
#define PDM_DRAW_SCREEN 	1
#define PDM_CUT_SCREEN  	2
#define MOTEC_SCREEN			3
#define END_RACE_SCREEN		4
#define CHASSIS_SCREEN		5
#define GENERAL_SCREEN		6
#define MIN_REFRESH 			100

#define MIN_SUS_POS				5
#define MAX_SUS_POS				20

/*
 * Defines a data stream that is relevant to one or more screens
 * 
 * value - 					double value of stream
 * warnThreshold - 	Value where data will enter a warning state
 * errThreshold - 	Value where data will enter an error state
 * refreshInterval -Maximum refresh frequency
 */
typedef struct {
	double value;
	double warnThreshold;
	double errThreshold;
	uint32_t refreshInterval;
	uint8_t wholeDigits;
	uint8_t decDigits;
} dataItem;


typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t size;
} screenItemInfo;


/* 
 * Defines an item that will be displayed on a specific screen
 * 
 * x - 							X coordinate
 * y - 							Y coordinate
 * size - 					Width to be drawn in.  Also used to determine how much to white out
 * 										during refresh
 * currentValue - 	Current value being displayed
 * data - 					Pointer to corresponding dataItem
 * refreshTime -			Time that the value was previously refreshed
 */
typedef struct {
	double currentValue;
	volatile dataItem * data;
	uint32_t refreshTime;
	screenItemInfo info;
	void (*redrawItem)(screenItemInfo *, volatile dataItem *);
} screenItem;

/*
 * Defines a screen
 * 
 * elements -	Array of screen Items that will be on that screen 
 * len - 			Length of element array
 */
typedef struct {
	screenItem * items;
	uint8_t len;
} screen;

typedef struct {
	char * errText;
	dataItem * item;
	uint8_t priority;
} errMsg;

screenItem raceScreenItems[8], pdmDrawItems[20], pdmCutItems[21], motecItems[30], endRaceItems[9], chassisItems[20], generalItems[6];
screen raceScreen, pdmDrawScreen, pdmCutScreen, motecScreen, endRaceScreen, chassisScreen, generalScreen;
screen* allScreens[7];

uint8_t screenNumber;

volatile uint16_t backgroundColor, foregroundColor, warningColor, errorColor;

// Uptimes
volatile dataItem paddleUptime, loggerUptime, swUptime, pdmUptime;

// Motec Data Stream
volatile dataItem rpm, throtPos, oilPress, oilTemp, waterTemp, lambda, manifoldPress, batVoltage, wheelSpeedFL, wheelSpeedFR, wheelSpeedRL, wheelSpeedRR, gpsLat, gpsLong, groundSpeed, driveSpeed, gpsSpeed, manifoldTemp, ambientTemp, ambientPress, fuelTemp, fuelPress, lambda1, lambda2, lambda3, lambda4, lcEnablity, fuelConsum, gpsAltitude, gpsTime, runTime, fuelInjDuty, fuelTrim;

// Tire Temps
volatile dataItem ttFLA[4], ttFL, ttFRA[4], ttFR, ttRLA[4], ttRL, ttRRA[4], ttRR;

// Wheel Buttons
volatile dataItem rotary[3], tRotary[2], switches[4], momentaries[4];

// PDM
volatile dataItem pdmTemp, pdmICTemp, pdmCurrentDraw, pdmVBat, pdm12v, pdm5v5, pdm5v, pdm3v3, pdmIGNdraw, pdmIGNcut, pdmINJdraw, pdmINJcut, pdmFUELdraw, pdmFUELNcut, pdmFUELPcut, pdmECUdraw, pdmECUNcut, pdmECUPcut, pdmWTRdraw, pdmWTRNcut, pdmWTRPcut, pdmFANdraw, pdmFANNcut, pdmFANPcut, pdmAUXdraw, pdmAUXcut, pdmPDLUdraw, pdmPDLUcut, pdmPDLDdraw, pdmPDLDcut, pdm5v5draw, pdm5v5cut, pdmBATdraw, pdmBATcut, pdmSTR0draw, pdmSTR0cut, pdmSTR1draw, pdmSTR1cut, pdmSTR2draw, pdmSTR2cut, pdmSTRdraw;

// Rear Analog Hub
volatile dataItem susPosRR, susPosRL, engOutput, battCurrent, radInputTemp, radOutputTemp, swirlTemp, swirlPress;

// Front Analog Hub
volatile dataItem susPosFR, susPosFL, brakePressFront, brakePressRear, steeringAngle, accelPedalPos0, accelPedalPos1;

// Paddle Shifting
volatile dataItem paddleTemp, gearPos, neutQueue, upQueue, downQueue, gearVoltage; 

// End Race Data
volatile dataItem endLogNum, endNumLaps, endFastLap, endTireTempFL, endTireTempFR, endTireTempRL, endTireTempRR, endAmbientTemp, endFuelConsum;

// Lap Time Ring Buffer
volatile dataItem lapTimeBuffer[20];
uint8_t lapTimeHead, numLaps;

errMsg errBuffer[20];
uint8_t errBufferHead, errBufferTail;

void initDataItems(void);
void initDataItem(volatile dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec);
void initAllScreens(void);
void initScreen(uint8_t num);
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, volatile dataItem *), volatile dataItem* data);
void changeScreen(uint8_t num);
void refreshScreenItems(void);
void clearScreen(void);
void resetScreenItems(void);
uint8_t initNightMode(uint8_t on);
void nightMode(uint8_t on);
double getMinLap(void);
void endRace(void);
void displayNoErrors(void);
void addError(char * errText, dataItem * item, uint8_t priority);

void redrawDigit(screenItemInfo * item, volatile dataItem * data);
void redrawGearPos(screenItemInfo * item, volatile dataItem * data);
void redrawFanSw(screenItemInfo * item, volatile dataItem * data);
void redrawPumpSw(screenItemInfo * item, volatile dataItem * data);
void redrawLCSw(screenItemInfo * item, volatile dataItem * data);
void redrawTireTemp(screenItemInfo * item, volatile dataItem * data);
void redrawSPBar(screenItemInfo * item, volatile dataItem * data);
void redrawRotary(screenItemInfo * item, volatile dataItem * data);

uint16_t tempColor(uint8_t temp);

#endif /* _FSAE_LCD_H */