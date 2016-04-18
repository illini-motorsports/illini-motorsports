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
#define NUM_SCREENS 			4
#define RACE_SCREEN 			0
#define PDM_DRAW_SCREEN 	1
#define PDM_CUT_SCREEN  	2
#define MOTEC_SCREEN			3
#define END_RACE_SCREEN		4
#define MIN_REFRESH 			20

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
	dataItem * data;
	uint32_t refreshTime;
	screenItemInfo info;
	void (*redrawItem)(screenItemInfo *, dataItem *);
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

screenItem raceScreenItems[5], pdmDrawItems[20], pdmCutItems[21], 
				motecItems[30], endRaceItems[9];
screen raceScreen, pdmDrawScreen, pdmCutScreen, motecScreen,
				endRaceScreen;
screen* allScreens[5];

uint8_t screenNumber;

// Uptimes
dataItem paddleUptime, loggerUptime, swUptime, pdmUptime;

// Motec Data Stream
dataItem rpm, throtPos, oilPress, oilTemp, waterTemp, lambda, manifoldPress, 
				batVoltage, wheelSpeedFL, wheelSpeedFR, wheelSpeedRL, wheelSpeedRR,
				gpsLat, gpsLong, groundSpeed, driveSpeed, gpsSpeed, manifoldTemp, 
				ambientTemp, ambientPress, fuelTemp, fuelPress, lambda1, lambda2, 
				lambda3, lambda4, lcEnablity, fuelConsum, gpsAltitude, gpsTime, runTime,
				fuelInjDuty, fuelTrim;

// Tire Temps
dataItem ttFL1, ttFL2, ttFL3, ttFL4, ttFL, ttFR1, ttFR2, ttFR3, ttFR4, ttFR, 
				ttRL1, ttRL2, ttRL3, ttRL4, ttRL, ttRR1, ttRR2, ttRR3, ttRR4, ttRR;

// Steering Wheel
dataItem swTemp, swSW1, swSW2, swSW3, swSW4, swROT1, swROT2, swROT3, swTROT1,
				swTROT2, swBUT1, swBUT2, swBUT3, swBUT4;

// PDM
dataItem pdmTemp, pdmICTemp, pdmCurrentDraw, pdmVBat, pdm12v, pdm5v5, pdm5v,
				pdm3v3, pdmIGNdraw, pdmIGNcut, pdmINJdraw, pdmINJcut, pdmFUELdraw, 
				pdmFUELNcut, pdmFUELPcut, pdmECUdraw, pdmECUNcut, pdmECUPcut, 
				pdmWTRdraw, pdmWTRNcut, pdmWTRPcut, pdmFANdraw, pdmFANNcut, pdmFANPcut,
				pdmAUXdraw, pdmAUXcut, pdmPDLUdraw, pdmPDLUcut, pdmPDLDdraw, pdmPDLDcut,
				pdm5v5draw, pdm5v5cut, pdmBATdraw, pdmBATcut, pdmSTR0draw, pdmSTR0cut,
				pdmSTR1draw, pdmSTR1cut, pdmSTR2draw, pdmSTR2cut, pdmSTRdraw;

// Paddle Shifting
dataItem paddleTemp, gearPos, neutQueue, upQueue, downQueue, gearVoltage;

// End Race Data
dataItem endLogNum, endNumLaps, endFastLap, endTireTempFL, endTireTempFR,
				endTireTempRL, endTireTempRR, endAmbientTemp, endFuelConsum;

// Lap Time Ring Buffer
dataItem lapTimeBuffer[20];
uint8_t lapTimeHead, numLaps;

errMsg errBuffer[20];
uint8_t errBufferHead, errBufferTail;

void initDataItems(void);
void initDataItem(dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec);
void initAllScreens(void);
void initScreen(uint8_t num);
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, dataItem *), dataItem* data);
void changeScreen(uint8_t num);
void refreshScreenItems(void);
void redrawDigit(screenItemInfo * item, dataItem * data);
void clearScreen(void);
double getMinLap(void);
void endRace(void);
void displayNoErrors(void);
void addError(char * errText, dataItem * item, uint8_t priority);


uint16_t tempColor(uint8_t temp);
void drawTireTemps(void);
void drawSuspensionPos(void);

#endif /* _FSAE_LCD_H */