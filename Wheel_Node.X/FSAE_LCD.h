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

#define BACKGROUND_COLOR 0x001F
#define FOREGROUND_COLOR 0xFBE0
#define WARNING_COLOR 0x001F
#define ERROR_COLOR 0x001F
#define NUM_SCREENS 1
#define RACE_SCREEN 0

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
	uint16_t x;
	uint16_t y;
	uint16_t size;
	double currentValue;
	dataItem * data;
	uint32_t refreshTime;
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

screenItem raceScreenItems[5];
screen raceScreen;
screen* allScreens[1];

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
dataItem ttFL1, ttFL2, ttFL3, ttFL4, ttFR1, ttFR2, ttFR3, ttFR4, ttRL1, ttRL2,
				ttRL3, ttRL4, ttRR1, ttRR2, ttRR3, ttRR4;

// Steering Wheel
dataItem swTemp, swSW1, swSW2, swSW3, swSW4, swROT1, swROT2, swROT3, swTROT1,
				swTROT2, swBUT1, swBUT2, swBUT3, swBUT4;

// PDM
dataItem pdmTemp, pdmICTemp, pdmCurrentDraw, pdmVBat, pdm12v, pdm5v5, pdm5v,
				pdm3v3, pdmIGNdraw, pdmIGNcut, pdmINJdraw, pdmINJcut, pdmFUELdraw, 
				pdmFUELNcut, pdmFUELPcut, pdmECUdraw, pdmECUNcut, pdmECUPcut, pdmWTRdraw, pdmWTRNcut, 
				pdmWTRPcut, pdmFANdraw, pdmFANNcut, pdmFANPcut, pdmAUXdraw, pdmAUXcut, 
				pdmPDLUdraw, pdmPDLUcut, pdmPDLDdraw, pdmPDLDcut, pdm5v5draw, pdm5v5cut, 
				pdmBATdraw, pdmBATcut, pdmSTR0draw, pdmSTR0cut, pdmSTR1draw, pdmSTR1cut, 
				pdmSTR2draw, pdmSTR2cut, pdmSTRdraw;

// Paddle Shifting
dataItem paddleTemp, gearPos, neutQueue, upQueue, downQueue, gearVoltage;

void initDataItems(void);
void initDataItem(dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec);
void initAllScreens(void);
void initScreen(uint8_t num);
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, 
				dataItem* data);
void changeScreen(uint8_t num);
void refreshScreenItems(void);
void redrawItem(screenItem * item);
void clearScreen(void);

#endif /* _FSAE_LCD_H */