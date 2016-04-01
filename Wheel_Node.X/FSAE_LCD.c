/**
 * FSAE LCD Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
#include "RA8875_driver.h"
#include "Wheel.h"
#include "FSAE_LCD.h"

// Motec Data Stream
dataItem rpm, throtPos, oilPress, oilTemp, waterTemp, lambda, manifoldPress, 
				batVoltage, wheelSpeedFL, wheelSpeedFR, wheelSpeedRL, wheelSpeedRR,
				gps, groundSpeed, driveSpeed, gpsSpeed, manifoldTemp, ambientTemp,
				ambientPress, fuelTemp, fuelPress, lambda1, lambda2, lambda3, lambda4,
				lcEnablity, fuelConsum;

// Tire Temps
dataItem ttFL1, ttFL2, ttFL3, ttFL4, ttFR1, ttFR2, ttFR3, ttFR4, ttRL1, ttRL2,
				ttRL3, ttRL4, ttRR1, ttRR2, ttRR3, ttRR4;

// Steering Wheel
dataItem swTemp, swSW1, swSW2, swSW3, swSW4, swROT1, swROT2, swROT3, swTROT1,
				swTROT2, swBUT1, swBUT2, swBUT3, swBUT4;

// PDM
dataItem pdmTemp, pdmICTemp, pdmCurrentDraw, pdmVBat, pdm12v, pdm5v5, pdm5v,
				pdm3v3, pdmIGNdraw, pdmIGNcut, pdmINJdraw, pdmINJcut, pdmFUELdraw, 
				pdmFUELNcut, pdmFUELPcut, pdmECUdraw, pdmECUcut, pdmWTRdraw, pdmWTRNcut, 
				pdmWTRPcut, pdmFANNdraw, pdmFANPdraw, pdmFANcut, pdmAUXdraw, pdmAUXcut, 
				pdmPLDUdraw, pdmPLDUcut, pdmPDLDdraw, pdmPDLDcut, pdm5v5draw, pdm5v5cut, 
				pdmBATdraw, pdmBATcut, pdmSTR0draw, pdmSTR0cut, pdmSTR1draw, pdmSTR1cut, 
				pdmSTR2draw, pdmSTR2cut, pdmSTRdraw;

// Paddle Shifting
dataItem paddleTemp, gearPos, neutQueue, upQueue, downQueue;

// Important Ones: oilTemp, oilPress, waterTemp, gearPos

// Race Screen - TopLeft, TopRight, BotLeft, BotRight, Center
screenItem raceScreenItems[5];
screen raceScreen;

uint8_t screenNumber;

void initDataItems(void){
	// Motec Vars
	initDataItem(&rpm,0,0,1000,2,1);
	initDataItem(&throtPos,0,0,1000,2,1);
	initDataItem(&oilPress,0,0,1000,2,1);
	initDataItem(&oilTemp,0,0,1000,2,1);
	initDataItem(&waterTemp,0,0,1000,2,1);
	initDataItem(&lambda,0,0,1000,2,1);
	initDataItem(&manifoldPress,0,0,1000,2,1);
	initDataItem(&batVoltage,0,0,1000,2,1);
	initDataItem(&wheelSpeedFL,0,0,1000,2,1);
	initDataItem(&wheelSpeedFR,0,0,1000,2,1);
	initDataItem(&wheelSpeedRL,0,0,1000,2,1);
	initDataItem(&wheelSpeedRR,0,0,1000,2,1);
	initDataItem(&gps,0,0,1000,2,1);
	initDataItem(&groundSpeed,0,0,1000,2,1);
	initDataItem(&driveSpeed,0,0,1000,2,1);
	initDataItem(&gpsSpeed,0,0,1000,2,1);
	initDataItem(&manifoldTemp,0,0,1000,2,1);
	initDataItem(&ambientTemp,0,0,1000,2,1);
	initDataItem(&ambientPress,0,0,1000,2,1);
	initDataItem(&fuelTemp,0,0,1000,2,1);
	initDataItem(&fuelPress,0,0,1000,2,1);
	initDataItem(&lambda1,0,0,1000,2,1);
	initDataItem(&lambda2,0,0,1000,2,1);
	initDataItem(&lambda3,0,0,1000,2,1);
	initDataItem(&lambda4,0,0,1000,2,1);
	initDataItem(&lcEnablity,0,0,1000,2,1);
	initDataItem(&fuelConsum,0,0,1000,2,1);

	// Tire Temps
	initDataItem(&ttFL1,0,0,1000,2,1);
	initDataItem(&ttFL2,0,0,1000,2,1);
	initDataItem(&ttFL3,0,0,1000,2,1);
	initDataItem(&ttFL4,0,0,1000,2,1);
	initDataItem(&ttFR1,0,0,1000,2,1);
	initDataItem(&ttFR2,0,0,1000,2,1);
	initDataItem(&ttFR3,0,0,1000,2,1);
	initDataItem(&ttFR4,0,0,1000,2,1);
	initDataItem(&ttRL1,0,0,1000,2,1);
	initDataItem(&ttRL2,0,0,1000,2,1);
	initDataItem(&ttRL3,0,0,1000,2,1);
	initDataItem(&ttRL4,0,0,1000,2,1);
	initDataItem(&ttRR1,0,0,1000,2,1);
	initDataItem(&ttRR2,0,0,1000,2,1);
	initDataItem(&ttRR3,0,0,1000,2,1);
	initDataItem(&ttRR4,0,0,1000,2,1);

	// Steering Wheel
	initDataItem(&swTemp,0,0,1000,2,1);
	initDataItem(&swSW1,0,0,1000,2,1);
	initDataItem(&swSW2,0,0,1000,2,1);
	initDataItem(&swSW3,0,0,1000,2,1);
	initDataItem(&swSW4,0,0,1000,2,1);
	initDataItem(&swROT1,0,0,1000,2,1);
	initDataItem(&swROT2,0,0,1000,2,1);
	initDataItem(&swROT3,0,0,1000,2,1);
	initDataItem(&swTROT1,0,0,1000,2,1);
	initDataItem(&swTROT2,0,0,1000,2,1);
	initDataItem(&swBUT1,0,0,1000,2,1);
	initDataItem(&swBUT2,0,0,1000,2,1);
	initDataItem(&swBUT3,0,0,1000,2,1);
	initDataItem(&swBUT4,0,0,1000,2,1);

	// Paddle Shifting
	initDataItem(&paddleTemp,0,0,1000,2,1);
	initDataItem(&gearPos,0,0,1000,2,1);
	initDataItem(&neutQueue,0,0,1000,2,1);
	initDataItem(&upQueue,0,0,1000,2,1);
	initDataItem(&downQueue,0,0,1000,2,1);
}

void initDataItem(dataItem* data, double warn, double err, uint32_t refresh, 
				uint8_t whole, uint8_t dec){
	data->value = 0;
	data->warnThreshold = warn;
	data->errThreshold = err;
	data->refreshInterval = refresh;
	data->wholeDigits = whole;
	data->decDigits = dec;
}

void initAllScreens(void){
	raceScreen.items = raceScreenItems;
	raceScreen.len = 5;
	initScreenItem(&(raceScreen.items[0]), 20, 20, 50, &oilTemp);
	initScreenItem(&(raceScreen.items[1]), 300, 20, 50, &waterTemp);
	initScreenItem(&(raceScreen.items[2]), 20, 150, 50, &oilPress);
	initScreenItem(&(raceScreen.items[3]), 300, 150, 50, 0x0);
	initScreenItem(&(raceScreen.items[4]), 100, 50, 100, &gearPos);
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, 
				dataItem* data){
	item->x = x;
	item->y = y;
	item->size = size;
	item->currentValue = 0;
	item->data = data;
	item->refreshTime = 0;
}

void initScreen(uint8_t num){
	if(num == 0){
		textMode();
		textSetCursor(0, 0);
		textTransparent(FOREGROUND_COLOR);
		textEnlarge(0);
		textWrite("OIL TEMP", 0);
		textSetCursor(0, 100);
		textWrite("OIL PRESS", 0);
	}
}

void changeScreen(uint8_t num){
	if( num < 0 || num > NUM_SCREENS) {return;}
	screenNumber = num;
	clearScreen();
	initScreen(num);
	refreshScreenItems();
}

void refreshScreenItems(void){
	
}

void clearScreen(void){
	fillScreen(BACKGROUND_COLOR);
}