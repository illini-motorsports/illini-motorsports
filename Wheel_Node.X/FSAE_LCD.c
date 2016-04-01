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



// Important Ones: oilTemp, oilPress, waterTemp, gearPos

// Race Screen - TopLeft, TopRight, BotLeft, BotRight, Center
screenItem raceScreenItems[5];
screen raceScreen;
screen* allScreens[1];

uint8_t screenNumber;

void initDataItems(void){
	// Motec Vars
	initDataItem(&rpm,0,0,1000,2,1);
	initDataItem(&throtPos,0,0,1000,2,1);
	initDataItem(&oilPress,0,0,1000,1,1);
	oilPress.value = 3.5;
	initDataItem(&oilTemp,0,0,1000,3,0);
	oilTemp.value = 50;
	initDataItem(&waterTemp,0,0,1000,3,0);
	waterTemp.value = 95;
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
	initDataItem(&gearPos,0,0,1000,1,0);
	gearPos.value = 4;
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
	allScreens[0] = &raceScreen;
	raceScreen.items = raceScreenItems;
	raceScreen.len = 5;
	initScreenItem(&(raceScreen.items[0]), 10, 30, 30, &oilTemp);
	initScreenItem(&(raceScreen.items[1]), 300, 30, 30, &waterTemp);
	initScreenItem(&(raceScreen.items[2]), 10, 150, 30, &oilPress);
	initScreenItem(&(raceScreen.items[3]), 300, 180, 30, 0x0);
	initScreenItem(&(raceScreen.items[4]), 150, 50, 100, &gearPos);
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, 
				dataItem* data){
	item->x = x;
	item->y = y;
	item->size = size;
	item->currentValue = 1;
	item->data = data;
	item->refreshTime = 0;
}

void initScreen(uint8_t num){
	if(num == 0){
		textMode();
		textSetCursor(0, 0);
		textTransparent(FOREGROUND_COLOR);
		textEnlarge(0);
		textWrite("OIL TMP", 0);
		textSetCursor(0, 100);
		textWrite("OIL PRESS", 0);
		textSetCursor(300, 0);
		textWrite("WTR TMP", 0);
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
	screen *currScreen = allScreens[screenNumber];
	int i;
	for(i = 0;i<currScreen->len;i++){
		if(currScreen->items[i].data == 0x0){
			continue;
		}
		if((currScreen->items[i]).currentValue != 
						(currScreen->items[i]).data->value){
			redrawItem(&(currScreen->items[i]));
		}
	}
}

void redrawItem(screenItem * item){
	uint16_t fillColor;
	if(item->currentValue >= item->data->warnThreshold){
		if(item->currentValue >= item->data->errThreshold){
			fillColor = ERROR_COLOR;
		}else{
			fillColor = WARNING_COLOR;
		}
	}else{
		fillColor = BACKGROUND_COLOR;
	}
	
	uint16_t wholeNums, decNums;
	wholeNums = item->data->wholeDigits;
	decNums = item->data->decDigits;
	uint16_t fillWidth = (item->size)*(wholeNums + decNums) + (item->size)*0.2*(wholeNums + decNums - 1);
	fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
	sevenSegmentDecimal(item->x, item->y, item->size, item->data->wholeDigits + item->data->decDigits, item->data->decDigits, FOREGROUND_COLOR, item->data->value);
	item->currentValue = item->data->value;
}

void clearScreen(void){
	fillScreen(BACKGROUND_COLOR);
}