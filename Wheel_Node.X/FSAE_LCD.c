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

// Initialize all the data streams
// This fn must be run before CAN is initialized
void initDataItems(void){
  // Motec Vars
  // Refresh Intervals
  // All Temp Channels - 500
  initDataItem(&rpm,0,0,MIN_REFRESH,5,0);
  initDataItem(&throtPos,0,0,MIN_REFRESH,3,1);
  initDataItem(&oilPress,0,0,MIN_REFRESH,1,2);
  initDataItem(&oilTemp,0,0,MIN_REFRESH,3,0);
  initDataItem(&waterTemp,0,0,MIN_REFRESH,3,0);
  initDataItem(&lambda,0,0,MIN_REFRESH,1,3);
  initDataItem(&manifoldPress,0,0,MIN_REFRESH,1,2);
  initDataItem(&batVoltage,0,0,MIN_REFRESH,2,2);
  initDataItem(&wheelSpeedFL,0,0,MIN_REFRESH,2,1);
  initDataItem(&wheelSpeedFR,0,0,MIN_REFRESH,2,1);
  initDataItem(&wheelSpeedRL,0,0,MIN_REFRESH,2,1);
  initDataItem(&wheelSpeedRR,0,0,MIN_REFRESH,2,1);
  initDataItem(&gpsLong,0,0,MIN_REFRESH,2,1); // Don't need
  initDataItem(&gpsLat,0,0,MIN_REFRESH,2,1); // Don't need
  initDataItem(&groundSpeed,0,0,MIN_REFRESH,2,1);
  initDataItem(&driveSpeed,0,0,MIN_REFRESH,2,1);
  initDataItem(&gpsSpeed,0,0,MIN_REFRESH,2,1);
  initDataItem(&manifoldTemp,0,0,MIN_REFRESH,3,0);
  initDataItem(&ambientTemp,0,0,MIN_REFRESH,3,0);
  initDataItem(&ambientPress,0,0,MIN_REFRESH,1,2);
  initDataItem(&fuelTemp,0,0,MIN_REFRESH,3,0);
  initDataItem(&fuelPress,0,0,MIN_REFRESH,1,2);
  initDataItem(&lambda1,0,0,MIN_REFRESH,1,3);
  initDataItem(&lambda2,0,0,MIN_REFRESH,1,3);
  initDataItem(&lambda3,0,0,MIN_REFRESH,1,3);
  initDataItem(&lambda4,0,0,MIN_REFRESH,1,3);
  initDataItem(&lcEnablity,0,0,MIN_REFRESH,1,0);
  initDataItem(&fuelConsum,0,0,MIN_REFRESH,2,1);
  initDataItem(&gpsAltitude,0,0,MIN_REFRESH,2,1); // ?
  initDataItem(&gpsTime,0,0,MIN_REFRESH,2,1); // ?
  initDataItem(&runTime,0,0,MIN_REFRESH,4,0);
  initDataItem(&fuelInjDuty,0,0,MIN_REFRESH,3,1);
  initDataItem(&fuelTrim,0,0,MIN_REFRESH,3,1);

  // Tire Temps
  initDataItem(&ttFLA[0],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFLA[1],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFLA[2],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFLA[3],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFL,0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFRA[0],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFRA[1],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFRA[2],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFRA[3],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttFR,0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRLA[0],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRLA[1],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRLA[2],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRLA[3],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRL,0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRRA[0],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRRA[1],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRRA[2],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRRA[3],0,0,MIN_REFRESH,2,1);
  initDataItem(&ttRR,0,0,MIN_REFRESH,2,1);

  // Paddle Shifting
  initDataItem(&paddleTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&gearPos,0,0,MIN_REFRESH,1,0);
  initDataItem(&neutQueue,0,0,MIN_REFRESH,1,0);
  initDataItem(&upQueue,0,0,MIN_REFRESH,1,0);
  initDataItem(&downQueue,0,0,MIN_REFRESH,1,0);
  initDataItem(&gearVoltage,0,0,MIN_REFRESH,1,2);

  // PDM
  initDataItem(&pdmTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmICTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmCurrentDraw,0,0,MIN_REFRESH,3,1);
  initDataItem(&pdmVBat,0,0,MIN_REFRESH,2,2);
  initDataItem(&pdm12v,0,0,MIN_REFRESH,2,2);
  initDataItem(&pdm5v5,0,0,MIN_REFRESH,1,2);
  initDataItem(&pdm5v,0,0,MIN_REFRESH,1,2);
  initDataItem(&pdm3v3,0,0,MIN_REFRESH,1,2);
  // Draw (^str) - 2,2
  // Cut (^str)) - 3,1
  initDataItem(&pdmIGNdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmIGNcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmINJdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmINJcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFUELdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFUELcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFUELPcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmECUdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmECUcut,0,0,MIN_REFRESH,0,1);
  initDataItem(&pdmECUPcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmWTRdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmWTRcut,0,0,MIN_REFRESH,0,1);
  initDataItem(&pdmWTRPcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFANdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFANcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmFANPcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmAUXdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmAUXcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmPDLUdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmPDLUcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmPDLDdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmPDLDcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdm5v5cut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmBATdraw,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmBATcut,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmSTRdraw,0,0,MIN_REFRESH,3,1);
  initDataItem(&pdmSTRcut,0,0,MIN_REFRESH,2,1);

  // PDM Bitmaps
  initDataItem(&STRenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&BVBATenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&PDLDenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&PDLUenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&AUXenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&FANenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&WTRenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&ECUenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&FUELenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&INJenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&IGNenabl,0,0,MIN_REFRESH,1,0);
  initDataItem(&BVBATpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&PDLDpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&PDLUpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&AUXpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&FANpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&WTRpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&ECUpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&FUELpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&INJpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&IGNpm,0,0,MIN_REFRESH,1,0);
  initDataItem(&KILLpdmSw,0,0,MIN_REFRESH,1,0);
  initDataItem(&ACT_DNpdmSw,0,0,MIN_REFRESH,1,0);
  initDataItem(&ACT_UPpdmSw,0,0,MIN_REFRESH,1,0);
  initDataItem(&ONpdmSw,0,0,MIN_REFRESH,1,0);
  initDataItem(&STRpdmSw,0,0,MIN_REFRESH,1,0);

  // Rear Analog Hub
  initDataItem(&susPosRR,0,0,MIN_REFRESH,2,1);
  initDataItem(&susPosRL,0,0,MIN_REFRESH,2,1);
  initDataItem(&engOutput,0,0,MIN_REFRESH,2,1);
  initDataItem(&battCurrent,0,0,MIN_REFRESH,2,1);
  initDataItem(&radInputTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&radOutputTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&swirlTemp,0,0,MIN_REFRESH,2,1);
  initDataItem(&swirlPress,0,0,MIN_REFRESH,2,1);

  // Front Analog Hub
  initDataItem(&susPosFR,0,0,MIN_REFRESH,2,1);
  initDataItem(&susPosFL,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakePressFront,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakePressRear,0,0,MIN_REFRESH,2,1);
  initDataItem(&steeringAngle,0,0,MIN_REFRESH,2,1);
  initDataItem(&accelPedalPos0,0,0,MIN_REFRESH,2,1);
  initDataItem(&accelPedalPos1,0,0,MIN_REFRESH,2,1);
  // Uptimes
  // uptimes - 4,0
  initDataItem(&paddleUptime,0,0,MIN_REFRESH,2,1);
  initDataItem(&loggerUptime,0,0,MIN_REFRESH,2,1);
  initDataItem(&swUptime,0,0,MIN_REFRESH,2,1);
  initDataItem(&pdmUptime,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakeMinFront,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakeMaxFront,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakeMinRear,0,0,MIN_REFRESH,2,1);
  initDataItem(&brakeMaxRear,0,0,MIN_REFRESH,2,1);
  brakeMinFront.value = 100;
  brakeMinRear.value = 100;

  // EndRace
  initDataItem(&endLogNum,0,0,1000,2,1);
  initDataItem(&endNumLaps,0,0,1000,2,1);
  initDataItem(&endFastLap,0,0,1000,2,1);
  initDataItem(&endTireTempFL,0,0,1000,2,1);
  initDataItem(&endTireTempFR,0,0,1000,2,1);
  initDataItem(&endTireTempRL,0,0,1000,2,1);
  initDataItem(&endTireTempRR,0,0,1000,2,1);
  initDataItem(&endAmbientTemp,0,0,1000,2,1);
  initDataItem(&endFuelConsum,0,0,1000,2,1);

  //Switches and Rotaries
  initDataItem(&rotary[0],0,0,MIN_REFRESH,1,0);
  initDataItem(&rotary[1],0,0,MIN_REFRESH,1,0);
  initDataItem(&rotary[2],0,0,MIN_REFRESH,1,0);
  initDataItem(&tRotary[0],0,0,MIN_REFRESH,1,0);
  initDataItem(&tRotary[1],0,0,MIN_REFRESH,1,0);
  initDataItem(&tRotary[2],0,0,MIN_REFRESH,1,0);
  initDataItem(&switches[0],0,0,MIN_REFRESH,1,0);
  initDataItem(&switches[1],0,0,MIN_REFRESH,1,0);
  initDataItem(&switches[2],0,0,MIN_REFRESH,1,0);
  initDataItem(&switches[3],0,0,MIN_REFRESH,1,0);
  initDataItem(&momentaries[0],0,0,MIN_REFRESH,1,0);
  initDataItem(&momentaries[1],0,0,MIN_REFRESH,1,0);
  initDataItem(&momentaries[2],0,0,MIN_REFRESH,1,0);
  initDataItem(&momentaries[3],0,0,MIN_REFRESH,1,0);

  fanSw[0] = &switches[0];
  fanSw[1] = &FANenabl;
  fuelSw[0] = &switches[1];
  fuelSw[1] = &FUELenabl;
  wtrSw[0] = &switches[2];
  wtrSw[1] = &WTRenabl;
}

void initDataItem(volatile dataItem* data, double warn, double err, uint32_t refresh, uint8_t whole, uint8_t dec){
  data->value = 0;
  data->warnThreshold = warn;
  data->errThreshold = err;
  data->refreshInterval = refresh;
  data->wholeDigits = whole;
  data->decDigits = dec;
}

// Initializes all the screen and screenitem variables in all
// the screens that might be displayed
void initAllScreens(void){
  // Initialize colors
  backgroundColor = RA8875_WHITE;
  foregroundColor = RA8875_BLACK;
  //initNightMode(switches[3].value);
  initNightMode(1); //TODO

  // All Screens Stuff
  allScreens[GENERAL_SCREEN] = &generalScreen;
  generalScreen.items = generalItems;
  generalScreen.len = 7;
  initScreenItem(&generalItems[0], 20, 30, 15, redrawRotary, &rotary[0]);
  initScreenItem(&generalItems[1], 20, 30, 15, redrawRotary, &rotary[1]);
  initScreenItem(&generalItems[2], 20, 30, 15, redrawRotary, &rotary[2]);
  initScreenItem(&generalItems[3], 20, 30, 15, redrawFanSw, &switches[0]);
  initScreenItem(&generalItems[4], 20, 30, 15, redrawFUELPumpSw, &switches[1]);
  initScreenItem(&generalItems[5], 20, 30, 15, redrawWTRPumpSw, &switches[2]);
  initScreenItem(&generalItems[6], 20, 30, 15, redrawShiftLightsRPM, &rpm);

  // Race Screen Stuff
  allScreens[RACE_SCREEN] = &raceScreen;
  raceScreen.items = raceScreenItems;
  raceScreen.len = 8;
  initScreenItem(&raceScreenItems[0], 120, 20, 15, redrawFanSw, *fanSw);
  initScreenItem(&raceScreenItems[1], 240, 20, 15, redrawFUELPumpSw,*fuelSw);
  initScreenItem(&raceScreenItems[2], 360, 20, 15, redrawWTRPumpSw, *wtrSw);
  initScreenItem(&raceScreenItems[3], 20, 70, 30, redrawDigit, &oilTemp);
  initScreenItem(&raceScreenItems[4], 330, 70, 30, redrawDigit, &waterTemp);
  initScreenItem(&raceScreenItems[5], 20, 190, 30, redrawDigit, &oilPress);
  initScreenItem(&raceScreenItems[6], 330, 180, 30, redrawDigit, &batVoltage);
  initScreenItem(&raceScreenItems[7], 170, 50, 100, redrawGearPos, &gearPos);

  // PDM stuff
  allScreens[PDM_DRAW_SCREEN] = &pdmDrawScreen;
  pdmDrawScreen.items = pdmDrawItems;
  pdmDrawScreen.len = 20;
  initScreenItem(&pdmDrawItems[1], 10, 50, 15, redrawDigit, &pdmIGNdraw);
  initScreenItem(&pdmDrawItems[2], 85, 50, 15, redrawDigit, &pdmINJdraw);
  initScreenItem(&pdmDrawItems[3], 160, 50, 15, redrawDigit, &pdmFUELdraw);
  initScreenItem(&pdmDrawItems[4], 235, 50, 15, redrawDigit, &pdmECUdraw);
  initScreenItem(&pdmDrawItems[5], 310, 50, 15, redrawDigit, &pdmWTRdraw);
  initScreenItem(&pdmDrawItems[6], 385, 50, 15, redrawDigit, &pdmFANdraw);
  initScreenItem(&pdmDrawItems[7], 10, 100, 15, redrawDigit, &pdmAUXdraw);
  initScreenItem(&pdmDrawItems[8], 85, 100, 15, redrawDigit, &pdmPDLUdraw);
  initScreenItem(&pdmDrawItems[9], 160, 100, 15, redrawDigit, &pdmPDLDdraw);
  initScreenItem(&pdmDrawItems[10], 235, 100, 15, redrawDigit, &pdm5v5draw);
  initScreenItem(&pdmDrawItems[11], 310, 100, 15, redrawDigit, &pdmBATdraw);
  initScreenItem(&pdmDrawItems[16], 385, 100, 15, redrawDigit, &pdmFUELPcut);
  initScreenItem(&pdmDrawItems[17], 10, 150, 15, redrawDigit, &pdmECUPcut);
  initScreenItem(&pdmDrawItems[18], 85, 150, 15, redrawDigit, &pdmWTRPcut);
  initScreenItem(&pdmDrawItems[19], 160, 150, 15, redrawDigit, &pdmFANPcut);
  initScreenItem(&pdmDrawItems[15], 280, 200, 15, redrawDigit, &pdmSTRdraw);
  initScreenItem(&pdmDrawItems[0], 370, 200, 15, redrawDigit, &pdmCurrentDraw);


  allScreens[PDM_CUT_SCREEN] = &pdmCutScreen;
  pdmCutScreen.items = pdmCutItems;
  pdmCutScreen.len = 21;
  initScreenItem(&pdmCutItems[0], 10, 50, 15, redrawDigit, &pdmTemp);
  initScreenItem(&pdmCutItems[1], 85, 50, 15, redrawDigit, &pdmICTemp);
  initScreenItem(&pdmCutItems[4], 310, 50, 15, redrawDigit, &pdm5v5);
  initScreenItem(&pdmCutItems[5], 385, 50, 15, redrawDigit, &pdm5v);
  initScreenItem(&pdmCutItems[6], 10, 100, 15, redrawDigit, &pdm3v3);
  initScreenItem(&pdmCutItems[7], 85, 100, 15, redrawDigit, &pdmIGNcut);
  initScreenItem(&pdmCutItems[8], 160, 100, 15, redrawDigit, &pdmINJcut);
  initScreenItem(&pdmCutItems[9], 235, 100, 15, redrawDigit, &pdmFUELcut);
  pdmECUcut.decDigits = 1;
  pdmECUcut.wholeDigits = 2;
  initScreenItem(&pdmCutItems[10], 160, 50, 15, redrawDigit, &pdmECUcut);
  initScreenItem(&pdmCutItems[12], 10, 150, 15, redrawDigit, &pdmFANcut);
  initScreenItem(&pdmCutItems[13], 85, 150, 15, redrawDigit, &pdmAUXcut);
  initScreenItem(&pdmCutItems[14], 160, 150, 15, redrawDigit, &pdmPDLUcut);
  initScreenItem(&pdmCutItems[15], 235, 150, 15, redrawDigit, &pdmPDLDcut);
  initScreenItem(&pdmCutItems[16], 310, 150, 15, redrawDigit, &pdm5v5cut);
  initScreenItem(&pdmCutItems[17], 385, 150, 15, redrawDigit, &pdmBATcut);
  initScreenItem(&pdmCutItems[18], 10, 200, 15, redrawDigit, &pdmSTRcut);

  //brake pressure
  initScreenItem(&pdmCutItems[19], 85, 200, 15, redrawDigit,&brakePressFront);
  initScreenItem(&pdmCutItems[20], 160, 200, 15, redrawDigit, &brakePressRear);

  initScreenItem(&pdmCutItems[2], 235, 200, 15, redrawDigit, &pdmVBat);
  initScreenItem(&pdmCutItems[3], 330, 200, 15, redrawDigit, &pdm12v);


  // MoTec Stuff
  allScreens[MOTEC_SCREEN] = &motecScreen;
  motecScreen.items = motecItems;
  motecScreen.len = 30;
  initScreenItem(&motecItems[0], 10, 30, 20, redrawDigit, &rpm);
  initScreenItem(&motecItems[1], 10, 30, 20, redrawDigit, &throtPos);
  initScreenItem(&motecItems[2], 10, 30, 20, redrawDigit, &oilPress);
  initScreenItem(&motecItems[3], 10, 30, 20, redrawDigit, &oilTemp);
  initScreenItem(&motecItems[4], 10, 30, 20, redrawDigit, &waterTemp);
  initScreenItem(&motecItems[5], 10, 30, 20, redrawDigit, &lambda);
  initScreenItem(&motecItems[6], 10, 30, 20, redrawDigit, &manifoldPress);
  initScreenItem(&motecItems[7], 10, 30, 20, redrawDigit, &batVoltage);
  initScreenItem(&motecItems[8], 10, 30, 20, redrawDigit, &wheelSpeedFL);
  initScreenItem(&motecItems[9], 10, 30, 20, redrawDigit, &wheelSpeedFR);
  initScreenItem(&motecItems[10], 10, 30, 20, redrawDigit, &wheelSpeedRL);
  initScreenItem(&motecItems[11], 10, 30, 20, redrawDigit, &wheelSpeedRR);
  initScreenItem(&motecItems[12], 10, 30, 20, redrawDigit, &gpsLat);
  initScreenItem(&motecItems[13], 10, 30, 20, redrawDigit, &gpsLong);
  initScreenItem(&motecItems[14], 10, 30, 20, redrawDigit, &groundSpeed);
  initScreenItem(&motecItems[15], 10, 30, 20, redrawDigit, &driveSpeed);
  initScreenItem(&motecItems[16], 10, 30, 20, redrawDigit, &gpsSpeed);
  initScreenItem(&motecItems[17], 10, 30, 20, redrawDigit, &manifoldTemp);
  initScreenItem(&motecItems[18], 10, 30, 20, redrawDigit, &ambientTemp);
  initScreenItem(&motecItems[19], 10, 30, 20, redrawDigit, &ambientPress);
  initScreenItem(&motecItems[20], 10, 30, 20, redrawDigit, &fuelTemp);
  initScreenItem(&motecItems[21], 10, 30, 20, redrawDigit, &fuelPress);
  initScreenItem(&motecItems[22], 10, 30, 20, redrawDigit, &lambda1);
  initScreenItem(&motecItems[23], 10, 30, 20, redrawDigit, &lambda2);
  initScreenItem(&motecItems[24], 10, 30, 20, redrawDigit, &lambda3);
  initScreenItem(&motecItems[25], 10, 30, 20, redrawDigit, &lambda4);
  initScreenItem(&motecItems[26], 10, 30, 20, redrawDigit, &lcEnablity);
  initScreenItem(&motecItems[27], 10, 30, 20, redrawDigit, &fuelConsum);
  initScreenItem(&motecItems[28], 10, 30, 20, redrawDigit, &gpsAltitude);
  initScreenItem(&motecItems[29], 10, 30, 20, redrawDigit, &gpsTime);
  initScreenItem(&motecItems[30], 10, 30, 20, redrawDigit, &fuelInjDuty);
  initScreenItem(&motecItems[31], 10, 30, 20, redrawDigit, &fuelTrim);

  // End Race Screen
  allScreens[END_RACE_SCREEN] = &endRaceScreen;
  endRaceScreen.items = endRaceItems;
  endRaceScreen.len = 9;
  initScreenItem(&endRaceItems[0], 10, 30, 20, redrawDigit, &endLogNum);
  initScreenItem(&endRaceItems[1], 10, 30, 20, redrawDigit, &endNumLaps);
  initScreenItem(&endRaceItems[2], 10, 30, 20, redrawDigit, &endFastLap);
  initScreenItem(&endRaceItems[3], 10, 30, 20, redrawDigit, &endTireTempFL);
  initScreenItem(&endRaceItems[4], 10, 30, 20, redrawDigit, &endTireTempFR);
  initScreenItem(&endRaceItems[5], 10, 30, 20, redrawDigit, &endTireTempRL);
  initScreenItem(&endRaceItems[6], 10, 30, 20, redrawDigit, &endTireTempRR);
  initScreenItem(&endRaceItems[3], 10, 30, 20, redrawTireTemp, &endTireTempFL);
  initScreenItem(&endRaceItems[4], 10, 30, 20, redrawTireTemp, &endTireTempFR);
  initScreenItem(&endRaceItems[5], 10, 30, 20, redrawTireTemp, &endTireTempRL);
  initScreenItem(&endRaceItems[6], 10, 30, 20, redrawTireTemp, &endTireTempRR);
  initScreenItem(&endRaceItems[7], 10, 30, 20, redrawDigit, &endAmbientTemp);
  initScreenItem(&endRaceItems[8], 10, 30, 20, redrawDigit, &endFuelConsum);

  allScreens[CHASSIS_SCREEN] = &chassisScreen;
  chassisScreen.items = chassisItems;
  chassisScreen.len = 20;
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttFL);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttFR);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttRL);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawDigit, &ttRR);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, ttFLA);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, ttFRA);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, ttRLA);
  initScreenItem(&chassisItems[0], 10, 30, 20, redrawTireTemp, ttRRA);

  //brake screen
  allScreens[BRAKE_SCREEN] = &brakeScreen;
  brakeScreen.items = brakeItems;
  brakeScreen.len = 8;
  initScreenItem(&brakeItems[0], 20, 180, 30, redrawDigit, &brakePressFront);
  initScreenItem(&brakeItems[1], 360, 180, 30, redrawDigit, &brakePressRear);
  initScreenItem(&brakeItems[2], 80, 60, 20, redrawDigit, &brakeMinFront);
  initScreenItem(&brakeItems[3], 80, 100, 20, redrawDigit, &brakeMaxFront);
  initScreenItem(&brakeItems[4],360, 60, 20, redrawDigit, &brakeMinRear);
  initScreenItem(&brakeItems[5], 360, 100, 20, redrawDigit, &brakeMaxRear);
  initScreenItem(&brakeItems[6], 190, 40, 20, redrawBrakeBar, &brakePressFront);
  initScreenItem(&brakeItems[7], 250, 40, 20, redrawBrakeBar, &brakePressRear);


  // Lap Time Stuff
  lapTimeHead = 0;
  numLaps = 0;
  int i;
  for(i=0;i<20;i++){
    initDataItem(&(lapTimeBuffer[i]),0,0,1000,2,1);
    lapTimeBuffer[i].value = -1;
  }
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, void (*redrawItem)(screenItemInfo *, volatile dataItem *, double), volatile dataItem* data){
  item->info.x = x;
  item->info.y = y;
  item->info.size = size;
  item->currentValue = 1;
  item->data = data;
  item->refreshTime = millis;
  item->redrawItem = redrawItem;
}

// Helper function for drawing labels and such on a new screen
void initScreen(uint8_t num){
  switch(num){
    case RACE_SCREEN:
      textMode();
      textEnlarge(1);
      textTransparent(foregroundColor);
      textSetCursor(0, 40);
      textWrite("OIL TEMP");
      textSetCursor(0, 140);
      textWrite("OIL PRESS");
      textSetCursor(330, 40);
      textWrite("WTR TMP");
      textSetCursor(330,140);
      textWrite("BAT V");
      textEnlarge(0);
      graphicsMode();
      break;

    case PDM_DRAW_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,30);
      textWrite("IGN DRW");
      textSetCursor(85, 30);
      textWrite("INJ DRW");
      textSetCursor(160, 30);
      textWrite("FUEL DRW");
      textSetCursor(235, 30);
      textWrite("ECU DRW");
      textSetCursor(310, 30);
      textWrite("WTR DRW");
      textSetCursor(385, 30);
      textWrite("FAN DRW");
      textSetCursor(10, 80);
      textWrite("AUX DRW");
      textSetCursor(85, 80);
      textWrite("PDLU DRW");
      textSetCursor(160,80);
      textWrite("PDLD DRW");
      textSetCursor(235, 80);
      textWrite("5V5 DRW");
      textSetCursor(310, 80);
      textWrite("BAT DRW");
      textSetCursor(385, 80);
      textWrite("FUEL P CUT");
      textSetCursor(10, 130);
      textWrite("ECU PCUT");
      textSetCursor(85, 130);
      textWrite("WTR PCUT");
      textSetCursor(160, 130);
      textWrite("FAN PCUT");
      textSetCursor(10, 180);
      textWrite("STR0 DRW");
      textSetCursor(100, 180);
      textWrite("STR1 DRW");
      textSetCursor(190, 180);
      textWrite("STR2 DRW");
      textSetCursor(280, 180);
      textWrite("STR DRW");
      textSetCursor(370, 180);
      textWrite("TOT DRW");
      graphicsMode();
      break;

    case PDM_CUT_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,30);
      textWrite("PDM TEMP");
      textSetCursor(85, 30);
      textWrite("IC TEMP");
      textSetCursor(160, 30);
      textWrite("BAT VOLT");
      textSetCursor(235, 30);
      textWrite("12V VOLT");
      textSetCursor(310, 30);
      textWrite("5V5 VOLT");
      textSetCursor(385, 30);
      textWrite("5V VOLT");
      textSetCursor(10, 80);
      textWrite("3V3 VOLT");
      textSetCursor(85, 80);
      textWrite("IGN CUT");
      textSetCursor(160,80);
      textWrite("INJ CUT");
      textSetCursor(235, 80);
      textWrite("FUEL NCUT");
      textSetCursor(310, 80);
      textWrite("ECU NCUT");
      textSetCursor(385, 80);
      textWrite("WTR NCUT");
      textSetCursor(10, 130);
      textWrite("FAN NCUT");
      textSetCursor(85, 130);
      textWrite("AUX CUT");
      textSetCursor(160, 130);
      textWrite("PDLU CUT");
      textSetCursor(235, 130);
      textWrite("PDLU CUT");
      textSetCursor(310,130);
      textWrite("5V5 CUT");
      textSetCursor(385, 130);
      textWrite("BAT CUT");
      textSetCursor(10, 180);
      textWrite("STR0 CUT");
      textSetCursor(85, 180);
      textWrite("BRK F");
      textSetCursor(160, 180);
      textWrite("BRK R");
      graphicsMode();
      break;

    case MOTEC_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;
    case END_RACE_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;
    case CHASSIS_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;

    case BRAKE_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textEnlarge(1);
      textSetCursor(20, 235);
      textWrite("BRK F");
      textSetCursor(360, 235);
      textWrite("BRK R");
      textSetCursor(20,60);
      textWrite("MIN F:");
      textSetCursor(20,100);
      textWrite("MAX F:");
      textSetCursor(300,60);
      textWrite("MIN R:");
      textSetCursor(300,100);
      textWrite("MAX R:");
      graphicsMode();
      textEnlarge(0);
      break;
  }
}

void changeAUXType(uint8_t num){
  if(num != auxNumber && screenNumber == RACE_SCREEN){
    fillRect(300, 140, 179, 135, backgroundColor);
    textMode();
    textTransparent(foregroundColor);
    textEnlarge(1);
    textSetCursor(330, 140);
    switch(num) {
      // Battery Voltage
      case 0:
        textWrite("BAT V");
        raceScreenItems[6].data = &batVoltage;
        raceScreenItems[6].info.x += 30;
        break;

        // Lambda
      case 1:
        textWrite("LAMBDA");
        raceScreenItems[6].data = &lambda;
        break;

        // RPM
      case 2:
        textWrite("RPM");
        raceScreenItems[6].data = &rpm;
        raceScreenItems[6].info.x -= 30;
        break;
    }
    textEnlarge(0);
    graphicsMode();
    auxNumber = num;
  }
}

// User called function to change which screen is being displayed
void changeScreen(uint8_t num){
  if( num < 0 || num > NUM_SCREENS) {return;}
  screenNumber = num;
  clearScreen();
  initScreen(num);
  resetScreenItems();
  refreshScreenItems();
}

// Function to asynchronously update all the items currently being displayed
// A number will only be redrawn if its dataItem value has changed, and it
// has surpassed its refresh interval
void refreshScreenItems(void){
  // change night mode if the switch was toggled
  //nightMode(switches[3].value); //TODO
  screen *currScreen = allScreens[GENERAL_SCREEN];
  int i;
  for(i = 0;i<currScreen->len;i++){
    screenItem * currItem = &currScreen->items[i];
    if(currItem->data && millis - currItem->refreshTime >= currItem->data->refreshInterval){
      currItem->redrawItem(&currItem->info, currItem->data, currItem->currentValue);
      currItem->currentValue = currItem->data->value;
      currItem->refreshTime = millis;
    }
  }

  currScreen = allScreens[screenNumber];
  for(i = 0;i<currScreen->len;i++){
    screenItem * currItem = &currScreen->items[i];
    if(currItem->data && millis - currItem->refreshTime >= currItem->data->refreshInterval){
      currItem->redrawItem(&currItem->info, currItem->data, currItem->currentValue);
      currItem->currentValue = currItem->data->value;
      currItem->refreshTime = millis;
    }
  }
}

// Redraw General Data
void redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  // Set Backround Color
  uint16_t fillColor;
  if(data->value >= data->warnThreshold){
    if(data->value >= data->errThreshold){
      fillColor = errorColor;
    }else{
      fillColor = warningColor;
    }
  }else{
    fillColor = backgroundColor;
  }
  // Calculate Fill Rectangle width
  uint16_t wholeNums, decNums;
  wholeNums = data->wholeDigits;
  decNums = data->decDigits;
  uint16_t fillWidth = (item->size)*(wholeNums+decNums)+(item->size)*0.2*(wholeNums+decNums-1);
  if(decNums){
    fillWidth += (item->size)/5;
  }
  fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
  sevenSegmentDecimal(item->x,item->y,item->size,wholeNums+decNums,decNums,foregroundColor,data->value);
}

// For Single Digits with Error and Neutral Displays
void redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 1.75, backgroundColor);
  if(data->value == 0){
    sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_N);
  }
  else if(data->value == 7){
    sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_E);
  }
  else{
    sevenSegmentDigit(item->x, item->y, item->size, foregroundColor, data->value);
  }
}

// For Fan Override Indicator
void redrawFanSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // Fan On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// For Water Pump Override Indicator
void redrawWTRPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // WTR On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// For Launch Control Override Indicator
void redrawFUELPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Override
  if(data[0].value){
    fillCircle(item->x, item->y, item->size, RA8875_GREEN);
  }
  // FUEL On, Switch Not toggled
  else if(data[1].value){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Load off, switch off
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

// Uses the 4 Tire Temp sensors to draw a color gradient tire
void redrawTireTemp(screenItemInfo * item, volatile dataItem * data, double currentValue){
  uint16_t fillColor = tempColor(data->value);
  uint16_t x = item->x;
  uint16_t y = item->y;
  uint16_t width = item->size / 4;
  uint16_t height = item->size * 2;
  fillCircleSquare(x,y,width*2,height,width,tempColor(data[0].value));
  fillCircleSquare(x+(2*width),y,width*2,height,width,tempColor(data[3].value));
  fillRect(x+width,y,width,height,tempColor(data[1].value));
  fillRect(x+(2*width),y,width,height,tempColor(data[2].value));
}

// Draws a bar with a height proportional to the suspension position
void redrawSPBar(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 5, backgroundColor);
  if(data->value > MIN_SUS_POS){
    uint16_t height = ((item->size*5)/(MAX_SUS_POS-MIN_SUS_POS))*(data->value-MIN_SUS_POS);
    if(height > MAX_SUS_POS){
      height = MAX_SUS_POS;
    }
    fillRect(item->x,item->y-(item->size*5)+height,item->size,height,RA8875_RED);
  }
}

//Draw a bar with height proportional to the brake pressure
void redrawBrakeBar(screenItemInfo * item, volatile dataItem * data, double currentValue){
  if(data->value == currentValue){
    return;
  }
  fillRect(item->x, item->y, item->size, item->size * 10, backgroundColor);
  if(data->value > MIN_BRAKE_PRESS){
    uint16_t height = ((item->size*10)/(MAX_BRAKE_PRESS-MIN_BRAKE_PRESS))*(data->value-MIN_BRAKE_PRESS);
    if(height > MAX_BRAKE_PRESS){
      height = MAX_BRAKE_PRESS;
    }
    fillRect(item->x,item->y-(item->size*10)+height,item->size,height,RA8875_RED);
  }
}

void redrawRotary(screenItemInfo * item, volatile dataItem * data, double currentValue){
  fillCircle(item->x, item->y, item->size, RA8875_RED);
  if(data->value == currentValue){
    return;
  }
  sevenSegmentDigit(item->x-(item->size/2.0),item->y-(item->size/2.0),item->size,RA8875_BLACK,data->value);
}

void redrawShiftLightsRPM(screenItemInfo * item, volatile dataItem * data, double currentValue) {
  uint8_t numFilled = (uint8_t)(9.0*(data->value/9000.0));
  uint8_t oldNumFilled = (uint8_t)(9.0*(currentValue/9000.0));
  if(numFilled == oldNumFilled){
    return;
  }
  if(numFilled > 9) numFilled = 9;
  uint64_t colorArray[9] = {0};

  int i;
  for(i=0;i<numFilled;i++){
    colorArray[i] = 0x00000000FFFF; // Blue
  }

  if (numFilled == 9) {
    tlc5955_set_main_blink(1, 0x0000FFFF0000);
  } else {
    tlc5955_set_main_blink(0, 0x0);
    tlc5955_write_main_colors(colorArray);
  }
}

void clearScreen(void){
  fillScreen(backgroundColor);
}

// Resets the current value of all screen items on a screen
// This means the
void resetScreenItems(void){
  int i = 0;
  screen *currScreen = allScreens[screenNumber];
  for(;i<currScreen->len;i++){
    if(currScreen->items[i].data){
      currScreen->items[i].currentValue = !(currScreen->items[i].data->value);
    }
  }
}

// Immediantly returns if
uint8_t initNightMode(uint8_t on){
  if(on){
    if(backgroundColor == RA8875_BLACK){
      warningColor = errorColor = backgroundColor;
      return 0;
    }
    backgroundColor = RA8875_BLACK;
    foregroundColor = RA8875_WHITE;
  }
  else{
    if(backgroundColor == RA8875_WHITE){
      warningColor = errorColor = backgroundColor;
      return 0;
    }
    backgroundColor = RA8875_WHITE;
    foregroundColor = RA8875_BLACK;
  }
  warningColor = errorColor = backgroundColor;
  return 1;
}

void nightMode(uint8_t on){
  if(initNightMode(on)){
    changeScreen(screenNumber);
  }
}

void addLap(double lapTime){
  lapTimeHead = (lapTimeHead+1)%20;
  lapTimeBuffer[lapTimeHead].value= lapTime;
  numLaps++;
}

double getMinLap(void){
  uint8_t counter = lapTimeHead;
  uint8_t iters = 0;
  double minLap = lapTimeBuffer[counter].value;
  while(lapTimeBuffer[counter].value != -1 && iters < 20){
    if(minLap > lapTimeBuffer[counter].value){
      minLap = lapTimeBuffer[counter].value;
    }
    counter = (counter+1)%20;
    iters++;
  }
  return minLap;
}

void endRace(void){
  endLogNum.value = 0;
  endNumLaps.value = numLaps;
  endFastLap.value = getMinLap();
  endTireTempFL.value = ttFL.value;
  endTireTempFR.value = ttFR.value;
  endTireTempRL.value = ttRL.value;
  endTireTempRR.value = ttRR.value;
  endAmbientTemp.value = ambientTemp.value;
  endFuelConsum.value = fuelConsum.value;
}

// Error Handling Stuff
void displayNoErrors(void){
  // Reset error area
  fillRect(0,200,WIDTH,HEIGHT-200,backgroundColor);
  drawChevron(30, 210, 30, 50, foregroundColor, backgroundColor);
  textMode();
  textSetCursor(100, 210);
  textTransparent(RA8875_BLACK);
  textEnlarge(1);
  textWrite("Illini Motorsports");
  graphicsMode();
}

uint16_t tempColor(uint8_t temp){
  return RA8875_RED;
}
