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
  uint16_t i;

  // Set default initialization for PDM data items
  for(i=0;i<PDM_DATAITEM_SIZE;i++){
    initDataItem(&pdmDataItems[i],0,0,2,1);
    pdmDataItems[i].thresholdDir = 1;
  }

  // Set default initialization for GCM data items
  for(i=0;i<GCM_DATAITEM_SIZE;i++){
    initDataItem(&gcmDataItems[i],0,0,1,0);
  }

  // Set default initializations for Motec data items
  for(i=0;i<MOTEC_DATAITEM_SIZE;i++) {
    initDataItem(&motecDataItems[i],0,0,2,1);
  }

  // Customized Motec dataitem initialization
  setDataItemDigits(&motecDataItems[ENG_RPM_IDX], 5, 0);
  setDataItemDigits(&motecDataItems[THROTTLE_POS_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[OIL_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[OIL_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[LAMBDA_IDX], 1, 3);
  setDataItemDigits(&motecDataItems[ENG_TEMP_IDX], 3, 0);
  motecDataItems[LAMBDA_IDX].warnThreshold = 1.1;
  motecDataItems[LAMBDA_IDX].errThreshold = 1.3;
  motecDataItems[LAMBDA_IDX].thresholdDir = 1;
  motecDataItems[OIL_PRES_IDX].warnThreshold = 1.2;
  motecDataItems[OIL_PRES_IDX].errThreshold = 1.2;
  motecDataItems[OIL_TEMP_IDX].thresholdDir = 1;
  motecDataItems[OIL_TEMP_IDX].warnThreshold = 120;
  motecDataItems[OIL_TEMP_IDX].errThreshold = 140;
  motecDataItems[ENG_TEMP_IDX].thresholdDir = 1;
  motecDataItems[ENG_TEMP_IDX].warnThreshold = 100;
  motecDataItems[ENG_TEMP_IDX].errThreshold = 110;
  motecDataItems[ENG_RPM_IDX].warnThreshold = 12000;
  motecDataItems[ENG_RPM_IDX].errThreshold = 13000;
  motecDataItems[ENG_RPM_IDX].thresholdDir = 1;
  motecDataItems[LAMBDA_IDX].warnThreshold = 1.1;
  motecDataItems[LAMBDA_IDX].errThreshold = 1.2;
  motecDataItems[LAMBDA_IDX].thresholdDir = 1;

  //Switches and Rotaries
  for(i=0;i<3;i++) {
    initDataItem(&wheelDataItems[i],0,0,2,1);
  }
  for(i=3;i<WHEEL_DATAITEM_SIZE;i++) {
    initDataItem(&wheelDataItems[i],0,0,1,0);
  }
}

void initDataItem(volatile dataItem* data, double warn, double err, uint8_t whole, uint8_t dec){
  data->value = 0;
  data->warnThreshold = warn;
  data->errThreshold = err;
  data->thresholdDir = 0;
  data->wholeDigits = whole;
  data->decDigits = dec;
}

void setDataItemDigits(volatile dataItem* data, uint8_t whole, uint8_t dec) {
  data->wholeDigits = whole;
  data->decDigits = dec;
}

// Initializes all the screen and screenitem variables in all
// the screens that might be displayed
void initAllScreens(void){
  // Initialize colors
  backgroundColor = RA8875_WHITE;
  foregroundColor = RA8875_BLACK;
  foregroundColor2 = RA8875_GREEN;

  // Race Screen Stuff
  allScreens[RACE_SCREEN] = &raceScreen;
  raceScreen.items = raceScreenItems;
  raceScreen.len = 7;
  initScreenItem(&raceScreenItems[0], 20, 90, 30, redrawDigit, &motecDataItems[OIL_TEMP_IDX], MIN_REFRESH);
  initScreenItem(&raceScreenItems[1], 360, 90, 30, redrawDigit, &motecDataItems[ENG_TEMP_IDX], MIN_REFRESH);
  initScreenItem(&raceScreenItems[2], 20, 210, 30, redrawDigit, &motecDataItems[OIL_PRES_IDX], MIN_REFRESH);
  initScreenItem(&raceScreenItems[3], 350, 210, 30, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX], MIN_REFRESH);
  initScreenItem(&raceScreenItems[4], 200, 20, 80, redrawGearPos, &gcmDataItems[GEAR_IDX], MIN_REFRESH);
  // TODO:You'll have to move this somewhere better, it's probably overlapping with things rn
  initScreenItem(&raceScreenItems[5], 200, 220, 15, redrawGCMMode, &gcmDataItems[MODE_IDX], MIN_REFRESH);
  initScreenItem(&raceScreenItems[6], 20, 30, 15, redrawKILLCluster, &pdmDataItems[KILL_SWITCH_IDX], MIN_REFRESH);

  // Test Screen Stuff
  allScreens[TEST_SCREEN] = &testScreen;
  testScreen.items = testScreenItems;
  testScreen.len = 3;
  // TODO: This orientation is gonna look super dumb, fix it
  initScreenItem(&testScreenItems[0], 80, 160, 50, redrawDigit, &motecDataItems[ENG_RPM_IDX], MIN_REFRESH);
  initScreenItem(&testScreenItems[1], 360, 60, 30, redrawDigit, &motecDataItems[THROTTLE_POS_IDX], MIN_REFRESH);
  initScreenItem(&testScreenItems[2], 5, 60, 30, redrawDigit, &motecDataItems[LAMBDA_IDX], MIN_REFRESH);
}

void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, double (*redrawItem)(screenItemInfo *, volatile dataItem *, double), volatile dataItem* data, uint32_t interval){
  item->info.x = x;
  item->info.y = y;
  item->info.size = size;
  item->currentValue = 1;
  item->data = data;
  item->refreshTime = millis;
  item->redrawItem = redrawItem;
  item->refreshInterval = interval;
}

// Helper function for drawing labels and such on a new screen
void initScreen(uint8_t num){
  switch(num){
    case RACE_SCREEN:
      textMode();
      textEnlarge(1);
      textTransparent(foregroundColor);
      textSetCursor(7, 40);
      textWrite("OIL TEMP");
      textSetCursor(0, 160);
      textWrite("OIL PRESS");
      textSetCursor(360, 40);
      textWrite("WTR TMP");
      textSetCursor(370,160);
      textWrite("BAT V");
      textEnlarge(0);
      graphicsMode();
      break;
    case TEST_SCREEN:
      textMode();
      textEnlarge(1);
      textTransparent(foregroundColor);
      // TODO: These also almost definitely need to be tweaked
      textSetCursor(220, 120);
      textWrite("RPM");
      textSetCursor(420, 20);
      textWrite("TP");
      textSetCursor(7, 20);
      textWrite("LAMBDA");
      textEnlarge(0);
      graphicsMode();
      break;
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
  screen* currScreen = allScreens[screenNumber];
  int i;
  for(i = 0;i<currScreen->len;i++){
    screenItem * currItem = &currScreen->items[i];
    volatile dataItem * data = currItem->data;
    if(data && millis - currItem->refreshTime >= currItem->refreshInterval){
      uint8_t warning = data->thresholdDir?data->value>=data->warnThreshold:data->value<=data->warnThreshold;
      if(warning && !data->warningState) {
        data->warningState = 1;
        warnCount++;
      }
      if(!warning && data->warningState) {
        data->warningState = 0;
        warnCount--;
      }
      currItem->currentValue = currItem->redrawItem(&currItem->info, currItem->data, currItem->currentValue);
      currItem->refreshTime = millis;
    }
  }
}

// Redraw General Data
double redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue){
  uint8_t warning = data->thresholdDir ? data->value >= data->warnThreshold: data->value <= data->warnThreshold;
  uint8_t error = data->thresholdDir ? data->value >= data->errThreshold: data->value <= data->errThreshold;
  uint8_t stale = millis - data->refreshTime > CAN_TIMEOUT;
  if(stale) {
    data->value = 0;
  }
  if(!error && !stale && !checkDataChange(data, currentValue)) {
    return data->value;
  }
  // Set Backround Color
  uint16_t fillColor;
  uint16_t numColor;
  if(warning){
    if(error){
      fillColor = errorColor;
      numColor = RA8875_WHITE;
    }else{
      fillColor = warningColor;
      numColor = RA8875_BLUE;
    }
  }
  else if(stale) {
    fillColor = errorColor;
  }
  else{
    fillColor = backgroundColor;
    numColor = foregroundColor;
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
  if(!stale && (!error || millis%500 > 200)) { // draw number if normal, or blink at 1hz
    sevenSegmentDecimal(item->x,item->y,item->size,wholeNums+decNums,decNums,numColor,data->value);
  }
  return data->value;
}

// For Single Digits with Error and Neutral Displays
double redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue){
  uint8_t stale = millis - data->refreshTime > CAN_TIMEOUT;

  // Set value to 8 (impossible value) if stale
  // Will force refresh on error condition change
  if(stale) {
    data->value = 8;
  }

  if(data->value == currentValue){
    return data->value;
  }
  uint16_t fillColor = stale ? errorColor : backgroundColor;
  fillRect(item->x, item->y, item->size, item->size * 1.75, fillColor);
  if(!stale) {
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
  return data->value;
}

uint8_t getShiftLightsRevRange(uint16_t rpm, uint8_t gear) {
  uint16_t maxRPM = shiftRPM[gear];
  int i;
  for(i = 9; i < 0; i--) {
    if(rpm > maxRPM - shiftLightSub[i]) {
      return i + 1;
    }
  }
}

double redrawKILLCluster(screenItemInfo * item, volatile dataItem * data, double currentValue) {
  uint8_t kill = data->value ? 1 : 0;
  if (kill && !tlc5955_get_cluster_warn(CLUSTER_RIGHT) ||
      !kill && tlc5955_get_cluster_warn(CLUSTER_RIGHT)) {
    if (!tlc5955_get_startup()) {
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, kill, RED, NO_OVR);
    }
  }
  return data->value;
}

// For GCM Mode Indicator
double redrawGCMMode(screenItemInfo * item, volatile dataItem * data, double currentValue){
  // Auto-Upshifting Engaged
  if(data->value == 1){
    fillCircle(item->x, item->y, item->size, RA8875_RED);
  }
  // Normal Mode
  else{
    fillCircle(item->x, item->y, item->size, RA8875_GREY);
  }
}

void clearScreen(void){
  fillScreen(backgroundColor);
}

// Resets the current value of all screen items on a screen
void resetScreenItems(void){
  int i = 0;
  screen *currScreen = allScreens[screenNumber];
  for(;i<currScreen->len;i++){
    if(currScreen->items[i].data){
      currScreen->items[i].currentValue = !(currScreen->items[i].data->value);
    }
  }
}

uint8_t initNightMode(uint8_t on){
  if(on){
    if(backgroundColor == RA8875_BLACK){
      warningColor = errorColor = RA8875_RED;
      return 0;
    }
    backgroundColor = RA8875_BLACK;
    foregroundColor = RA8875_WHITE;
    foregroundColor2 = RA8875_CYAN;
    warningColor = RA8875_YELLOW;
    errorColor = RA8875_RED;
  }
  else{
    if(backgroundColor == RA8875_WHITE){
      warningColor = RA8875_YELLOW;
      errorColor = RA8875_RED;
      return 0;
    }
    backgroundColor = RA8875_WHITE;
    foregroundColor = RA8875_BLACK;
    foregroundColor2 = RA8875_GREEN;
  }
  warningColor = RA8875_YELLOW;
  errorColor = RA8875_RED;
  return 1;
}

void nightMode(uint8_t on){
  if(initNightMode(on)){
    changeScreen(screenNumber);
  }
}

uint8_t checkDataChange(volatile dataItem *data, double currentValue) {
  return 1;//(int) (pow(10,data->decDigits)*(data->value-currentValue));
}
