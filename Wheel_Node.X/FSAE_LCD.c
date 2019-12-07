/**
 * FSAE LCD Library
 *
 * Processor:   PIC32MZ2048EFM100
 * Compiler:    Microchip XC32
 * Author:      Jake Leonard
 * Created:     2015-2016
 */
// #include "RA8875_driver.h"
#include "Wheel.h"
#include "FSAE_LCD.h"
#include "FSAE_BUFFER.h"

uint16_t shiftRPM[7] = {12444,11959,12155,12100,11150,13000,13000}; //theres only gears 1-5

// Initialize all the data streams
// This fn must be run before CAN is initialized
void initDataItems(void){
  uint16_t i;

  // Set default initialization for PDM data items
  for(i=0;i<PDM_DATAITEM_SIZE;i++){
    initDataItem(&pdmDataItems[i],200,200,2,1);
    pdmDataItems[i].thresholdDir = 1;
  }
  for(i=FUEL_CUT_IDX;i<ECU_CUT_P_IDX;i++){
    setDataItemDigits(&pdmDataItems[i], 3, 0);
  }
  pdmDataItems[VBAT_RAIL_IDX].warnThreshold = 12.5;
  pdmDataItems[VBAT_RAIL_IDX].errThreshold = 12;
  pdmDataItems[VBAT_RAIL_IDX].thresholdDir = 0;
  pdmDataItems[VBAT_RAIL_IDX].decDigits = 2;

  // Customized PDM dataitem initialization
  pdmDataItems[STR_DRAW_IDX].wholeDigits = 3;

  // Set default initialization for GCM data items
  for(i=0;i<GCM_DATAITEM_SIZE;i++){
    initDataItem(&gcmDataItems[i],0,0,1,0);
  }

  // Customized GCM dataitem initialization
  gcmDataItems[GEAR_VOLT_IDX].decDigits = 2;
  gcmDataItems[GEAR_VOLT_IDX].wholeDigits = 2;
  gcmDataItems[FORCE_IDX].decDigits = 2;
  gcmDataItems[FORCE_IDX].wholeDigits = 2;

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
  setDataItemDigits(&motecDataItems[MANIFOLD_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[MANIFOLD_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[ENG_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[VOLT_ECU_IDX], 2, 2);
  setDataItemDigits(&motecDataItems[AMBIENT_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[FUEL_PRES_IDX], 1, 2);
  setDataItemDigits(&motecDataItems[FUEL_TEMP_IDX], 3, 0);
  setDataItemDigits(&motecDataItems[RUN_TIME_IDX], 4, 0);
  setDataItemDigits(&motecDataItems[FUEL_INJ_DUTY_IDX], 3, 1);
  setDataItemDigits(&motecDataItems[FUEL_TRIM_IDX], 3, 1);
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

  // Tire temps
  for(i=0;i<TIRETEMP_DATAITEM_SIZE;i++) {
    initDataItem(&tireTempDataItems[i],0,0,2,1);
  }

  // SPM
  for(i=0;i<SPM_DATAITEM_SIZE;i++) {
    initDataItem(&spmDataItems[i],0,0,2,1);
  }

  //Switches and Rotaries
  for(i=0;i<3;i++) {
    initDataItem(&wheelDataItems[i],0,0,2,1);
  }
  for(i=3;i<WHEEL_DATAITEM_SIZE;i++) {
    initDataItem(&wheelDataItems[i],0,0,1,0);
  }
  
  //IMU g readings
  for(i = 0; i < IMU_DATAITEM_SIZE; i++)
  {
      initDataItem(&imuDataItems[i], 0, 0, 1, 2);
  }
/*
  // Special array declerations
  fanSw[0] = &wheelDataItems[SW_FAN_IDX];
  fanSw[1] = &pdmDataItems[FAN_ENABLITY_IDX];
  fuelSw[0] = &wheelDataItems[SW_FUEL_IDX];
  fuelSw[1] = &pdmDataItems[FUEL_ENABLITY_IDX];
  wtrSw[0] = &wheelDataItems[SW_WTR_IDX];
  wtrSw[1] = &pdmDataItems[WTR_ENABLITY_IDX];
  shiftLights[0] = &motecDataItems[ENG_RPM_IDX];
  shiftLights[1] = &gcmDataItems[GEAR_IDX];
  gForce[0] = &imuDataItems[LATERAL_G_IDX];
  gForce[1] = &imuDataItems[LONGITUDINAL_G_IDX];
 */
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
  //This is probably the place to initialize the buffers!!
  buffer * nonprty_buffer  = malloc(sizeof(nonprty_buffer));
  init_buffer(nonprty_buffer, 0);
  // Initialize colors
  backgroundColor = RA8875_WHITE;
  foregroundColor = RA8875_BLACK;
  foregroundColor2 = RA8875_GREEN;

  // Race Screen Stuff
  allScreens[RACE_SCREEN] = &raceScreen;
  raceScreen.items = raceScreenItems;
  raceScreen.len = 11;
  //initScreenItem(&raceScreenItems[0], 120, 20, 15, redrawFanSw, *fanSw);
  //initScreenItem(&raceScreenItems[1], 240, 20, 15, redrawFUELPumpSw,*fuelSw);
  //initScreenItem(&raceScreenItems[2], 360, 20, 15, redrawWTRPumpSw, *wtrSw);
  initScreenItem(&raceScreenItems[3], 20, 90, 30, redrawDigit, &motecDataItems[OIL_TEMP_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&raceScreenItems[4], 360, 90, 30, redrawDigit, &motecDataItems[ENG_TEMP_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&raceScreenItems[5], 20, 210, 30, redrawDigit, &motecDataItems[OIL_PRES_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&raceScreenItems[6], 350, 210, 30, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&raceScreenItems[7], 200, 20, 80, redrawGearPos, &gcmDataItems[GEAR_IDX], MIN_REFRESH, nonprty_buffer);
  //initScreenItem(&raceScreenItems[8], 20, 30, 15, redrawShiftLightsRPM,*shiftLights);
  initScreenItem(&raceScreenItems[9], 20, 30, 15, redrawKILLCluster, &pdmDataItems[KILL_SWITCH_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&raceScreenItems[10], 180, 210, 30, redrawDigit, &motecDataItems[LAMBDA_IDX], MIN_REFRESH, nonprty_buffer);
//  initScreenItem(&raceScreenItems[10], 180, 210, 30, redrawDigit, &motecDataItems[THROTTLE_POS_IDX], MIN_REFRESH, nonprty_buffer); //debug throttle instead of lambda

  // PDM stuff
  allScreens[PDM_DRAW_SCREEN] = &pdmDrawScreen;
  pdmDrawScreen.items = pdmDrawItems;
  pdmDrawScreen.len = 33;
  initScreenItem(&pdmDrawItems[1], 10, 30, 15, redrawDigit, &pdmDataItems[IGN_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[2], 85, 30, 15, redrawDigit, &pdmDataItems[INJ_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[3], 160, 30, 15, redrawDigit, &pdmDataItems[FUEL_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[4], 235, 30, 15, redrawDigit, &pdmDataItems[ECU_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[5], 310, 30, 15, redrawDigit, &pdmDataItems[WTR_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[6], 385, 30, 15, redrawDigit, &pdmDataItems[FAN_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[7], 10, 80, 15, redrawDigit, &pdmDataItems[AUX_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[8], 85, 80, 15, redrawDigit, &pdmDataItems[PDLU_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[9], 160, 80, 15, redrawDigit, &pdmDataItems[PDLD_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[10], 235, 80, 15, redrawDigit, &pdmDataItems[ABS_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[11], 310, 80, 15, redrawDigit, &pdmDataItems[BVBAT_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[15], 385, 80, 15, redrawDigit, &pdmDataItems[STR_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[16], 10, 130, 15, redrawDigit, &pdmDataItems[IGN_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[17], 85, 130, 15, redrawDigit, &pdmDataItems[INJ_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[18], 160, 130, 15, redrawDigit, &pdmDataItems[FUEL_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[19], 235, 130, 15, redrawDigit, &pdmDataItems[ECU_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[20], 310, 130, 15, redrawDigit, &pdmDataItems[WTR_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[21], 385, 130, 15, redrawDigit, &pdmDataItems[FAN_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[22], 10, 180, 15, redrawDigit, &pdmDataItems[AUX_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[23], 85, 180, 15, redrawDigit, &pdmDataItems[PDLU_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[24], 160, 180, 15, redrawDigit, &pdmDataItems[PDLD_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[25], 235, 180, 15, redrawDigit, &pdmDataItems[ABS_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[26], 310, 180, 15, redrawDigit, &pdmDataItems[BVBAT_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[27], 160, 230, 15, redrawDigit, &pdmDataItems[FUEL_CUT_P_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[28], 235, 230, 15, redrawDigit, &pdmDataItems[FAN_CUT_P_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[29], 310, 230, 15, redrawDigit, &pdmDataItems[WTR_CUT_P_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[30], 385, 230, 15, redrawDigit, &pdmDataItems[ECU_CUT_P_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[31], 20, 30, 15, redrawShiftLightsRPM, (volatile dataItem*) shiftLights, MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmDrawItems[32], 20, 30, 15, redrawKILLCluster, &pdmDataItems[KILL_SWITCH_IDX], MIN_REFRESH, nonprty_buffer);

  allScreens[PDM_GRID_SCREEN] = &pdmGridScreen;
  pdmGridScreen.items = pdmGridItems;
  pdmGridScreen.len = 13;
  initScreenItem(&pdmGridItems[1], 10, 30, 15, redrawDigit, &pdmDataItems[IGN_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[2], 85, 30, 15, redrawDigit, &pdmDataItems[INJ_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[3], 160, 30, 15, redrawDigit, &pdmDataItems[FUEL_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[4], 235, 30, 15, redrawDigit, &pdmDataItems[ECU_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[5], 310, 30, 15, redrawDigit, &pdmDataItems[WTR_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[6], 385, 30, 15, redrawDigit, &pdmDataItems[FAN_DRAW_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[7], 10, 70, 15, redrawDigit, &pdmDataItems[IGN_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[8], 85, 70, 15, redrawDigit, &pdmDataItems[INJ_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[9], 160, 70, 15, redrawDigit, &pdmDataItems[FUEL_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[10], 235, 70, 15, redrawDigit, &pdmDataItems[ECU_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[11], 310, 70, 15, redrawDigit, &pdmDataItems[WTR_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmGridItems[12], 385, 70, 15, redrawDigit, &pdmDataItems[FAN_CUT_IDX], MIN_REFRESH, nonprty_buffer);

  allScreens[PDM_CUT_SCREEN] = &pdmCutScreen;
  pdmCutScreen.items = pdmCutItems;
  pdmCutScreen.len = 21;
  initScreenItem(&pdmCutItems[0], 10, 50, 15, redrawDigit, &pdmDataItems[PCB_TEMP_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[1], 85, 50, 15, redrawDigit, &pdmDataItems[IC_TEMP_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[2], 235, 200, 15, redrawDigit, &pdmDataItems[VBAT_RAIL_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[3], 330, 200, 15, redrawDigit, &pdmDataItems[V12_RAIL_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[4], 310, 50, 15, redrawDigit, 0x0, MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[5], 385, 50, 15, redrawDigit, 0x0, MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[6], 10, 100, 15, redrawDigit, 0x0, MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[7], 85, 100, 15, redrawDigit, &pdmDataItems[IGN_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[8], 160, 100, 15, redrawDigit, &pdmDataItems[INJ_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[9], 235, 100, 15, redrawDigit, &pdmDataItems[FUEL_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[10], 160, 50, 15, redrawDigit,  &pdmDataItems[ECU_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[12], 10, 150, 15, redrawDigit,  &pdmDataItems[FAN_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[13], 85, 150, 15, redrawDigit,  &pdmDataItems[AUX_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[14], 160, 150, 15, redrawDigit, &pdmDataItems[PDLU_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[15], 235, 150, 15, redrawDigit, &pdmDataItems[PDLD_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[16], 310, 150, 15, redrawDigit, &pdmDataItems[ABS_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[17], 385, 150, 15, redrawDigit, &pdmDataItems[BVBAT_CUT_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&pdmCutItems[18], 10, 200, 15, redrawDigit,  0x0, MIN_REFRESH, nonprty_buffer);

  // throttle position screen
  allScreens[THROTTLE_SCREEN] = &throttleScreen;
  throttleScreen.items = throttleItems;
  throttleScreen.len = 3;
  initScreenItem(&throttleItems[0], 50, 40, 50, redrawDigit, &motecDataItems[THROTTLE_POS_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&throttleItems[1], 350, 70, 100, redrawGearPos, &gcmDataItems[GEAR_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&throttleItems[2], 50, 150, 50, redrawDigit, &motecDataItems[ENG_RPM_IDX], MIN_REFRESH, nonprty_buffer);

  // IMU g readings screen
  // coordinates and sizes are still to be decided upon
  allScreens[IMU_SCREEN] = &imuScreen;
  imuScreen.items = imuItems;
  imuScreen.len = 3;
  initScreenItem(&imuItems[0], 360, 107, 20, redrawDigit, &imuDataItems[LATERAL_G_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&imuItems[1], 203, 235, 20, redrawDigit, &imuDataItems[LONGITUDINAL_G_IDX], MIN_REFRESH, nonprty_buffer);
  initScreenItem(&imuItems[2], 239, 97, 90, redrawGforceGraph, (volatile dataItem*) gForce, 10, nonprty_buffer);
  
  /*
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
  */
}

/* Modifying redraw item input to include ring buffer piointer
 * Not needed in screenItem but needed for instantiation
 */
void initScreenItem(screenItem* item, uint16_t x, uint16_t y, uint16_t size, double (*redrawItem)(screenItemInfo *, volatile dataItem *, double, buffer * buf_ptr), volatile dataItem* data, uint32_t interval, buffer * buf_ptr){
  item->info.x = x;
  item->info.y = y;
  item->info.size = size;
  item->currentValue = 1;
  item->data = data;
  item->refreshTime = millis;
  item->redrawItem = redrawItem;
  item->refreshInterval = interval;
  item->ring_buf = buf_ptr;
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
      textSetCursor(200,160);
      textWrite("LAMBDA");
      textEnlarge(0);
      graphicsMode();
      break;

    case PDM_DRAW_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,10);
      textWrite("IGN DRW");
      textSetCursor(85, 10);
      textWrite("INJ DRW");
      textSetCursor(160, 10);
      textWrite("FUEL DRW");
      textSetCursor(235, 10);
      textWrite("ECU DRW");
      textSetCursor(310, 10);
      textWrite("WTR DRW");
      textSetCursor(385, 10);
      textWrite("FAN DRW");
      textSetCursor(10, 60);
      textWrite("AUX DRW");
      textSetCursor(85, 60);
      textWrite("PDLU DRW");
      textSetCursor(160,60);
      textWrite("PDLD DRW");
      textSetCursor(235, 60);
      textWrite("ABS DRW");
      textSetCursor(310, 60);
      textWrite("BAT DRW");
      textSetCursor(385, 60);
      textWrite("STR DRW");
      textSetCursor(10,110);
      textWrite("IGN CUT");
      textSetCursor(85, 110);
      textWrite("INJ CUT");
      textSetCursor(160, 110);
      textWrite("FUEL CUT");
      textSetCursor(235, 110);
      textWrite("ECU CUT");
      textSetCursor(310, 110);
      textWrite("WTR CUT");
      textSetCursor(385, 110);
      textWrite("FAN CUT");
      textSetCursor(10, 160);
      textWrite("AUX CUT");
      textSetCursor(85, 160);
      textWrite("PDLU CUT");
      textSetCursor(160,160);
      textWrite("PDLD CUT");
      textSetCursor(235, 160);
      textWrite("ABS CUT");
      textSetCursor(310, 160);
      textWrite("BAT CUT");
      textSetCursor(160, 210);
      textWrite("FUEL PCT");
      textSetCursor(235, 210);
      textWrite("FAN PCT");
      textSetCursor(310, 210);
      textWrite("WTR PCT");
      textSetCursor(385, 210);
      textWrite("ECU PCT");
      textEnlarge(2);
      textSetCursor(10, 210);
      textWrite("PDM");
      textEnlarge(0);
      graphicsMode();
      break;

    case LAMBDA_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,10);
      textEnlarge(0);
      graphicsMode();
      break;

   case THROTTLE_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      graphicsMode();
      break;

/*
    case PDM_DRAW_SCREEN:
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(10,10);
      textWrite("IGN DRW");
      textSetCursor(85, 10);
      textWrite("INJ DRW");
      textSetCursor(160, 10);
      textWrite("FUEL DRW");
      textSetCursor(235, 10);
      textWrite("ECU DRW");
      textSetCursor(310, 10);
      textWrite("WTR DRW");
      textSetCursor(385, 10);
      textWrite("FAN DRW");
      graphicsMode();
      //Horizontal Lines
      drawLine(80,  30, 0, 200, foregroundColor);
      drawLine(155, 30, 0, 200, foregroundColor);
      drawLine(230, 30, 0, 200, foregroundColor);
      drawLine(305, 30, 0, 200, foregroundColor);
      drawLine(380, 30, 0, 200, foregroundColor);
      //Vertical Lines
      drawLine(10, 30, 0, 200, foregroundColor);
      drawLine(10, 30, 0, 200, foregroundColor);
      drawLine(10, 30, 0, 200, foregroundColor);
      drawLine(10, 30, 0, 200, foregroundColor);
      drawLine(10, 30, 0, 200, foregroundColor);
      break;
*/
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
      
    case IMU_SCREEN:
      //to be determined
      textMode();
      textTransparent(foregroundColor);
      textSetCursor(364, 85);
      textWrite("LATERAL");
      textSetCursor(207, 213);
      textWrite("LONGITUDINAL");
      textEnlarge(0);
      graphicsMode();
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
        raceScreenItems[6].data = &pdmDataItems[VBAT_RAIL_IDX];
        raceScreenItems[6].info.x += 30;
        break;

        // Lambda
      case 1:
        textWrite("LAMBDA");
        raceScreenItems[6].data = &motecDataItems[LAMBDA_IDX];
        break;

        // RPM
      case 2:
        textWrite("RPM");
        raceScreenItems[6].data = &motecDataItems[ENG_RPM_IDX];
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
      currItem->currentValue = currItem->redrawItem(&currItem->info, currItem->data, currItem->currentValue, currItem->ring_buf);
      currItem->refreshTime = millis;
    }
  }
}

// Redraw General Data, currentValue is MINREFRESH
/*  fillRect: 1 digit: 1
 *  TODO will need to find way to give this context awareness of a buffer, probably just add another input
 */
double redrawDigit(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  uint8_t warning = data->thresholdDir ? data->value >= data->warnThreshold: data->value <= data->warnThreshold;
  uint8_t error = data->thresholdDir ? data->value >= data->errThreshold: data->value <= data->errThreshold;
  uint8_t stale = millis - data->refreshTime > 1000;
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
  cmd_struct_digit cmd_data;
  // fillRect(item->x, item->y, fillWidth, (item->size)*1.75, fillColor);
  cmd_data.fillW       = fillWidth;         //size of rectangle
  cmd_data.colorFill   = fillColor;
  cmd_data.colorNumber = numColor;   //color values, TODO map these out sometime Diablo side
  cmd_data.numActual   = data->value;    //Value to be printed
  cmd_data.numWhole    = wholeNums;    //Whole digits before decimal point
  cmd_data.numDecimal  = decNums;  //digits after decimal point
  cmd_data.drawNumber  = 0;
  if(!stale && (!error || millis%500 > 200)) { // draw number if normal, or blink at 1hz
    // sevenSegmentDecimal(item->x,item->y,item->size,wholeNums+decNums,decNums,numColor,data->value);
    cmd_data.drawNumber  = 1;
  }
  /*Create cmd struct using function inputs, allocate memory for it?
    If we allocate memory on the heap for this, then diablo will have to free structs
    after reading from the buff. So, for now declare on stack and simply set equal when pushing. 
  */
  cmd_struct rDigit; /* This all should occur before returning*/ 
  rDigit.msg_type = dredrawDigit;
  // rDigit.cmd_digit = ; //TODO find useful information and put it here, in form of a struct
  rDigit.x = item->x;
  rDigit.y = item->y;
  rDigit.size = item->size;

  //Pass this struct into buffer 
  blocking_push(rDigit, buf_ptr);
  return data->value;
}

// For Single Digits with Error and Neutral Displays
/* fill rectangle: 1
 * digit: 1 (supports N, E as well) 
 * On Diablo side, keep the gear checking for N or Error
 */
double redrawGearPos(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  if(data->value == currentValue){
    return data->value;
  }
  cmd_struct_gear cmd_data;
  cmd_data.colorBG = backgroundColor;
  // fillRect(item->x, item->y, item->size, item->size * 1.75, backgroundColor); //erases a rect at this location
  cmd_data.gear = data->value; //might be casting issue here, but Diablo side will decide this drawing.
  if(data->value == 0){
    // sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_N);
  }
  else if(data->value == 7){
    // sevenSegment(item->x, item->y, item->size, foregroundColor, SEVEN_SEG_E);
  }
  else{
    // sevenSegmentDigit(item->x, item->y, item->size, foregroundColor, data->value);
  }
  cmd_struct rGear;
  rGear.msg_type = dredrawGearPos,
    /* union args */
  // rGear.cmd_gear = 
  rGear.x = item->x;
  rGear.y = item->y;
  rGear.size = item->size;

  blocking_push(rGear, buf_ptr);
  return data->value;
}

// For Fan Override Indicator
/* fill circle: 1
 */
double redrawFanSw(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  volatile dataItem** dataArray = (volatile dataItem**) data;
  // Override
  cmd_struct_fan cmd_data;
  if(dataArray[0]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_GREEN);
    cmd_data.colorC = RA8875_GREEN;
  }
  // Fan On, Switch Not toggled
  else if(dataArray[1]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_RED);
    cmd_data.colorC = RA8875_RED;
  }
  // Load off, switch off
  else{
    // fillCircle(item->x, item->y, item->size, RA8875_GREY);
    cmd_data.colorC = RA8875_GREY;
  }
  //Create cmd struct and populate it
  cmd_struct rFan;
  rFan.msg_type = dredrawFanSw;
  rFan.cmd_fan = cmd_data;
  rFan.x = item->x;
  rFan.y = item->y;
  rFan.size = item->size;

  blocking_push(rFan, buf_ptr);
  return data->value;
}

// For GCM Mode Indicator
/* fill Cirlce: 1
 */
double redrawGCMMode(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  // Auto-Upshifting Engaged
  cmd_struct_gcm cmd_data;
  if(data->value == 1){
    // fillCircle(item->x, item->y, item->size, RA8875_GREEN);
    cmd_data.colorC = RA8875_GREEN;
  }
  // Normal Mode
  else{
    // fillCircle(item->x, item->y, item->size, RA8875_GREY);
    cmd_data.colorC = RA8875_GREY;
  }
  //Create cmd struct and populate it
  cmd_struct rGCM;
  rGCM.msg_type = dredrawGCMMode;
  rGCM.x = item->x;
  rGCM.y = item->y;
  rGCM.size = item->size;

  blocking_push(rGCM, buf_ptr); 
  return data->value;
}

// For Water Pump Override Indicator
/* fill Circle: 1
 *
*/
double redrawWTRPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  volatile dataItem** dataArray = (volatile dataItem**) data;
  cmd_struct_wtr cmd_data;
  // Override
  if(dataArray[0]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_GREEN);
    cmd_data.colorC = RA8875_GREEN;
  }
  // WTR On, Switch Not toggled
  else if(dataArray[1]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_RED);
    cmd_data.colorC = RA8875_RED;
  }
  // Load off, switch off
  else{
    // fillCircle(item->x, item->y, item->size, RA8875_GREY);
    cmd_data.colorC = RA8875_GREY;
  }
  
  //Create cmd struct, populate it
  cmd_struct rWTR;
  rWTR.msg_type = dredrawWTRPumpSw;
  rWTR.cmd_wtr = cmd_data;
  rWTR.x = item->x;
  rWTR.y = item->y;
  rWTR.size = item->size;

  blocking_push(rWTR, buf_ptr); 
  return data->value;
}

// For Launch Control Override Indicator
/* fill Circle: 1
 */
double redrawFUELPumpSw(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  volatile dataItem** dataArray = (volatile dataItem**) data;
  // Override
  cmd_struct_fuel cmd_data;
  if(dataArray[0]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_GREEN);
    cmd_data.colorC = RA8875_GREEN;
  }
  // FUEL On, Switch Not toggled
  else if(dataArray[1]->value){
    // fillCircle(item->x, item->y, item->size, RA8875_RED);
    cmd_data.colorC = RA8875_RED;
  }
  // Load off, switch off
  else{
    // fillCircle(item->x, item->y, item->size, RA8875_GREY);
    cmd_data.colorC = RA8875_GREY;
  }

  //Create cmd struct using function inputs
  cmd_struct rFUEL;
  rFUEL.msg_type = dredrawFUELPumpSw;
  rFUEL.cmd_fuel = cmd_data;
  rFUEL.x = item->x;
  rFUEL.y = item->y;
  rFUEL.size = item->size;

  blocking_push(rFUEL, buf_ptr);
  return data->value;
}

// Uses the 4 Tire Temp sensors to draw a color gradient tire
/* fill circle square (?): 2
 * fill rect: 2
 */
double redrawTireTemp(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  volatile dataItem** dataArray = (volatile dataItem**) data;
  uint16_t fillColor = tempColor(data->value);
  uint16_t x = item->x;
  uint16_t y = item->y;
  uint16_t width = item->size / 4;
  uint16_t height = item->size * 2;
  cmd_struct_tire cmd_data;
  cmd_data.width = width;
  cmd_data.height = height;
  //look into fillCircleSquare,
  // fillCircleSquare(x,y,width*2,height,width,tempColor(dataArray[0]->value));
  cmd_data.color0 = dataArray[0]->value;
  // fillCircleSquare(x+(2*width),y,width*2,height,width,tempColor(dataArray[3]->value));
  cmd_data.color3 = dataArray[3]->value;
  // fillRect(x+width,y,width,height,tempColor(dataArray[1]->value));
  cmd_data.color1 = dataArray[1]->value;
  // fillRect(x+(2*width),y,width,height,tempColor(dataArray[2]->value));
  cmd_data.color2 = dataArray[2]->value;
  
  //Need dataArray[0 through 4]->value for color, maybe width and height
  //Create cmd struct using function inputs
  cmd_struct rTire;
  rTire.msg_type = dredrawTireTemp;
  rTire.cmd_tire = cmd_data;
  rTire.x = item->x;
  rTire.y = item->y;
  rTire.size = item->size;

  blocking_push(rTire, buf_ptr);
  return dataArray[0]->value;
}

// Draws a bar with a height proportional to the suspension position
/* fillRect: 2
 */
double redrawSPBar(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  if(data->value == currentValue){
    return data->value;
  }
  // fillRect(item->x, item->y, item->size, item->size * 5, backgroundColor);
  cmd_struct_sp cmd_data;
  cmd_data.colorBG = backgroundColor;
  uint16_t height = 0;
  cmd_data.height = height;     //initialize these two outside of conditional
  cmd_data.colorC = RA8875_RED;
  if(data->value > MIN_SUS_POS){
    height = ((item->size*5)/(MAX_SUS_POS-MIN_SUS_POS))*(data->value-MIN_SUS_POS);
    if(height > MAX_SUS_POS){
      height = MAX_SUS_POS;
    }
    cmd_data.height = height;
    // fillRect(item->x,item->y-(item->size*5)+height,item->size,height,RA8875_RED);
  }

  //Need to hide the original thing (the first fillRect), then color red, maybe height
  //Create cmd struct using function inputs
  cmd_struct rSP;
  rSP.msg_type = dredrawSPBar;
  rSP.cmd_sp = cmd_data;
  rSP.x = item->x;
  rSP.y = item->y;
  rSP.size = item->size;

  blocking_push(rSP, buf_ptr); 
  return data->value;
}

//Draw a bar with height proportional to the brake pressure
/* fillRect: 2
 */
double redrawBrakeBar(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  if(data->value == currentValue){
    return data->value;
  }
  cmd_struct_brake cmd_data;
  cmd_data.colorBG = backgroundColor;
  // fillRect(item->x, item->y, item->size, item->size * 10, backgroundColor);
  cmd_data.height = 0;
  cmd_data.colorC = RA8875_RED;
  if(data->value > MIN_BRAKE_PRESS){
    uint16_t height = ((item->size*10)/(MAX_BRAKE_PRESS-MIN_BRAKE_PRESS))*(data->value-MIN_BRAKE_PRESS);
    if(height > MAX_BRAKE_PRESS){
      height = MAX_BRAKE_PRESS;
    }
    cmd_data.height = height;
    // fillRect(item->x,item->y-(item->size*10)+height,item->size,height,RA8875_RED);
  }
  
  //Need color red, data->value - MIN_BRAKE_PRESS determines height
  //Create cmd struct using function inputs
  cmd_struct rBrake;
  rBrake.msg_type = dredrawBrakeBar;
  rBrake.cmd_brake = cmd_data;
  rBrake.x = item->x;
  rBrake.y = item->y;
  rBrake.size = item->size;

  blocking_push(rBrake, buf_ptr);
  return data->value;
}

/* fill circle: 1
 * digit: 1
 */
double redrawRotary(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr){
  // fillCircle(item->x, item->y, item->size, RA8875_RED);
  if(data->value == currentValue){
    return data->value;
  }
  cmd_struct_rotary cmd_data;
  cmd_data.val = data->value;
  cmd_data.colorC = RA8875_BLACK;
  // sevenSegmentDigit(item->x-(item->size/2.0),item->y-(item->size/2.0),item->size,RA8875_BLACK,data->value);
  
  //Need digit, black, data->value
  //Create cmd struct using function inputs
  cmd_struct rRotary;
  rRotary.msg_type = dredrawRotary;
  rRotary.cmd_rotary = cmd_data;
  rRotary.x = item->x;
  rRotary.y = item->y;
  rRotary.size = item->size;

  blocking_push(rRotary, buf_ptr); 
  return data->value;
}

uint8_t _getShiftLightsRevRange(uint16_t rpm, uint8_t gear) {
  uint16_t maxRPM = shiftRPM[gear];
  if (rpm > maxRPM) {
    return 10;
  } else if (rpm > maxRPM - REV_SUB_9) {
    return 9;
  } else if (rpm > maxRPM - REV_SUB_8) {
    return 8;
  } else if (rpm > maxRPM - REV_SUB_7) {
    return 7;
  } else if (rpm > maxRPM - REV_SUB_6) {
    return 6;
  } else if (rpm > maxRPM - REV_SUB_5) {
    return 5;
  } else if (rpm > maxRPM - REV_SUB_4) {
    return 4;
  } else if (rpm > maxRPM - REV_SUB_3) {
    return 3;
  } else if (rpm > maxRPM - REV_SUB_2) {
    return 2;
  } else if (rpm > maxRPM - REV_SUB_1) {
    return 1;
  } else {
    return 0;
  }
}

/* does not seem to draw to screen, uses blinks on leds
 */
double redrawShiftLightsRPM(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr) {
  volatile dataItem** dataArray = (volatile dataItem**) data;
  uint16_t rpm = (uint16_t) dataArray[0]->value;
  uint8_t gear = (uint8_t) dataArray[1]->value;
  uint8_t num_leds = _getShiftLightsRevRange(rpm, gear);

  if (num_leds == 10) {
    tlc5955_set_main_blink(1, GRN, NO_OVR);
  } else {
    uint8_t i;
    uint64_t colorArray[9] = {0};
    uint64_t drawColor = warnCount?RED:BLU;
    for (i = 0; i < num_leds; i++) {
      colorArray[i] = drawColor;
    }
    tlc5955_set_main_blink(0, 0x0, NO_OVR);
    tlc5955_write_main_colors(colorArray);
  }
  return data->value;
  
  cmd_struct_shift cmd_data;
  //Nothing drawn, probably dont need to send anything?
  //Create cmd struct using function inputs
  cmd_struct rShift;
  rShift.msg_type = dredrawShiftLightsRPM;
  rShift.cmd_shift = cmd_data;
  rShift.x = item->x;
  rShift.y = item->y;
  rShift.size = item->size;

  blocking_push(rShift, buf_ptr); 
}

/* again, tlc5955 is leds only
 */
double redrawKILLCluster(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr) {
  uint8_t kill = data->value ? 1 : 0;
  if (kill && !tlc5955_get_cluster_warn(CLUSTER_RIGHT) ||
      !kill && tlc5955_get_cluster_warn(CLUSTER_RIGHT)) {
    if (!tlc5955_get_startup()) {
      tlc5955_set_cluster_warn(CLUSTER_RIGHT, kill, RED, NO_OVR);
    }
  }
  return data->value;
  
  cmd_struct_kill cmd_data;
  //Again, nothing drawn so probably dont need to send anythign??
  //Create cmd struct using function inputs
  cmd_struct rKILL;
  rKILL.msg_type = dredrawKILLCluster;
  rKILL.cmd_kill = cmd_data;
  rKILL.x = item->x;
  rKILL.y = item->y;
  rKILL.size = item->size;

  blocking_push(rKILL, buf_ptr); 
}

//For drawing a visual representation of g-forces
/* fill circle: (erases 1) 1
 * draw circle: 4
 * fill rect: 2
 */
double redrawGforceGraph(screenItemInfo * item, volatile dataItem * data, double currentValue, buffer * buf_ptr)
{
  int i = 0;
  volatile dataItem** dataArray = (volatile dataItem**) data;
          
  //constants for various markers representing different g values on the graph
  uint16_t maxRadius, radii[4], maxG;
    
  //take a "snapshot" of the gForce CAN message
  double lateralSnap, longitSnap;
  lateralSnap = dataArray[0]->value;
  longitSnap = dataArray[1]->value;
  //initialize constants
  maxRadius = item->size;
  maxG = (-1 * LATERAL_G_OFFSET);
    
  //coordinates to draw the moving indicator
  uint16_t dot_x = item->x + (((lateralSnap) / maxG) * item->size);
  uint16_t dot_y = item->y + (((longitSnap ) / maxG) * item->size);
  uint16_t oldx;
  uint16_t oldy;

  cmd_struct_gforce cmd_data;
    
  //determine where to erase the previous dot by mapping currentValue to coordinates
  double lateral, longit;
  lateral = fmod(currentValue, 10000);
  if(lateral > 999)
  {
    lateral -= 1000;
    lateral = lateral * -1;
  }
  lateral = lateral / 100;

  longit = (int)currentValue / 10000;
  if(longit > 999)
  {
    longit-=1000;
    longit = longit * -1;
  }
  longit = longit / 100;

  oldx = item->x + (((lateral) / maxG) * item->size);
  oldy = item->y + (((longit) / maxG) * item->size);
  
  //radius of the moving indicator
  uint16_t dotRad = item->size / 15;

  //erase the old indicator
  // fillCircle(oldx, oldy, item->size/10, backgroundColor);
  //draw the moving dot
  // fillCircle(dot_x, dot_y, dotRad, foregroundColor2);
    
  //derive radii and draw axis lines
  for(i = 2; i >= 0; i--)
  {
      radii[i] = ((i + 1) * maxRadius) / maxG ;
      cmd_data.radii[i] = radii[i];
      // drawCircle(item->x, item->y, radii[i], foregroundColor);
  }
  cmd_data.maxG = maxG;
  cmd_data.maxRadius = maxRadius;
  cmd_data.colorBG = backgroundColor;
  cmd_data.colorFG = foregroundColor;
  cmd_data.colorFG2 = foregroundColor2;
  // drawCircle(item->x, item->y, maxRadius, errorColor);
  
  // fillRect(item->x - item->size, item->y - item->size / 160, 2 * item->size, item->size / 80, foregroundColor);
  // fillRect(item->x - item->size / 160, item->y - item->size, item->size / 80, 2 * item->size, foregroundColor);
        

    
    
    //map the horizontal and lateral G snapshots to a double
  lateralSnap = lateralSnap * 100;
  lateralSnap = (int) lateralSnap;
  if(lateralSnap < 0)
  {
    lateralSnap = lateralSnap * -1;
    lateralSnap += 1000;
  }
  longitSnap = longitSnap * 100;
  longitSnap = (int) longitSnap;
  longitSnap = longitSnap * 10000;
  if(longitSnap < 0)
  {
    longitSnap = longitSnap * -1;
    longitSnap += 10000000;
  }
  cmd_data.latsnap = lateralSnap;
  cmd_data.longsnap = longitSnap;
  //setting the "currentValue" to allow the function to erase a localized area in the future
    
  /*Oh god oH FUCK
    so this code will snapshot the g's from CAN, i guess pass those? Make diablo graph that shit
    It should be much easier to create a standalone g-force graph with solely new g-information
    on Diablo side, using the Dialblo software suite. 
  */
  //Create cmd struct using function inputs
  cmd_struct rGforce;
  rGforce.msg_type = dredrawGforceGraph;
  rGforce.cmd_gforce = cmd_data;
  rGforce.x = item->x;
  rGforce.y = item->y;
  rGforce.size = item->size;

  blocking_push(rGforce, buf_ptr);
  return longitSnap + lateralSnap;
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
      //warningColor = errorColor = backgroundColor;
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
     // warningColor = errorColor = backgroundColor;
      warningColor = RA8875_YELLOW;
      errorColor = RA8875_RED;
      return 0;
    }
    backgroundColor = RA8875_WHITE;
    foregroundColor = RA8875_BLACK;
    foregroundColor2 = RA8875_GREEN;
  }
  //warningColor = errorColor = backgroundColor;
  warningColor = RA8875_YELLOW;
  errorColor = RA8875_RED;
  return 1;
}

void nightMode(uint8_t on){
  if(initNightMode(on)){
    changeScreen(screenNumber);
  }
}

uint16_t tempColor(uint8_t temp){
  return RA8875_RED;
}

uint8_t checkDataChange(volatile dataItem *data, double currentValue) {
  return 1;//(int) (pow(10,data->decDigits)*(data->value-currentValue));
}