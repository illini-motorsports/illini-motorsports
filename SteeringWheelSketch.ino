/*
	SteeringWheelSketch.ino
	v1.5
  
	Written by: Joe Sheehan
	University of Illinois Formula SAE Team 2011

	Modified by: George Schwieters
	University of Illinois Formula SAE Team 2013
  
	Target: Atmega328p
  
	This sketch was written for the 2013 Rev. of the LEDControlBoard.
	This sketch was written to work independently of the DataAcqBoard.
  
	Description:
	This sketch receives data from the Motec over RS232, and shifts out
	control data to the display drivers on the LEDBoard. 

	TODO:
	-None

	Change Log:
	v1.5
	- added more constants for more intuitive code
	- added constant parameter optimized pin operation macros (digitalWriteFast.h)
	- modifed LED bar scaling
	- added LED bar warning for excessive oil temp or water temp
	- removed oil pressure from LED bar graph and added oil tmep
	- added AFR display
	- added battery display
	- modififed for direct communication with Motec (removed I2C)
	- made code more efficent (getting rid of unnecesary int declarations etc.)
	v1.4
	- Added Oil Temperature readings and display
	v1.3
	- Fixed bug in requestEvent() only sending 2 bytes instead of 4.
	v1.2
	- Modified RPM thresholds in updateShiftLights()
	- Adjusted LED bar scaling to correct range (100)
	v1.1 
	- Added I2C request handler and corresponding message constants.
	v1.0 
	- Initial upload
*/

#include <LedControl.h>
#include <digitalWriteFast.h>

//////////////////////////////
//	Functionality control	//
//////////////////////////////

//#define CRC
//#define OTHER_TEXT
//#define GEAR_DETECT
//#define TROLL_STARTUP
//#define STARTUP
//#define BRIGHTNESS

// 7 Segment Letters and Numbers
const byte SEV_SEG_NULL = 0x00;
const byte SEV_SEG_A = 0x77;
const byte SEV_SEG_B = 0x1F;	// 8
const byte SEV_SEG_C = 0x4E;
const byte SEV_SEG_d = 0x3D;
const byte SEV_SEG_E = 0x4F;
const byte SEV_SEG_F = 0x47;
//const byte SEV_SEG_g = 0x;	// 9
const byte SEV_SEG_H = 0x37;
const byte SEV_SEG_I = 0x30;	// 1
const byte SEV_SEG_i = 0x10;
//const byte SEV_SEG_J = 0x;
const byte SEV_SEG_k = 0x57;
const byte SEV_SEG_L = 0x0E;
const byte SEV_SEG_M0 = 0x66;
const byte SEV_SEG_M1 = 0x72;
const byte SEV_SEG_m0 = 0x15;	// n
const byte SEV_SEG_m1 = 0x11;
const byte SEV_SEG_n = 0x15;
const byte SEV_SEG_N = 0x76;
const byte SEV_SEG_O = 0x7E;	// 0
const byte SEV_SEG_P = 0x67;
//const byte SEV_SEG_q = 0x;
const byte SEV_SEG_R = 0x75;
const byte SEV_SEG_r = 0x05;
const byte SEV_SEG_S = 0x5B;	// 5
const byte SEV_SEG_T = 0x70;	// 7
const byte SEV_SEG_t = 0x0F;
const byte SEV_SEG_u = 0x3E;
//const byte SEV_SEG_v = 0x;
//const byte SEV_SEG_w = 0x;
//const byte SEV_SEG_x = 0x;
//const byte SEV_SEG_y = 0x;
//const byte SEV_SEG_z = 0x;

const byte SEV_SEG_DEG = 0x63;

const byte SEV_SEG_0 = 0x7E;
//const byte SEV_SEG_1 = 0x;
const byte SEV_SEG_2 = 0x6D;
//const byte SEV_SEG_3 = 0x;
//const byte SEV_SEG_4 = 0x;
//const byte SEV_SEG_5 = 0x;
//const byte SEV_SEG_6 = 0x;
//const byte SEV_SEG_7 = 0x;
//const byte SEV_SEG_8 = 0x;
//const byte SEV_SEG_9 = 0x;

// Alphanumeric Gears
const byte ALPHA_NULL = 0x00;
const byte ALPHA_N = 0xB6;
const byte ALPHA_1 = 0x06;
const byte ALPHA_2 = 0x5B;
const byte ALPHA_3 = 0x4F;
const byte ALPHA_4 = 0x66;
const byte ALPHA_5 = 0x6D;
const byte ALPHA_6 = 0x7D;

// Bar Graph Filling
const byte BAR_NULL = 0x00;
const byte BAR_1 = 0x01;
const byte BAR_2 = 0x03;
const byte BAR_3 = 0x07;
const byte BAR_4 = 0x0F;
const byte BAR_5 = 0x1F;
const byte BAR_6 = 0x3F;
const byte BAR_7 = 0x7F;
const byte BAR_8 = 0xFF;
const byte EXTRA = 0xC0;

// Shift Lights Filling
const byte SHIFT_NULL = 0x00;
const byte SHIFT_1 = 0x80;
const byte SHIFT_2 = 0xC0;
const byte SHIFT_3 = 0xE0;
const byte SHIFT_4 = 0xF0;
const byte SHIFT_5 = 0xF8;
const byte SHIFT_6 = 0xFC;
const byte SHIFT_7 = 0xFE;
const byte SHIFT_8 = 0xFF;

// Shift Lights Warning Filling
const byte RED_WARN = 0x38;		// 0011 1000
const byte BLUE_WARN = 0x07;	// 0000 0111

// RPM Ranges and LED control
//
// the defines determine if the specifc LED pattern is to be utilized and
// then the constant sets the range for it
#define REV_RANGE_1
const unsigned int REV_RANGE1 = 8000;	// original
#define REV_RANGE_2
//const unsigned int REV_RANGE2 = 8500;	// original
const unsigned int REV_RANGE2 = 9000;
#define REV_RANGE_3
const unsigned int REV_RANGE3 = 10000;	// original
#define REV_RANGE_4
const unsigned int REV_RANGE4 = 10500;
#define REV_RANGE_5
const unsigned int REV_RANGE5 = 11000;	// original
#define REV_RANGE_6
const unsigned int REV_RANGE6 = 11166;
#define REV_RANGE_7
const unsigned int REV_RANGE7 = 11333;
//
// RPM_RANGE8 is always used along with REV_LIMIT
const unsigned int REV_RANGE8 = 11500;	// original
const unsigned int REV_LIMIT = 12500;	// original

// LEDControl
const boolean SHIFT_BAR_CTRL = 0;
const boolean DIGIT_CTRL = 1;

// Pin Declarations
const byte LEFT_DISP_PIN = 10;
const byte RIGHT_DISP_PIN = 2;
const byte BRIGHT_DOWN_PIN = 9;
const byte BRIGHT_UP_PIN = 14;
const byte ALPHA_DATA_PIN = 17;
const byte ALPHA_CLK_PIN = 16;
const byte ALPHA_LAT_PIN = 15;

//	ECU
//
// Channel Indicies and Constants
const byte NUM_CHANNELS = 8;
const byte HEADER_LENGTH = 4;
const byte CRC_LENGTH = 4;
const byte DATA_LENGTH = NUM_CHANNELS * 2;
const byte ECU_MSG_LENGTH = DATA_LENGTH + /*CRC_LENGTH*/ + HEADER_LENGTH; // 4 header and 4 CRC bytes
const unsigned int BAUD = 19200;
const byte RPM = 0;
const byte WATER = 1;
const byte OIL_P = 2;
const byte OIL_T = 3;
const byte GEAR = 4;
const byte G_SPEED = 5;
const byte AF = 6;
const byte BATT = 7;

//	Button Constants
//
// Data Selection
const byte LEFT = 0;
const byte RIGHT = 1;
const byte LEFT_BAR = 2;
const byte RIGHT_BAR = 3;
const byte EXTRA_BAR = 4;
#ifdef BRIGHTNESS
// Brightness Adj.
const byte DOWN = 0;
const byte UP = 1;
#endif

//	Display State Constants
//
// Data (these must match the array of display row vectors)
const byte RPM_DISPLAY = 0;
const byte SPEED_DISPLAY = 1;
const byte OIL_P_DISPLAY = 2;
const byte OIL_T_DISPLAY = 3;
const byte WATER_DISPLAY = 4;
const byte AIR_FUEL_DISPLAY= 5;
const byte BATTERY_DISPLAY = 6;
const byte TMR_DISPLAY = 7;
// Text
const byte TRAC_DISPLAY = 0;
const byte ON_DISPLAY = 1;
const byte OFF_DISPLAY = 2;
const byte FUEL_DISPLAY = 3;
const byte RICH_DISPLAY = 4;
const byte LEAN_DISPLAY = 5;

//	Other Constants
//
#ifdef GEAR_DETECT
const float RPM_PER_MPH[] = {339.85, 247.39, 197.56, 170.45, 152.66, 140.81};
#endif
const unsigned int TEXT_HOLD_DELAY = 1000;	// data selection text display hold time (ms)
const byte BAR_BLINK_DELAY = 100;			// data selection blinker (ms)
const byte RPM_BLINK_DELAY = 70;			// shift lights (ms)
const byte WARNING_BLINK_DELAY = 70;		// LED bar graphs (ms)
const float STOICH = 14.70;
//const float KPA_TO_PSI = 0.145;
//const float KMPH_TO_MPH = 0.621;
const unsigned int MAX_ET = 1150;		// in celcious (greater by one order of magnitude to match data scaling)
const unsigned int MAX_OT = 1900;		// in celcious (greater by one order of magnitude to match data scaling)

//These are arrays that define the led states for the 
//  Alphanumeric, shift lights, bar graphs, and seven segs
const byte gearArray[] = {
	ALPHA_N, ALPHA_1, ALPHA_2, ALPHA_3, ALPHA_4, ALPHA_5, ALPHA_6, ALPHA_NULL};
const byte barGraphArray[] = {
	BAR_NULL, BAR_1, BAR_2, BAR_3, BAR_4, BAR_5, BAR_6, BAR_7, BAR_8};
const byte displayTexts[8][4] = {						//The order of these must match the constants for each sensor display index
	{SEV_SEG_R, SEV_SEG_P, SEV_SEG_M0, SEV_SEG_M1},		//RPM , Tachometer (RPM)
	{SEV_SEG_M0, SEV_SEG_M1, SEV_SEG_P, SEV_SEG_H},		//M PH, Drive Speed (MPH)
	{SEV_SEG_O, SEV_SEG_P, SEV_SEG_S, SEV_SEG_I},		//OPS1, Oil Pressure (PSI)
	{SEV_SEG_O, SEV_SEG_t, SEV_SEG_DEG, SEV_SEG_C},		//Ot*C, Oil Temperature (Celcious)
	{SEV_SEG_E, SEV_SEG_t, SEV_SEG_DEG, SEV_SEG_C},		//Et*C, Engine Temp (Celcious)
	{SEV_SEG_A, SEV_SEG_F, SEV_SEG_R, SEV_SEG_NULL},	//AFR , Air to Fuel Ratio (14.7 Stoich)
	{SEV_SEG_B, SEV_SEG_A, SEV_SEG_t, SEV_SEG_t},		//BAtt, Battery (Volts)
	{SEV_SEG_t, SEV_SEG_m0, SEV_SEG_m1, SEV_SEG_r}		//tm r, millis() timer display (minutes and seconds)
};
#ifdef OTHER_TEXT
const byte otherDisplayTexts[6][4] = {
	{SEV_SEG_T, SEV_SEG_C, SEV_SEG_S, SEV_SEG_NULL},	//TC5, Traction Control System
	{SEV_SEG_O, SEV_SEG_n, SEV_SEG_NULL, SEV_SEG_NULL},	//On
	{SEV_SEG_O, SEV_SEG_F, SEV_SEG_F, SEV_SEG_NULL},	//OFF
	{SEV_SEG_F, SEV_SEG_u, SEV_SEG_E, SEV_SEG_L},		//FUEL
	{SEV_SEG_r, SEV_SEG_I, SEV_SEG_C, SEV_SEG_H},		//rICH, rich fuel map
	{SEV_SEG_L, SEV_SEG_E, SEV_SEG_A, SEV_SEG_n}		//LEAn, lean fuel map
};
#endif

//////////////////////////////////////
//	Global Variable Declarations	//
//////////////////////////////////////

// data buffers
unsigned int channelData[NUM_CHANNELS] = {0};
#ifndef BRIGHTNESS
unsigned int channelAverage[NUM_CHANNELS] = {0};
unsigned int channelExtreme[NUM_CHANNELS] = {0};
#endif
//byte waterC = 0;
//byte drivenSpeed = 0;		// average of the driven wheel speeds
#ifdef GEAR_DETECT
static byte gear = 7;
int gear_hist[25] = {0};
#endif

// flags
#ifdef OTHER_TEXT
boolean noBarBlink = false;
#endif
boolean leftBarBlink = true;
boolean rightBarBlink = true;
boolean rightBarWarn = false;
boolean leftBarWarn = false;
boolean revLimit = false;
boolean displayHold[2] = {false, false};

// states
byte brightness = 15;
#ifndef BRIGHTNESS
boolean averageOn = 0;		// flags for max/min and average displays for the data values
boolean extremeOn = 0;		//
boolean resetOn = 0;		//
#endif
byte shiftLightsState = SHIFT_NULL;			// for blinking the shift lights
byte leftBarGraphState = barGraphArray[0];	// for LED bar graph blinking
byte rightBarGraphState = barGraphArray[0];	// for LED bar graph blinking
byte shiftLightBlueState = SHIFT_NULL;		// for LED shift light blinking
byte shiftLightRedState = SHIFT_NULL;		// for LED shift light blinking
byte displayStates[2] = {OIL_T_DISPLAY, WATER_DISPLAY}; // [0] is LEFT and [1] is RIGHT.
byte leftDisplayButtonState = HIGH, rightDisplayButtonState = HIGH, brightnessDownButtonState = HIGH,
		brightnessUpButtonState = HIGH;

// timing
unsigned long rpmBlinkTimer, textLeftHoldTimer, textRightHoldTimer, leftBlinkTimer, rightBlinkTimer,
				otherTextHoldTimer, rightBarBlinkTimer, leftBarBlinkTimer;

// This unions is used for easy conversion of data received on RS232
union {
	unsigned int var;
	byte buf[2];
} FromInt;

/* Macro: updateETBar()
 * Description: Converts the water temerapture variable into an 8 step scale and
 *              displays it on the right led bar display.
 */
#define updateETBar() displays.setRow(SHIFT_BAR_CTRL, RIGHT_BAR, barGraphArray[map(channelData[WATER], 0, MAX_ET, 0, 8)])

/* Macro: updateOTBar()
 * Description: Converts the oil temperature variable into an 8 step scale and
 *              displays it on the left led bar display.
 */
#define updateOTBar() displays.setRow(SHIFT_BAR_CTRL, LEFT_BAR, barGraphArray[map(channelData[OIL_T], 0, MAX_OT, 0, 8)])

/* Macro: C2F(int a)
 * Description: Converts Celcius to Fahrenheit (when the data is offset by 10)
 */
//#define C2F(a) (1.8 * (a) + 320)

// Create an object to control two cascaded MAX7219 chips
//  The library sets the pinModes for us.
LedControl displays = LedControl(11, 8, 12, 2);

void setup() {

	// setup pin I/O
	pinModeFast(LEFT_DISP_PIN, INPUT);
	pinModeFast(RIGHT_DISP_PIN, INPUT);
	pinModeFast(BRIGHT_DOWN_PIN, INPUT);
	pinModeFast(BRIGHT_UP_PIN, INPUT);
	pinModeFast(ALPHA_DATA_PIN, OUTPUT);
	pinModeFast(ALPHA_CLK_PIN, OUTPUT);
	pinModeFast(ALPHA_LAT_PIN, OUTPUT);

	//Start MAX7219 chips. Blank the displays.
	for(byte index = 0; index < displays.getDeviceCount(); index++) {
		displays.shutdown(index, false);
	}

	// set initial brightness to the maximum
	displays.setIntensity(DIGIT_CTRL, 15);
	displays.setIntensity(SHIFT_BAR_CTRL, 15);

	// begin commmunication
	Serial.begin(BAUD);

#ifdef STARTUP
	//Blink the lights to make it look like a "boot up" sequence
	startupSequence();

	// flash the text for the intial sensors that are defaulted to
	updateText(RIGHT);
	updateText(LEFT);
#endif

#ifndef BRIGHTNESS
	resetOn = true;
#endif

	// delay and blink timers
	rpmBlinkTimer = textLeftHoldTimer = textRightHoldTimer = leftBlinkTimer = rightBlinkTimer \
		= otherTextHoldTimer = rightBarBlinkTimer = leftBarBlinkTimer = millis();
}

void loop() {

	// Variables
	static byte l_disp_butt, r_disp_butt;
#ifdef BRIGHTNESS
	static byte bright_down_butt, bright_up_butt;
#endif
	static byte current_gear = 0;

	// update current data
	takeReadings();

	// check if a button has been pressed
	if(l_disp_butt != digitalReadFast(LEFT_DISP_PIN)) {
		// update the pin state since it's different now
		l_disp_butt = digitalReadFast(LEFT_DISP_PIN);
		// only update when button is low
		if(!l_disp_butt) {
			if(++displayStates[LEFT] > 7)
				displayStates[LEFT] = 0;
			updateText(LEFT);
		}
	}
	// see above
	if(r_disp_butt != digitalReadFast(RIGHT_DISP_PIN)) {
		r_disp_butt = digitalReadFast(RIGHT_DISP_PIN);
		if(!r_disp_butt) {
			if(++displayStates[RIGHT] > 7)
				displayStates[RIGHT] = 0;
			updateText(RIGHT);
		}
	}
#ifdef BRIGHTNESS
	// check if a birghtness button has been pressed
	if(bright_down_butt != digitalReadFast(BRIGHT_DOWN_PIN)) {
		// button state is different from before update the state variable
		bright_down_butt = digitalReadFast(BRIGHT_DOWN_PIN);
		// only update when button is low
		if(!bright_down_butt)
			adjustBrightness(DOWN);
	}
	// see above
	else if(bright_up_butt != digitalReadFast(BRIGHT_UP_PIN)) {
		bright_up_butt = digitalReadFast(BRIGHT_UP_PIN);
		if(!bright_up_butt)
			adjustBrightness(UP); 
	}
#else

	// update flags for average and max/min display along with the reset flag
	if(digitalReadFast(BRIGHT_UP_PIN))
		averageOn = false;
	else
		averageOn = true;

	if(digitalReadFast(BRIGHT_DOWN_PIN))
		extremeOn = false;
	else
		extremeOn = true;

	if(digitalReadFast(BRIGHT_DOWN_PIN) || digitalReadFast(BRIGHT_UP_PIN))
		resetOn = false;
	else
		resetOn = true;
#endif

	//This loop determines which variables should be displayed on the left 
	// and right seven segment displays based on the driver's display 
	// selections and calls the appropriate function to convert that variable
	// into seven segment data and display it on either display. 
	for(byte i = 0; i < 2; i++) {
		//If we are displaying a newly selected parameter, this
		//  will prevent it from being immediately overwritten.
		if(displayHold[i] == true)
			continue;

		switch(displayStates[i]) {
			case RPM_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateRPM(i, channelExtreme[RPM]);
				else if(averageOn)
					updateRPM(i, channelAverage[RPM]);
				else
#endif
					updateRPM(i, channelData[RPM] - 2500);
				break;
			case SPEED_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay(i, channelExtreme[G_SPEED]);
				else if(averageOn)
					updateDisplay(i, channelAverage[G_SPEED]);
				else
#endif
					updateDisplay(i, channelData[G_SPEED]);
				break;
			case OIL_P_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay(i, channelExtreme[OIL_P]);
				else if(averageOn)
					updateDisplay(i, channelAverage[OIL_P]);
				else
#endif
					updateDisplay(i, channelData[OIL_P]);
				break;
			case OIL_T_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay(i, channelExtreme[OIL_T]);
				else if(averageOn)
					updateDisplay(i, channelAverage[OIL_T]);
				else
#endif
					updateDisplay(i, channelData[OIL_T]);
				break; 
			case AIR_FUEL_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay_Decimal(i, channelExtreme[AF]);
				else if(averageOn)
					updateDisplay_Decimal(i, channelAverage[AF]);
				else
#endif
					updateDisplay_Decimal(i, channelData[AF]);
				break; 
			case WATER_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay(i, channelExtreme[WATER]);
				else if(averageOn)
					updateDisplay(i, channelAverage[WATER]);
				else
#endif
					updateDisplay(i, channelData[WATER]);
				break; 
			case BATTERY_DISPLAY:
#ifndef BRIGHTNESS
				if(extremeOn)
					updateDisplay_Decimal(i, channelExtreme[BATT]);
				else if(averageOn)
					updateDisplay_Decimal(i, channelAverage[BATT]);
				else
#endif
					updateDisplay_Decimal(i, channelData[BATT]);
				break; 
			case TMR_DISPLAY:
				updateTimeDisplay(i, millis());
				break;
			default:
				break;
		}
	}

	// Self explanatory
	updateShiftLights();
	if(revLimit)
		blinkShiftLights();

	//This code overtakes the left led bar display when the driver has 
	// pushed the left display cycle button. While the text is being held
	// in the corresponding seven segment, the appropriate LED in the sequence
	// will blink at .1 second intervals. The "noBarBlink" parameter is for 
	// use with text displays that don't utilize a banked selection, i.e.
	// the TCS ON/OFF text.
	if(displayHold[LEFT]/* && !noBarBlink*/) {
		if(millis() - leftBlinkTimer > BAR_BLINK_DELAY) {
			leftBarBlink ^= true;
			displays.setLed(SHIFT_BAR_CTRL, 2, displayStates[LEFT], leftBarBlink);
			leftBlinkTimer = millis();
		}
		//When our test hold timer has expired, clear the seven segment displays
		if(millis() - textLeftHoldTimer > TEXT_HOLD_DELAY) {
			displayHold[LEFT] = false;
			for(byte i = 0; i < 4; i++)
				displays.setRow(DIGIT_CTRL, i, SEV_SEG_NULL);
		}
	}
	//Otherwise, the left led bar always displays oil temperature
	else
		if(channelData[OIL_T] < MAX_OT) {
			displays.setRow(0, EXTRA_BAR, EXTRA);
			updateOTBar();
			leftBarWarn = false;
		}
		else {
			leftBarGraphWarn();
			leftBarWarn = true;
		}

	//See above!
	if(displayHold[RIGHT]/* && !noBarBlink*/) {
		if(millis() - rightBlinkTimer > BAR_BLINK_DELAY) {
			rightBarBlink ^= true;
			displays.setLed(SHIFT_BAR_CTRL, 3, displayStates[RIGHT], rightBarBlink);
			rightBlinkTimer = millis();
		}
		if(millis() - textRightHoldTimer > TEXT_HOLD_DELAY) {
			displayHold[RIGHT] = false;
			for(byte i = 4; i < 8; i++)
				displays.setRow(DIGIT_CTRL, i, SEV_SEG_NULL);
		}
	}
	else
		if(channelData[WATER] < MAX_ET) {
			displays.setRow(0, EXTRA_BAR, EXTRA);
			updateETBar();
			rightBarWarn = false;
		}
		else {
			rightBarGraphWarn();
			rightBarWarn = true;
		}

	// figure out what gear we are in
	if(channelData[GEAR] / 10 != current_gear) {
		current_gear = channelData[GEAR] / 10;
		digitalWriteFast(ALPHA_LAT_PIN, LOW);
		shiftOut(ALPHA_DATA_PIN, ALPHA_CLK_PIN, MSBFIRST, gearArray[current_gear]);
		digitalWriteFast(ALPHA_LAT_PIN, HIGH);
	}


#ifdef GEAR_DETECT
	// figure out what gear we are in
	if(computeCurrentGear()) {
		digitalWriteFast(ALPHA_LAT_PIN, LOW);
		shiftOut(ALPHA_DATA_PIN, ALPHA_CLK_PIN, MSBFIRST, gearArray[gear]);
		digitalWriteFast(ALPHA_LAT_PIN, HIGH);
		gearChanged = false;
	}
#endif

#ifdef OTHER_TEXT
	//Takes care of holding other text on the displays for the specified hold time.
	if(noBarBlink == true) {
		if(millis() - otherTextHoldTimer > TEXT_HOLD_DELAY) {
			displayHold[RIGHT] = displayHold[LEFT] = false;
			noBarBlink = false;
			displays.clearDisplay(1);
		}
	}
#endif

}

//////////////////////
//	User Functions	//
//////////////////////


/* Function: takeReadings
 * 
 * Parameters: none
 *
 * Description: Reads PWM inputs from the Motec M400 and converts 
 *              the data into the scaled readings for the wheel.
 */
void takeReadings() {

	byte ECUBuffer[ECU_MSG_LENGTH] = {0};

	while(Serial.available()) {
		// check for the begining of a message
		if(Serial.peek() == 0x82) {
			// get the ECU message
			for(byte i = 0; i < ECU_MSG_LENGTH; i++) {
				// wait till there is a byte availible for reading
				while(!Serial.available());
				ECUBuffer[i] = Serial.read();
			}

			// clear out data currently in the serial buffers so we have new data next time we read
			byte count = Serial.available();
			for(byte i = 0; i < count; i++)
				Serial.read();

			// check if the recieved message is error free
			if(ECUBuffer[0] != 0x82 || ECUBuffer[1] != 0x81 || ECUBuffer[2] != 0x80 || ECUBuffer[3] != NUM_CHANNELS)
				return;

			// place data in our data buffer reordered for aruino byte order (Intel)
			byte index = 0;
			for(byte i = 0; i < NUM_CHANNELS * 2; i += 2) {
				FromInt.buf[1] = ECUBuffer[i + HEADER_LENGTH];
				FromInt.buf[0] = ECUBuffer[i + HEADER_LENGTH + 1];
				channelData[index++] = FromInt.var; 
			}

			// stop gear postion from going all over the place while the car is sitting
			if(channelData[G_SPEED] < 50) {
				channelData[GEAR] = 70;
			}

			// get data in the correct units
			channelData[AF] *= STOICH;
//			drivenSpeed = (channelData[RRWSS] + channelData[RLWSS]) * KMPH_TO_MPH / 2;
//			channelData[OIL_P] *= KPA_TO_PSI;				(Motec outputs this data in PSI already)
//			channelData[OIL_T] = C2F(channelData[OIL_T]);	(Celcious is prefered by engine team)
//			waterC = (byte)channelData[WATER];				(Water data is staying in celcious no need for extra variable)
//			channelData[WATER] = C2F(channelData[WATER]);	(Celcious is prefered by engine team)

			// fuck with RPM for different shift light timing
			channelData[RPM] = channelData[RPM] + 2500;

#ifndef BRIGHTNESS
			if(resetOn)
				for(byte i = 0; i < NUM_CHANNELS; i++) {
					channelExtreme[i] = channelData[i];
					channelAverage[i] = channelData[i];
				}
			else {
				// get minimums
				if(channelExtreme[AF] > channelData[AF])
					channelExtreme[AF] = channelData[AF];
				if(channelExtreme[OIL_P] > channelData[OIL_P])
					channelExtreme[OIL_P] = channelData[OIL_P];
				// get maximums
				if(channelExtreme[OIL_T] < channelData[OIL_T])
					channelExtreme[OIL_T] = channelData[OIL_T];
				if(channelExtreme[WATER] < channelData[WATER])
					channelExtreme[WATER] = channelData[WATER];
				if(channelExtreme[G_SPEED] < channelData[G_SPEED])
					channelExtreme[G_SPEED] = channelData[G_SPEED];
				if(channelExtreme[BATT] < channelData[BATT])
					channelExtreme[BATT] = channelData[BATT];
				if(channelExtreme[RPM] < channelData[RPM])
					channelExtreme[RPM] = channelData[RPM];

				// get averages
				for(byte i = 0; i < NUM_CHANNELS; i++) {
					if(channelData[G_SPEED] > 80)
						channelAverage[i] = (channelData[i] + channelAverage[i]) / 2;
				}
			}
#endif

			// data has been updated time to leave
			return;
		}
		else
			Serial.read(); //discard byte
	}
}

#ifdef BRIGHTNESS
/* Function: adjustBrightness
 * 
 * Parameters: adjustment - either a 0 or 1 signaling increase or decrease
 *
 * Description: Adjusts brightness in one increment from 1 to 15. 
 */
void adjustBrightness(const byte adjustment) {

	// check which way to increment the brightness
	if(adjustment == UP) {
		if(++brightness > 15)
			brightness = 15;
	}
	// adjustment is down
	else {
		if(--brightness < 1)
			brightness = 1;
	}

	// set displays
	displays.setIntensity(SHIFT_BAR_CTRL, brightness);
	displays.setIntensity(DIGIT_CTRL, brightness);

	return;
}
#endif

/* Function: updateText
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Writes text output to the seven segment displays, sets the 
 *              corresponding LED bar to all on, and begins the hold and 
 *              blink timers. 
 */
void updateText(const byte whichDisplay) {

	//LEFT Display
	if(whichDisplay == LEFT) {
		displays.setRow(DIGIT_CTRL, 3, displayTexts[displayStates[whichDisplay]][0]);
		displays.setRow(DIGIT_CTRL, 2, displayTexts[displayStates[whichDisplay]][1]);
		displays.setRow(DIGIT_CTRL, 1, displayTexts[displayStates[whichDisplay]][2]);
		displays.setRow(DIGIT_CTRL, 0, displayTexts[displayStates[whichDisplay]][3]);
		displays.setRow(SHIFT_BAR_CTRL, LEFT_BAR, BAR_8);
		textLeftHoldTimer = leftBlinkTimer = millis();
		displayHold[whichDisplay] = true;
	}

	//RIGHT Display
	else {
		displays.setRow(DIGIT_CTRL, 7, displayTexts[displayStates[whichDisplay]][0]);
		displays.setRow(DIGIT_CTRL, 6, displayTexts[displayStates[whichDisplay]][1]);
		displays.setRow(DIGIT_CTRL, 5, displayTexts[displayStates[whichDisplay]][2]);
		displays.setRow(DIGIT_CTRL, 4, displayTexts[displayStates[whichDisplay]][3]);
		displays.setRow(SHIFT_BAR_CTRL, RIGHT_BAR, BAR_8);
		textRightHoldTimer = rightBlinkTimer = millis();
		displayHold[whichDisplay] = true;
	}

	return;
}

#if (0)
/* Function: updateOtherText
 * 
 * Parameters: leftDisplayIndex - index into otherDisplayTexts array
 *             rightDisplayIndex - "                               "
 *
 * Description: Writes the other text options to both of the seven 
 *              segments, as they arise.  Text is selected by the inputs. 
 */
boolean updateOtherText(const byte leftDisplayIndex, const byte rightDisplayIndex, unsigned long * otherTextHoldTimer, boolean * displayHold) {

	//LEFT Display
	displays.setRow(DIGIT_CTRL, 3, otherDisplayTexts[leftDisplayIndex][0]);
	displays.setRow(DIGIT_CTRL, 2, otherDisplayTexts[leftDisplayIndex][1]);
	displays.setRow(DIGIT_CTRL, 1, otherDisplayTexts[leftDisplayIndex][2]);
	displays.setRow(DIGIT_CTRL, 0, otherDisplayTexts[leftDisplayIndex][3]);

	//RIGHT Display
	displays.setRow(DIGIT_CTRL, 7, otherDisplayTexts[rightDisplayIndex][0]);
	displays.setRow(DIGIT_CTRL, 6, otherDisplayTexts[rightDisplayIndex][1]);
	displays.setRow(DIGIT_CTRL, 5, otherDisplayTexts[rightDisplayIndex][2]);
	displays.setRow(DIGIT_CTRL, 4, otherDisplayTexts[rightDisplayIndex][3]);

	//Only one timer is needed since we are controlling both displays
	*otherTextHoldTimer = millis();
	displayHold[LEFT] = displayHold[RIGHT] = true;

	return true;
}
#endif

/* Function: updateDisplay
 * 
 * Parameters:	whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *				data - the data to be displayed
 *
 * Description: Converts the passed variable received from Motec into
 *              a 3 digit display for use on a 7-seg and displays it.
 *				Hundreds place is only displayed when non-zero.
 */
void updateDisplay(const byte whichDisplay, const int data) {

	const byte hundreds = (byte)((data % 10000) / 1000);
	const byte tens = (byte)((data % 1000) / 100);
	const byte ones = (byte)(data % 100)/ 10;

	if(whichDisplay == LEFT) {
		// display hundreds digit if non-zero
		if(hundreds)
			displays.setDigit(DIGIT_CTRL, 2, hundreds, false);
		else
			displays.setRow(DIGIT_CTRL, 2, SEV_SEG_NULL);
		// display tens and ones digits
		displays.setDigit(DIGIT_CTRL, 1, tens, false);
		displays.setDigit(DIGIT_CTRL, 0, ones, false);    
	}
	else {
		// see above
		if(hundreds)
			displays.setDigit(DIGIT_CTRL, 6, hundreds, false);
		else
			displays.setRow(DIGIT_CTRL, 6, SEV_SEG_NULL);
		displays.setDigit(DIGIT_CTRL, 5, tens, false);
		displays.setDigit(DIGIT_CTRL, 4, ones, false);
	}

	return;
}

/* Function: updateDisplay_Decimal
 * 
 * Parameters:	whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *				data - the data to be written to the display
 *
 * Description: Converts .01 scaled variable received from Motec into
 *              a 3 digit display for use on a 7-seg and displays it. 
 */
void updateDisplay_Decimal(const byte whichDisplay, const int data) {

	const byte tens = (byte)((data % 10000) / 1000);
	const byte ones = (byte)((data % 1000) / 100);
	const byte tenths = (byte)((data % 100) / 10);

	if(whichDisplay == LEFT) {
		displays.setDigit(DIGIT_CTRL, 2, tens, false);
		displays.setDigit(DIGIT_CTRL, 1, ones, true);
		displays.setDigit(DIGIT_CTRL, 0, tenths, false);
	}
	else {
		displays.setDigit(DIGIT_CTRL, 6, tens, false);
		displays.setDigit(DIGIT_CTRL, 5, ones, true);
		displays.setDigit(DIGIT_CTRL, 4, tenths, false);
	}

	return;
}

/* Function: updateRPM
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *
 * Description: Converts the RPM variable received from the data board into
 *              a four digit display for use on a 7-seg and displays it. 
 */
void updateRPM(const byte whichDisplay, const int data) {

	byte tenThousands = (byte)(data / 10000);
	byte thousands = (byte)((data % 10000) / 1000);
	byte hundreds = (byte)((data % 1000) / 100);
	byte tens = (byte)((data % 100) / 10);

	if(tens > 5)
		if(hundreds < 9)
			hundreds++;
		else {
			hundreds = 0;
			if(thousands < 9)
				thousands++;
			else {
				tenThousands++;
				thousands = 0;
			}
		}

	tens = 0;

	if(whichDisplay == LEFT) {
		displays.setDigit(DIGIT_CTRL, 3, tenThousands, false);
		displays.setDigit(DIGIT_CTRL, 2, thousands, true);
		displays.setDigit(DIGIT_CTRL, 1, hundreds, false);
		displays.setDigit(DIGIT_CTRL, 0, tens, false);
	}
	else {
		displays.setDigit(DIGIT_CTRL, 7, tenThousands, false);
		displays.setDigit(DIGIT_CTRL, 6, thousands, true);
		displays.setDigit(DIGIT_CTRL, 5, hundreds, false);
		displays.setDigit(DIGIT_CTRL, 4, tens, false);
	}

	return;
}

/* Function: updateShiftLights
 * 
 * Parameters: none
 *
 * Description: Checks the current RPM and determines what the shift lights to display
 */
void updateShiftLights() {

	// don't update shift lights when we are warning the driver
	if (rightBarWarn || leftBarWarn)
		return;

	// check if RPM has been at rev limit recently
	if(revLimit) {
		// check if we are still at the rev limit
		if(channelData[RPM] <= REV_LIMIT)
			revLimit = false;
		else {
			return;
		}
	}
	else
		// check we are at the rev limit now
		if(channelData[RPM] > REV_LIMIT) {
			revLimit = true;
			// light up all shift lights
			shiftLightsState = SHIFT_8;
			displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_8);
			displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_8);
			return;
		}

	// check what RPM range we are in and light up the appropriate LEDs
	if(channelData[RPM] > REV_RANGE8) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_8);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_8);
	}
#ifdef REV_RANGE_7
	else if(channelData[RPM] > REV_RANGE7) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_7);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_7);
	}
#endif
#ifdef REV_RANGE_6
	else if(channelData[RPM] > REV_RANGE6) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_6);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_6);
	}
#endif
#ifdef REV_RANGE_5
	else if(channelData[RPM] > REV_RANGE5) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_5);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_5);
	}
#endif
#ifdef REV_RANGE_4
	else if(channelData[RPM] > REV_RANGE4) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_4);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_4);
	}
#endif
#ifdef REV_RANGE_3
	else if(channelData[RPM] > REV_RANGE3) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_3);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_3);
	}
#endif
#ifdef REV_RANGE_2
	else if(channelData[RPM] > REV_RANGE2) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_2);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_2);
	}
#endif
#ifdef REV_RANGE_1
	else if(channelData[RPM] > REV_RANGE1) {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_1);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_1);
	}
#endif
	else {
		displays.setRow(SHIFT_BAR_CTRL, 0, SHIFT_NULL);
		displays.setRow(SHIFT_BAR_CTRL, 1, SHIFT_NULL);
	}

	return;
}

/* Function: blinkShiftLights
 * 
 * Parameters: none
 *
 * Description: When called this will blink the shift lights indicating the rev limit has been reached
 */
void blinkShiftLights() {

	// don't update shift lights when we are warning the driver
	if (rightBarWarn || leftBarWarn)
		return;

	if(millis() - rpmBlinkTimer > RPM_BLINK_DELAY) {
		// XOR to flip the bits from 0x00 to 0xFF and vice versa
		shiftLightsState ^= SHIFT_8;
		displays.setRow(SHIFT_BAR_CTRL, 0, shiftLightsState);
		displays.setRow(SHIFT_BAR_CTRL, 1, shiftLightsState);
		rpmBlinkTimer = millis();
	}

	return;
}

/* Function: leftBarGraphWarn
 *
 * Parameters: none
 *
 * Description: When called this will blink the left LED bar graph indicating the oil temperature is excessive
 */
void leftBarGraphWarn() {
	if(millis() - leftBarBlinkTimer > WARNING_BLINK_DELAY) {
		// XOR to flip the bits from 0x00 to 0xFF and vice versa
		leftBarGraphState ^= BAR_8;
		shiftLightRedState ^= RED_WARN;
		displays.setRow(SHIFT_BAR_CTRL, 0, shiftLightBlueState | shiftLightRedState);
		displays.setRow(SHIFT_BAR_CTRL, 1, shiftLightBlueState | shiftLightRedState);
		displays.setRow(SHIFT_BAR_CTRL, LEFT_BAR, leftBarGraphState);
		leftBarBlinkTimer = millis();
	}
}

/* Function: rightBarGraphWarn
 * 
 * Parameters: none
 *
 * Description: When called this will blink the left LED bar graph indicating the water temperature is excessive
 */
void rightBarGraphWarn() {
	if(millis() - rightBarBlinkTimer > WARNING_BLINK_DELAY) {
		// XOR to flip the bits from 0x00 to 0xFF and vice versa
		rightBarGraphState ^= BAR_8;
		shiftLightBlueState ^= BLUE_WARN;
		displays.setRow(SHIFT_BAR_CTRL, 0, shiftLightBlueState | shiftLightRedState);
		displays.setRow(SHIFT_BAR_CTRL, 1, shiftLightBlueState | shiftLightRedState);
		displays.setRow(SHIFT_BAR_CTRL, RIGHT_BAR, rightBarGraphState);
		rightBarBlinkTimer = millis();
	}
}

/* Function: updateTimeDisplay
 * 
 * Parameters: whichDisplay - either a 0 or 1 for LEFT or RIGHT 7-seg display
 *             time - the time data that's been passed to be displayed
 *
 * Description: Converts a time variable received from the data board into
 *              a four digit display for use on a 7-seg and displays it. 
 *
 *              Capable of displaying a time of less than 10 minutes in a 
 *              min.secs.tenths format, i.e. 1.20.5 would be 1min, 20.5secs
 */
void updateTimeDisplay(const byte whichDisplay, const unsigned long time) {

	const byte minutes = (byte)(((time / 1000) / 60) % 10);
	const byte secondTens = (byte)((time % 60000) / 10000);
	const byte secondOnes = (byte)((time % 10000) / 1000);
	const byte tenths = (byte)((time % 1000) / 100);

	if(whichDisplay == LEFT) {
		displays.setDigit(DIGIT_CTRL, 3, minutes, true);
		displays.setDigit(DIGIT_CTRL, 2, secondTens, false);
		displays.setDigit(DIGIT_CTRL, 1, secondOnes, true);
		displays.setDigit(DIGIT_CTRL, 0, tenths, false);    
	}
	else {
		displays.setDigit(DIGIT_CTRL, 7, minutes, true);
		displays.setDigit(DIGIT_CTRL, 6, secondTens, false);
		displays.setDigit(DIGIT_CTRL, 5, secondOnes, true);
		displays.setDigit(DIGIT_CTRL, 4, tenths, false);    
	}

	return;
}

#ifdef GEAR_DETECT
/* Function: computeCurrentGear
 * 
 * Parameters: none
 *
 * Description: Calculates current gear based on driven wheel speed 
 *              and engine rpm. 
 */
boolean computeCurrentGear() {

	long perc_diff[6];
	int count[6];
	byte closest_gear = 0;
	byte computedGear;

	if(drivenSpeed < 1) {
		computedGear = 0;
	}
	else {
		const long ratio = channelData[RPM] / drivenSpeed;
		for(byte i = 0; i < 6; i++) {
			perc_diff[i] = 2 * fabs(ratio - RPM_PER_MPH[i]) / (ratio + RPM_PER_MPH[i]);
			if (perc_diff[i] < perc_diff[closest_gear])
				closest_gear = i;
		}

		for(byte i = 0; i < 24; i++) {
			gear_hist[i] = gear_hist[i + 1];
			count[gear_hist[i]]++;
		}

		gear_hist[24] = closest_gear;
		count[closest_gear]++;
		computedGear = gear;

		for(byte i = 0; i < 6; i++) {
			if(count[i] > count[computedGear])
				computedGear = i;
		}
	}

	if(computedGear != gear) {
		gear = computedGear;
		return true;
	}

	return false;
}
#endif
