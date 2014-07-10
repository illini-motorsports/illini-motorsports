#include <string>
#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#pragma once

/*MACROS*/
#define ACCEL_1 0x070
#define ACCEL_2 0x080
#define TIRE_TEMP_FL 0x3F4
#define TIRE_TEMP_FR 0x3FC
#define TIRE_TEMP_RL 0x3F0
#define TIRE_TEMP_RR 0x3F8
#define F_HUB_FAST 0x060
#define F_HUB_SLOW 0x110
#define R_HUB_FAST 0x050
#define R_HUB_SLOW 0x100
#define MOTEC_0 0x200
#define MOTEC_1 0x201
#define MOTEC_2 0x202
#define MOTEC_3 0x203
#define THERMO_0 0x250
#define THERMO_1 0x251

#define REAR_HUB_NUM 0
#define FRONT_HUB_NUM 3
#define TIMESTAMP_SIZE 4
#define ID_SIZE 2

#define YAW_ACCEL_SCALE .0125
#define LONG_LAT_ACCEL_SCALE .0001274
#define YAW_RATE_SCALE .005
#define TIRE_TEMP_SCALE .1
#define SUS_POT_SCALE 0.012207
#define OTHER_SUS_POT_SCALE 0.0148866
#define BRAKE_PRESS_SCALE 0.7629395
#define STEER_ANG_SCALE 0.0878906
#define THERMO_SCALE .25
#define RPM_SCALE 1
#define TP_SCALE .1
#define OP_SCALE .1
#define OT_SCALE .1
#define WSS_SCALE .1
#define PULSE_WIDTH_SCALE .5
#define ET_SCALE .1
#define LAMBDA_SCALE .001
#define MAP_SCALE .1
#define BATT_VOLT_SCALE .01

class Can_Message
{
public:
	Can_Message(void);
	Can_Message(int theID, unsigned char theTime[4], unsigned char theData[8], int theSize);
	~Can_Message(void);
	void Can_Message::print(ofstream * outStream, int n);
	int id;
	double time;
	unsigned char data[8];
	int size;
private:
	void printID(ofstream *outStream);
	void printData(ofstream *outStream) ;
	void printAccel1(ofstream *outStream) const;
	void printAccel2(ofstream *outStream) const;
	void printTireTempFL(ofstream *outStream) const;
	void printTireTempFR(ofstream *outStream) const;
	void printTireTempRL(ofstream *outStream) const;
	void printTireTempRR(ofstream *outStream) const;
	void printFrontHubFast(ofstream *outStream) const;
	void printFrontHubSlow(ofstream *outStream) const;
	void printRearHubFast(ofstream *outStream) const;
	void printRearHubSlow(ofstream *outStream) const;
	void printMotec0(ofstream *outStream) const;
	void printMotec1(ofstream *outStream) const;
	void printMotec2(ofstream *outStream) const;
	void printMotec3(ofstream *outStream) const;
	void printThermo0(ofstream *outStream) const;
	void printThermo1(ofstream *outStream) const;
};

