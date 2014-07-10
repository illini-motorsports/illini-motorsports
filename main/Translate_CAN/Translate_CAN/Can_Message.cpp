#include "Can_Message.h"
#include <fstream>
#include <map>
#include <iostream>
using namespace std;

Can_Message::Can_Message(void)
{
}

Can_Message::Can_Message(int theID, unsigned char theTime[4], unsigned char theData[8], int theSize)
{
	id = theID;
	time = theTime[3] * 256. + theTime[2] * 1. + (theTime[0] * 1. + theTime[1] * 256.) / 32768. - 1.;
	for(int i = 0; i < theSize; i++)
	{
		data[i] = theData[i];
	}
	size = theSize;
}

void Can_Message::print(ofstream *outStream, int n)
{

	printData(outStream); 

}

void Can_Message::printData(ofstream *outStream) 
{
	switch (id)
	{
		case ACCEL_1:
				printAccel1(outStream);
				break;
		case ACCEL_2:
				printAccel2(outStream);
				break;
		case TIRE_TEMP_FL:
				printTireTempFL(outStream);
				break;
		case TIRE_TEMP_FR:
				printTireTempFR(outStream);
				break;
		case TIRE_TEMP_RL:
				printTireTempRL(outStream);
				break;
		case TIRE_TEMP_RR:
				printTireTempRR(outStream);
				break;
		case F_HUB_FAST:
				printFrontHubFast(outStream);
				break;
		case F_HUB_SLOW:
				printFrontHubSlow(outStream);
				break;
		case R_HUB_FAST:
				printRearHubFast(outStream);
				break;
		case R_HUB_SLOW:
				printRearHubSlow(outStream);
				break;
		case MOTEC_0:
				printMotec0(outStream);
				break;
		case MOTEC_1:
				printMotec1(outStream);
				break;
		case MOTEC_2:
				printMotec2(outStream);
				break;
		case MOTEC_3:
				printMotec3(outStream);
				break;
		case THERMO_0:
				printThermo0(outStream);
				break;
		case THERMO_1:
				printThermo1(outStream);
				break;
		default:
				*outStream<<"Not Recongnized" << endl;
				break;
	}
}
void Can_Message::printAccel1(ofstream *outStream) const
{
	/*calculate accel data*/
	int sum = data[0] + data[1] * 256;
	sum = sum - 0x8000;

	double yaw_rate = sum * YAW_RATE_SCALE;
	sum = data[4] + data[5] * 256;
	sum = sum - 0x8000;

	double accy = sum * LONG_LAT_ACCEL_SCALE;

	*outStream<<"Yaw Rate,"<<yaw_rate<<","<<time<<endl;
	*outStream<<"Lateral Acceleration,"<<accy<<","<<time<<endl;
} 

void Can_Message::printAccel2(ofstream *outStream) const
{
	/*calculate accel data*/
	int sum = data[0] + data[1] * 256;
	sum = sum - 0x8000;
	double yaw_acc = sum * YAW_ACCEL_SCALE;

	sum = data[4] + data[5] * 256;
	sum = sum - 0x8000;
	double accx = sum * LONG_LAT_ACCEL_SCALE;

	*outStream<<"Yaw Acceleration,"<<yaw_acc<<","<<time<<endl;
	*outStream<<"Longitudinal Acceleration,"<<accx<<","<<time<<endl;
} 


void Can_Message::printTireTempFL(ofstream *outStream) const
{
	/*calculate tire temp data */
	int sum = data[0] * 256 + data[1];
	double Chan1Result = sum * TIRE_TEMP_SCALE;

	sum = data[2] * 256 + data[3];
	double Chan3Result = sum * TIRE_TEMP_SCALE;

	sum = data[4] * 256 + data[5];
	double Chan6Result = sum * TIRE_TEMP_SCALE;

	sum = data[6] * 256 + data[7];
	double Chan8Result = sum * TIRE_TEMP_SCALE;

	*outStream<<"Tire Temperature FL Ch 1,"<< Chan1Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FL Ch 3,"<< Chan3Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FL Ch 6,"<< Chan6Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FL Ch 8,"<< Chan8Result <<","<< time <<endl;
} 

void Can_Message::printTireTempFR(ofstream *outStream) const
{
	/*calculate tire temp data */
	int sum = data[0] * 256 + data[1];
	double Chan1Result = sum * TIRE_TEMP_SCALE;

	sum = data[2] * 256 + data[3];
	double Chan3Result= sum * TIRE_TEMP_SCALE;

	sum = data[4] * 256 + data[5];
	double Chan6Result= sum * TIRE_TEMP_SCALE;

	sum = data[6] * 256 + data[7];
	double Chan8Result = sum * TIRE_TEMP_SCALE;

	*outStream<<"Tire Temperature FR Ch 1,"<< Chan1Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FR Ch 3,"<< Chan3Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FR Ch 6,"<< Chan6Result <<","<< time <<endl;
	*outStream<<"Tire Temperature FR Ch 8,"<< Chan8Result <<","<< time <<endl;
} 

void Can_Message::printTireTempRL(ofstream *outStream) const
{
	/*calculate tire temp data */
	int sum = data[0] * 256 + data[1];
	double Chan1Result = sum * TIRE_TEMP_SCALE;

	sum = data[2] * 256 + data[3];
	double Chan3Result= sum * TIRE_TEMP_SCALE;

	sum = data[4] * 256 + data[5];
	double Chan6Result= sum * TIRE_TEMP_SCALE;

	sum = data[6] * 256 + data[7];
	double Chan8Result = sum * TIRE_TEMP_SCALE;

	*outStream<<"Tire Temperature RL Ch 1,"<< Chan1Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RL Ch 3,"<< Chan3Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RL Ch 6,"<< Chan6Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RL Ch 8,"<< Chan8Result <<","<< time <<endl;
} 

void Can_Message::printTireTempRR(ofstream *outStream) const
{
	/*calculate tire temp data */
	int sum = data[0] * 256 + data[1];
	double Chan1Result = sum * TIRE_TEMP_SCALE;

	sum = data[2] * 256 + data[3];
	double Chan3Result= sum * TIRE_TEMP_SCALE;

	sum = data[4] * 256 + data[5];
	double Chan6Result= sum * TIRE_TEMP_SCALE;

	sum = data[6] * 256 + data[7];
	double Chan8Result = sum * TIRE_TEMP_SCALE;

	*outStream<<"Tire Temperature RR Ch 1,"<< Chan1Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RR Ch 3,"<< Chan3Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RR Ch 6,"<< Chan6Result <<","<< time <<endl;
	*outStream<<"Tire Temperature RR Ch 8,"<< Chan8Result <<","<< time <<endl;
} 

void Can_Message::printFrontHubFast(ofstream *outStream) const
{
	/*calculate suspension pot data */
	unsigned int sum = data[0] + data[1] * 256;
	double SusPotFrResult = sum * SUS_POT_SCALE;

	sum = data[2] + data[3] * 256;
	double SusPotFlResult = sum * SUS_POT_SCALE;

	*outStream<<"Suspension Position FL,"<< SusPotFlResult <<","<< time <<endl;
	*outStream<<"Suspension Position FR,"<< SusPotFrResult <<","<< time <<endl;
}

void Can_Message::printFrontHubSlow(ofstream *outStream) const
{
	/*calculate brake pressure data */
	unsigned int sum  = data[0] + data[1] * 256;
	double BrkPrs1Result = (sum - 409.60) * BRAKE_PRESS_SCALE;

	sum  = data[2] + data[3] * 256;
	double BrkPrs0Result = (sum - 409.60) * BRAKE_PRESS_SCALE;

	/*calculate steering angle data */
	sum = data[2] + data[3] * 256;
	double SteerAngleResult = sum * STEER_ANG_SCALE;

	*outStream<<"Brake Pressure 0,"<< BrkPrs0Result <<","<< time <<endl;
	*outStream<<"Brake Pressure 1,"<< BrkPrs1Result <<","<< time <<endl;
	*outStream<<"Steering Angle,"<< SteerAngleResult <<","<< time <<endl;
}

void Can_Message::printRearHubFast(ofstream *outStream) const
{
	/*calculate suspension pot data */
	unsigned int sum = data[0] + data[1] * 256;
	double SusPotRlResult = sum * SUS_POT_SCALE; // in mm

	sum = data[2] + data[3] * 256;
	double SusPotRrResult = sum * OTHER_SUS_POT_SCALE; // in mm

	*outStream<<"Suspension Position RL,"<< SusPotRlResult <<","<< time <<endl;
	*outStream<<"Suspension Position RR,"<< SusPotRrResult <<","<< time <<endl;
}

void Can_Message::printRearHubSlow(ofstream *outStream) const
{
	return;
}

void Can_Message::printMotec0(ofstream *outStream) const
{
	//calculate Motec data
	int sum = data[0] * 256 + data[1];
	double rpm = sum * RPM_SCALE;

	sum = data[2] * 256 + data[3];
	double tp = sum * TP_SCALE;

	sum = data[4] * 256 + data[5];
	double op = sum * OP_SCALE;

	sum = data[6] * 256 + data[7];
	double ot = sum * OT_SCALE;

	*outStream<<"RPM,"<< rpm <<","<< time <<endl;
	*outStream<<"Throttle Position,"<< tp <<","<< time <<endl;
	*outStream<<"Oil Pressure,"<< op <<","<< time <<endl;
	*outStream<<"Oil Temperature,"<< ot <<","<< time <<endl;
}

void Can_Message::printMotec1(ofstream *outStream) const
{
	//calculate Motec data
	int sum = data[0] * 256 + data[1];
	double et = sum * ET_SCALE;

	sum = data[2] * 256 + data[3];
	double lam = sum * LAMBDA_SCALE;

	sum = data[4] * 256 + data[5];
	double map = sum * MAP_SCALE;

	sum = data[6] * 256 + data[7];
	double volt = sum * BATT_VOLT_SCALE;

	*outStream<<"Engine Temperature,"<< et <<","<< time <<endl;
	*outStream<<"Lambda,"<< lam <<","<< time <<endl;
	*outStream<<"Manifold Pressure,"<< map <<","<< time <<endl;
	*outStream<<"Battery Voltage,"<< volt <<","<< time <<endl;
}

void Can_Message::printMotec2(ofstream *outStream) const
{
	//calculate Motec data
	int sum = data[0] * 256 + data[1];
	double fl = sum * WSS_SCALE;

	sum = data[2] * 256 + data[3];
	double fr = sum * WSS_SCALE;

	sum = data[4] * 256 + data[5];
	double rl = sum * WSS_SCALE;

	sum = data[6] * 256 + data[7];
	double rr = sum * WSS_SCALE;

	*outStream<<"Wheel Speed FL,"<< fl <<","<< time <<endl;
	*outStream<<"Wheel Speed FR,"<< fr <<","<< time <<endl;
	*outStream<<"Wheel Speed RL,"<< rl <<","<< time <<endl;
	*outStream<<"Wheel Speed RR,"<< rr <<","<< time <<endl;
}

void Can_Message::printMotec3(ofstream *outStream) const
{
	//calculate Motec data
	int sum = data[0] * 256 + data[1];
	double inj = sum * PULSE_WIDTH_SCALE;

	*outStream<<"Injector Pulse Width,"<< inj <<","<< time <<endl;
}

void Can_Message::printThermo0(ofstream *outStream) const
{
	//calculate thermocouple data
	int sum = data[0] * 256 + data[1];
	double thermo0 = sum * THERMO_SCALE;

	sum = data[2] * 256 + data[3];
	double thermo1 = sum * THERMO_SCALE;

	sum = data[4] * 256 + data[5];
	double thermo2 = sum * THERMO_SCALE;

	sum = data[6] * 256 + data[7];
	double thermo3 = sum * THERMO_SCALE;

	*outStream<<"Thermocouple0,"<< thermo0 <<","<< time <<endl;
	*outStream<<"Thermocouple1,"<< thermo1 <<","<< time <<endl;
	*outStream<<"Thermocouple2,"<< thermo2 <<","<< time <<endl;
	*outStream<<"Thermocouple3,"<< thermo3 <<","<< time <<endl;
}

void Can_Message::printThermo1(ofstream *outStream) const
{
	//calculate thermocouple data
	int sum = data[0] * 256 + data[1];
	double thermo4 = sum * THERMO_SCALE;

	*outStream<<"Thermocouple0,"<< thermo4 <<","<< time <<endl;
}

Can_Message::~Can_Message(void)
{
}