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
	time = theTime[3]*256. + theTime[2]*1.  +  ((theTime[1] & 0x7F)*256.+ (theTime[0])*1.)/32768.;
	for(int i=0;i<theSize;i++)
	{
		data[i] = theData[i];
	}
	size = theSize;
}

void Can_Message::oldprint(ofstream * outStream, int n)
{

	//result = new std::string("");
	
	*outStream << "Msg # " << n << endl << "\tid: " << std::hex << (unsigned long)id << std::dec << "\ttime: " << time << endl;

	
	for(int i=0;i<size;i++)
	{
		*outStream << "\t\t[" << i << "] = " << (int)data[i] << endl;
	}

}


void Can_Message::print(ofstream * outStream)
{
	if(time < 4000)
	{
		*outStream << time << ","; // write time if not absurd
	}
	else
	{
		*outStream << ","; // skip time if absurd
	}
	switch(id)
	{
		case 0x070:
			*outStream << ((unsigned int)((data[0] + (data[1] << 8)) ^ 0x8000)) * 0.005 << "," << ((unsigned int)((data[4] + (data[5] << 8))) * .0001274)  << "," ;
			break;
		case 0x080:
			*outStream << ((unsigned int)((data[0] + (data[1] << 8)) ^ 0x8000)) * 0.0125 << "," << ((unsigned int)((data[4] + (data[5] << 8)) ^0x8000) * .0001274)  << "," ;
			break;
		case 0x3F0:
			*outStream << ((signed int)((data[0] << 8) + data[1])) * 0.1 << "," << ((signed int)((data[2] << 8) + data[3])) * 0.1  << "," << ((signed int)((data[4] << 8) + data[5])) * 0.1 << "," << ((signed int)((data[6] << 8) + data[7])) * 0.1 << "," ;
			break;
		case 0x3F4:
			*outStream << ((signed int)((data[0] << 8) + data[1])) * 0.1 << "," << ((signed int)((data[2] << 8) + data[3])) * 0.1  << "," << ((signed int)((data[4] << 8) + data[5])) * 0.1 << "," << ((signed int)((data[6] << 8) + data[7])) * 0.1 << "," ;
			break;
		case 0x3F8:
			*outStream << ((signed int)((data[0] << 8) + data[1])) * 0.1 << "," << ((signed int)((data[2] << 8) + data[3])) * 0.1  << "," << ((signed int)((data[4] << 8) + data[5])) * 0.1 << "," << ((signed int)((data[6] << 8) + data[7])) * 0.1 << "," ;
			break;
		case 0x3FC:
			*outStream << ((signed int)((data[0] << 8) + data[1])) * 0.1 << "," << ((signed int)((data[2] << 8) + data[3])) * 0.1  << "," << ((signed int)((data[4] << 8) + data[5])) * 0.1 << "," << ((signed int)((data[6] << 8) + data[7])) * 0.1 << "," ;
			break;
		case 0x060:
			*outStream << (unsigned)((data[0] << 8) + data[1]) << "," << (unsigned)((data[2] << 8) + data[3])  << "," ;
			break;
		case 0x110:
			*outStream << (float)((data[0] << 8) + data[1])* 0.7629 << "," << (float)((data[2] << 8) + data[3]) * 0.7629 << "," <<  (float)((data[4] << 8) + data[5]) << "," ;
			break;
		case 0x050:
			*outStream << (unsigned)((data[0] << 8) + data[1]) << "," << (unsigned)((data[2] << 8) + data[3])  << "," ;
			break;
		default:
			*outStream << (unsigned)data[0] << "," << (unsigned)data[1] << "," << (unsigned)data[2] << "," << (unsigned)data[3] << "," << (unsigned)data[4] << "," << (unsigned)data[5] << "," << (unsigned)data[6] << "," << (unsigned)data[7] << "," ;
			break;
	}
}




Can_Message::~Can_Message(void)
{
}
