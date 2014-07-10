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
	time = theTime[3] * 256. + theTime[2] * 1.  +  (theTime[1] * 256.+ theTime[0] * 1.) / 32768. - 1;
	for(int i=0;i<theSize;i++)
	{
		data[i] = theData[i];
	}
	size = theSize;
}

void Can_Message::print(ofstream * outStream, int n)
{

	//result = new std::string("");
	
	*outStream << "Msg # " << n << endl << "\tid: " << std::hex << (unsigned long)id << std::dec << "\ttime: " << time << endl;

	
	for(int i=0;i<size;i++)
	{
		*outStream << "\t\t[" << i << "] = " << (int)data[i] << endl;
	}

}


Can_Message::~Can_Message(void)
{
}
