#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Can_Message.h"
#include <map>
#include <set>

using namespace std;

#define TIMESTAMP_SIZE 4

static map<int,int> sizeMap;
set<int> idSet;
map<int, vector<Can_Message *> > MsgListMap;

void betterPrint(ofstream * outStream);


int main(int argc, char *argv[])
{
	sizeMap[0x070] = 8;
	sizeMap[0x080] = 8;
	sizeMap[0x576] = 8;
	sizeMap[0x3F0] = 8;
	sizeMap[0x3F4] = 8;
	sizeMap[0x3F8] = 8;
	sizeMap[0x3FC] = 8;
	sizeMap[0x060] = 4;
	sizeMap[0x110] = 6;
	sizeMap[0x050] = 4;
	sizeMap[0x100] = 0;
	sizeMap[0x200] = 8;
	sizeMap[0x210] = 8;

	idSet.insert(0x070);
	idSet.insert(0x080);
	idSet.insert(0x576);
	idSet.insert(0x3F0);
	idSet.insert(0x3F4);
	idSet.insert(0x3F8);
	idSet.insert(0x3FC);
	idSet.insert(0x060);
	idSet.insert(0x110);
	idSet.insert(0x050);
	idSet.insert(0x200);
	idSet.insert(0x210);

	if(argc != 3)
	{
		cout << "Usage: " << argv[0] << " <infile> <outfile>" << endl;
	//	return -1;
	}
	


	unsigned char id[2];
	unsigned char data[8];
	unsigned char time[4];
	int MsgCnt = 0;


	//Opens for reading the file
	ifstream infile;
	if(argc != 2)
	{
		infile.open( "2000.CSV", ifstream::binary ); // binary required; otherwise it stops on EOF symbol...or something
	}
	else
	{
		infile.open( argv[1] , ifstream::binary ); 
	}
	//ifstream infile ( argv[1] );
	if(!infile)
	{
		printf ("input stream could not open");
		exit(1);
	}
	ofstream outfile;
	outfile.open( "out.txt" );
	//ofstream outfile( argv[2] );
	if(!outfile)
	{
		printf ("output stream could not open");
		exit(1);
	}
	long int linecount = 0;
	while (infile)
	{



		//outfile << "Message #: " << MsgCnt << endl;


		// put 2 char into id
		for(int i=0;i<2;i++)
		{
			id[i] = infile.get();
			linecount++;
		}
		
		int idVal = id[0] + id[1]*256;
		int size = sizeMap[idVal];
		if(size != 0) // filter out fake messages
		{
			MsgCnt++;
			//outfile << "\tid:" << someInt << endl;




			for(int i=0;i<size;i++)
			{
				data[i] = infile.get();
				linecount++;
			}
			//if(linecount < 512) UNCOMMENT to show that timestamps don't write occasionally after 512
			//{

				for(int i=0;i<4;i++)
				{
				
					time[i] = infile.get();
					linecount++;
				}
			//}
			/*for(int i=0;i<size;i++)
			{
				outfile << "\t\t[" << i << "]: "<< (int)(data[i]) << endl;
			}*/
			if(idSet.find(idVal) != idSet.end()) // if it was in the set of known ids, add it
			{
				Can_Message * tempMsg = new Can_Message(idVal, time, data, size);
				MsgListMap[idVal].push_back(tempMsg);
			}
			
			/*if(idSet.find(idVal) == idSet.end()) // if it wasn't in the set of known ids, add it
			{
				idSet.insert(idVal);
			}*/

			//MsgList.back()->print(&outfile, MsgCnt);
			//outfile << (tempString);
			//free(tempString);
			if (MsgCnt >= 1000000)
			{
				break;
			}
		}
		else
		{
			//outfile << "\nskipped msg id:" << someInt << endl;
		}
	}

	//outfile << "\n\n\tMessage Count: " << MsgCnt << endl;

	//outfile << "status: " << infile.flags() << endl;
	betterPrint(&outfile);
}


void printHead(ofstream * outStream, int id)
{
	switch(id)
	{
		case 0x070:
			*outStream << "time,Yaw Rate 1 (deg/s),Y axis acceleration(g),";
			break;
		case 0x080:
			*outStream << "time,Yaw Angular Acceleration 1 (deg/s^2),X axis acceleration(g),";
			break;
		case 0x3F0:
			*outStream << "time,Tire Temp 1 Channel 1 (deg),Tire Temp 1 Channel 3 (deg),Tire Temp 1 Channel 6 (deg),Tire Temp 1 Channel 8 (deg),";
			break;
		case 0x3F4:
			*outStream << "time,Tire Temp 2 Channel 1 (deg),Tire Temp 2 Channel 3 (deg),Tire Temp 2 Channel 6 (deg),Tire Temp 2 Channel 8 (deg),";
			break;
		case 0x3F8:
			*outStream << "time,Tire Temp 3 Channel 1 (deg),Tire Temp 3 Channel 3 (deg),Tire Temp 3 Channel 6 (deg),Tire Temp 3 Channel 8 (deg),";
			break;
		case 0x3FC:
			*outStream << "time,Tire Temp 4 Channel 1 (deg),Tire Temp 4 Channel 3 (deg),Tire Temp 4 Channel 6 (deg),Tire Temp 4 Channel 8 (deg),";
			break;
		case 0x060:
			*outStream << "time,Sus. Pot. FL,Sus. Pot. FR,";
			break;
		case 0x110:
			*outStream << "time,Front Hub Brake Pressure 0 (psi),Front Hub Brake Pressure 1 (psi),Steering angle(deg),";
			break;
		case 0x050:
			*outStream << "time,Sus. Pot. RL,Sus. Pot. RR,";
			break;
		default:
			*outStream << "time, byte 0, byte 1, byte 2, byte 3, byte 4, byte 5, byte 6, byte 7,";
			break;
	}

}

void printcommas(ofstream * outStream, int id)
{
	switch(id)
	{
		case 0x070:
			*outStream << ",,,";
			break;
		case 0x080:
			*outStream << ",,,";
			break;
		case 0x3F0:
			*outStream << ",,,,,";
			break;
		case 0x3F4:
			*outStream << ",,,,,";
			break;
		case 0x3F8:
			*outStream << ",,,,,";
			break;
		case 0x3FC:
			*outStream << ",,,,,";
			break;
		case 0x060:
			*outStream << ",,,";
			break;
		case 0x110:
			*outStream << ",,,,";
			break;
		case 0x050:
			*outStream << ",,,";
			break;
		default:
			*outStream << ",,,,,,,,,";
			break;
	}
}





void betterPrint(ofstream * outStream) // what this function needs to do is to take the map of can message ids, and first write the name of the field, then the field in the same position.
{
	for( auto it= idSet.begin(); it != idSet.end();it++)
	{
		printHead(outStream, *it);
		*outStream << ","; // some space inbetween each message horizontally
	}
	*outStream << "\n"; // a new line.

	for(unsigned int i=0;i<100000;i++)
	{
		for( auto it= idSet.begin(); it != idSet.end();it++)
		{
			if(i < MsgListMap[*it].size())
			{
				MsgListMap[*it][i]->print(outStream);
				*outStream << ","; // some space inbetween each message horizontally
			}
			else
			{
				printcommas(outStream, *it);
				*outStream << ","; // some space inbetween each message horizontally
			}
		}
		*outStream << "\n";
	}

}



