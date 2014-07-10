#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Can_Message.h"
#include <map>

using namespace std;

static map<int,int> sizeMap;

int main(int argc, char *argv[])
{
	sizeMap[0x050] = 4;
	sizeMap[0x060] = 4;
	sizeMap[0x070] = 8;
	sizeMap[0x080] = 8;
	sizeMap[0x3F0] = 8;
	sizeMap[0x3F4] = 8;
	sizeMap[0x3F8] = 8;
	sizeMap[0x3FC] = 8;
	sizeMap[0x100] = REAR_HUB_NUM * 2;
	sizeMap[0x110] = FRONT_HUB_NUM * 2;
	sizeMap[0x250] = 8;
	sizeMap[0x251] = 2;
	sizeMap[0x200] = 8;
	sizeMap[0x201] = 8;
	sizeMap[0x202] = 8;
	sizeMap[0x203] = 2;

	if(argc != 3)
	{
		cout << "Usage: " << argv[0] << " <infile> <outfile>" << endl;
	}

	unsigned char id[ID_SIZE];
	unsigned char data[8];
	unsigned char time[TIMESTAMP_SIZE];
	int MsgCnt = 0;
	vector<Can_Message *> MsgList; // vector for storing all the messages

	//Opens for reading the file
	ifstream infile;
	if(argc != 2)
	{
		infile.open("2000.txt", ifstream::binary ); // binary required; otherwise it stops on EOF symbol...or something
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
	outfile.open("out.csv");
	if(!outfile)
	{
		printf ("output stream could not open");
		exit(1);
	}
	long int linecount = 0;
	while (infile)
	{
		// put 2 char into id
		for(int i=0;i<ID_SIZE;i++)
		{
			id[i] = infile.get();
			linecount++;
		}

		int someInt = id[0] + id[1]*256;
		int size = sizeMap[someInt];
		if(size != 0) // filter out fake messages
		{
			MsgCnt++;

			for(int i=0;i<size;i++)
			{
				data[i] = infile.get();
				linecount++;
			}

			for(int i=0;i<4;i++)
			{
				time[i] = infile.get();
				linecount++;
			}

			Can_Message * tempMsg = new Can_Message(someInt, time, data, size);
			MsgList.push_back(tempMsg);

			MsgList.back()->print(&outfile, MsgCnt);
			delete tempMsg; // this could be done somewhere else, but it's done here because the vector isn't quite used

			/* cut short the output file
			if (MsgCnt >= 100000)
			{
				break;
			}
			*/
		}
		else
		{
			outfile << "\nskipped msg id:" << someInt << endl;
		}
	}

	outfile << "\n\n\tMessage Count: " << MsgCnt << endl;

	outfile << "status: " << infile.flags() << endl;
}
