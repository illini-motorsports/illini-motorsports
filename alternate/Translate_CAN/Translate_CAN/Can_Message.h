#include <string>
#include <iostream>
#include <fstream>
#include <map>
using namespace std;

#pragma once




class Can_Message
{
public:
	Can_Message(void);
	Can_Message(int theID, unsigned char theTime[4], unsigned char theData[8], int theSize);
	~Can_Message(void);
	void print(ofstream * outStream);
	void oldprint(ofstream * outStream, int n);
	void printHead(ofstream * outStream, int id);
	int id;
	double time;
	unsigned char data[8];
	int size;
};

