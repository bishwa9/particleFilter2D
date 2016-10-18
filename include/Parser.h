//*****************************************************************
//  Parser.h
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This header file contains the parser class declaration.
//*****************************************************************

#include <cstdio>
#include <cstring>
#include <cstdlib>

#include <fstream>
#include <sstream>
#include <vector>

#include "Types.h"

using namespace std;

//*****************************************************************
// Parser Class
//*****************************************************************

class Parser
{
private:

public:
	Parser();

	map_type *_my_map;
	vector<log_type*> *_logData;

	void new_hornetsoft_map(int size_x, int size_y);
	int read_beesoft_map(const char *mapName);
	int read_log_data(const char *logName);
};

//*****************************************************************
// End of File
//*****************************************************************