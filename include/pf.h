//*****************************************************************
//  pf.h
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This header file contains the class definition of the particle
//	filter.
//*****************************************************************

#ifndef _PF_H
#define _PF_H

#include <list>

#include "types.h"

using namespace std;

typedef list<int>* particle_type;

class pf
{
private:
	//map
	map_type *_map;
	//list of particles representing 
	list< particle_type > *_curSt;
	//list of particles representing 
	list< particle_type > *_nxtSt;

	//don't call the default constructor
	pf();
public:
	pf(map_type *map);
	~pf();

	/* SENSOR UPDATE */
	void sensor_update( log_type *data );

	/* MOTION UPDATE */
	void motion_update( log_type *data );

}; //end class pf

#endif