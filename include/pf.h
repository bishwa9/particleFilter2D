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

typedef list<float>* particle_type;

class pf
{
private:
	//map
	map_type *_map;
	//list of particles representing 
	list< particle_type > *_curSt;
	//list of particles representing 
	list< particle_type > *_nxtSt;
	//maximum number of particles (upper limit if using adaptive)
	int _maxP;

	//don't call the default constructor
	pf();

	//Helper functions
	float getParticleWeight( particle_type particle, log_type *data );

	void resampleW( list< particle_type > *resampledSt, list<float> *Ws );

	void init();
public:
	pf(map_type *map, int max_particles);
	~pf();

	/* RESET PARTICLE FILTER */
	void reset();

	/* SENSOR UPDATE */
	void sensor_update( log_type *data );

	/* MOTION UPDATE */
	void motion_update( log_type *data );

}; //end class pf

#endif