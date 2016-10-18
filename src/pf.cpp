//*****************************************************************
//  pf.cpp
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This soruce file contains the class implementation of the particle
//	filter.
//*****************************************************************

#include "pf.h"

/* CONSTRUCTOR */
pf::pf(map_type *map, int max_particles):
	_map( map ), 
	_curSt( new list< particle_type >() ), 
	_nxtSt( new list< particle_type >() ),
	_maxP( max_particles )
{
	init();
}

/* DESTRUCTOR */
pf::~pf()
{
	_map = NULL; // map will be destroyed by parser

	delete _curSt;
	delete _nxtSt;
}

/* INITIALIZE */
void pf::init()
{
//TODO: look at map and create initial list of particles
}

/* RESET THE FILTER */
void pf::reset()
{
	//clear nxt and cur states
	_curSt->clear();
	_nxtSt->clear();
	//reinitialize
	init();
}

/* SENSOR UPDATE */
//Helper function: uses sensor model and map to get weight of a particle
float pf::getParticleWeight( particle_type particle, log_type *data )
{
//TODO
	return 0.0;
}

//Helper function: uses low variance sampling to resample based on particle weights
void pf::resampleW( list< particle_type > *resampledSt, list<float> *Ws )
{
//TODO
}

//Main function to update state using laser reading
void pf::sensor_update( log_type *data )
{
	list<float> *weights = new list<float>();
	//iterate through all particles (TODO: Look into parallelizing)
	for( particle_type x_m : *_curSt )
	{
		//get P(Z|X,map) = weight of particle
		float weight_x_m = getParticleWeight( x_m, data );

		//store weight in list
		weights->push_back(weight_x_m);
	}

	//using covariance
	resampleW( _nxtSt, weights );
}

/* MOTION UPDATE */
void pf::motion_update( log_type *data )
{
	//TODO: update the next state for now
}