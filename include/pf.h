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

#include <vector>
#include <stdlib.h>
#include <mutex>
#include <random>

#include "Types.h"
#include "bmm.h"
#include "settings.h"

#if PARALLELIZE == 1
#include <omp.h>
#endif

using namespace std;

typedef struct 

{
	float x;
	float y;
	float bearing;
} particle_type;

class pf
{
private:

// EVERY THING IS PUBLIC FOR UNIT TESTING
public:
	//map
	mutex _mapMutex;
	map_type *_map;
	//vector of particles representing 
	mutex _curStMutex;
	vector< particle_type* > *_curSt;
	//vector of particles representing 
	mutex _nxtStMutex;
	vector< particle_type* > *_nxtSt;
	//maximum number of particles (upper limit if using adaptive)
	int _maxP;

	//range finder model
	beamMeasurementModel *_bmm;

	//motion model gaussian sampler
	default_random_engine *_generator;

	//don't call the default constructor
	pf();

	//Helper functions

	float RandomFloat(float min, float max);

	vector<float> *expectedReadings( particle_type particle ) const;

	float getParticleWeight( particle_type particle, log_type *data ) const;

	void resampleW( vector< particle_type *> *resampledSt, vector<float> *Ws );

	particle_type motion_sample(particle_type u, float sigma=0.1) const;

	void init();
//public:
	static const int beam_fov = 180; 
	static const int beam_resolution = 1; 
	static const int res_x = 10;
	static const int res_y = 10;
	static constexpr float obst_thres = 0.9;

	pf(map_type *map, int max_particles);
	~pf();

	int convToGrid_x(float x) const;
	int convToGrid_y(float y) const;

	const vector< particle_type *> *access_st() const;
	const map_type *access_map() const;

	/* RESET PARTICLE FILTER */
	void reset();

	/* SENSOR UPDATE */
	void sensor_update( log_type *data );

	/* MOTION UPDATE */
	void motion_update( log_type *data );

}; //end class pf

#endif