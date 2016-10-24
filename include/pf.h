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

#ifdef PARALLELIZE
#include <omp.h>
#endif

#include "Types.h"
#include "bmm.h"

using namespace std;

typedef vector<float>* particle_type;
enum particle_description {X_POS=0, Y_POS, BEARING_POS};

class pf
{
private:
	//map
	map_type *_map;
	//vector of particles representing 
	vector< particle_type > *_curSt;
	//vector of particles representing 
	vector< particle_type > *_nxtSt;
	//maximum number of particles (upper limit if using adaptive)
	int _maxP;

	//range finder model
	beamMeasurementModel *_bmm;

	//don't call the default constructor
	pf();

	//Helper functions
	vector<float> *expectedReadings( particle_type particle ) const;

	float getParticleWeight( particle_type particle, log_type *data ) const;

	void resampleW( vector< particle_type > *resampledSt, vector<float> *Ws );

	void init();
public:
	static const int beam_fov = 180; 
	static const int beam_resolution = 1; 
	static const int res_x = 10;
	static const int res_y = 10;
	static constexpr float obst_thres = 0.5;

	pf(map_type *map, int max_particles);
	~pf();

	int convToGrid_x(float x) const;
	int convToGrid_y(float y) const;

	/* RESET PARTICLE FILTER */
	void reset();

	/* SENSOR UPDATE */
	void sensor_update( log_type *data );

	/* MOTION UPDATE */
	void motion_update( log_type *data );

}; //end class pf

#endif