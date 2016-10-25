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
	_curSt( new vector< particle_type* >() ), 
	_nxtSt( new vector< particle_type* >() ),
	_maxP( max_particles ),
	_bmm( new beamMeasurementModel() ),
	_generator( new default_random_engine )
{
	init();
}

/* DESTRUCTOR */
pf::~pf()
{
	_map = NULL; // map will be destroyed by parser

	delete _curSt;
	delete _nxtSt;
	delete _bmm;
}

/* INITIALIZE */
float pf::RandomFloat(float min, float max)
{
	float r = (float)rand() / (float)RAND_MAX;
	return min + r * (max - min);
}

void pf::init()
{
	for (int i = 0; i < _maxP; i++) 
	{
		particle_type *particle;

		particle->x = RandomFloat( _map->min_x, _map->max_x);
		particle->y = RandomFloat( _map->min_y, _map->max_y);
		particle->bearing = RandomFloat(0.0, 2 * M_PI);

		int x_ = convToGrid_x(particle->x);
		int y_ = convToGrid_y(particle->y);

		while (_map->cells[x_][y_] == -1 || _map->cells[x_][y_] <= obst_thres) 
		{
			particle->x = RandomFloat( _map->min_x, _map->max_x);
			particle->y = RandomFloat( _map->min_y, _map->max_y);
		}

		_curSt->push_back(particle);
	}
}

/* RESET THE FILTER */
void pf::reset()
{
	//unique_lock<mutex> lock_curSt(_curStMutex);
	//unique_lock<mutex> lock_nxtSt(_nxtStMutex);

	//clear nxt and cur states
	_curSt->clear();
	_nxtSt->clear();
	//reinitialize
	init();
}

const vector< particle_type *> *pf::access_st() const
{
	return _curSt;
}
const map_type *pf::access_map() const
{
	return _map;
}

int pf::convToGrid_x(float x) const
{
	return static_cast<int>( x/res_x );
}

int pf::convToGrid_y(float y) const
{
	return static_cast<int>( y/res_y );
}

/* SENSOR UPDATE */
//Helper function: uses the map to get a vector of expected readings
float euclid(float x1, float y1, float x2, float y2)
{
	float x = x1 - x2; //calculating number to square in next step
	float y = y1 - y2;
	float dist;

	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);                  

	return dist;
}

vector<float> *pf::expectedReadings( particle_type particle ) const
{
//TODO
	//unique_lock<mutex> lock_map(_mapMutex); //released when exiting this function

	vector<float> *expected = new vector<float>(beam_fov / beam_resolution);
	float particle_bearing = particle.bearing;
	int x0 = convToGrid_x( particle.x );
	int y0 = convToGrid_y( particle.y );
	float range = 0.0;
	_bmm->get_param(MAX_RANGE, &range);

	float **grid_data = _map->cells;

	//use ray casting to get all grid locations to check
#if PARALLELIZE == 1
#pragma omp parallel for
#endif
	for(int beam = 0; beam < beam_fov; beam+=beam_resolution)
	{
		float beam_dir = beam*beam_resolution;
		float m = tan(beam_dir);

		// Bersenham's ray casting
		float error = -1.0;
		float d_error = beam_dir;
		int y = y0;
		int x = x0;
		float d = 0.0;
		while(true)
		{
			float tentative_d = euclid(x0, y0, x, y);
			if(tentative_d >= range )
			{
				d = range;
				break;
			}
			else if( grid_data[x][y] >= obst_thres )
			{
				d = tentative_d;
				break;
			}

			error += d_error;
			if( error >= 0.0 )
			{
				y += 1;
				error += error-1;
			}
			x += 1;
		}
		expected->at(beam) = d;
	}
	//calculate the expected readings
	return expected;
}

//Helper function: uses sensor model and map to get weight of a particle
float pf::getParticleWeight( particle_type particle, log_type *data ) const
{
	vector<float> *exp_readings = expectedReadings( particle );
	vector<float> *ws = new vector<float>(beam_fov / beam_resolution);
	float *beamReadings = data->r;

	// iterate through each beam
#if PARALLELIZE == 1
#pragma omp parallel for
#endif
	for(int beam = 0; beam < beam_fov; beam+=beam_resolution)
	{
		float reading = beamReadings[beam];

		// for each beam get expected range reading
		float expected_reading = exp_readings->at(beam);

		// get weight from the actual reading for that beam
		_bmm->set_param(P_HIT_U, expected_reading);
		float p_reading = _bmm->getP(reading);
		ws->at(beam) = p_reading; 
	}

	// calculate total weight
	float tot = 1.0;
	for(float w : *ws)
	{
		tot *= w; //TODO: sum of log
	}
	return tot;
}

//Helper function: uses low variance sampling to resample based on particle weights
void pf::resampleW( vector< particle_type *> *resampledSt, vector<float> *Ws )
{
	float r = RandomFloat(0.0, (float) 1/_curSt->size());
	float c = Ws->at(0);
	int idx = 0;
	for (int i = 0; i < _curSt->size(); i++)
	{
		particle_type *particle;
		float new_weight = r + static_cast<float>(i) / _curSt->size();
		while (c < new_weight)
		{
			idx++;
			c += (*Ws)[idx]; 
		}
		particle = (*_curSt)[idx];
		resampledSt->push_back(particle);
	}
}

//Main function to update state using laser reading
void pf::sensor_update( log_type *data )
{
	//unique_lock<mutex> lock_curSt(_curStMutex);
	//unique_lock<mutex> lock_nxtSt(_nxtStMutex);

	vector<float> *weights = new vector<float>();
	//iterate through all particles (TODO: Look into parallelizing)
	for( particle_type *x_m : *_curSt )
	{
		//get P(Z|X,map) = weight of particle
		float weight_x_m = getParticleWeight( *x_m, data );

		//store weight in vector
		weights->push_back(weight_x_m);
	}

	//using low covariance sampling
	_nxtSt->clear();
	resampleW( _nxtSt, weights );
}

/* MOTION UPDATE */
particle_type pf::motion_sample(particle_type u, float sigma) const
{
	//sample x
	normal_distribution<float> x_norm(u.x, sigma);
	float dx_sample = x_norm(*_generator);
	//sample y
	normal_distribution<float> y_norm(u.y, sigma);
	float dy_sample = y_norm(*_generator);
	//sample bearing
	normal_distribution<float> bearing_norm(u.bearing, sigma);
	float db_sample = bearing_norm(*_generator);

	particle_type dP;
	dP.x = dx_sample;
	dP.y = dy_sample;
	dP.bearing = db_sample;

	return dP;
}

void pf::motion_update( log_type *data )
{
	float *meatOfData = data->r;
	float dx = meatOfData[0];
	float dy = meatOfData[1];
	float db = meatOfData[2];
	float ts = meatOfData[3];

	particle_type dP_u;
	dP_u.x = dx;
	dP_u.y = dy;
	dP_u.bearing = db;

	//TODO: update the next state for now
	//unique_lock<mutex> lock_curSt(_curStMutex);
	//unique_lock<mutex> lock_map(_mapMutex);

	float **grid_data = _map->cells;
	
#if PARALLELIZE == 1
#pragma omp parallel for
#endif
	for (particle_type *particle : *_curSt)
	{
		particle_type dP = motion_sample(dP_u);

		particle->x += dP.x;
		particle->y += dP.y;

		int x_ = convToGrid_x(particle->x);
		int y_ = convToGrid_y(particle->y);

		if( grid_data[x_][y_] > obst_thres )
		{
			particle->x -= dP.x;
			particle->y -= dP.y;
		}

		particle->bearing += dP.bearing;
	    particle->bearing = fmod(particle->bearing, 2*M_PI);
		if (particle->bearing < 0) particle->bearing += 2*M_PI;
	}
}