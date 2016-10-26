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
pf::pf(map_type *map, int max_particles, int pf_num):
	_map( map ), 
	_curSt( new vector< particle_type* >() ), 
	_nxtSt( new vector< particle_type* >() ),
	_maxP( max_particles ),
	_bmm( new beamMeasurementModel() ),
	_generator( new default_random_engine )
#if UI == 1
	,
	_pfNum(pf_num),
	_mapWindowName( "Particle Filter_" + to_string(_pfNum) ),
	_mapMat( new Mat(_map->size_x, _map->size_y, CV_32F) ),
	_clearMapMat( new Mat(_map->size_x, _map->size_y, CV_32F) )
#endif
{
#if UI == 1
	for (unsigned int i = 0; i < _mapMat->rows; i++)
	{
		for (unsigned int j = 0; j < _mapMat->cols; j++)
		{
			if (_map->cells[i][j] > 0.0)
			{
				_mapMat->at<float>(i, j) =  _map->cells[i][j]; 
			}
		} 
	}
	cvtColor(*_mapMat, *_mapMat, COLOR_GRAY2BGR);
	_mapMat->copyTo(*_clearMapMat);
#endif
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

// TESTER : ABHISHEK
// TESTED!
void pf::init()
{
	for (int i = 0; i < _maxP; i++) 
	{
		particle_type *particle = new particle_type;

		particle->x = RandomFloat( _map->min_x, _map->max_x);
		particle->y = RandomFloat( _map->min_y, _map->max_y);
		particle->bearing = RandomFloat(0.0, 2 * M_PI) - M_PI;

		int x_ = particle->x;
		int y_ = particle->y;

		while (_map->cells[x_][y_] == -1 || _map->cells[x_][y_] <= obst_thres) 
		{
			particle->x = RandomFloat( _map->min_x, _map->max_x);
			particle->y = RandomFloat( _map->min_y, _map->max_y);
			x_ = particle->x;
			y_ = particle->y;
		}

		particle->x = x_;
		particle->y = y_;

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
	return static_cast<int>( x );
}

int pf::convToGrid_y(float y) const
{
	return static_cast<int>( y );
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


// TESTER : BISHWAMOY SINHA FUCKING ROY
// TEST : BMM IS DONE, FOR A PARTICLE TEST EXPECTED READINGS
// TESTING DONE
vector<float> *pf::expectedReadings( particle_type *particle ) const
{
	//unique_lock<mutex> lock_map(_mapMutex); //released when exiting this function

	vector<float> *expected = new vector<float>(beam_fov / beam_resolution);
	float particle_bearing = particle->bearing;
	float particle_x = particle->x;
	float particle_y = particle->y;

	float max_range = 0.0;
	_bmm->get_param(MAX_RANGE, &max_range);

	float **grid_data = _map->cells;

	//transformation matrices
	Eigen::Matrix<float, 3, 3> _H_m_r(3,3);
	Eigen::Matrix<float, 3, 3> _H_r_s(3,3);
	Eigen::Matrix<float, 3, 3> _H_s_p(3,3);

	_H_m_r << 1.0, 0.0, particle_x,
			  0.0, 1.0, particle_y,
			  0.0, 0.0, 1.0;

	_H_r_s << cos(particle_bearing), -sin(particle_bearing), 0,
			  sin(particle_bearing), cos(particle_bearing) , 0,
			  0					   , 0                     , 1;
#if PARALLELIZE == 1
#pragma omp parallel for
#endif
	for(int beam_deg = 0; beam_deg < beam_fov; beam_deg+=beam_resolution)
	{
		float beam_rad = static_cast<float>(beam_deg) * M_PI / 180.0;

		float x1_s = max_range *  sin(beam_rad);
		float y1_s = max_range * -cos(beam_rad);

		_H_s_p << 1, 0, x1_s + 2.50,
				  0, 1, y1_s,
				  0, 0, 1;

		Eigen::Matrix<float, 3, 3> _H_s_m_1(3,3);
		_H_s_m_1 = _H_m_r * _H_r_s * _H_s_p;

		int x1_m = convToGrid_x( _H_s_m_1(0, 2) );
		int y1_m = convToGrid_y( _H_s_m_1(1, 2) );
		int x0_m = convToGrid_x( particle_x );
		int y0_m = convToGrid_y( particle_y );
		/*int x_iter = x0_m;
		int y_iter = y0_m;*/

		float expected_range = 0.0;

		LineIterator lit(*_clearMapMat, Point(x0_m, y0_m), Point(x1_m, y1_m));


		for(int i = 0; i < lit.count; ++i, ++lit)
		{
			Point pt = lit.pos();
			if(pt.x == x1_m && pt.y == y1_m)
			{
				expected_range = max_range;
				break;
			}

			if(pt.x >= _map->max_x ||  //out of map range
					pt.y >= _map->max_y || 
					pt.x <= _map->min_x || 
					pt.y <= _map->min_y)
			{
				expected_range = euclid(x0_m, y0_m, pt.x, pt.y);
				break;
			}

			if( grid_data[pt.x][pt.y] <= obst_thres )
			{
				expected_range = euclid(x0_m, y0_m, pt.x, pt.y);
				break;
			}
		}


		/*while(true)
		{
			//printf("x_iter: %d, y_iter: %d ", x_iter, y_iter);
			if(x_iter == x1_m && y_iter == y1_m)
			{
				expected_range = max_range;
				break;
			}
			else if(x_iter >= _map->max_x ||  //out of map range
					y_iter >= _map->max_y || 
					x_iter <= _map->min_x || 
					y_iter <= _map->min_y)
			{
				expected_range = euclid(x0_m, y0_m, x_iter, y_iter);
				break;
			}
			else if( grid_data[x_iter][y_iter] <= obst_thres )
			{
				expected_range = euclid(x0_m, y0_m, x_iter, y_iter);
				break;
			}
			x_iter += (x_iter == x1_m) ? 0 : xInc_m;
			y_iter += (y_iter == y1_m) ? 0 : yInc_m;
		}*/
		int beam = static_cast<int>( beam_deg / beam_resolution );
		expected->at(beam) = expected_range;
	}

	//calculate the expected readings
	return expected;
}

//Helper function: uses sensor model and map to get weight of a particle
float pf::getParticleWeight( particle_type *particle, log_type *data ) const
{
	vector<float> *exp_readings = expectedReadings( particle );
	vector<float> *ws = new vector<float>( static_cast<int>(beam_fov / beam_resolution) );
	float *beamReadings = data->r;

	// iterate through each beam
#if PARALLELIZE == 1
#pragma omp parallel for
#endif
	for(int beam_deg = 0; beam_deg < beam_fov; beam_deg+=beam_resolution)
	{
		int beam = static_cast<int>(beam_deg / beam_resolution);
		
		float reading = beamReadings[beam];

		// for each beam get expected range reading
		float expected_reading = exp_readings->at(beam);

		// get weight from the actual reading for that beam
		_bmm->set_param(P_HIT_U, expected_reading);
		float p_reading = _bmm->getP(reading);
		ws->at(beam) = p_reading; 
	}

	// calculate total weight
	float tot = 0.0;
	for(float w : *ws)
	{
		tot += log(w); //TODO: sum of log
	}
	return tot;
}

// TESTER : ABHISHEK
// TEST : TEST AFTER VISUALIZER IS DONE
//Helper function: uses low variance sampling to resample based on particle weights
void pf::resampleW( vector< particle_type *> *resampledSt, vector<float> *Ws )
{
	float r = RandomFloat(0.0, (float) 1/_curSt->size());
	float c = Ws->at(0);
	int idx = 0;
	for (int i = 0; i < _curSt->size(); i++)
	{
		float new_weight = r + static_cast<float>(i) / _curSt->size();
		while (c < new_weight)
		{
			idx++;
			c += (*Ws)[idx]; 
		}
		particle_type *particle = (*_curSt)[idx];
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
	float tot_w = 0.0;
	for( particle_type *x_m : *_curSt )
	{
		//get P(Z|X,map) = weight of particle
		float weight_x_m = getParticleWeight( x_m, data );

		//store weight in vector
		weights->push_back(weight_x_m);
		tot_w += weight_x_m;
	}

	//normalize
	for(int i = 0; i < weights->size(); i++)
	{
		weights->at(i) /= tot_w;
	}

	//using low covariance sampling
	resampleW( _nxtSt, weights );
	delete _curSt;
	_curSt = new vector<particle_type*>(*_nxtSt);
	_nxtSt->clear();
	printf("Size: %lu\n", _curSt->size());
}

/* MOTION UPDATE */
// TESTER : ERIC
// TEST : PARSE MAP AND LOG FILE, ACCESS ODOMETRY DATA ITERATIVELY, CHECK STATE UPDATES
particle_type pf::motion_sample(particle_type u, particle_type sigma) const
{
	//sample x
	normal_distribution<float> x_norm(u.x, sigma.x);
	float dx_sample = x_norm(*_generator);
	//sample y
	normal_distribution<float> y_norm(u.y, sigma.y);
	float dy_sample = y_norm(*_generator);
	//sample bearing
	normal_distribution<float> bearing_norm(u.bearing, sigma.bearing);
	float db_sample = bearing_norm(*_generator);

	particle_type dP;
	dP.x = dx_sample;
	dP.y = dy_sample;
	dP.bearing = db_sample;

	return dP;
}

void pf::motion_update( log_type *data, log_type *prev_data)
{
	// detla x, y, bearing 
	particle_type dP_u;
	dP_u.x = data->x - prev_data->x;
	dP_u.y = data->y - prev_data->y;
	dP_u.bearing = data->theta - prev_data->theta;

	// variance 
	particle_type sigma; 
	sigma.x = 10;
	sigma.y = 10;
	sigma.bearing = 0.1;

	//TODO: update the next state for now
	//unique_lock<mutex> lock_curSt(_curStMutex);
	//unique_lock<mutex> lock_map(_mapMutex);
	
/*#if PARALLELIZE == 1
#pragma omp parallel for
#endif*/

	for (particle_type *particle : *_curSt)
	{
		particle_type *nxtParticle = new particle_type;

		// sample from guassian
		particle_type dP_guas = motion_sample(dP_u, sigma);
		particle_type dP; 


		// transformation matrix
		Eigen::Matrix<float, 3, 3> _H_m_r(3,3); 
		Eigen::Matrix<float, 3, 3> _H_r_b(3,3); 
		Eigen::Matrix<float, 3, 3> _H_b_p(3,3);
		Eigen::Matrix<float, 3, 3> _H_m_p(3,3);

		// map to robot
		_H_m_r << 1, 0, particle->x, 
				  0, 1, particle->y, 
				  0, 0, 1;

		// robot to bearing 
		_H_r_b << cos(particle->bearing), -sin(particle->bearing), 0, 
				  sin(particle->bearing),  cos(particle->bearing), 0, 
				  0, 0, 1;

		// bearing to new dx, dy
		_H_b_p << 1, 0, dP_guas.x, 
				  0, 1, dP_guas.y, 
				  0, 0, 1;

		// map to new dx, dy
		_H_m_p = _H_m_r * _H_r_b * _H_b_p;

		// dx, dy, dtheta
		dP.x = _H_m_p(0,2) - particle->x;
		dP.y = _H_m_p(1,2) - particle->y;
		dP.bearing = dP_guas.bearing;

		// apply update
		nxtParticle->x = particle->x + dP.x;
		nxtParticle->y = particle->y + dP.y;
		nxtParticle->bearing = particle->bearing + dP.bearing;

		// check max, min bounds
		if (nxtParticle->x < 0 ) 
			nxtParticle->x = 0;
		else if (nxtParticle->x > _map->max_x)
			nxtParticle->x = _map->max_x;

		if (nxtParticle->y < 0) 
			nxtParticle->y = 0;
		else if (nxtParticle->y > _map->max_y)
			nxtParticle->y = _map->max_y;

		if (nxtParticle->bearing < -M_PI)
			nxtParticle->bearing = M_PI + fmod(nxtParticle->bearing, M_PI);
		else if (nxtParticle->bearing > M_PI)
			nxtParticle->bearing = fmod(nxtParticle->bearing, M_PI) - M_PI;

		// push state
		_nxtSt->push_back(nxtParticle);

		/*printf("x: %f + %f = %f\n", particle->x, dP.x, nxtParticle->x);
		printf("y: %f + %f = %f\n", particle->y, dP.y, nxtParticle->y);
		printf("bearing: %f + %f = %f\n", particle->bearing, dP.bearing, nxtParticle->bearing);*/
	}
}

#if UI == 1
//TESTED
	void pf::draw_map() const
	{
		imshow( _mapWindowName, *_mapMat );
		waitKey(10);
	}

	void pf::erase_shapes()
	{
		delete _mapMat;
		_mapMat = new Mat(_map->size_x, _map->size_y, CV_32F);
		_clearMapMat->copyTo(*_mapMat);
	}
//TESTED
	void pf::draw_particles() const
	{
		for(particle_type *particle : *_curSt)
		{
			//printf("%f %f | ", particle->y, particle->x);
			circle( *_mapMat, Point(particle->y, particle->x), 2, Scalar(0,0,255), -1, 8 );
		}
		printf("\n");
		imshow( _mapWindowName, *_mapMat );
		waitKey(10);
	}

	void pf::draw_range(particle_type *particle, vector<float> *readings) const
	{
		circle( *_mapMat, Point(particle->y, particle->x), 20, Scalar(0,0,255), -1, 8 );

		//transformation matrices
		Eigen::Matrix<float, 3, 3> _H_m_r(3,3);
		Eigen::Matrix<float, 3, 3> _H_r_s(3,3);
		Eigen::Matrix<float, 3, 3> _H_s_p(3,3);

		_H_m_r << 1.0, 0.0, particle->x,
				  0.0, 1.0, particle->y,
				  0.0, 0.0, 1.0;

		_H_r_s << cos(particle->bearing), -sin(particle->bearing), 0,
				  sin(particle->bearing), cos(particle->bearing) , 0,
				  0					   , 0                     , 1;

		for(int beam = 0; beam < beam_fov; beam++)
		{
			float beam_deg = beam * beam_resolution;
			float beam_rad = beam_deg * M_PI / 180.0;

			float x1_s = readings->at(beam) *  sin(beam_rad);
			float y1_s = readings->at(beam) * -cos(beam_rad);

			_H_s_p << 1, 0, x1_s + 2.50,
					  0, 1, y1_s,
					  0, 0, 1;

			Eigen::Matrix<float, 3, 3> _H_s_m_1(3,3);
			_H_s_m_1 = _H_m_r * _H_r_s * _H_s_p;

			int x1_m = convToGrid_x( _H_s_m_1(0, 2) );
			int y1_m = convToGrid_y( _H_s_m_1(1, 2) );

			line(*_mapMat, 
				Point(particle->y, particle->x), 
				Point(y1_m, x1_m),
				Scalar(0,255,0));
		}
		imshow( _mapWindowName, *_mapMat );
		waitKey(10);
	}
#endif