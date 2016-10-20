//*****************************************************************
//  MotionModel.cpp
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This source file contains the motion model class definitions.
//*****************************************************************

#include "MotionModel.h"
#include "types.h"

// TODO: add curr_motion_update, prev_motion_update, and new list of particles
void MotionModel::MotionUpdate(log_type *data) 
{
  	// find difference between previous motion update and current update
  	// be sensitive to theta roll over
  	std::list<float>::iterator curr_it = curr_motion_update.begin();
  	std::list<float>::iterator prev_it = prev_motion_update.begin();

  	float delta_x = *curr_it - *prev_it; 
  	curr_it++;
  	prev_it++;

  	float delta_y = *curr_it - *prev_it; 
  	curr_it++;
  	prev_it++;

  	float delta_theta = *curr_it - *prev_it; 

  	if (delta_theta >  2 || delta_theta < 2)  // catch roll over
  	{
  		if ( *prev_it > *curr_it) // from positive to negative 
  		{
  			delta_theta = (pi - *prev_it) + (pi + *curr_it);
  		}
  		else  // from negative to positive
  		{
  			delta_theta = (pi + *prev_it) + (pi - *curr_it);
  		}
  	}

  	// apply delta x,y,theta to each particle 
  	std::list<std::list<float>> updated_data; // list of info: x,y,theta
  	std::list<std::list<float>>::iterator it_particles = updated_data.begin(); // list of particles

	for (//TODO: iterate over particles in data  ..... xxx particle : _curSt)
	{
		// for each particle apply delta_x, delta_y, delta_theta
		// be mindful of theta roll over
		std::list<float>::iterator it_particle_data = particle.begin(); // pointer to start of nested list

		float new_x = *it_particle_data + delta_x; 
		it_particle_data++;

		float new_y = *it_particle_data + delta_y;
		it_particle_data++;

		float new_theta = *it_particle_data + delta_theta;

		// catch roll over
		if (new_theta > pi)
		{
			new_theta -= 2*pi;  
		}
		else if (new_theta < -pi) 
		{
			new_theta += 2*pi;
		}

		// create list of parameters (x,y,theta)
		std::list<float> single_particle_list = {new_x, new_y, new_theta};

		// insert particle info list into list of particles
		updated_data.insert (it_particles, single_particle_list); 
	}
}