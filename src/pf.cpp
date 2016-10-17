#include "pf.h"

/* CONSTRUCTOR */
pf::pf(map_type *map):_map( map ), _curSt( new list< particle_type >() ), _nxtSt( new list< particle_type >() )
{
	//Initialize the state
	
}

/* DESTRUCTOR */
pf::~pf()
{
	_map = NULL; // map will be destroyed by parser

	delete _curSt;
}

/* SENSOR UPDATE */
void pf::sensor_update( log_type *data )
{

}

/* MOTION UPDATE */
void pf::motion_update( log_type *data )
{
	//update the next state for now
}