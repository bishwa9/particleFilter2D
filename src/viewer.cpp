#if UI == 1

#include "viewer.h"

viewer::viewer(pf *filter):_keepRunning(true), _filter(filter)
{}

viewer::~viewer()
{
	//whoever created the pf object is responsible of destroying it
}

void viewer::set_running(bool running_)
{
	_keepRunning = running_;
}

bool viewer::get_running() const
{
	return _keepRunning;
}

void viewer::draw()
{
	while(!_keepRunning)
	{
		//lock pf
		
	}
}

#endif