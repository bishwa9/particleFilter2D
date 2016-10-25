#ifndef VIEWER_H
#define VIEWER_H

#if UI == 1

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "pf.h"

class viewer
{
private:
	pf *_filter;
	bool _keepRunning;

public:
	viewer(pf *filter);
	~viewer();

	void set_running(bool running_);
	bool get_running() const;

	void draw();
};

#endif

#endif