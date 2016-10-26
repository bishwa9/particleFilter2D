//*****************************************************************
//  main.cpp
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This is the main source file.
//**************************************************************

#include <iostream>

//#define MAP_UNIT_TEST
//#define BMM_UNIT_TEST
//#define VISUALIZE_UNIT_TEST
#define MM_UNIT_TEST
//#define MM_OR_SENSOR

#ifdef VISUALIZE_UNIT_TEST
	#include "Parser.h"
	#include "pf.h"
#endif

#ifdef MAP_UNIT_TEST
	#include "Parser.h"
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	using namespace cv;
#endif

#ifdef BMM_UNIT_TEST
	#include <fstream>
	#include "bmm.h"

	using namespace std;
#endif

#ifdef MM_UNIT_TEST
	#include "Parser.h"
	#include "pf.h"

	using namespace std;
	using namespace Eigen;
#endif

#ifdef MM_OR_SENSOR
	#include "Parser.h"
	#include "pf.h"
	#include <fstream>
	#include "bmm.h"

	using namespace std;
	using namespace Eigen;
#endif

// Unit test
#ifdef MAP_UNIT_TEST

int main(int argc, char **argv)
{
	if( argc == 2 ) {
		Parser *parse = new Parser();
		
		string map_name(argv[1]);
		parse->read_beesoft_map(map_name.c_str());

		// Check Map information
		cout << "Map information: " << endl;
		cout << "Resolution " << parse->_my_map->resolution << " SizeX " << parse->_my_map->size_x << " SizeY " << parse->_my_map->size_y << endl;
		cout << "Min_Max X " << parse->_my_map->min_x << " " << parse->_my_map->max_x << endl;
		cout << "Min_Max Y " << parse->_my_map->min_y << " " << parse->_my_map->max_y << endl;
		cout << "Offset X " << parse->_my_map->offset_x << " Offset Y " << parse->_my_map->offset_y << endl;

		// visualize
		Mat image = Mat::zeros( parse->_my_map->size_x, parse->_my_map->size_y, CV_32FC1 );

		cout << "Image Property" << endl;
		cout << "Row: " << image.rows << " Col: " << image.cols << endl;
		cout << "Step: " << image.step << " Dim: " << image.dims << endl;
		cout << "ElemSize: " << image.elemSize() << " Depth: " << image.depth() << endl;
		cout << "Channels: " << image.channels() << endl;

		// unsigned char *imgMat = (unsigned char*)(image.data);
		for (unsigned int i = 0; i < image.rows; i++)
			for (unsigned int j = 0; j < image.cols; j++){
				if (parse->_my_map->cells[i][j] > 0.0)
					image.at<float>(i, j) = parse->_my_map->cells[i][j]; 
			} 
	  	
	  	string filename = "./data/log/robotdata1.log";
		parse->read_log_data(filename.c_str());

	  	imshow("Image",image);
		waitKey( 0 );

	} else {
		cout << "ERROR: Enter dat file name" << endl;
	}
	
	return 0;
}

#endif

#ifdef BMM_UNIT_TEST
int main()
{
	ofstream p_file("dist.txt");
	beamMeasurementModel bmm;
	int resolution = 1;
	float maxRange = 0.0;

	if( !bmm.get_param(MAX_RANGE, &maxRange) )
	{
		printf("ERROR!\n");
		return 0;
	}
	
	for(float x_t = 0; x_t < maxRange; x_t+=resolution)
	{
		float prob = bmm.getP( x_t );
		p_file << prob << endl;
	}
	p_file.close();
	return 0;
}
#endif

#ifdef VISUALIZE_UNIT_TEST

int main(int argc, char **argv)
{
	if( argc == 3 ) {
		Parser *parse = new Parser();
		
		string map_name(argv[1]);
		parse->read_beesoft_map(map_name.c_str());

		// Check Map information
		cout << "Map information: " << endl;
		cout << "Resolution " << parse->_my_map->resolution << " SizeX " << parse->_my_map->size_x << " SizeY " << parse->_my_map->size_y << endl;
		cout << "Min_Max X " << parse->_my_map->min_x << " " << parse->_my_map->max_x << endl;
		cout << "Min_Max Y " << parse->_my_map->min_y << " " << parse->_my_map->max_y << endl;
		cout << "Offset X " << parse->_my_map->offset_x << " Offset Y " << parse->_my_map->offset_y << endl;

	  	string filename(argv[2]);
		parse->read_log_data(filename.c_str());
		pf filter(parse->_my_map, 1000, 0);

		for(int i = 0; i < parse->_logData->size(); i++)
		{
			log_type* dat = parse->_logData->at(i);

			if(dat->type == L_DATA)
			{
				filter.sensor_update(dat);
				filter.erase_shapes();
				filter.draw_particles();
			}	
		}
	} else {
		cout << "ERROR: Enter dat file name" << endl;
	}
	
	return 0;
}

#endif

#ifdef MM_UNIT_TEST

int main(int argc, char **argv)
{
	if( argc == 3 ) {
		Parser *parse = new Parser();
		
		string map_name(argv[1]);
		parse->read_beesoft_map(map_name.c_str());

		// Check Map information
		cout << "Map information: " << endl;
		cout << "Resolution " << parse->_my_map->resolution << " SizeX " << parse->_my_map->size_x << " SizeY ";
		cout << parse->_my_map->size_y << endl;
		cout << "Min_Max X " << parse->_my_map->min_x << " " << parse->_my_map->max_x << endl;
		cout << "Min_Max Y " << parse->_my_map->min_y << " " << parse->_my_map->max_y << endl;
		cout << "Offset X " << parse->_my_map->offset_x << " Offset Y " << parse->_my_map->offset_y << endl;
	  	
	  	string filename(argv[2]);
		parse->read_log_data(filename.c_str());  //output of log 

		pf pf(parse->_my_map, 1, 0); 

		int prev_ptr, curr_ptr;
		for (int i = 0; i < parse->_logData->size() - 1; i++) {
			/*while (parse->_logData[0][i]->type == L_DATA) {
				i++;
				if (i == parse->_logData->size())
					break;
			}
			if (i == parse->_logData->size())
				break;

			prev_ptr = i;
			curr_ptr = i+1;
			if (curr_ptr == parse->_logData->size())
					break;
			while (parse->_logData[0][curr_ptr]->type == L_DATA) {
				curr_ptr++;
				if (curr_ptr == parse->_logData->size())
					break;
			}
			if (curr_ptr >= parse->_logData->size())
				break;*/
			prev_ptr = i;
			curr_ptr = i+ 1;

			printf("\n%d %d\n", prev_ptr, curr_ptr);
			pf.motion_update(parse->_logData->at(curr_ptr), parse->_logData->at(prev_ptr));
			printf("Current State: %f %f %f\n", pf._curSt->at(0)->x, pf._curSt->at(0)->y, pf._curSt->at(0)->bearing);
			printf("Next State: %f %f %f\n", pf._nxtSt->at(0)->x, pf._nxtSt->at(0)->y, pf._nxtSt->at(0)->bearing);
			delete pf._curSt;
			pf._curSt = new vector<particle_type*>(*pf._nxtSt);
			pf._nxtSt->clear();

			// pf.erase_shapes();
			pf.draw_particles();
		}

	} else {
		cout << "ERROR: Enter dat file name" << endl;
	}
	
	return 0;
}

#endif

#ifdef MM_OR_SENSOR
/*int main(int argc, char **argv)
{

}*/

#endif 
