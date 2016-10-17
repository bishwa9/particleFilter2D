//*****************************************************************
//  main.cpp
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This is the main source file.
//**************************************************************

#include "Parser.h"

#define MAP_UNIT_TEST

#ifdef MAP_UNIT_TEST
	#include <opencv2/core/core.hpp>
	#include <opencv2/highgui/highgui.hpp>
	using namespace cv;
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