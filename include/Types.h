//*****************************************************************
//  Types.h
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This header file contains the structs definitions to store map data and robot (laser and odometry) data.
//*****************************************************************

#ifndef TYPES_H
#define TYPES_H

#define RANGE_LEN 180
#define PI 3.1415926

typedef struct {
	int resolution, size_x, size_y;
	float offset_x, offset_y;
	int min_x, max_x, min_y, max_y;
	float **cells;
} map_type;

enum LogType
{
	L_DATA = 0, O_DATA = 1
};

typedef struct {
	LogType type;
	float x, y, xl, yl;
	float theta, thetal;
	double ts;
	float* r; 
} log_type;

#endif