//*****************************************************************
//  Parser.cpp
//  16831 Statistical Techniques, Fall 2016
//  Project 3: Robot Localization
//
//  Created by Abhishek Bhatia, Bishwamoy Sinha Roy, Eric Markvicka.
//
//  This source file contains the parser class definitions.
//*****************************************************************

#include "Parser.h"

Parser::Parser() {
	_my_map = new map_type();
	_logData = new vector<log_type *>();
}

void Parser::new_hornetsoft_map(int size_x, int size_y)
{
	int i;
	_my_map->cells = (float **)calloc(size_x, sizeof(float *));
	for(i = 0; i < size_x; i++)
		_my_map->cells[i] = (float *)calloc(size_y, sizeof(float));
}

int Parser::read_beesoft_map(const char *mapName)
{
	int x, y, count;
	float temp;
	char line[256];
	FILE *fp;

	if((fp = fopen(mapName, "rt")) == NULL) {
		fprintf(stderr, "# Could not open file %s\n", mapName);
		return -1;
	}
	fprintf(stderr, "# Reading map: %s\n", mapName);
	while((fgets(line, 256, fp) != NULL)
		  && (strncmp("global_map[0]", line , 13) != 0)) {
		if(strncmp(line, "robot_specifications->resolution", 32) == 0)
			if(sscanf(&line[32], "%d", &(_my_map->resolution)) != 0)
				#ifdef DEBUG
				printf("# Map resolution: %d cm\n", _my_map->resolution);
				#endif		
		if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
			if(sscanf(&line[35], "%g", &(_my_map->offset_x)) != 0) {
				_my_map->offset_x = _my_map->offset_x;
				#ifdef DEBUG
				printf("# Map offsetX: %g cm\n", _my_map->offset_x);
				#endif			
			}
		if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) {
			if (sscanf(&line[35], "%g", &(_my_map->offset_y)) != 0) {
				_my_map->offset_y = _my_map->offset_y;
				#ifdef DEBUG
				printf("# Map offsetY: %g cm\n", _my_map->offset_y);
				#endif			
			}
		}
	}
	
	if(sscanf(line,"global_map[0]: %d %d", &_my_map->size_y, &_my_map->size_x) != 2) {
		fprintf(stderr, "ERROR: corrupted file %s\n", mapName);
		fclose(fp);
		return -1;
	}
	#ifdef DEBUG
	printf("# Map size: %d %d\n", _my_map->size_x, _my_map->size_y);
	#endif	
	new_hornetsoft_map(_my_map->size_x, _my_map->size_y);
	
	_my_map->min_x = _my_map->size_x;
	_my_map->max_x = 0;
	_my_map->min_y = _my_map->size_y;
	_my_map->max_y = 0;
	count = 0;
	for(x = 0; x < _my_map->size_x; x++)
		for(y = 0; y < _my_map->size_y; y++, count++) {
			fscanf(fp,"%e", &temp);
			if(temp < 0.0)
				_my_map->cells[x][y] = -1;
			else {
				if(x < _my_map->min_x)
					_my_map->min_x = x;
				else if(x > _my_map->max_x)
					_my_map->max_x = x;
				if(y < _my_map->min_y)
					_my_map->min_y = y;
				else if(y > _my_map->max_y)
					_my_map->max_y = y;
				_my_map->cells[x][y] = temp;	   
			}
			#ifdef DEBUG
			printf("%e ",_my_map->cells[x][y]);
			#endif		
		}
	#ifdef DEBUG
	printf("\n");
	#endif
	fclose(fp);
	return 0;
}

int Parser::read_log_data(const char *logName){
	ifstream logFile(logName); 
	if (!logFile.is_open()){
		fprintf(stderr, "Couldn't open file %s.\n", logName);
		return -1;
	}
	string line;

	_logData->clear();
	fprintf(stderr, "# Reading log: %s\n", logName);
	while (getline(logFile, line)){
		log_type *data = new log_type();
		char type;
		istringstream ss(line);
		ss >> type;
		if (line[0] == 'O'){
			data->type = O_DATA;
			ss >> data->x >> data->y;
			ss >> data->theta >> data->ts;
		}
		else if (line[0] == 'L'){
			data->r = new float[RANGE_LEN];
			data->type = L_DATA;
			ss >> data->x >> data->y >> data->theta;
			ss >> data->xl >> data->yl >> data->thetal;
			for (unsigned int i = 0; i < RANGE_LEN; i++)
				ss >> data->r[i];
			ss >> data->ts;
		}
		else{
			fprintf(stderr, "Wrong data format at line %d\n", (int)_logData->size());
			return -1;
		}
		_logData->push_back(data);
		#ifdef DEBUG
		cout << _logData->back()->type << " " << _logData->back()->x << " ";
		cout << _logData->back()->y << " " << _logData->back()->ts << endl;
		#endif
	}
	logFile.close();
	return 1;
}

//*****************************************************************
// End of File
//*****************************************************************