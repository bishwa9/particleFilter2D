OPENCV=`pkg-config opencv --cflags --libs`
FLAGS=-std=c++11
OMP_FLAG=-fopenmp
SRC_FILES=./src/Parser.cpp ./src/pf.cpp ./src/bmm.cpp ./src/main.cpp
INCLUDE_FILES=./include/

all:
	g++ $(FLAGS) $(SRC_FILES) -I $(INCLUDE_FILES) -I/usr/local/include/eigen3/ -o ./bin/main $(OPENCV)

parallel:
	g++ $(FLAGS) $(OMP_FLAG) $(SRC_FILES) -I $(INCLUDE_FILES) -o ./bin/main $(OPENCV)

clean:
	$(RM) ./bin/*