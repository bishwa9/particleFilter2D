all:
	g++ -std=c++11 `pkg-config --cflags --libs opencv` ./src/bee-map.cpp -I ./include/ -o ./bin/map_read

clean:
	$(RM) ./bin/*