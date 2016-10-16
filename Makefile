all:
	g++ -std=c++11 ./src/bee-map.cpp -I ./include/ -o ./bin/map_read `pkg-config opencv --cflags --libs`

clean:
	$(RM) ./bin/*