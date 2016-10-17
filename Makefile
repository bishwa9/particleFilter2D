all:
	g++ -std=c++11 ./src/Parser.cpp ./src/main.cpp -I ./include/ -o ./bin/main `pkg-config opencv --cflags --libs`

clean:
	$(RM) ./bin/*