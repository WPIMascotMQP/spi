all: spi.o

spi.o: spi.cpp
	g++ -c -g -Wall --std=c++11 spi.cpp -lwiringPi -o spi
