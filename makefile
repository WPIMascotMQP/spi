all: spi.o

spi.o: spi.cpp
	g++ spi.cpp -lwiringPi -o spi