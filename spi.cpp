/******************************************************************************
spitest.cpp
Raspberry Pi SPI interface demo
Byron Jacquot @ SparkFun Electronics>
4/2/2014
https://github.com/sparkfun/Pi_Wedge

A brief demonstration of the Raspberry Pi SPI interface, using the SparkFun
Pi Wedge breakout board.

Resources:

This example makes use of the Wiring Pi library, which streamlines the interface
to the the I/O pins on the Raspberry Pi, providing an API that is similar to the
Arduino.  You can learn about installing Wiring Pi here:
http://wiringpi.com/download-and-install/

The wiringPi SPI API is documented here:
https://projects.drogon.net/raspberry-pi/wiringpi/spi-library/

The init call returns a standard file descriptor.  More detailed configuration
of the interface can be performed using ioctl calls on that descriptor.
See the wiringPi SPI implementation (wiringPi/wiringPiSPI.c) for some examples.
Parameters configurable with ioctl are documented here:
http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/spi/spidev

Hardware connections:

This file interfaces with the SparkFun Serial 7 Segment display:
https://www.sparkfun.com/products/11629

The board was connected as follows:
(Raspberry Pi)(Serial 7 Segment)
GND  -> GND
3.3V -> Vcc
CE1  -> SS (Shift Select)
SCK  -> SCK
MOSI -> SDI
MISO -> SDO

To build this file, I use the command:
>  g++ spitest.cpp -lwiringPi

Then to run it, first the spi kernel module needs to be loaded.  This can be
done using the GPIO utility.
> gpio load spi
> ./a.out

This test uses the single-segment mode of the 7 segment display.  It shifts a
bit through the display characters, lighting a single character of each at a
time.

Development environment specifics:
Tested on Raspberry Pi V2 hardware, running Raspbian.
Building with GCC 4.6.3 (Debian 4.6.3-14+rpi1)

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include <cstddef>
#include <cstring>
#include <bitset>
#include <iostream>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <wiringPiSPI.h>
#include <unistd.h>

#include <string>

#define BUFFER_SIZE 64
#define PATTERN_LEN 10

// Immediate variable type between float and long
typedef long float_i;

// Flags for the flag byte of each command
enum flags {
	FLAG1 = 0x80, // 10000000
	FLAG2 = 0x40, // 01000000
	FLAG3 = 0x20, // 00100000
	FLAG4 = 0x10, // 00010000
	FLAG5 = 0x08, // 00001000
	FLAG6 = 0x04, // 00000100
	FLAG7 = 0x02, // 00000010
	FLAG8 = 0x01, // 00000001
};

// Holds the hex representation of each command
enum cmds {
	PATTERN = 0xFF,
	MOTORPOSITION = 0x01,

};

// Holds the byte index for different parts of the commmand
enum byte_index {
	flag_byte = 0,
	cmd_byte = 1,
};

void handleCommand(unsigned char* buffer);
bool findCommand(unsigned char* buffer);

std::string getStringHex(unsigned char* buffer, size_t length);
size_t overwriteBytes(unsigned char* buffer, size_t byte_start, unsigned char* buf, size_t inc_start, size_t byte_inc);
size_t encodePattern(unsigned char* buffer, size_t byte_start);
size_t encodeInt16(unsigned char* buffer, size_t byte_start, int16_t num);
size_t encodeInt32(unsigned char* buffer, size_t byte_start, int32_t num);
size_t encodeFloat(unsigned char* buffer, size_t byte_start, float num);

int16_t decodeInt16(unsigned char* buffer, size_t byte_start);
int32_t decodeInt32(unsigned char* buffer, size_t byte_start);
float decodeFloat(unsigned char* buffer, size_t byte_start);

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 0;

void gen_random(unsigned char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}

int main() {
    int fd, result;
    unsigned char buffer[BUFFER_SIZE];
    //unsigned char bufferRecieve[100];

    std::cout << "Initializing" << std::endl ;

    // Configure the interface.
    // CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 9600);

    std::cout << "Init result: " << fd << std::endl;

    sleep(1);

    // Do a one-hot bit selection for each field of the display
    // It displays gibberish, but tells us that we're correctly addressing all
    // of the segments.
    //for(int i = 1; i <= 0x7f; i <<= 1) {

    int i = 1;
    while(1) {
        std::string input;
        // the decimals, colon and apostrophe dots
        std::getline(std::cin, input);
        std::string delimiter = " ";
        std::string token1 = input.substr(0, input.find(delimiter));
        std::string token2 = input.substr(input.find(delimiter) + 1, input.length() - 1);
        int16_t index = (int16_t) std::stoi(token1);
        float radians = std::stof(token2);
        std::cout << token1 << " " << index << "|" << token2 << " " << radians << std::endl;

        memset(buffer, 0, BUFFER_SIZE);
        unsigned char* buffer = new unsigned char[BUFFER_SIZE];
        size_t current_byte = 0;

        current_byte += encodePattern(buffer, current_byte);
        buffer[current_byte++] = 0x00; // Flag Byte
        buffer[current_byte++] = MOTORPOSITION;
        current_byte += encodeInt16(buffer, current_byte, index);
        current_byte += encodeFloat(buffer, current_byte, radians);
        current_byte += encodePattern(buffer, current_byte);
        unsigned char original[BUFFER_SIZE];
        overwriteBytes(original, 0, buffer, 0, BUFFER_SIZE);
        std::cout << "Sending:  " << getStringHex(buffer, current_byte) << std::endl;

        result = wiringPiSPIDataRW(CHANNEL, buffer, BUFFER_SIZE);
        while(!findCommand(buffer) || memcmp(buffer, original, BUFFER_SIZE)) {
			result = wiringPiSPIDataRW(CHANNEL, buffer, BUFFER_SIZE);
        }

        std::cout << "Received: "  << getStringHex(buffer, BUFFER_SIZE) << std::endl;
        handleCommand(buffer);

        // Pause so we can see them
        //sleep(1);
    }
    // clear display again
    buffer[0] = 0x76;
    wiringPiSPIDataRW(CHANNEL, buffer, 1);
}

/**
 Handles the command in the given buffer
 @param buffer The buffer where the command is (must start at beginning of buffer)
 */
void handleCommand(unsigned char* buffer) {
  printf("Received Command: %s\n", buffer);
	if(buffer[cmd_byte] == MOTORPOSITION) {
		// If command is the current motor position
		size_t current_byte = 2;
		int index = (int) decodeInt16(buffer, current_byte);
		current_byte += 2;
		double radians = (double) decodeFloat(buffer, current_byte);
		printf("Received Motor Radians: %d %f\n", index, radians);
	}
}

/**
 Finds a valid command in the given buffer
 If a command was found, writies that command into the beginning of the buffer
 @param buffer
 @return Whether a command was found or not
 */
bool findCommand(unsigned char* buffer) {
	unsigned char buf[BUFFER_SIZE];
	overwriteBytes(buf, 0, buffer, 0, BUFFER_SIZE);

	size_t patterns[2];
	size_t lengths[2];
	size_t start_bypte = 0;
	size_t counter = 0;
  	size_t current = 0;
	bool on_one = false;

	// Parse throught each byte in the buffer
	for(size_t i = 0; i < BUFFER_SIZE; i++) {
		// If found pattern byte
		if(buffer[i] == PATTERN) {
			// If not on pattern yet
			if(!on_one) {
				on_one = true;
				start_bypte = i;
				counter++;
			// If on pattern add to counter
			} else if(on_one) {
				counter++;
			}
		} else {
			// If on one but less length or longer than pattern length
			if(on_one && (counter < PATTERN_LEN || counter > PATTERN_LEN)) {
				start_bypte = 0;
				counter = 0;
				on_one = false;
			}
			// If on one which is exactly  pattern length
			if(on_one && counter == PATTERN_LEN) {
				patterns[current] = start_bypte;
				lengths[current] = counter;
				start_bypte = 0;
				counter = 0;
				on_one = false;
		        current++;
		        if(current > 2) {
		          return false;
		        }
			}
		}
	}
	if(current == 2) {
		overwriteBytes(buffer, 0, buf, patterns[0] + lengths[0],
			patterns[1] - patterns[0] - lengths[0]);
		return true;
	}
	return false;
}
/**
 Gets the std::string represention of the given buffer
 @param buffer The given buffer to represent
 @param length The length of the buffer to represent
 @return The std::string hex value
 */
std::string getStringHex(unsigned char* buffer, size_t length) {
	char hex[length * 2 + 1];
	for(size_t i = 0; i < length; i++) {
   		sprintf(hex + 2 * i, "%.2x", buffer[i]);
	}
	hex[length * 2 + 1] = '\0';

	char output[length * 2 + 1];
	sprintf(output, "0x%s", hex);
	std::string s(output);
	return s;
}

/**
 Overwrites the bytes in the given buffer with bytes from the other given buffer
 @param buffer The buffer to write into
 @param byte_start The starting location to write to
 @param buf The buffer to write from
 @param int_start The starting location to write from
 @param byte_inc The number of bytes to write
 @return The number of bytes written
*/
size_t overwriteBytes(unsigned char* buffer, size_t byte_start,
  unsigned char* buf, size_t inc_start, size_t byte_inc) {
	for(size_t i = 0; i < byte_inc; i++) {
		buffer[byte_start + i] = buf[inc_start + i];
	}
	return byte_inc;
}

/**
 Writes the pattern into the given buffer at the given position
 @param buffer The buffer to write into
 @param byte_start The starting byte to writing into
 @return The number of bytes written
 */
size_t encodePattern(unsigned char* buffer, size_t byte_start) {
	buffer[byte_start + 0] = PATTERN;
	buffer[byte_start + 1] = PATTERN;
	buffer[byte_start + 2] = PATTERN;
	buffer[byte_start + 3] = PATTERN;
	buffer[byte_start + 4] = PATTERN;
	buffer[byte_start + 5] = PATTERN;
	buffer[byte_start + 6] = PATTERN;
	buffer[byte_start + 7] = PATTERN;
	buffer[byte_start + 8] = PATTERN;
	buffer[byte_start + 9] = PATTERN;
	return PATTERN_LEN;
}

/**
 Encodes a 16 bit int into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 16 bit int to encode
 @return The number of bytes encoded
 */
size_t encodeInt16(unsigned char* buffer, size_t byte_start, int16_t num) {
	buffer[byte_start + 0] = (num >> 8) & 0xFF;
	buffer[byte_start + 1] = num & 0xFF;
	return sizeof(int16_t);
}

/**
 Encodes a 32 bit int into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 32 bit int to encode
 @return The number of bytes encoded
 */
size_t encodeInt32(unsigned char* buffer, size_t byte_start, int32_t num) {
	buffer[byte_start + 0] = (num >> 24) & 0xFF;
	buffer[byte_start + 1] = (num >> 16) & 0xFF;
	buffer[byte_start + 2] = (num >> 8) & 0xFF;
	buffer[byte_start + 3] = num & 0xFF;
	return sizeof(int32_t);
}

/**
 Encodes a 32 bit float into the given buffer
 @param buffer The buffer to write into
 @param byte_start The starting byte to write into
 @param num The 32 bit float to encode
 @return The number of bytes encoded
 */
size_t encodeFloat(unsigned char* buffer, size_t byte_start, float num) {
	float_i number = *(float_i*) &num;
	buffer[byte_start + 0] = (number >> 24) & 0xFF;
	buffer[byte_start + 1] = (number >> 16) & 0xFF;
	buffer[byte_start + 2] = (number >> 8) & 0xFF;
	buffer[byte_start + 3] = number & 0xFF;
	return sizeof(float);
}

/**
 Decodes a 16 bit int from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The 16 bit int decoded
 */
int16_t decodeInt16(unsigned char* buffer, size_t byte_start) {
	unsigned char buf[2];
	buf[0] = buffer[byte_start + 1];
	buf[1] = buffer[byte_start + 0];
	int16_t number = *(int16_t*) &buf;

	return number;
}

/**
 Decodes a 32 bit int from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The 32 bit int decoded
 */
int32_t decodeInt32(unsigned char* buffer, size_t byte_start) {
	unsigned char buf[4];
	buf[0] = buffer[byte_start + 3];
	buf[1] = buffer[byte_start + 2];
	buf[2] = buffer[byte_start + 1];
	buf[3] = buffer[byte_start + 0];
	int32_t number = *(int32_t*) &buf;

	return number;
}

/**
 Decodes a float from given buffer
 @param buffer The buffer to read from
 @param byte_start The starting byte to read from
 @return The float decoded
 */
float decodeFloat(unsigned char* buffer, size_t byte_start) {
	unsigned char buf[4];
	buf[0] = buffer[byte_start + 3];
	buf[1] = buffer[byte_start + 2];
	buf[2] = buffer[byte_start + 1];
	buf[3] = buffer[byte_start + 0];
	float number = *(float*) &buf;

	return number;
}
