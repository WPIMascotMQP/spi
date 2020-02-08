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

#include <bitset>
#include <iostream>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <wiringPiSPI.h>
#include <unistd.h>

using namespace std;

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 0;

int main()
{
    int fd, result;
    unsigned char buffer[5];
    //unsigned char bufferRecieve[100];

    cout << "Initializing" << endl ;

    // Configure the interface.
    // CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 9600);

    cout << "Init result: " << fd << endl;

    sleep(1);

    // Do a one-hot bit selection for each field of the display
    // It displays gibberish, but tells us that we're correctly addressing all
    // of the segments.
    //for(int i = 1; i <= 0x7f; i <<= 1) {

    int i = 1;
    while(1) {
        // the decimals, colon and apostrophe dots
        memset(buffer, 0, 5);
        buffer[0] = 'a';
        buffer[1] = 'b';
        buffer[2] = 'c';
        buffer[3] = 'd';
        buffer[4] = '\0';
        std::bitset<8> a(buffer[0]);
        std::bitset<8> b(buffer[1]);
        std::bitset<8> c(buffer[2]);
        std::bitset<8> d(buffer[3]);
        cout << "Sending:  " << a << b << c << d << endl;

        result = wiringPiSPIDataRW(CHANNEL, buffer, 5);
        cout << "Received: "  << buffer << endl;

        // Pause so we can see them
        //sleep(1);
    }
    // clear display again
    buffer[0] = 0x76;
    wiringPiSPIDataRW(CHANNEL, buffer, 1);
}
