/*
    SerialPort Class

    Copyright 2016-2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

class SerialPort {
	public:
		char* COMport;    // Number of COM port
		speed_t baudrate;  // Port baud rate
		int portDescr;     // Port descriptor

	public:
		SerialPort();
		SerialPort(char* port, speed_t baud);
		~SerialPort();

		void setCOM(char* port);
		void setBAUD(speed_t baud);
		int connect();
		int disconnect();
		int readSerial(unsigned char* buf, int num_byte);
		int writeSerial(unsigned char* buf, int num_byte);
};
