/*
    Robotous RFT sensor Lib

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

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <bitset>
#include <sys/time.h>

#include "RFT_IF_PACKET_Rev1.0.h"
#include "serial_port.h"

using namespace std;

bool robotous_read_model_name(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_serial_number(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_firmware_version(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_filter_setting(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_force_once(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_read_force(SerialPort USBport, CRT_RFT_IF_PACKET& packet, bool save);
void robotous_read_current_sensor_settings(SerialPort USBport, CRT_RFT_IF_PACKET& packet);

bool robotous_set_baud_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char buad_rate);
bool robotous_set_filter(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char filter_type, unsigned char cutoff_frequency);
void robotous_set_stop_force_out(SerialPort USBport, CRT_RFT_IF_PACKET& packet);
bool robotous_set_output_rate(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char output_rate);
void robotous_set_bias(SerialPort USBport, CRT_RFT_IF_PACKET& packet, unsigned char bias_is_on);


