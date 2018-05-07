/*
    ROS node to read finger's voltages from serial port

    Copyright 2018 Università della Campania Luigi Vanvitelli

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


#include "serial/serial.h"
#include <ros/ros.h>

#include "wsg_50_common_sun/Tactile.h"
#include <Helper1.0.h>


using namespace std;

//*******GLOBAL ROS VARS******//
ros::NodeHandle* n;
//----------------------------//


//==============MAIN================//

int main(int argc, char *argv[]){

    ros::init(argc,argv,"read_tactile_serial");

    n = new ros::NodeHandle("~");

    /**** CHECK PARAMS ****/
    string serial_port = string("");
    n->param("serial_port" , serial_port, string("/dev/ttyUSB0") );
    unsigned long baud = 0;
    string str_baud = string("");
    n->param("baud_rate" , str_baud, string("500000") );
    sscanf(str_baud.c_str(), "%lu", &baud);
    int serialTimeout = 0;
    n->param("serial_timeout" , serialTimeout, 1000 );
    int voltages_count;
    n->param("voltages_count" , voltages_count, 25 );
    string frame_id = string("");
    n->param("frame_id" , frame_id, string("fingertip0") );
    string topic_name = string("");
    n->param("topic" , topic_name, string("/tactile") );

    /*** INIT SERIAL ****/	
    serial::Serial my_serial(serial_port, baud, serial::Timeout::simpleTimeout(serialTimeout));

   /*** CHECK ***/
   if(!my_serial.isOpen()){
   	cout << BOLDRED << "ERROR - SERIAL PORT " << BOLDYELLOW << serial_port << BOLDRED << " is not open!" << CRESET <<endl;
   	exit(-1);
   }
   cout << BOLDGREEN << "SERIAL PORT " << BOLDYELLOW << serial_port << BOLDGREEN << " OPEN - OK" << CRESET << endl;
   
   // ==== Tactile msg ====
   wsg_50_common_sun::Tactile finger_voltages;
	finger_voltages.voltages.data.resize(voltages_count);
	//finger_voltages.header.stamp = myTime; //ros::Time::now(); //Please change to a time saved as soon as we got data
   finger_voltages.header.frame_id = frame_id;
	finger_voltages.voltages.layout.dim.resize(1);
   finger_voltages.voltages.layout.dim[0].label="voltage";
   finger_voltages.voltages.layout.dim[0].size=voltages_count;
   finger_voltages.voltages.layout.dim[0].stride=1;
   finger_voltages.voltages.layout.data_offset=0;
   
   // ======= PUBLISHER
   ros::Publisher pubTactile = n->advertise<wsg_50_common_sun::Tactile>( topic_name ,1);
   
   //init buffers
   const int dim_buffer = voltages_count*2;
   uint8_t b2write[1], readBytes[dim_buffer];
   //size_t bytes_wrote;
   b2write[0] = 'a';
 	
   //mean
   double voltage_prec[voltages_count];
   for(int i = 0; i<voltages_count; i++ )
 		voltage_prec[i] = 0.0;
   
   //**** ROS MAIN LOOP  ***//
   while(ros::ok()){
   	
      /*	bytes_wrote = */ my_serial.write(b2write,1);
		my_serial.read(readBytes, dim_buffer);
			
		finger_voltages.header.stamp = ros::Time::now();
		
		for (int i = 0; i < voltages_count; i++) {
			finger_voltages.voltages.data[i] = (double)(readBytes[i*2] + (readBytes[i*2+1]&0b00001111)*255) * 3.3/4096.0;
			finger_voltages.voltages.data[i] = (finger_voltages.voltages.data[i] + voltage_prec[i])/2.0;
			voltage_prec[i] = finger_voltages.voltages.data[i];
		}
		
		pubTactile.publish(finger_voltages);	
   	
   }

}

