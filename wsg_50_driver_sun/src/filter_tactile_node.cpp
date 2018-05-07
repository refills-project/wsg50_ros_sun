/*
    ROS node to filter tactile voltages

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

#include <ros/ros.h>

#include <stdio.h>
#include <iostream>

#include <wsg_50_common_sun/Tactile.h>

#include <ros/package.h>

//------new
#include <Helper1.0.h>
#include <ANN1.0.h>
#include <linear_filter.h>
#include <vector>

using namespace std;

//==========GLOBAL VARS========//
int voltages_count;
Vector<> *voltages;
Vector<> *voltages_filter;

//*******GLOBAL ROS VARS******//
ros::NodeHandle* n;
//----------------------------//
//*******PUBLISHERS**********//
ros::Publisher pubVoltagesFilter;
//----------------------------//
//**********MSGS*************//
wsg_50_common_sun::Tactile msgVoltageFilter;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//


//====================================//


//==========SERVICES=========//


//====================================//


//==========TOPICs CALLBKs=========//
void readV( const wsg_50_common_sun::Tactile::ConstPtr& msg  ){
	for(int i = 0; i<voltages_count; i++){
		(*voltages)[i] = msg->voltages.data[i];
	}
	
	msgVoltageFilter.voltages.data.resize(25);
	msgVoltageFilter.header.stamp = msg->header.stamp;
    msgVoltageFilter.header.frame_id = msg->header.frame_id;
	msgVoltageFilter.voltages.layout.dim.resize(1);
    msgVoltageFilter.voltages.layout.dim[0].label = msg->voltages.layout.dim[0].label;
    msgVoltageFilter.voltages.layout.dim[0].size = msg->voltages.layout.dim[0].size;
    msgVoltageFilter.voltages.layout.dim[0].stride = msg->voltages.layout.dim[0].stride;
    msgVoltageFilter.voltages.layout.data_offset = msg->voltages.layout.data_offset;
	
}

//====================================//


//==============MAIN================//

int main(int argc, char *argv[]){

	ros::init(argc,argv,"filter_voltages");

	n = new ros::NodeHandle("~");

    /**** CHECK PARAMS ****/
    n->param("voltages_count" , voltages_count, 25 );
    string topic_name = string("");
    n->param("topic" , topic_name, string("/tactile") );
	/************************************/

    /******INIT ROS MSGS**********/
	msgVoltageFilter.voltages.data.resize(voltages_count);
	/********************/

    /*******INIT ROS PUB**********/
	//Voltage_filter pub
	pubVoltagesFilter = n->advertise<wsg_50_common_sun::Tactile>( topic_name + "/filter",1);
    /***************************/
	

    /*******INIT ROS SUB**********/
	//Status subscriber
	ros::Subscriber subVoltage = n->subscribe( topic_name ,1,readV);
    /***************************/

	/*******INIT FILTER**********/
	string path("");
    path = ros::package::getPath("wsg_50_driver_sun");
    path = path + "/Filter_files/";
	string tmp_path = path;
	tmp_path += "meta.txt";
	Vector<2> metaV = readFileV(tmp_path.c_str(),2);
	int order = metaV[0];
	double rate = metaV[1];
	tmp_path = path + "A.txt";
	Matrix<> A = readFileM(tmp_path.c_str(), order,order);
	tmp_path = path + "B.txt";
	Matrix<> B = readFileM(tmp_path.c_str(), order,1);
	tmp_path = path + "C.txt";
	Matrix<> C = readFileM(tmp_path.c_str(), 1,order);
	tmp_path = path + "D.txt";
	Matrix<> D = readFileM(tmp_path.c_str(), 1,1);
	Vector<> x0 = Zeros(order);
	std::vector<LinearFilter> filter;

	for(int ii = 0; ii<voltages_count;ii++){
		filter.push_back(LinearFilter(order, 1, 1));
		filter[ii].Set_Filter(x0, A, B, C, D);
	}
	voltages = new Vector<>(Zeros(voltages_count));
	voltages_filter = new Vector<>(Zeros(voltages_count));
		
	ros::Rate loop_rate(rate);
	
	filter[0].display();
	cout << YELLOW << "Rate = " << BOLDYELLOW << rate << CRESET << endl;
	cout << BOLDGREEN << "FILTER INIT OK " << CRESET << endl;

/*============LOOP==============*/

	while(ros::ok()){
	
		for(int ii = 0; ii<voltages_count; ii++){
			Vector<1> tmp = Zeros;
			tmp[0] = (*voltages)[ii];
			tmp = filter[ii].apply_filter(tmp);
			(*voltages_filter)[ii] = tmp[0];
			msgVoltageFilter.voltages.data[ii] = (*voltages_filter)[ii];
		}

		pubVoltagesFilter.publish( msgVoltageFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}

    
/*==============================*/
	
	return 0;
}


