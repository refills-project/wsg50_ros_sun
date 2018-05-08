/*
    ROS node to combine wrenches of fingers

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

#include <geometry_msgs/WrenchStamped.h>


//------new
#include <Helper1.0.h>

using namespace std;

//==========GLOBAL VARS========//
Matrix<3,3> e_R_0 = Identity, e_R_1 = Identity;
Vector<3> _0_r_e = Zeros, _1_r_e = Zeros;


//*******GLOBAL ROS VARS******//
ros::NodeHandle* n;
//----------------------------//
//*******PUBLISHERS**********//
ros::Publisher pubWrench;
//----------------------------//
//**********MSGS*************//
geometry_msgs::WrenchStamped totalWrench;
//====================================//


//=========LOCAL FCNs INTERFACEs=======//

geometry_msgs::WrenchStamped trWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg , Matrix<3,3> R, Vector<3> r);
void updateWrench();

//====================================//


//==========SERVICES=========//

//====================================//


//==========TOPICs CALLBKs=========//

geometry_msgs::WrenchStamped trWrench0;
void readWrench0( const geometry_msgs::WrenchStamped::ConstPtr& msg ){
			 
    trWrench0 = trWrench( msg, e_R_0 , _0_r_e );	 
    updateWrench();

}

geometry_msgs::WrenchStamped trWrench1;
void readWrench1( const geometry_msgs::WrenchStamped::ConstPtr& msg ){
	 
    trWrench1 = trWrench( msg, e_R_1 , _1_r_e );
    updateWrench();

}


//====================================//


//==============MAIN================//

int main(int argc, char *argv[]){

	ros::init(argc,argv,"combine_wrench");

	n = new ros::NodeHandle("~");

    /***** Params **********/ // (FROM FILE!)
    e_R_0 = Data(1.0, 0.0,  0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0);

    e_R_1 = Data(-1.0, 0.0,  0.0,
                  0.0, 1.0,  0.0,
                  0.0, 0.0, -1.0);
   
   //From topic (OR FROM FILE)
    _0_r_e = makeVector(0.0, 0.0, 0.04);
    _1_r_e = makeVector(0.0, 0.0, 0.04);

    string topic_0("");
    n->param("topic_0" , topic_0, string("/wrench0") );
    string topic_1("");
    n->param("topic_1" , topic_1, string("/wrench1") );

    string out_topic("");
    n->param("out_topic" , out_topic, string("/total_wrench") );

    ros::Subscriber subWrench0 = n->subscribe( topic_0 ,1, readWrench0);
    ros::Subscriber subWrench1 = n->subscribe( topic_1 ,1, readWrench1);

    /*******INIT ROS PUB**********/
    //Force pub
	pubWrench = n->advertise<geometry_msgs::WrenchStamped>( out_topic,1);
    /***************************/

	totalWrench.header.frame_id = "graspCenter";


    //Output
    cout << BOLDBLUE "COMBINE WRENCH NODE" << endl << CRESET << 
        "e_R_0: " << endl << e_R_0 << endl <<
        "e_R_1: " << endl << e_R_1 << endl <<
        "_0_r_e: " << endl << _0_r_e << endl <<
        "_1_r_e: " << endl << _1_r_e << endl <<
        "inputTopic_0: " << BOLDYELLOW << topic_0 << endl << CRESET <<
        "inputTopic_1: " << BOLDYELLOW << topic_1 << endl <<
        BOLDBLUE "=========================" << CRESET << endl;


/*============LOOP==============*/
    ros::spin();
/*==============================*/
	
	return 0;
}



/*===========LOCAL FCNs IMPL==========*/


geometry_msgs::WrenchStamped trWrench( const geometry_msgs::WrenchStamped::ConstPtr& msg , Matrix<3,3> R, Vector<3> r){

    Vector<3> f;
    f[0] = msg->wrench.force.x;
    f[1] = msg->wrench.force.y;
    f[2] = msg->wrench.force.z;

    Vector<3> m;
    m[0] = msg->wrench.torque.x;
    m[1] = msg->wrench.torque.y;
    m[2] = msg->wrench.torque.z;

    Vector<3> f_tr;
    Vector<3> m_tr;

    f_tr = R*f;
    m_tr = R*( skew(r)*f + m );

    geometry_msgs::WrenchStamped outMsg;
    outMsg.header.stamp = msg->header.stamp;
    outMsg.header.frame_id = msg->header.frame_id;

    outMsg.wrench.force.x = f_tr[0];
    outMsg.wrench.force.y = f_tr[1];
    outMsg.wrench.force.z = f_tr[2];

    outMsg.wrench.torque.x = m_tr[0];
    outMsg.wrench.torque.y = m_tr[1];
    outMsg.wrench.torque.z = m_tr[2];

    totalWrench.header.stamp = msg->header.stamp;

    return outMsg;

}

void updateWrench(){

    totalWrench.wrench.force.x = trWrench0.wrench.force.x + trWrench1.wrench.force.x;
    totalWrench.wrench.force.y = trWrench0.wrench.force.y + trWrench1.wrench.force.y;
    totalWrench.wrench.force.z = trWrench0.wrench.force.z + trWrench1.wrench.force.z;

    totalWrench.wrench.torque.x = trWrench0.wrench.torque.x + trWrench1.wrench.torque.x;
    totalWrench.wrench.torque.y = trWrench0.wrench.torque.y + trWrench1.wrench.torque.y;
    totalWrench.wrench.torque.z = trWrench0.wrench.torque.z + trWrench1.wrench.torque.z;

    pubWrench.publish(totalWrench);

}
