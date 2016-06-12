#include <ros/ros.h>
#include "JointJoyCtrl.h"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"joint_joy_ctrl_node");
	ros::Time::init();

	h4r_joint_joy_ctrl::JointJoyCtrl node;
	return node.run();
}





