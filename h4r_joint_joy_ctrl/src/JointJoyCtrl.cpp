/*
 * JointJoyCtrl.cpp
 *
 *  Created on: 10.06.2016
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2016)
 */

#include "JointJoyCtrl.h"
#include <sstream>

/**
 * @todo add license and move to source file
 * Class source
 * h4r_joint_joy_ctrl
 * JointJoyCtrl
 */
namespace h4r_joint_joy_ctrl
{


JointJoyCtrl::JointJoyCtrl()
:n_()
,nh_("~")
,rate_(10)
,deadman_(0)
,sub_joy_(n_.subscribe("/joy",1,&JointJoyCtrl::joyCallback,this))
{
  nh_.getParam("rate",rate_);
  nh_.getParam("deadman",deadman_);

  for (int j = 0; ; ++j)
  {
	  std::stringstream ss;
	  ss << "joint" <<j;

	  std::string joint;
	  if(!nh_.getParam(ss.str()+"_name",joint))
	  {
		  break;
	  }

	  int axis;
	  if(!nh_.getParam(ss.str()+"_axis",axis))
	  {
		  ROS_ERROR_STREAM(ss.str()<<" axis missing");
		  exit(1);
		  break;
	  }

	  std::string topic;
	  if(!nh_.getParam(ss.str()+"_topic",topic))
	  {
		  ROS_ERROR_STREAM(ss.str()<<" topic missing");
		  exit(1);
		  break;
	  }

	  double multiplicator;
	  if(!nh_.getParam(ss.str()+"_multiplicator",multiplicator))
	  {
		  ROS_ERROR_STREAM(ss.str()<<" multiplicator missing");
		  exit(1);
		  break;
	  }

	  pubs_joints_.push_back(nh_.advertise<std_msgs::Float64>(topic,1,false));

  }
}

JointJoyCtrl::~JointJoyCtrl()
{

}

void JointJoyCtrl::joyCallback(const sensor_msgs::Joy::ConstPtr &msg)
{

}


int JointJoyCtrl::run()
{
	ros::Rate loop_rate(rate_);

	while(ros::ok())
	{

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

}/* namespace h4r_joint_joy_ctrl */
