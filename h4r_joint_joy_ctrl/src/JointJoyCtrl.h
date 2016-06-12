

#ifndef JOINTJOYCTRL_H_
#define JOINTJOYCTRL_H_

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

namespace h4r_joint_joy_ctrl
{
/**
 * @todo add license and move to header file
 * h4r_joint_joy_ctrl
 * JointJoyCtrl
 */
class JointJoyCtrl
{
private:
	ros::Subscriber sub_joy_;
	std::vector<ros::Publisher> pubs_joints_;

	ros::NodeHandle n_;
	ros::NodeHandle nh_;

	int deadman_;
	double rate_;


	void joyCallback(const sensor_msgs::Joy::ConstPtr &msg);

public:


	JointJoyCtrl();
	virtual ~JointJoyCtrl();
	int run();
};/* class JointJoyCtrl */



} /* namespace h4r_joint_joy_ctrl */



#endif /* JOINTJOYCTRL_H_ */
