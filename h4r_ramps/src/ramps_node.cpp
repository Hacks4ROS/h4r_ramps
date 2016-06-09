#include <ros/ros.h>
#include <ros/master.h>
#include <iostream>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <serial/serial.h>

#include "RampsHardwareInterface.h"
using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ramps_node");
	ros::Time::init();

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	ros::CallbackQueue queue;
	nh.setCallbackQueue(&queue);
	n.setCallbackQueue(&queue);

	ros::AsyncSpinner spinner(4, &queue);
	spinner.start();

	serial::Serial serial;


	int rate = 10;
	nh.getParam("rate", rate);
	ROS_INFO_STREAM("Rate: "<<rate);
	ros::Rate loop_rate(rate);


	std::string port = "/dev/ttyACM0";

	nh.getParam("port", port);
	ROS_INFO_STREAM("Port: "<<port);
	serial::Timeout timeout=serial::Timeout::simpleTimeout(30);



	int baudrate = 9600;
	nh.getParam("baudrate", baudrate);


	int bytesize = 8;
	nh.getParam("bytesizes", bytesize);
	if (bytesize > 8 && bytesize < 5)
	{
		ROS_ERROR("Bytesize not supported");
		exit(1);
	}

	int stopbits=1;
	nh.getParam("stopbits", stopbits);
	if (stopbits > 3 && stopbits < 1)
	{
		ROS_ERROR("Stopbits not supported (supported is 1-3(where 3 => 1.5))");
		exit(1);
	} else {
		if(stopbits==3)
			stopbits=serial::stopbits_one_point_five;
	}

	int flowcontrol=2;
	nh.getParam("flowcontrol", flowcontrol);
	if (flowcontrol > 3 && flowcontrol < 1)
	{
		ROS_ERROR("flowcontrol not supported (supported is 0-2)");
		exit(1);
	}


	int parity=0;
	nh.getParam("parity", parity);
	if (parity > 4 && parity < 0)
	{
		ROS_ERROR("flowcontrol not supported (supported is 0-4)");
		exit(1);
	}
	serial.setTimeout(timeout);
	serial.setPort(port);
	serial.setBaudrate(baudrate);
	serial.setBytesize(serial::bytesize_t(bytesize));
	serial.setStopbits((serial::stopbits_t) stopbits);
	serial.setFlowcontrol((serial::flowcontrol_t) flowcontrol);
	serial.setParity((serial::parity_t) parity);
	ROS_INFO_STREAM("Serial: Baud:"<<baudrate<<" Stop: "<<stopbits<< " Flow: "<<flowcontrol<<" Bytesize: "<<bytesize<< " Parity: "<<parity);


	std::string modestr="position";
	nh.getParam("mode", modestr);


	h4r_ros_ramps::RampsHardwareInterface::control_mode_t mode;


	if(modestr=="position")
	{
		mode=h4r_ros_ramps::RampsHardwareInterface::POSITION;
	}
	else if(modestr=="velocity")
	{
		mode=h4r_ros_ramps::RampsHardwareInterface::SPEED;
	}
	else
	{
		ROS_ERROR_STREAM("Unknown mode "<<modestr);
		exit(1);
	}


	h4r_ros_ramps::RampsHardwareInterface robot(serial,mode);
	controller_manager::ControllerManager cm(&robot, n);
	ros::Time ts = ros::Time::now();

	serial.open();
	sleep(1);

	while (ros::ok() && serial.isOpen())
	{
		ros::Duration d = ros::Time::now() - ts;
		robot.read();
		ts = ros::Time::now();
		cm.update(ts, d);
		robot.write();
		loop_rate.sleep();
	}

	spinner.stop();

	return 0;
}
