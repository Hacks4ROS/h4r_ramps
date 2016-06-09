/*
 * This file (RampsHardwareInterface.cpp) is part of h4r_ramps.
 * Date: 10.06.2016
 *
 * Author: Christian Holl
 * http://github.com/Hacks4ROS
 *
 * h4r_ramps is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * h4r_ramps is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with h4r_ramps.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "RampsHardwareInterface.h"

#include <iostream>



namespace h4r_ros_ramps
{

RampsHardwareInterface::RampsHardwareInterface(serial::Serial &serial, control_mode_t mode)
:serial_(&serial)
,motorcon_(serial)
,mode(mode)
{
	registerInterface(&jnt_pos_interface);
	registerInterface(&jnt_state_interface);
	registerInterface(&jnt_vel_interface);


	const int size=5;


	vel_=new double[5];
	pos_=new double[5];
	eff_=new double[5];
	cmd_pos_=new double[5];
	cmd_vel_=new double[5];
	last_cmd_pos_=new double[5];
	last_cmd_vel_=new double[5];

	for (int i = 0; i < 5; ++i) {
		vel_[i]=0;
		pos_[i]=0;
		eff_[i]=0;
		cmd_pos_[i]=0;
		cmd_vel_[i]=0;
		last_cmd_pos_[i]=NAN;
		last_cmd_vel_[i]=NAN;
	}


	for (int j = 0; j < 5; ++j)
	{

		std::string joint_name;
		switch(j)
		{
		case 0:
		case 1:
		case 2:
			joint_name="joint_";
			joint_name+='x'+j;
			break;

		case 3:
		case 4:
			joint_name="joint_e";
			joint_name+='0'+(j-3);
			break;
		}

		hardware_interface::JointStateHandle state_handle(joint_name, &pos_[j] , &vel_[j],&eff_[j]);
		jnt_state_interface.registerHandle(state_handle);

		hardware_interface::JointHandle joint_handle_pos(jnt_state_interface.getHandle(joint_name), &cmd_pos_[j]);
		hardware_interface::JointHandle joint_handle_vel(jnt_state_interface.getHandle(joint_name), &cmd_vel_[j]);
		jnt_pos_interface.registerHandle(joint_handle_pos);
		jnt_vel_interface.registerHandle(joint_handle_vel);

	}
}

RampsHardwareInterface::~RampsHardwareInterface() {
	delete[] vel_;
	delete[] pos_;
	delete[] eff_;
	delete[] cmd_pos_;
	delete[] last_cmd_pos_;
	delete[] last_cmd_vel_;
	delete[] cmd_vel_;
}


void RampsHardwareInterface::write()
{

	bool changes_pos=false;
	bool changes_vel=false;
	for (int i = 0;  i < 5; ++ i)
	{
		if(last_cmd_pos_[i]!=cmd_pos_[i])
		{
			last_cmd_pos_[i]=cmd_pos_[i];
			changes_pos=true;
		}

		if(last_cmd_vel_[i]!=cmd_vel_[i])
		{
			last_cmd_vel_[i]=cmd_vel_[i];
			changes_vel=true;
		}
	}


	if(changes_pos)
	{
		motorcon_.getOutputData()->header.cmd=SerialMotorControl<5>::CMD_SET_GOAL;
		for (int i = 0;  i < 5; ++ i)
		{
			motorcon_.getOutputData()->data[i]=cmd_pos_[i];
			ROS_INFO_STREAM("POS "<< i<<": "<<cmd_pos_[i]);
		}
		motorcon_.sendOutput();
		ROS_INFO("SEND POS!");
	}

	if(changes_vel)
	{
		motorcon_.getOutputData()->header.cmd=SerialMotorControl<5>::CMD_SET_VELOCITY;
		for (int i = 0;  i < 5; ++ i)
		{
			motorcon_.getOutputData()->data[i]=cmd_vel_[i];
		}
		motorcon_.sendOutput();
		ROS_INFO("SEND VEL!");
	}

};

void RampsHardwareInterface::read()
{

	motorcon_.queryPosition();

	if(motorcon_.receiveCommand())
	{
		ROS_INFO("GOT DATA!");
		switch (motorcon_.getReceivedData()->header.cmd)
		{
			case SerialMotorControl<5>::CMD_RESP_POS:

				for (int i = 0; i < 5; ++i)
				{
					pos_[i]=motorcon_.getReceivedData()->data[i];
					ROS_INFO_STREAM("POS "<<i<<": "<<pos_[i]);
				}

				break;
			default:
				break;
		}
	}
}

} /*namespace*/
