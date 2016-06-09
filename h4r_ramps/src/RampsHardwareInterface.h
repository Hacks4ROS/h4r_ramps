/*
 * This file (RampsHardwareInterface.h) is part of h4r_ramps.
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

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/controller_info.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <control_toolbox/pid.h>
#include <list>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <serial/serial.h>
#include "../ArduinoPlatformIO/ArduinoRamps/src/SerialMotorControl.h"



#ifndef RAMPSHARDWAREINTERFACE_H_
#define RAMPSHARDWAREINTERFACE_H_



namespace h4r_ros_ramps
{



	using namespace std;
	using namespace hardware_interface;
	using namespace serial_motor_control;

	class RampsHardwareInterface : public hardware_interface::RobotHW
	{
	public:
		typedef enum
		{
			POSITION,
			SPEED
		}control_mode_t;

	private:
		SerialMotorControlSerialROS<5> motorcon_;

		double *pos_;
		double *eff_;
		double *vel_;
		double *cmd_pos_;
		double *cmd_vel_;
		double *last_cmd_vel_;
		double *last_cmd_pos_;

		control_mode_t mode;

		serial::Serial *serial_;

		hardware_interface::JointStateInterface jnt_state_interface;
		hardware_interface::PositionJointInterface jnt_pos_interface;
		hardware_interface::VelocityJointInterface jnt_vel_interface;
	public:



		RampsHardwareInterface(serial::Serial &serial, control_mode_t mode);
		virtual ~RampsHardwareInterface();
		void write();
		void read();

	};
}
#endif /* RAMPSHARDWAREINTERFACE_H_ */
