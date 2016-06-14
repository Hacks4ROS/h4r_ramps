#include <Arduino.h>
#include <pins_arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FastCRC.h>
#include <TimerOne.h>
#include "pins.h"
#include "SerialMotorControl.h"
#include "pcInt.h"

#include "ArduEncoder.h"





bool pos_nVel=true;

AccelStepper stepperX(AccelStepper::DRIVER,X_STEP_PIN,X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER,Y_STEP_PIN,Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER,Z_STEP_PIN,Z_DIR_PIN);
AccelStepper stepperE0(AccelStepper::DRIVER,Y_STEP_PIN,Y_DIR_PIN);
AccelStepper stepperE1(AccelStepper::DRIVER,Z_STEP_PIN,Z_DIR_PIN);


MultiStepper multistepper;

using namespace serial_motor_control;
SerialMotorControlArduino<5> proto(Serial);


void stepper()
{
	if(pos_nVel)
	{
		multistepper.run();
	}
	else
	{
		for (int s = 0; s < 5; ++s) {
			multistepper.getStepper(s)->runSpeed();
		}
	}
}



void setup()
{


	Serial.begin(9600);
//	Serial.setInterruptPriority(10);

	pinMode(X_ENABLE_PIN,OUTPUT);
	pinMode(Y_ENABLE_PIN,OUTPUT);
	pinMode(Z_ENABLE_PIN,OUTPUT);
	pinMode(E_ENABLE_PIN,OUTPUT);

	digitalWrite(X_ENABLE_PIN,LOW);
	digitalWrite(Y_ENABLE_PIN,LOW);
	digitalWrite(Z_ENABLE_PIN,LOW);
	digitalWrite(E_ENABLE_PIN,HIGH);

	stepperX.setAcceleration(100);
	stepperY.setAcceleration(100);
	stepperZ.setAcceleration(100);
	stepperE0.setAcceleration(100);
	stepperE1.setAcceleration(100);

	stepperX.setMaxSpeed(1000);
	stepperY.setMaxSpeed(1000);
	stepperZ.setMaxSpeed(1000);
	stepperE0.setMaxSpeed(1000);
	stepperE1.setMaxSpeed(1000);

	multistepper.addStepper(stepperX);
	multistepper.addStepper(stepperY);
	multistepper.addStepper(stepperZ);
	multistepper.addStepper(stepperE0);
	multistepper.addStepper(stepperE1);

    Timer1.initialize(100);
    Timer1.attachInterrupt(stepper);

}

void loop()
{
	if(proto.receiveCommand())
	{
		bool end = false;
		for(int i=0; i<5 && !end;i++)
		{
			AccelStepper *stepper = multistepper.getStepper(i);
			if(stepper)
			{
				switch(proto.getReceivedData()->header.cmd)
				{

				case SerialMotorControl<5>::CMD_REQ_STOP:
					stepper->setSpeed(0);
					break;

				case SerialMotorControl<5>::CMD_SET_MAX_ACCELERATION:
						stepper->setAcceleration(proto.getReceivedData()->data[i]);
					break;

				case SerialMotorControl<5>::CMD_SET_MAX_VELOCITY:
						stepper->setMaxSpeed(proto.getReceivedData()->data[i]);

					break;

				case SerialMotorControl<5>::CMD_SET_GOAL:
						multistepper.moveTo((long *)proto.getReceivedData()->data);
						pos_nVel=true;
						end=true;
					break;

				case SerialMotorControl<5>::CMD_SET_VELOCITY:
						stepper->setSpeed(proto.getReceivedData()->data[i]);
						pos_nVel=false;
					break;

				case SerialMotorControl<5>::CMD_REQ_POS:
					    proto.getOutputData()->data[i]=stepper->currentPosition();
						proto.getOutputData()->header.cmd=SerialMotorControl<5>::CMD_RESP_POS;
						if(i==2)
							proto.sendOutput();
					break;
				default:
					break;
				}
			}
			else
			{
				break;
			}
		}

	}

}


