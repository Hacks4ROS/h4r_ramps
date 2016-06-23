#include <Arduino.h>
#include <pins_arduino.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <FastCRC.h>
#include <TimerOne.h>
#include "pins.h"
#include "SerialMotorControl.h"
#include <pcInt.h>

#include "ArduEncoder.h"

bool pos_nVel=true;
bool led=0;

AccelStepper stepperX(AccelStepper::DRIVER,X_STEP_PIN,X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER,Y_STEP_PIN,Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER,Z_STEP_PIN,Z_DIR_PIN);
AccelStepper stepperE0(AccelStepper::DRIVER,E0_STEP_PIN,E0_DIR_PIN);
AccelStepper stepperE1(AccelStepper::DRIVER,E1_STEP_PIN,E1_DIR_PIN);

MultiStepper multistepper;

using namespace serial_motor_control;
SerialMotorControlArduino<5> proto(Serial);


void step()
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

	stepperX.setAcceleration(10000);
	stepperY.setAcceleration(10000);
	stepperZ.setAcceleration(10000);
	stepperE0.setAcceleration(10000);
	stepperE1.setAcceleration(10000);

	stepperX.setEnablePin(X_ENABLE_PIN);
	stepperY.setEnablePin(Y_ENABLE_PIN);
	stepperZ.setEnablePin(Z_ENABLE_PIN);
	stepperE0.setEnablePin(E0_ENABLE_PIN);
	stepperE1.setEnablePin(E1_ENABLE_PIN);


	stepperX.setPinsInverted(false,false,true);
	stepperY.setPinsInverted(false,false,true);
	stepperZ.setPinsInverted(false,false,true);
	stepperE0.setPinsInverted(false,false,true);
	stepperE1.setPinsInverted(false,false,true);

	stepperX.setMaxSpeed(10000);
	stepperY.setMaxSpeed(10000);
	stepperZ.setMaxSpeed(10000);
	stepperE0.setMaxSpeed(10000);
	stepperE1.setMaxSpeed(10000);

	multistepper.addStepper(stepperX);
	multistepper.addStepper(stepperY);
	multistepper.addStepper(stepperZ);
	multistepper.addStepper(stepperE0);
	multistepper.addStepper(stepperE1);



	Timer1.initialize(100);
    Timer1.attachInterrupt(step);

    pinMode(LED_BUILTIN,OUTPUT);

//	ardu_encoder::ArduEncoder EncoderX(512,20 /*A*/,21 /*B*/,false/*X4*/,-1/*I*/);
}

void loop()
{



	if(proto.receiveCommand())
	{

		led=!led;
		digitalWrite(LED_BUILTIN,led);
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
						stepper->enableOutputs();
						pos_nVel=false;
					break;

				case SerialMotorControl<5>::CMD_REQ_POS:
					    proto.getOutputData()->data[i]=stepper->currentPosition();
						proto.getOutputData()->header.cmd=SerialMotorControl<5>::CMD_RESP_POS;
						if(i==2)
							proto.sendOutput();
					break;

				case SerialMotorControl<5>::CMD_SET_OUTPUTS:
					if(proto.getReceivedData()->data[i])
					{
						stepper->enableOutputs();
					}
					else
					{
						stepper->disableOutputs();
					}
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


