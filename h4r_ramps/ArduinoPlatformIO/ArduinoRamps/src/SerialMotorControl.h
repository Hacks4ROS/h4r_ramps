/*
 * SerialMotorControl.h
 *
 *  Created on: 08.06.2016
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2016)
 */

#ifndef SERIALMOTORCONTROL_H_
#define SERIALMOTORCONTROL_H_

#ifdef linux
#include <string>
#include <boost/crc.hpp>
#include <serial/serial.h>
#endif


namespace serial_motor_control
{

template <uint8_t MOTORS=3>
class SerialMotorControl
{

private:



	typedef uint8_t header_raw_t[4];
	typedef struct
	{
		uint8_t mark1;
		uint8_t cmd;
		uint8_t mark2;
		uint8_t motors_cnt;
	}header_t;

	typedef int32_t data_array_t[MOTORS];
	typedef uint8_t crc_t;


	typedef union
	{
		struct
		{
			union
			{
				header_raw_t header_raw;
				header_t header;
			};

			data_array_t data;
			crc_t crc;
		};
			uint8_t buf[];
	}send_receive_struct_t;

	send_receive_struct_t output_;
	send_receive_struct_t input_;

	typedef enum
	{
		RECV_FIRST_MARK,
		RECV_COMMAND,
		RECV_SECOND_MARK,
		RECV_DATA,
		RECV_CRC
	}recv_state_t;

	recv_state_t state_;
	uint8_t data_state_;

public:
	typedef enum
	{
		CMD_E_MOTOR_NUM='E',
		CMD_E_CRC='C',

		CMD_SET_MAX_ACCELERATION='A',
		CMD_SET_MAX_VELOCITY='V',
		CMD_SET_VELOCITY='T',
		CMD_REQ_STOP='S',

		CMD_RESP_POS='P',
		CMD_REQ_POS='p',

		CMD_SET_OUTPUTS='E',

		CMD_SET_GOAL='G',
	}command_t;

protected:

	virtual bool data_on_serial()=0;

	virtual uint8_t readByte()=0;

	virtual size_t readBytes(uint8_t *buffer, size_t size)=0;

	virtual void writeBytes(const uint8_t  * const data, size_t size)=0;

	virtual void flushOutput()=0;
	virtual void flushInput()=0;

	virtual uint8_t crc8(const uint8_t  * const data, size_t size)=0;

	template <typename T>
	void writeBytes(const T  * const data, size_t size)
	{
		writeBytes((uint8_t *)data,size);
	}

	void quickCommand(command_t cmd)
	{
		uint8_t header[4]="---";
		header[1]=cmd;
		writeBytes(header,3);
		flushOutput();
	}
public:


	void sendOutput()
	{
		output_.crc=crc8((uint8_t *)output_.data,sizeof(data_array_t));
		writeBytes(output_.buf, sizeof(send_receive_struct_t));
		flushOutput();
	}

	const send_receive_struct_t * const getReceivedData()
	{
		return &input_;
	}

	send_receive_struct_t *getOutputData()
	{
		return &output_;
	}

	SerialMotorControl()
	:state_(RECV_FIRST_MARK)
	,data_state_(0)
	{
		strcpy((char*)output_.header_raw,"-P-");
		strcpy((char*)input_.header_raw,"-G-");
		output_.header_raw[3]=MOTORS; //Amount of available motors
		input_.header_raw[3]=MOTORS;
		data_state_=RECV_FIRST_MARK;
	}

	virtual ~SerialMotorControl(){}




	void sendStop()
	{
		quickCommand(CMD_REQ_STOP);
	}

	void queryPosition()
	{
		quickCommand(CMD_REQ_POS);
	}


	bool receiveCommand()
	{
		while(data_on_serial())
		{

			if(state_!=RECV_DATA)
			{
				input_.buf[data_state_]=readByte();//TODO compiler warning above bounds??? Something wrong here? Check array bounds later

			}
			else //DATA BULK READING
			{
				size_t missing_bytes=sizeof(send_receive_struct_t)-data_state_;

				data_state_+=readBytes(input_.buf+data_state_,missing_bytes);

				if(data_state_>=sizeof(send_receive_struct_t))
				{
					state_=RECV_CRC;
				}
				else
				{
					continue;
				}
			}

			switch(state_)
			{
			case RECV_DATA:
				continue;
				break;
			case RECV_FIRST_MARK:
				if(input_.header.mark1=='-')
				{
					state_=RECV_COMMAND;
				}
				else
				{
					data_state_=0;
					continue;
				}
				break;

			case RECV_COMMAND:
				state_=RECV_SECOND_MARK;
				break;

			case RECV_SECOND_MARK:
				if(input_.header.mark2=='-')
				{
					state_=RECV_DATA;
				}
				else
				{
					data_state_=0;
					state_=RECV_FIRST_MARK;
					continue;
				}

				//SIMPLE SHORT COMMANDS (NO DATA AND CRC)
				switch(input_.header.cmd)
				{
				case CMD_REQ_STOP:
				case CMD_REQ_POS:
				case CMD_E_MOTOR_NUM:
				case CMD_E_CRC:
					data_state_=0;
					state_=RECV_FIRST_MARK;
					return true;
				}
				break;

			case RECV_CRC:
				{
					state_=RECV_FIRST_MARK;
					data_state_=0;
					//TODO byte order?
					if(input_.crc==crc8( (uint8_t*)input_.data , sizeof(data_array_t) ) )
					{
						return true;
					}
					else
					{
						state_=RECV_FIRST_MARK;
						data_state_=0;
						writeBytes("-C-",3);
						flushInput();
						return false;
					}
				}
				break;
			}
		data_state_++;
		}
		return false;
	}
};

#ifdef ARDUINO
#include <FastCRC.h>
template <uint8_t MOTORS=3>
class SerialMotorControlArduino : public SerialMotorControl<MOTORS>
{
	Stream *serial_;
public:
	SerialMotorControlArduino(Stream &serial)
	:serial_(&serial)
	{}

protected:


	void flushOutput(){};
	void flushInput(){serial_->flush();};

	bool data_on_serial()
	{
		return !!serial_->available();
	}

	uint8_t readByte()
	{
		return serial_->read();
	}

	size_t readBytes(uint8_t *buffer, size_t size)
	{
		return serial_->readBytes(buffer,size);
	}

	void writeBytes(const uint8_t  * const data, size_t size)
	{
		serial_->write(data,size);
	}

	uint8_t crc8(const uint8_t  * const data, size_t size)
	{
		FastCRC8 crc8;
		return crc8.maxim(data,size);
	}

};
#endif

#ifdef linux
template <uint8_t MOTORS=3>
class SerialMotorControlSerialROS : public SerialMotorControl<MOTORS>
{
	serial::Serial *serial_;
public:
	SerialMotorControlSerialROS(serial::Serial &serial)
	:serial_(&serial)
	{}

protected:
	void flushOutput(){serial_->flushOutput();};
	void flushInput(){serial_->flushInput();};

	bool data_on_serial()
	{
		return !!serial_->available();
	}

	uint8_t readByte()
	{
		return serial_->read().c_str()[0];
	}

	size_t readBytes(uint8_t *buffer, size_t size)
	{
		return serial_->read(buffer,size);
	}

	void writeBytes(const uint8_t  * const data, size_t size)
	{
		serial_->write(data,size);
	}

	uint8_t crc8(const uint8_t  * const data, size_t size)
	{
		boost::crc_optimal <8, 0x31, 0, 0, true, true> crc_8_maxim;
		crc_8_maxim=std::for_each( data, data + size, crc_8_maxim );
		return crc_8_maxim();
	}
};
#endif


} /* namespace serial_motor_control */

#endif /* SERIALMOTORCONTROL_H_ */
