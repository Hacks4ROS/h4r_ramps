/*
 * ArduEncoder.h
 *
 *  Created on: 14.06.2016
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2016)
 */

#ifndef ARDUENCODER_H_
#define ARDUENCODER_H_

#include <Arduino.h>
#include <inttypes.h>
#include <pcInt.h>


#define INTWRAP_DEFINE(PIN,NUM)\
static void PIN##_int##NUM(){ interrupt_array[NUM]->PIN();}

#define INTWRAPS(PIN)\
	INTWRAP_DEFINE(PIN,0);\
	INTWRAP_DEFINE(PIN,1);\
	INTWRAP_DEFINE(PIN,2);\
	INTWRAP_DEFINE(PIN,3);\
	INTWRAP_DEFINE(PIN,4)

#define CASE_INTFCTS(NUM)\
case NUM:\
	a_fct_ptr=a_int##NUM;\
	b_fct_ptr=b_int##NUM;\
	i_fct_ptr=i_int##NUM;\
	break



namespace ardu_encoder
{



class ArduEncoder
{
	static int num;
	static ArduEncoder *interrupt_array[5];


	int32_t reset_pos_;
	int32_t position_;
	int32_t ticks_per_round_;

	bool mode_x4_;
	bool error_;

	typedef enum
	{
		id_A,
		id_B,
		id_I,
	}PinID_t;

	typedef struct
	{
		volatile uint8_t * port;
		uint8_t mask;
	}pcPortMask_t;

	void tick(PinID_t channel)
	{


		bool current;
		bool other;

		uint8_t A=( (*ports[id_A].port) & ports[id_A].mask );
		uint8_t B=( (*ports[id_B].port) & ports[id_B].mask );


		switch(channel)
		{
		case id_A:
//			if(last_chan_state_[id_A]==A)
//				return;

			current=!A;
			other=B;
			last_chan_state_[id_A]=A;
			break;
		case id_B:
//			if(last_chan_state_[id_B]==B)
//				return;

			current=B;
			other=A;
			last_chan_state_[id_B]=B;
			break;
		default:
			return;
		}

		switch(current^other)
		{
		case 0:
			position_++;
			break;
		case 1:
			position_--;
			break;
		}
	}

	pcPortMask_t ports[3];
	uint8_t last_chan_state_[3];
public:
	void a()
	{
		tick(id_A);
	}

	void b()
	{
		tick(id_B);
	}

	void i()
	{
		Serial.println("i");
	}

	INTWRAPS(a);
	INTWRAPS(b);
	INTWRAPS(i);

	ArduEncoder(int32_t ticks_per_round, int8_t pinA, int8_t pinB, bool x4, int8_t pinI=-1);

	virtual ~ArduEncoder();

	int32_t getPosition()
	{
		return position_;
	}

	void reset(int32_t position=0)
	{
		position_=position;
		reset_pos_=position;
	}
};



} /* namespace ardu_encoder */

#endif /* ARDUENCODER_H_ */
