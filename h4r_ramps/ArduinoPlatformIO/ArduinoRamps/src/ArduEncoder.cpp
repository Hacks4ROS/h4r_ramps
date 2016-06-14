/*
 * ArduEncoder.cpp
 *
 *  Created on: 14.06.2016
 *      Author: Christian Holl
 *      
 * @todo insert LICENSE!
 *      All rights reserved! (©2016)
 */

#include "ArduEncoder.h"



namespace ardu_encoder
{


ArduEncoder::ArduEncoder(uint8_t pinA, uint8_t pinB, uint8_t pinI)
:position_(0)
{

	interrupt_array[num]=this;

	void (*a_fct_ptr)()=0;
	void (*b_fct_ptr)()=0;
	void (*i_fct_ptr)()=0;

	switch(num)
	{
	CASE_INTFCTS(0);
	CASE_INTFCTS(1);
	CASE_INTFCTS(2);
	CASE_INTFCTS(3);
	CASE_INTFCTS(4);
	}

	for (int i = 0; i < 3; ++i)
	{
		uint8_t pin;

		enum
		{
			PCINT,
			INT,
		};
		uint8_t interrupt=0;
		void (*i_fct)();
		switch(i)
		{
		case A:
			pin=pinA;
			interrupt = digitalPinToInterrupt(pinA);
			i_fct=a_fct_ptr;
			break;

		case B:
			pin=pinB;
			interrupt = digitalPinToInterrupt(pinB);
			i_fct=b_fct_ptr;
			break;


		case I:

			if(pinI<0) /*PinI is optional*/
				break;

			pin=pinI;
			interrupt = digitalPinToInterrupt(pinI);
			i_fct=i_fct_ptr;
			break;

		default:

			break;
		}

		ports[i].mask=digitalPinToBitMask(pin);
		ports[i].port=(volatile uint8_t*)digitalPinToPort(pin);

		last_[i]=(*ports[i].port)&ports[i].mask;

		if(interrupt!=NOT_AN_INTERRUPT)
		{
			PCattachInterrupt(pin,i_fct,CHANGE);
		}
		else
		{
			attachInterrupt(pin,i_fct,CHANGE);
		}

		num++;
	}
}

ArduEncoder::~ArduEncoder()
{
	// TODO Auto-generated destructor stub
}

} /* namespace ardu_encoder */
