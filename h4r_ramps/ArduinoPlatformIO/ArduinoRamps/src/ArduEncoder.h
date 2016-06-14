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

//forward declarations
class ArduEncoder;
//template <ArduEncoder* enc>
//class ArduEncoderWrapper;






class ArduEncoder
{
	static int num;
	static ArduEncoder *interrupt_array[5];
	INTWRAPS(a);
	INTWRAPS(b);
	INTWRAPS(i);

	int32_t position_;

	typedef enum
	{
		A,
		B,
		I,
	}PinID_t;


	typedef struct
	{
		volatile uint8_t * port;
		uint8_t mask;
	}pcPortMask_t;


	pcPortMask_t ports[3];
	uint8_t last_[3];


	void a()
	{
		uint8_t curr = (*ports[A].port)&ports[A].mask;

		last_[A]=curr;
	}

	void b()
	{
		uint8_t curr = (*ports[B].port)&ports[B].mask;

		last_[B]=curr;
	}

	void i()
	{
		uint8_t curr = (*ports[I].port)&ports[I].mask;


		last_[I]=curr;
	}






public:
	ArduEncoder(uint8_t pinA, uint8_t pinB, uint8_t pinI=-1);

	virtual ~ArduEncoder();

	int32_t getPosition()
	{
		return position_;
	}

	void reset(int32_t position=0)
	{
		position_=position;
	}


};

//template <ArduEncoder* enc>
//class ArduEncoderWrapper
//{
//public:
//	static void intFcta ()
//	{
//		enc->a();
//	}
//};


} /* namespace ardu_encoder */

#endif /* ARDUENCODER_H_ */
