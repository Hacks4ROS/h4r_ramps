#ifndef PCINT_H_
#define PCINT_H_

#include <Arduino.h>
#include <pins_arduino.h>

/*
 * attach an interrupt to a specific pin using pin change interrupts.
 */
 void PCattachInterrupt(uint8_t pin, void (*userFunc)(void), int mode);
 void PCdetachInterrupt(uint8_t pin);




#endif /* PCINT_H_ */
