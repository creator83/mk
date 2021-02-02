#include "device.h"              // Device header


#ifndef GPIO_H
#define GPIO_H

class Gpio
{
//variables
public:
	
	enum class mode {Input, Output};
	enum class PP {PullDown, PullUp};
	enum class state {Off, On};

protected:
	uint8_t prt;
	PORT_Type * portPtr;
	GPIO_Type * gpioPtr;
private:

//functions
public:
	Gpio ();
	Gpio(port p );
	Gpio(uint8_t p );
};

#endif
