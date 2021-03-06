#include "device.h"               // Device header
#include "gpio.h"

#ifndef PORT_H
#define PORT_H



class Port: protected Gpio
{
//functions
	uint32_t value_;
public:
	Port (port prt, uint32_t value);
	Port (port prt, mux mx, uint32_t value);
	void set(uint32_t value);
	void set();
	void set (uint32_t value, bool st);
	void clear (uint32_t value);
	void clear ();
	void togle (uint32_t value);
};

#endif



