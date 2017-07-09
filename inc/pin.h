#include "device.h"                  // Device header
#include "gpio.h"


#ifndef PIN_H
#define PIN_H

class Pin: protected Gpio
{
//variables
public:

private:
	uint8_t pin_;
//functions
public:
	Pin (){}
	Pin (Port prt, uint8_t p , mux mx);
	Pin (uint8_t prt, uint8_t p , mux mx);
	Pin (Port prt, uint8_t p);
	Pin (Port prt, uint8_t p , PP m);
	void setPort (Port);
	void direction (mode m);
	void setIn (PP pp_);
	void setOut ();
	void set();
	void set (bool st);
	void clear ();
	void togle ();
	bool state ();
	PORT_Type * getPort ();

	uint8_t & getPin ();
};

#endif



