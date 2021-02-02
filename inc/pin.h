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
	Pin (port prt, uint8_t p , mux mx);
	Pin (uint8_t prt, uint8_t p , mux mx);
	Pin (port prt, uint8_t p);
	Pin (port prt, uint8_t p , PP m);
	void setPort (port);
	void direction (mode m);
	void setIn (PP pp_);
    void setPull (PP pp_);
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



