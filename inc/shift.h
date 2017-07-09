#include "MKL26Z4.h"                 // Device header
#include "spi.h"
#include "pin.h"



#ifndef SHIFT_H
#define SHIFT_H

class Shift
{
private:
	Pin cs;
	Spi * mod;

//functions
public:
	Shift (Spi &, Gpio::Port, uint8_t);
	Shift (Spi &);
	void setMode ();
	void send (uint8_t data);
};
#endif
