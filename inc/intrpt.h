#include "device.h"                // Device header
#include "pin.h"

#ifndef INTRPT_H
#define INTRPT_H

class Intrpt
{
//variables
public:
	enum class mode {lowState = 8, risingEdge, fallingEdge, eitherEdge, highState};
private:
	Pin pin_;

public:
	Intrpt (port, uint8_t, mode, Gpio::PP = Gpio::PP::PullUp);
	void clearFlag ();
};

#endif



