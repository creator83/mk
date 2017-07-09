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
	//static IRQn source [4];
public:
	Intrpt (Gpio::Port, uint8_t, mode, Gpio::PP = Gpio::PP::PullUp);
	void clearFlag ();
};

#endif



