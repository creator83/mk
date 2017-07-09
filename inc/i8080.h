#include "MKL26Z4.h"
#include "port.h"
#include "pin.h"

#ifndef I8080_H
#define I8080_H

namespace i8080def
{
	//Data bus 0-7
	const Gpio::Port lowDataPort = Gpio::Port::C;
	const uint32_t lowDataPins = 0xFF;
	//Data bus 8-15
	const Gpio::Port highDataPort = Gpio::Port::B;
	const uint32_t highDataPins = 0xFF0000;
	//CS
	const Gpio::Port csPort = Gpio::Port::C;
	const uint8_t csPin = 8;
	//RS
	const Gpio::Port rsPort = Gpio::Port::C;
	const uint8_t rsPin = 10;
	//WR
	const Gpio::Port wrPort = Gpio::Port::C;
	const uint8_t wrPin = 11;
	//RD
	const Gpio::Port rdPort = Gpio::Port::C;
	const uint8_t rdPin = 12;
	//RST
	const Gpio::Port rstPort = Gpio::Port::C;
	const uint8_t rstPin = 9;
}


class I8080
{
//variables
public:
	Port high, low;
	Pin rs, rst, wr, rd, cs;
protected:

private:



//functions
public:
	I8080();
	void index(uint16_t indx);
	void data(uint16_t dta);
	void putData (uint16_t dta);
	void wReg (uint16_t indx , uint16_t dta);
	void strob ();
protected:

private:


};

#endif
