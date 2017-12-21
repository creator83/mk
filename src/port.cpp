#include "port.h"

Port::Port (port prt, uint32_t value)
:Gpio(prt), value_(value)
{
	union
	{
		uint32_t full;
		uint16_t half[2];
	}val;
	val.full = value_;
	portPtr->GPCLR = (val.half[0]<<16| (uint8_t)Gpio::mode::Output << PORT_PCR_MUX_SHIFT);
	portPtr->GPCHR = (val.half[1]<<16| (uint8_t)Gpio::mode::Output << PORT_PCR_MUX_SHIFT);
	gpioPtr->PDDR |= value;

}

Port::Port (port prt, mux mx, uint32_t value)
:Gpio(prt)
{
	union
	{
		uint32_t full;
		uint16_t half[2];
	}val;
	val.full = value;
	portPtr->GPCLR = (val.half[0]<<16| (uint8_t)mx << PORT_PCR_MUX_SHIFT);
	portPtr->GPCHR = (val.half[1]<<16| (uint8_t)mx << PORT_PCR_MUX_SHIFT);
}

void Port::set(uint32_t value)
{
	gpioPtr->PSOR  |= value;
}

void Port::set()
{
	gpioPtr->PSOR  |= value_;
}

void Port::set (uint32_t value, bool st)
{

}

void Port::clear (uint32_t value)
{
	gpioPtr->PCOR  |= value;
}

void Port::clear ()
{
	gpioPtr->PCOR  |= value_;
}

void Port::togle (uint32_t value)
{
	gpioPtr->PTOR  |= value;
}
