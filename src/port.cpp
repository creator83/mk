#include "port.h"

Port::Port (Gpio::Port prt, uint32_t value)
:Gpio(prt), value_(value)
{
	union
	{
		uint32_t full;
		uint16_t half[2];
	}val;
	val.full = value_;
	PortBase[Gpio::prt]->GPCLR = (val.half[0]<<16| (uint8_t)Gpio::mode::Output << PORT_PCR_MUX_SHIFT);
	PortBase[Gpio::prt]->GPCHR = (val.half[1]<<16| (uint8_t)Gpio::mode::Output << PORT_PCR_MUX_SHIFT);
	GpioBase[Gpio::prt]->PDDR |= value;

}

Port::Port (Gpio::Port prt, mux mx, uint32_t value)
:Gpio(prt)
{
	union
	{
		uint32_t full;
		uint16_t half[2];
	}val;
	val.full = value;
	PortBase[Gpio::prt]->GPCLR = (val.half[0]<<16| (uint8_t)mx << PORT_PCR_MUX_SHIFT);
	PortBase[Gpio::prt]->GPCHR = (val.half[1]<<16| (uint8_t)mx << PORT_PCR_MUX_SHIFT);
}

void Port::set(uint32_t value)
{
	GpioBase[Gpio::prt]->PSOR  |= value;
}

void Port::set()
{
	GpioBase[Gpio::prt]->PSOR  |= value_;
}

void Port::set (uint32_t value, bool st)
{

}

void Port::clear (uint32_t value)
{
	GpioBase[Gpio::prt]->PCOR  |= value;
}

void Port::clear ()
{
	GpioBase[Gpio::prt]->PCOR  |= value_;
}

void Port::togle (uint32_t value)
{
	GpioBase[Gpio::prt]->PTOR  |= value;
}
