#include "pin.h"

Pin::Pin (Gpio::Port prt, uint8_t p , mux mx)
:Gpio(prt)
{
	pin_ = p;
	PortBase[Gpio::prt]->PCR[pin_]= (uint8_t)mx << PORT_PCR_MUX_SHIFT;

}

Pin::Pin (uint8_t prt, uint8_t p , mux mx)
:Gpio(prt)
{
	pin_ = p;
	PortBase[Gpio::prt]->PCR[pin_]= (uint8_t)mx << PORT_PCR_MUX_SHIFT;
}

Pin::Pin (Port prt, uint8_t p )
:Gpio(prt)
{
	pin_ = p;
	PortBase[Gpio::prt]->PCR[pin_] = (uint8_t)Gpio::mux::GPIO << PORT_PCR_MUX_SHIFT;
	GpioBase[Gpio::prt]->PDDR |= 1 << pin_;
}

Pin::Pin (Port prt, uint8_t p , PP m)
:Gpio(prt)
{
	pin_ = p;
	PortBase[Gpio::prt]->PCR[pin_] = (uint8_t)Gpio::mux::GPIO << PORT_PCR_MUX_SHIFT;
	GpioBase[Gpio::prt]->PDDR &= ~(1 << pin_);
	PortBase[Gpio::prt]->PCR[pin_] |= PORT_PCR_PE_MASK;
	PortBase[Gpio::prt]->PCR[pin_] &= ~PORT_PCR_PS_MASK;
	PortBase[Gpio::prt]->PCR[pin_] |= (uint8_t)m << PORT_PCR_PS_SHIFT;
}

void Pin::setPort (Port)
{

}

void Pin::direction (mode m)
{
	GpioBase[Gpio::prt]->PDDR &= ~(1 << pin_);
	GpioBase[Gpio::prt]->PDDR |= ((uint8_t)m << pin_);
}

void Pin::setIn (PP pp_)
{
	GpioBase[Gpio::prt]->PDDR &= ~(1 << pin_);
	PortBase[Gpio::prt]->PCR[pin_] |= 1 << PORT_PCR_PE_MASK;
	PortBase[Gpio::prt]->PCR[pin_] &= ~ (1 << PORT_PCR_PS_MASK);
	PortBase[Gpio::prt]->PCR[pin_] |= (uint8_t)pp_ << PORT_PCR_PS_MASK;
}

void Pin::setOut ()
{
	GpioBase[Gpio::prt]->PDDR |= 1 << pin_;
}


void Pin::set()
{
	GpioBase[Gpio::prt]->PSOR |= 1 << pin_;
}

void Pin::set (bool st)
{
	GpioBase[Gpio::prt]->PCOR |= 1 << pin_;
	GpioBase[Gpio::prt]->PSOR |= st << pin_;
}

void Pin::clear ()
{
	GpioBase[Gpio::prt]->PCOR |= 1 << pin_;
}

void Pin::togle ()
{
	GpioBase[Gpio::prt]->PTOR |= 1 << pin_;
}

bool Pin::state ()
{
	return (GpioBase[Gpio::prt]->PDIR&(1 << pin_));
}

PORT_Type * Pin::getPort ()
{
	return PortBase[prt];
}

uint8_t & Pin::getPin ()
{
	return pin_;
}
