#include "pin.h"

Pin::Pin (port prt, uint8_t p , mux mx)
:Gpio(prt)
{
	pin_ = p;
	portPtr->PCR[pin_]= (uint8_t)mx << PORT_PCR_MUX_SHIFT;

}

Pin::Pin (uint8_t prt, uint8_t p , mux mx)
:Gpio(prt)
{
	pin_ = p;
	portPtr->PCR[pin_]= (uint8_t)mx << PORT_PCR_MUX_SHIFT;
}

Pin::Pin (port prt, uint8_t p )
:Gpio(prt)
{
	pin_ = p;
	portPtr->PCR[pin_] = (uint8_t)Gpio::mux::GPIO << PORT_PCR_MUX_SHIFT;
	gpioPtr->PDDR |= 1 << pin_;
}

Pin::Pin (port prt, uint8_t p , PP m)
:Gpio(prt)
{
	pin_ = p;
	portPtr->PCR[pin_] = (uint8_t)Gpio::mux::GPIO << PORT_PCR_MUX_SHIFT;
	gpioPtr->PDDR &= ~(1 << pin_);
	portPtr->PCR[pin_] |= PORT_PCR_PE_MASK;
	portPtr->PCR[pin_] &= ~PORT_PCR_PS_MASK;
	portPtr->PCR[pin_] |= (uint8_t)m << PORT_PCR_PS_SHIFT;
}

void Pin::setPort (port)
{

}

void Pin::direction (mode m)
{
	gpioPtr->PDDR &= ~(1 << pin_);
	gpioPtr->PDDR |= ((uint8_t)m << pin_);
}

void Pin::setIn (PP pp_)
{
	gpioPtr->PDDR &= ~(1 << pin_);
	portPtr->PCR[pin_] |= 1 << PORT_PCR_PE_MASK;
	portPtr->PCR[pin_] &= ~ (1 << PORT_PCR_PS_MASK);
	portPtr->PCR[pin_] |= (uint8_t)pp_ << PORT_PCR_PS_MASK;
}

void Pin::setOut ()
{
	gpioPtr->PDDR |= 1 << pin_;
}


void Pin::set()
{
	gpioPtr->PSOR |= 1 << pin_;
}

void Pin::set (bool st)
{
	gpioPtr->PCOR |= 1 << pin_;
	gpioPtr->PSOR |= st << pin_;
}

void Pin::clear ()
{
	gpioPtr->PCOR |= 1 << pin_;
}

void Pin::togle ()
{
	gpioPtr->PTOR |= 1 << pin_;
}

bool Pin::state ()
{
	return (gpioPtr->PDIR&(1 << pin_));
}

PORT_Type * Pin::getPort ()
{
	return portPtr;
}

uint8_t & Pin::getPin ()
{
	return pin_;
}
