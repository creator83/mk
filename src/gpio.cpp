#include "gpio.h"

Gpio::Gpio ()
{

}

Gpio::Gpio (port p)
{
	prt = static_cast <uint8_t> (p);
	//takt port
	SIM->SCGC5 |= (0x200 << prt);
	portPtr = ((PORT_Type *)portAddress[prt]);
	gpioPtr = ((GPIO_Type *)gpioAddress[prt]);
}


Gpio::Gpio (uint8_t p)
{
	prt = p;
	//takt port
	SIM->SCGC5 |= (0x200 << prt);
	portPtr = ((PORT_Type *)portAddress[prt]);
	gpioPtr = ((GPIO_Type *)gpioAddress[prt]);
}



