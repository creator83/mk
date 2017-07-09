#include "shift.h"


Shift::Shift(Spi &s, Gpio::Port prt, uint8_t pn_)
:cs (prt, pn_)
{
	mod = &s;
	setMode();
}

Shift::Shift (Spi &s)
{
	mod = &s;
	setMode();
}

void Shift::setMode ()
{
	//settings SPI
	mod->setCpha(Spi::Cpha::first);
	mod->setCpol(Spi::Cpol::neg);
	mod->setDivision(Spi::Division::div32);
	mod->setFrameSize(Spi::Size::bit8);
	mod->start();
}

void Shift::send (uint8_t data)
{
	while (!mod->flagSptef());
	mod->putDataDl(data);
}




