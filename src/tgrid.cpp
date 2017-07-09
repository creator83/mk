#include "tgrid.h"

Tgrid::Tgrid (Xpt2046 &d, uint8_t hor, uint8_t ver)
:h(hor), v(ver)
{
	driver = &d;
	xGrid = driver->getdX()/hor;
	yGrid = driver->getdY()/ver;
}

uint16_t & Tgrid::getXgrid ()
{
	return xGrid;
}

uint16_t & Tgrid::getYgrid ()
{
	return yGrid;
}

uint8_t & Tgrid::getV ()
{
	return v;
}

Xpt2046 * Tgrid::getDriver ()
{
	return driver;
}
