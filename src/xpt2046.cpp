#include "xpt2046.h"

Xpt2046::modeF Xpt2046::func [3] = {&Xpt2046::getDataSpi, &Xpt2046::getDataSoft};


Xpt2046::Xpt2046 (Spi &d, Gpio::Port cs_, uint16_t csp, Gpio::Port irq_, uint16_t irqp)
:cs(cs_, csp), irq (irq_, irqp, Intrpt::mode::fallingEdge),
 x(0), y(0), Xmin(300), Ymin(300), dX (3300), dY(3300), ptrF(0),
 spiDriver (&d)
{
	NVIC_EnableIRQ(PORTA_IRQn);
	spiDriver->setCpol(Spi::Cpol::neg);
	spiDriver->setCpha(Spi::Cpha::first);
	spiDriver->setDivision(Spi::Division::div32);
	spiDriver->setFrameSize(Spi::Size::bit8);
	cs.set ();
	spiDriver->start();
	cs.clear();
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl(0x80);
	while (!spiDriver->flagSprf());
	uint8_t dummy = spiDriver->getDataDl();
	spiDriver->putDataDl(0x00);
	while (!spiDriver->flagSprf());
	dummy = spiDriver->getDataDl();
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl(0x00);
	while (!spiDriver->flagSprf());
	dummy = spiDriver->getDataDl();
	cs.set();
}

Xpt2046::Xpt2046 (Spis &d, Gpio::Port irq_, uint16_t irqp)
:irq (irq_, irqp, Intrpt::mode::fallingEdge),
 x(0), y(0), Xmin(300), Ymin(300), dX (3300), dY(3300), ptrF(1),
 softDriver (&d)
{
	softDriver->chipEnable();
	softDriver->transmite(0x80);
	uint8_t dummy = softDriver->receive();
	softDriver->transmite(0x00);
	dummy = softDriver->receive();
	softDriver->transmite(0x00);
	dummy = softDriver->receive();
	cs.set();
}

Xpt2046::Xpt2046 (Flexio &d, Gpio::Port cs_, uint16_t csp, Gpio::Port irq_, uint16_t irqp)
:cs(cs_, csp), irq (irq_, irqp, Intrpt::mode::fallingEdge),
 x(0), y(0), Xmin(300), Ymin(300), dX (3300), dY(3300), ptrF(2)
{
	fDriver = &d;
}

void Xpt2046::getData ()
{
	(this->*(Xpt2046::func[ptrF]))();
}
void Xpt2046::getDataSpi ()
{
	uint16_t tempX, tempY;
	cs.clear();
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl (channelX);
	while (!spiDriver->flagSprf());
	uint8_t dummy = spiDriver->getDataDl();
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl (0);
	while (!spiDriver->flagSprf());
	tempX = spiDriver->getDataDl();
	tempX <<= 8;
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl (0);
	while (!spiDriver->flagSprf());
	tempX |= spiDriver->getDataDl();
	tempX >>=3;
	delay_us(100);
	spiDriver->putDataDl (channelY);
	while (!spiDriver->flagSprf());
	dummy = spiDriver->getDataDl();
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl (0);
	while (!spiDriver->flagSprf());
	tempY = spiDriver->getDataDl();
	tempY <<= 8;
	while (!spiDriver->flagSptef());
	spiDriver->putDataDl (0);
	while (!spiDriver->flagSprf());
	tempY |= spiDriver->getDataDl();
	tempY >>=3;
	cs.set();
	y = tempX-Xmin;
	x = 4096 - tempY-Ymin;
}

void Xpt2046::getDataSoft ()
{
	uint16_t tempX, tempY;
	softDriver->chipEnable();
	softDriver->transmite (channelX);
	uint8_t dummy = softDriver->receive();
	softDriver->transmite (0);
	tempX = softDriver->receive();
	tempX <<= 8;
	softDriver->transmite (0);
	tempX |= softDriver->receive();
	tempX >>=3;
	delay_us(100);
	softDriver->transmite (channelY);
	dummy = softDriver->receive();
	softDriver->transmite (0);
	tempY = softDriver->receive();
	tempY <<= 8;
	softDriver->transmite (0);
	tempY |= softDriver->receive();
	tempY >>=3;
	softDriver->chipDisable();
	y = tempX-Xmin;
	x = 4096 - tempY-Ymin;
}

uint16_t & Xpt2046::getX ()
{
	return x;
}

uint16_t & Xpt2046::getY ()
{
	return y;
}

uint16_t & Xpt2046::getdX ()
{
	return dX;
}

uint16_t & Xpt2046::getdY ()
{
	return dY;
}

void Xpt2046::clearFlag ()
{
	irq.clearFlag();
}



