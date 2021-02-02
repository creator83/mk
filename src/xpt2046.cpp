#include "xpt2046.h"

Xpt2046::Xpt2046 (Spi &s, Intrpt & irq_)
:spiDriver(&s), irq(&irq_), x(0), y(0), Xmin(300), Ymin(300), dX (3300), dY(3300), ptrF(0)
{
	//NVIC_EnableIRQ(PORTA_IRQn);
	/*spiDriver->setCpol(Spi::Cpol::neg);
	spiDriver->setCpha(Spi::Cpha::first);
	spiDriver->setDivision(Spi::Division::div32);
	spiDriver->setFrameSize(Spi::Size::bit8);
	cs.set ();
	spiDriver->start();*/
    //settings spi -> txFifo
    init();
}

void Xpt2046::init (){
    
    //continues mode
    spiDriver->continuousModeEnable();
    spiDriver->transmit (0x80);
    spiDriver->transmit (0x00);
    spiDriver->continuousModeDisable();
    spiDriver->transmit (0x00);
   	/*cs.clear();
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
	cs.set(); */
}

void Xpt2046::getData (){
	//(this->*(Xpt2046::func[ptrF]))();
}
void Xpt2046::getDataSpi (){
	uint16_t tempX, tempY;
	/*cs.clear();
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
	cs.set();*/
    spiDriver->continuousModeEnable();
  
    uint16_t dummy = spiDriver->exchange (channelX);
    tempX = spiDriver->exchange (0);
    tempX <<= 8;
    spiDriver->setDelayAfterTransfer(Spi::scaler::div16, Spi::prescaler::div3);
    tempX |= spiDriver->exchange (0);
    tempX >>=3;
    
    dummy = spiDriver->exchange (channelY);
    spiDriver->setDelayAfterTransfer(Spi::scaler::div2, Spi::prescaler::div1);
    tempY = spiDriver->exchange (0);
    tempY <<= 8;
    tempY |= spiDriver->exchange (0);
    tempY >>=3;
    
	y = tempX-Xmin;
	x = 4096 - tempY-Ymin;
}

uint16_t & Xpt2046::getX (){
	return x;
}

uint16_t & Xpt2046::getY (){
	return y;
}

uint16_t & Xpt2046::getdX (){
	return dX;
}

uint16_t & Xpt2046::getdY (){
	return dY;
}

void Xpt2046::clearFlag (){
}



