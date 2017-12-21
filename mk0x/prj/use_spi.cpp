#include "device.h"
#include "tact.h"
#include "delay.h"
#include "spi.h"
#include "ili9341.h"
#include "colors16bit.h"


int main()
{
	Tact::getInstance();
	Spi spi0(numberSpi::SPI_0, Spi::ctarNumber::ctar0, Spi::csNumber::cs0);
	spi0.setCpol();
	spi0.setCpha();
	spi0.setFrameSize();
	spi0.setBaudrate(Spi::division::div64);
	
	
	spi0.transmit(0x0F);
	
	Spi spiDisplay (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
	Ili9341 display (spiDisplay, port::A, 2, port::A, 3);
	display.fillScreen(colors16bit::BLUE);
	
	while (1)
	{
	}
}


