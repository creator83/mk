#include "device.h"
#include "tact.h"
#include "delay.h"
#include "spi.h"
#include "ili9341.h"
#include "colors16bit.h"
#include "dma.h"


uint32_t source[]={1,2,3,4,5,6};
uint32_t dest[6];
int main()
{
	Tact::getInstance();
	Spi spi0(numberSpi::SPI_0, Spi::ctarNumber::ctar0, Spi::csNumber::cs0);
	spi0.setCpol();
	spi0.setCpha();
	spi0.setFrameSize();
	spi0.setBaudrate(Spi::division::div64);
	Dma dma0(dmaChannel::ch0);
	dma0.setSource((uint32_t)source);
	dma0.setDestination((uint32_t)dest);
	dma0.setSsize(Dma::size::bit32);
	dma0.setDsize(Dma::size::bit32);
	dma0.setMinorLoop(6);
	dma0.setOffsetDestination(0);
	dma0.setOffsetSource(0);
	dma0.start();
	
	spi0.transmit(0x0F);
	
	Spi spiDisplay (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
	Ili9341 display (spiDisplay, port::A, 2, port::A, 3);
	display.fillScreen(colors16bit::BLUE);
	
	while (1)
	{
	}
}


