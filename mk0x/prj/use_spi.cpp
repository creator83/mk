#include "device.h"
#include "tact.h"
#include "delay.h"
#include "spi.h"
#include "ili9341.h"
#include "colors16bit.h"
#include "dma.h"


uint8_t source[]={1,2,3,4,5,6};
uint8_t dest[6];
int main()
{
	Tact::getInstance();
	Pin cs(port::D, 0, Gpio::mux::Alt2);
	Pin sck(port::D, 1, Gpio::mux::Alt2);
	Pin mosi(port::D, 2, Gpio::mux::Alt2);
	Pin miso(port::D, 3, Gpio::mux::Alt2);
	Spi spi0(numberSpi::SPI_0, Spi::ctarNumber::ctar0, Spi::csNumber::cs0);
	spi0.setCpol();
	spi0.setCpha();
	spi0.setFrameSize(Spi::fSize::bit_16);
	spi0.setBaudrate(Spi::division::div4);

	
	Dma dma0(dmaChannel::ch0);
	//dma0.setSource((uint32_t)source);
	DMA0->CR |= DMA_CR_HALT_MASK;
	//while (!(DMA0->CR&(1<<31)));
	DMA0->TCD[0].SADDR = (uint32_t)source;
	
	DMA0->TCD[0].SOFF = DMA_SOFF_SOFF(1);

	
	//*((uint32_t*)0x40009004) = 5;
	//DMA0->TCD[0].SLAST = -6;
	//dma0.setOffsetSource(1);
	//dma0.setDestination((uint32_t)dest);
	DMA0->TCD[0].DADDR = (uint32_t)dest;
	DMA0->TCD[0].DOFF = DMA_DOFF_DOFF(1);
	//dma0.setOffsetDestination(1);
	//DMA0->TCD[0].DLAST_SGA = -6;
	//dma0.setSsize(Dma::size::bit32);
	//DMA0->TCD[0].ATTR |= DMA_ATTR_DSIZE(1);
	DMA0->TCD[0].ATTR = 2<<8;
	DMA0->TCD[0].ATTR |= DMA_ATTR_SSIZE(1);
	DMA0->TCD[0].NBYTES_MLNO = 6;
	DMA0->TCD[0].CITER_ELINKNO = 1;
	//DMA0->TCD[0].BITER_ELINKNO = 1;
	//dma0.setDsize(Dma::size::bit32);
	//dma0.setMinorLoop(6);
	DMA0->CR &=~ DMA_CR_HALT_MASK;
	DMA0->TCD[0].CSR |= DMA_CSR_START_MASK;
	//dma0.start();

	/*spi0.transmit(0x0F);

	Spi spiDisplay (numberSpi::SPI_0, Spi::ctarNumber::ctar1, Spi::csNumber::cs0);
	Ili9341 display (spiDisplay, port::A, 2, port::A, 3);
	display.fillScreen(colors16bit::BLUE);*/
	
	while (1)
	{
		spi0.transmit(0x0FF0);
		delay_ms(10);
		spi0.transmit(0xF00F);
	}
}


