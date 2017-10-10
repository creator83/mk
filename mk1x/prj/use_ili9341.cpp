#include "MKL26Z4.h"                // Device header
#include "tact.h"
#include "delay.h"
#include "ili9341.h"
#include "colors16bit.h"


Tact frq;
Spi spi0 (Spi::SPI_N::SPI_0);


int main()
{
	Pin cs (Gpio::Port::D, 0, Gpio::mux::Alt2);
	Pin clk (Gpio::Port::D, 1, Gpio::mux::Alt2);
	Pin mosi (Gpio::Port::D, 2, Gpio::mux::Alt2);
	Pin miso (Gpio::Port::D, 3, Gpio::mux::Alt2);
	Ili9341 display (spi0, Gpio::Port::D, 4, Gpio::Port::D, 5);
	/*Dma dma0 (Dma::dmaChannel::ch0);
	dma0.enableDmaMux(Dma::dmaMux::spi0Tx);
	display.setDma(dma0);*/
	display.fillScreen(colors16bit::BLACK);
	delay_ms(500);
	while (1)
	{
		display.fillScreen(colors16bit::RED);
		delay_ms(500);
		display.fillScreen(colors16bit::GREEN);
		delay_ms(500);

	}
}
