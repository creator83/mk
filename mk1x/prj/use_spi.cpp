#include "MKL26Z4.h"
#include "pin.h"
#include "tact.h"
#include "delay.h"
#include "spi.h"


Tact frq;
Spi spi0 (Spi::SPI_N::SPI_1);

uint8_t arr [] = {0xc0, 0xfe, 0x07, 0xe0};

int main()
{
	spi0.setMode(Spi::Mode::hardware);
	Pin cs (Gpio::Port::D, 4, Gpio::mux::Alt2);
	Pin sck (Gpio::Port::D, 5, Gpio::mux::Alt2);
	Pin mosi (Gpio::Port::D, 6, Gpio::mux::Alt2);
	Pin miso (Gpio::Port::D, 7, Gpio::mux::Alt2);

	spi0.setCpha(Spi::Cpha::first);
	spi0.setCpol(Spi::Cpol::neg);
	spi0.setDivision(Spi::Division::div32);
	spi0.setFrameSize(Spi::Size::bit8);
	spi0.start();
	SPI1->C3 |= SPI_C3_FIFOMODE_MASK ;

  while (1)
  {
	  for (uint8_t i=0;i<4;++i)
	  {
		  spi0.putDataDl(arr[i]);
		  while (SPI1->S & SPI_S_TXFULLF_MASK);
	  }
	  delay_ms(100);
  }
}


