#include "ili9341.h"

Ili9341::Ili9341(Spi &d, port po, uint8_t p, port rstpo, uint8_t rstpi)
:dc (po, p), rst (rstpo, rstpi)
{
	driver = &d;
    driver->setCpol (Spi::cpol::neg);
    driver->setCpha (Spi::cpha::first);
    driver->setFrameSize (Spi::fSize::bit_8);
	driver->setBaudrate(Spi::division::div128);
	init ();
	driver->setBaudrate(Spi::division::div2);
}

void Ili9341::fillScreen (uint16_t color)
{
	setColoumn(0, 319);
	setPage (0, 239);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::fSize::bit_16);
	dc.set();
	for (uint32_t n = 0; n < 76800; n++) {
		driver->transmit(color);
	}
	driver->setFrameSize(Spi::fSize::bit_8);
}

void Ili9341::setCursor (uint16_t x , uint16_t y)
{

}

void Ili9341::setArea (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2)
{
	command(ILI9341_COLUMN_ADDR);
	data16(x1);
	data16(x2);
	
	command(ILI9341_PAGE_ADDR);
	data16(y1);
	data16(y2);
}

void Ili9341::data8 (uint8_t dta)
{
	dc.set();
	driver->transmit(dta);
}


void Ili9341::data16 (uint16_t dta)
{
	dc.set();
	driver->setFrameSize(Spi::fSize::bit_16);
	driver->transmit(dta);
	driver->setFrameSize(Spi::fSize::bit_8);
}

void Ili9341::command (uint8_t com)
{
	dc.clear();
	driver->transmit(com);
}

void Ili9341::init ()
{
	rst.set();
	delay_ms(5);
	rst.clear();
	delay_ms(20);
	rst.set();
	delay_ms(150);

	/*command (ili9341Commands::softwareReset);
	delay_ms(1000);*/
	command (ili9341Commands::powerControl1);
	data8(0x25);

	command (ili9341Commands::powerControl2);
	data8(0x11);

	command (ili9341Commands::vcomControl1);
	data8(0x2B);
	data8(0x2B);

	command (ili9341Commands::vcomControl2);
	data8(0x86);

	//orient
	command (ili9341Commands::memryAccessControl);
	data8(0x28);

	command (ili9341Commands::pixelFormatSet);
	data8(0x55);

	command (ili9341Commands::frameControl);
	data8(0x00);
	data8(0x18);

	command (ili9341Commands::pixelFormatSet);
	data8(0x55);

	command (ili9341Commands::displayFunctionControl);
	data8(0x0A);
	data8(0x82);
	data8(0x27);

	command (ili9341Commands::sleepOut);
	delay_ms(120);

	command (ili9341Commands::displayOn);
}

void Ili9341::setPage (uint16_t y1, uint16_t y2)
{
	command (ili9341Commands::pageAddressSet);
	data16 (y1);
	data16 (y2);
}

void Ili9341::setColoumn (uint16_t x1, uint16_t x2)
{
	command (ili9341Commands::coloumnAddressSet);
	data16 (x1);
	data16 (x2);
}

void Ili9341::setPosition (uint16_t x, uint16_t y)
{
	setColoumn (x, x);
	setPage (y, y);
	command (ili9341Commands::memoryWrite);
}
