#include "ili9341.h"

Ili9341::Ili9341(Spi &d, Gpio::Port po, uint8_t p, Gpio::Port rstpo, uint8_t rstpi)
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

void Ili9341::setDma (Dma &d)
{
	/*dma = &d;
	dma->setDsize(Dma::size::bit16);
	dma->setSsize(Dma::size::bit16);
	dma->setDestination((uint32_t)&driver->getSpiPtr()->DL);
	dma->enableDmaMux(Dma::dmaMux::spi1Tx);
	//driver->setDma(d);
	dma->setIncDestination(false);
	DMA0->DMA[dma->getChannel()].DCR |= DMA_DCR_CS_MASK;*/
}

void Ili9341::fillScreen (uint16_t color)
{
		setColoumn(0, 319);
		setPage (0, 239);
		command(ili9341Commands::memoryWrite);
		driver->setFrameSize(Spi::fSize::bit16);
		dc.set();
		for (uint32_t n = 0; n < 76800; n++) {
			while (!driver->flagSptef());
			driver->putDataDh(color>>8);
			driver->putDataDl(color);
		}
		driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::fillScreenDma (const uint16_t * color)
{
	setColoumn(0, 319);
	setPage (0, 239);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(false);
	dataDma (color, 153600);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::setCursor (uint16_t x , uint16_t y)
{

}

void Ili9341::setArea (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2)
{
	command(ILI9341_COLUMN_ADDR);
	data(x1 >> 8);
	data(x1 & 0xFF);
	data(x2 >> 8);
	data(x2 & 0xFF);

	command(ILI9341_PAGE_ADDR);
	data(y1 >> 8);
	data(y1 & 0xFF);
	data(y2 >> 8);
	data(y2 & 0xFF);

}

void Ili9341::data (uint8_t dta)
{
    driver->setFrameSize(Spi::fSize::bit_8);
	dc.set();
	while (!driver->flagSptef());
	driver->transmit(dta);
	while (!driver->flagSprf());
	uint8_t temp = driver->getDataDl();
}


void Ili9341::data16 (uint16_t dta)
{
	dc.set();
	//PTE->PCOR |= 1 << 16;
	while (!dc.state());
	while (!driver->flagSptef());
	driver->putDataDh(dta >> 8);
	driver->putDataDl(dta);
	//PTE->PSOR |= 1 << 16;
}

void Ili9341::dataDma (const uint16_t * buf, uint32_t n)
{
	dma->setSource((uint32_t )buf);
	dma->setLength(n);
	DMA0->DMA[dma->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::transmit);
	while (!dma->flagDone());
	DMA0->DMA[dma->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	dma->clearFlags();
	driver->disableDma(Spi::dma::transmit);
}

void Ili9341::dataDma (uint32_t address, uint32_t n)
{
	dma->setSource(address);
	dma->setLength(n);
	DMA0->DMA[dma->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::transmit);
	while (!dma->flagDone());
	DMA0->DMA[dma->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	dma->clearFlags();
	driver->disableDma(Spi::dma::transmit);
}

void Ili9341::dataDma8 (const uint8_t * buf, uint32_t n)
{
	dma->setDsize(Dma::size::bit8);
	dma->setSsize(Dma::size::bit8);
	dma->setSource((uint32_t )buf);
	dma->setLength(n);
	DMA0->DMA[dma->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::transmit);
	while (!dma->flagDone());
	DMA0->DMA[dma->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	dma->clearFlags();
	driver->disableDma(Spi::dma::transmit);
	dma->setDsize(Dma::size::bit16);
	dma->setSsize(Dma::size::bit16);
}


void Ili9341::command (uint8_t com)
{
	dc.clear();
	while (!driver->flagSptef());
	driver->putDataDl(com);
	while (!driver->flagSprf());
	uint8_t temp = driver->getDataDl();
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
	data(0x25);

	command (ili9341Commands::powerControl2);
	data(0x11);

	command (ili9341Commands::vcomControl1);
	data(0x2B);
	data(0x2B);

	command (ili9341Commands::vcomControl2);
	data(0x86);

	//orient
	command (ili9341Commands::memryAccessControl);
	data(0x28);

	command (ili9341Commands::pixelFormatSet);
	data(0x55);

	command (ili9341Commands::frameControl);
	data(0x00);
	data(0x18);

	command (ili9341Commands::pixelFormatSet);
	data(0x55);

	command (ili9341Commands::displayFunctionControl);
	data(0x0A);
	data(0x82);
	data(0x27);

	command (ili9341Commands::sleepOut);
	delay_ms(120);

	command (ili9341Commands::displayOn);
}

void Ili9341::setPage (uint16_t y1, uint16_t y2)
{
	command (ili9341Commands::pageAddressSet);
	data (y1>>8);
	data (y1&0xFF);
	data (y2>>8);
	data (y2&0xFF);
}

void Ili9341::setColoumn (uint16_t x1, uint16_t x2)
{
	command (ili9341Commands::coloumnAddressSet);
	data (x1>>8);
	data (x1&0xFF);
	data (x2>>8);
	data (x2&0xFF);
}

void Ili9341::setPosition (uint16_t x, uint16_t y)
{
	setColoumn (x, x);
	setPage (y, y);
	command (ili9341Commands::memoryWrite);
}

void Ili9341::pixel (uint16_t x , uint16_t y, const uint16_t color)
{
	command (ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	data16 (color);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, Font & s)
{
	uint16_t colors [2] = {fon, color};
	uint8_t byte = s.getWidth()>>3;
	uint8_t pix = s.getWidth()%8;
	uint16_t num;

	if (pix){
		num = (ch-s.getShift())*(byte+1)*s.getHeight();
	}
	else num = (ch-s.getShift())*byte*s.getHeight();

	const uint8_t * ptrFont = s.getFont(num);

	uint16_t lArea = s.getWidth()*s.getHeight();

	uint16_t arrSymbol [lArea];
	uint16_t * arr=arrSymbol;

	for (uint8_t i=0;i<s.getHeight();++i)
	{
		for (uint8_t j=0;j<byte;++j)
		{
			for (int8_t k=7;k>=0;--k)
			{
				bool temp = (*ptrFont)&(1 << k);
				*arr++ = colors [temp];
			}
			++ptrFont;
		}
		for (int8_t k=7, j=0;j<pix;--k, ++j)
		{
			bool temp = (*ptrFont)&(1 << k);
			*arr++ = colors [temp];
		}
		if (pix) ++ptrFont;
	}

	setArea(x, y, x+s.getWidth()-1, y+s.getHeight()-1);

	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(true);
	dataDma (arrSymbol, lArea*2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, Font &f, int8_t interval)
{
	uint16_t tempX = x;
	while (*str)
	{
		symbol (tempX, y, color, fon, *str++, f);
		tempX +=  f.getWidth() + interval;
	}
}

void Ili9341::drawArr (uint16_t x , uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width)
{

}

void Ili9341::drawPic (uint16_t x , uint16_t y, const uint16_t *arr, uint16_t length, uint16_t height)
{
	setArea(x, y, x+length-1, y+height-1);
	command(ili9341Commands::memoryWrite);
	/*driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	for (uint32_t n = 0; n < (length+1)*(height+1); n++) {
		while (!driver->flagSptef());
		driver->putDataDh(arr[n]>>8);
		driver->putDataDl(arr[n]);
	}
	driver->setFrameSize(Spi::Size::bit8);*/
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(true);
	dataDma (arr, (height)*(length)*2+2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::drawPic (uint16_t x , uint16_t y, uint32_t address, uint16_t length, uint16_t height)
{
	setArea(x, y, x+length-1, y+height-1);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(true);
	dataDma (address, (height)*(length)*2+2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::drawPic8 (uint16_t x , uint16_t y, const uint8_t *arr, uint16_t length, uint16_t height)
{
	setArea(x, y, x+length, y+height);
	command(ili9341Commands::memoryWrite);
	dc.set();
	dma->setIncSource(true);
	dataDma8 (arr, height*length*2);
}

void Ili9341::horLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick)
{
	setArea(x, y, x+length, y+thick-1);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(false);
	dataDma (color, length*thick*2+thick*2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::verLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick)
{
	setArea(x, y, x+thick-1, y+length);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(false);
	dataDma (color, length*thick*2+2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::line (uint16_t x, uint16_t y, uint16_t color, uint16_t length, uint8_t thick)
{

}

void Ili9341::rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height, uint8_t thick)
{
	horLine(x, y, color, length, thick);
	horLine(x, y+height-thick, color, length, thick);
	verLine(x, y, color, height, thick);
	verLine(x+length, y, color, height, thick);
}

void Ili9341::rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height)
{
	setArea(x, y, x+length, y+height);
	command(ili9341Commands::memoryWrite);
	driver->setFrameSize(Spi::Size::bit16);
	dc.set();
	dma->setIncSource(false);
	dataDma (color, height*length*2+(height+1)*2);
	driver->setFrameSize(Spi::Size::bit8);
}

void Ili9341::gradientVer (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height)
{
	uint16_t grad = 5;//sizeof (color);
	uint8_t dHeight = height/grad;
	for (uint16_t i=0;i<grad;++i){
		rectangle (x, y+dHeight*i, &color [i], length, dHeight);
	}
}

