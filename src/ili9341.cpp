#include "ili9341.h"

uint8_t Ili9341::counter;

uint16_t Ili9341::resolutionDef [2][2]={{320, 240},
                                        {400, 240}};

Ili9341::Ili9341(Spi &s, Dma &d, Dma &m, port dcPort_, uint8_t dcPin_, port rstPort_, uint8_t rstPin_)
:dc (dcPort_, dcPin_), rst (rstPort_, rstPin_), spiDriver(&s), mem2spi(&d), mem2mem (&m)
{
    counter=0;
    //settings spi driver
    spiDriver->setCpol (Spi::cpol::neg);
    spiDriver->setCpha (Spi::cpha::first);
    spiDriver->setFrameSize (Spi::fSize::bit_8);  
	spiDriver->setBaudrate(Spi::scaler::div2);
    
    spiDriver->moduleDisable();
    spiDriver->txFifoEnable();
    spiDriver->clearAllFlag();
    spiDriver->moduleEnable();
    spiDriver->enableTransmiteDmaRequest();
    
    mem2spi->enableDmaMux(dmaMux::spi0Tx);
    mem2spi->setOffsetSource(4);
    mem2spi->setDestination ((uint32_t)&SPI0->PUSHR); 
    mem2spi->setOffsetDestination (0);
    mem2spi->setDsize (Dma::size::bit32);
    mem2spi->setSsize (Dma::size::bit32);
    mem2spi->setCountMnrLoop(4);
    mem2spi->setSource((uint32_t)frameBuffer);
    
    mem2mem->enableDmaMux (dmaMux::dma0);
    mem2mem->setDsize (Dma::size::bit32);
    mem2mem->setSsize (Dma::size::bit32);
    mem2mem->setOffsetDestination (4);
    mem2mem->setCountMnrLoop (4);
    mem2mem->setDestination((uint32_t)frameBuffer);
	init ();
    spiDriver->doubleBrEnable();
}
Ili9341::Ili9341(Spi &s, port dcPort_, uint8_t dcPin_, port rstPort_, uint8_t rstPin_)
:dc (dcPort_, dcPin_), rst (rstPort_, rstPin_), spiDriver(&s)
{
    counter=0;
    //settings spi driver
    spiDriver->setCpol (Spi::cpol::neg);
    spiDriver->setCpha (Spi::cpha::first);
    spiDriver->setFrameSize (Spi::fSize::bit_8);  
	spiDriver->setBaudrate(Spi::scaler::div2);
    
    spiDriver->moduleDisable();
    spiDriver->txFifoEnable();
    spiDriver->clearAllFlag();
    spiDriver->moduleEnable();
    spiDriver->enableTransmiteDmaRequest();
    
	init ();
    spiDriver->doubleBrEnable();
}
void Ili9341::setResolution(resolution r){
    horisontal = resolutionDef[static_cast<uint8_t>(r)][0];
    vertical = resolutionDef[static_cast<uint8_t>(r)][1];
    resolutionData = horisontal*vertical;
}
void Ili9341::fillScreen (uint16_t color)
{
	setColoumn(0, horisontal);
	setPage (0, vertical);
	//command(ili9341Commands::memoryWrite);
    command(Ili9341::commands::memoryWrite);
	spiDriver->setFrameSize(Spi::fSize::bit_16);
	dc.set();
	for (uint32_t n = 0; n < resolutionData; n++) {
		spiDriver->transmit(color);
	}
	spiDriver->setFrameSize(Spi::fSize::bit_8);
}
void Ili9341::fillScreenDma (uint16_t color){
    setColoumn(0, horisontal);
	setPage (0, vertical);
    uint8_t retain=calcCounter (resolutionData);
    command(Ili9341::commands::memoryWrite);
    spiDriver->setFrameSize(Spi::fSize::bit_16);
    //settings spi mode dma
	dc.set();
    frameBuffer [1023] = 0x18010000|color;
    
    spiDriver->clearAllFlag();
    currentColor = 0x10010000|color;
    
    mem2mem->setSource((uint32_t)&currentColor);
    
    mem2mem->setDLast(-4092);
    mem2mem->setOffsetSource (0);
    mem2mem->setCountMjrLoop (1023);
    mem2mem->setDREQ();
    mem2mem->startRequest();
    //settings dma driver
    //mem2spi->enableInterruptMajor();
    while (!mem2mem->flagDone());
    mem2spi->setDREQ();
    if (counter){
        mem2spi->setCountMjrLoop(1024);
        mem2spi->setSLast(-4096);
        mem2spi->startRequest();
    }
    while (counter);
    if (retain){
        counter++;
        mem2spi->setCountMjrLoop(retain);
        mem2spi->startRequest();
    }
    while (counter);
    spiDriver->clearAllFlag();
    spiDriver->setFrameSize(Spi::fSize::bit_8);
}
void Ili9341::fillArea (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color){
    setArea(x1, y1, x2, y2);
    uint32_t size = (x2-x1)*(y2-y1);
    uint8_t retain=calcCounter (size);
    command(Ili9341::commands::memoryWrite);
    spiDriver->setFrameSize(Spi::fSize::bit_16);
    //settings spi mode dma
	dc.set();
    frameBuffer [size-1] = 0x18010000|color;
    spiDriver->clearAllFlag();
    currentColor = 0x10010000|color;
    mem2mem->setSource((uint32_t)&currentColor);
    mem2mem->setDestination((uint32_t)frameBuffer); 
    mem2mem->setOffsetSource (0);
    if (counter){
        mem2mem->setCountMjrLoop(1023);
        mem2spi->startRequest();
    }
    mem2mem->setCountMjrLoop (1023);
    mem2mem->setDREQ();
    mem2mem->startRequest();
    //settings dma driver
    while (!mem2mem->flagDone());
    mem2spi->setDREQ();
    if (counter){
        mem2spi->setCountMjrLoop(1024);
        mem2spi->setSLast(-4096);
        mem2spi->startRequest();
    }
    while (counter);
    if (retain){
        counter++;
        mem2spi->setCountMjrLoop(retain);
        mem2spi->startRequest();
    }
    while (counter);
    spiDriver->clearAllFlag();
    spiDriver->setFrameSize(Spi::fSize::bit_8);
}
void Ili9341::drawFromArray (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2, uint32_t ptrAddress){
    setArea(x1, y1, x2, y2);
    uint32_t size = (x2-x1)*(y2-y1);
    command(Ili9341::commands::memoryWrite);
    spiDriver->setFrameSize(Spi::fSize::bit_16);
    //settings spi mode dma
	dc.set();
    mem2spi->setSource(ptrAddress);
    mem2spi->setOffsetSource(0);
}
void Ili9341::setCursor (uint16_t x , uint16_t y){

}

void Ili9341::setArea (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2)
{
	//command(ILI9341_COLUMN_ADDR);
    command(Ili9341::commands::coloumnAddressSet);
	data16(x1);
	data16(x2);
	
	//command(ILI9341_PAGE_ADDR);
    command(Ili9341::commands::pageAddressSet);
	data16(y1);
	data16(y2);
}

uint8_t Ili9341::calcCounter (uint32_t val){
    counter = val/1024;
    return val%1024;
}

void Ili9341::data8 (uint8_t dta)
{
	dc.set();
	spiDriver->transmit(dta);
}


void Ili9341::data16 (uint16_t dta)
{
	dc.set();
	spiDriver->setFrameSize(Spi::fSize::bit_16);
	spiDriver->transmit(dta);
	spiDriver->setFrameSize(Spi::fSize::bit_8);
}

void Ili9341::command (commands com)
{   
	dc.clear();
	spiDriver->transmit(static_cast<uint8_t>(com));
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
	//command (ili9341Commands::powerControl1);
    command(Ili9341::commands::powerControl1);
	data8(0x25);

	//command (ili9341Commands::powerControl2);
    command(Ili9341::commands::powerControl2);
	data8(0x11);

	//command (ili9341Commands::vcomControl1);
    command(Ili9341::commands::vcomControl1);
	data8(0x2B);
	data8(0x2B);

	//command (ili9341Commands::vcomControl2);
    command(Ili9341::commands::vcomControl2);
	data8(0x86);

	//orient
	//command (ili9341Commands::memryAccessControl);
    command(Ili9341::commands::memoryAccessControl);
	data8(0x28);

	//command (ili9341Commands::pixelFormatSet);
    command(Ili9341::commands::pixelFormatSet);
	data8(0x55);

	//command (ili9341Commands::frameControl);
    command(Ili9341::commands::frameControl);
	data8(0x00);
	data8(0x18);

	//command (ili9341Commands::pixelFormatSet);
    command(Ili9341::commands::pixelFormatSet);
	data8(0x55);

	//command (ili9341Commands::displayFunctionControl);
    command(Ili9341::commands::displayFunctionControl);
	data8(0x0A);
	data8(0x82);
	data8(0x27);

	//command (ili9341Commands::sleepOut);
    command(Ili9341::commands::sleepOut);
	delay_ms(120);

	//command (ili9341Commands::displayOn);
    command(Ili9341::commands::displayOn);
}

void Ili9341::setPage (uint16_t y1, uint16_t y2)
{
	//command (ili9341Commands::pageAddressSet);
    command(Ili9341::commands::pageAddressSet);
	data16 (y1);
	data16 (y2);
}

void Ili9341::setColoumn (uint16_t x1, uint16_t x2)
{
	//command (ili9341Commands::coloumnAddressSet);
    command(Ili9341::commands::coloumnAddressSet);
	data16 (x1);
	data16 (x2);
}

void Ili9341::setPosition (uint16_t x, uint16_t y)
{
	setColoumn (x, x);
	setPage (y, y);
	//command (ili9341Commands::memoryWrite);
    command(Ili9341::commands::memoryWrite);
}
void Ili9341::dmaTransfer (uint16_t n){
    mem2spi->setCountMnrLoop(4);
    mem2spi->setCountMjrLoop(n);
    mem2spi->startRequest();
}
