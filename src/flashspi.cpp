#include "flashspi.h"



Flash::Flash (Spi &d, Gpio::Port p, uint8_t pin)
:cs(p, pin), dummy (0xEEEE)
{
	driver = &d;
	driver->setCpol(Spi::Cpol::neg);
	driver->setCpha(Spi::Cpha::first);
	driver->setDivision(Spi::Division::div8);
	driver->setFrameSize(Spi::Size::bit8);
	cs.set();
	driver->start();
	delay_ms(5);
	powerUp ();
}

void Flash::writeByte (uint8_t)
{

}

void Flash::writePage (const uint8_t * buffer, uint32_t addr, uint16_t n)
{
	writeEnable ();
	cs.clear();
	command(flashCommands::PageProgram);
	command(addr>>16);
	command(addr>>8);
	command(addr);
	for (uint16_t i=0;i<n;++i)
	{
		command(*buffer++);
	}
	cs.set();
	while (flagBusy());
}

void Flash::writePage16 (const uint16_t * buffer, uint32_t addr, uint16_t n)
{
	writeEnable ();
	driver->setFrameSize (Spi::Size::bit16);
	cs.clear();
	command16(flashCommands::PageProgram, addr>>16);
	command16(addr>>8, addr);
	for (uint16_t i=0;i<n;++i)
	{
		command16(*buffer>>8, *buffer);
		++buffer;
	}
	cs.set();
	driver->setFrameSize (Spi::Size::bit8);
	while (flagBusy());
}

void Flash::write (uint8_t * buffer, uint32_t addr, uint16_t n)
{

}

void Flash::readID ()
{
	/*cs.clear();
	while (!driver->flagSptef());
	driver->putDataDl(ManufactDeviceID);
	while (!driver->flagSprf());
	uint8_t dummy = driver->getDataDl();
	while (!driver->flagSptef());
	driver->putDataDl(0);
	while (!driver->flagSprf());
	manufacturId = driver->getDataDl();
	while (!driver->flagSptef());
	driver->putDataDl(0);
	while (!driver->flagSprf());
	deviceId = driver->getDataDl()<<8;
	while (!driver->flagSptef());
	driver->putDataDl(0);
	while (!driver->flagSprf());
	deviceId |= driver->getDataDl();
	cs.set();*/
}

uint16_t Flash::readStatus1 ()
{
	cs.clear();
	command(flashCommands::ReadStatusReg1);
	uint8_t status = command(0);
	cs.set();
	return status;
}

uint16_t Flash::readStatus2 ()
{
	cs.clear();
	command(flashCommands::ReadStatusReg2);
	uint8_t status = command(0);
	cs.set();
	return status;
}

uint16_t Flash::readStatus3 ()
{
	cs.clear();
	command(flashCommands::ReadStatusReg3);
	uint8_t status = command(0);
	cs.set();
	return status;
}

void Flash::writeStatus (uint8_t val)
{
	cs.clear();
	command(flashCommands::WriteStatusReg);
	command(val);
	cs.set();
}

void Flash::writeEnable ()
{
	cs.clear();
	command(flashCommands::WriteEnable);
	cs.set();
}

void Flash::writeDisable ()
{
	/*cs.clear();
	while (!driver->flagSptef());
	driver->putDataDl(WriteDisable);
	while (!driver->flagSptef());
	cs.set();*/
}

void Flash::read (uint8_t * buffer, uint32_t addr, uint16_t n)
{
	cs.clear();
	command(flashCommands::ReadData);
	command(addr>>16);
	command(addr>>8);
	command(addr);
	for (uint16_t i=0;i<n;++i)
	{
		*buffer++ = command(0);
	}
	cs.set();
}

void Flash::read16 (uint16_t * buffer, uint32_t addr, uint16_t n)
{
	driver->setFrameSize (Spi::Size::bit16);
	cs.clear();
	command16(flashCommands::ReadData, addr>>16);
	command16(addr>>8, addr);
	for (uint16_t i=0;i<n;++i)
	{
		*buffer++ = command16(0, 0);
	}
	cs.set();
	driver->setFrameSize (Spi::Size::bit8);
}

void Flash::read16Dma (uint16_t * buffer, uint32_t addr, uint32_t n)
{
	driver->setFrameSize (Spi::Size::bit16);
	cs.clear();
	command16(flashCommands::ReadData, addr>>16);
	command16(addr>>8, addr);
	transmitter->setIncDestination(false);
	while (!driver->flagSptef());
	driver->putDataDh(0);
	driver->putDataDl(0);
	dataDma ((uint32_t)buffer, n);
	cs.set();
	driver->setFrameSize (Spi::Size::bit8);
}

void Flash::txToDma (uint32_t dest, uint32_t addr, uint32_t n)
{
	driver->setFrameSize (Spi::Size::bit16);
	cs.clear();
	command16(flashCommands::ReadData, addr>>16);
	command16(addr>>8, addr);
	transmitter->setIncDestination(true);
	while (!driver->flagSptef());

	//settings transmitter
	transmitter->setDestination(dest);
	DMA0->DMA[transmitter->getChannel()].DSR_BCR &= ~ DMA_DSR_BCR_BCR_MASK;
	transmitter->setLength(n);
	DMA0->DMA[transmitter->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::receive);

	//settings txDummy
	DMA0->DMA[txDummy->getChannel()].DSR_BCR &= ~ DMA_DSR_BCR_BCR_MASK;
	txDummy->setLength(n);
	DMA0->DMA[txDummy->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::transmit);
	//timer->start();


	while (!transmitter->flagDone());
	DMA0->DMA[transmitter->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	transmitter->clearFlags();
	driver->disableDma(Spi::dma::receive);

	DMA0->DMA[txDummy->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	txDummy->clearFlags();
	driver->disableDma(Spi::dma::transmit);

	cs.set();
	driver->setFrameSize (Spi::Size::bit8);
}

void Flash::dataDma (uint32_t dest, uint32_t n)
{
	transmitter->setDestination(dest);
	transmitter->setLength(n);
	DMA0->DMA[transmitter->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::receive);
	while (!transmitter->flagDone());
	DMA0->DMA[transmitter->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	transmitter->clearFlags();
	driver->disableDma(Spi::dma::receive);
}

void Flash::txDum (uint32_t n)
{
	driver->setFrameSize (Spi::Size::bit16);
	cs.clear();
	//settings txDummy
	txDummy->setLength(n);
	DMA0->DMA[txDummy->getChannel()].DCR |= DMA_DCR_ERQ_MASK;
	driver->enableDma(Spi::dma::transmit);
	timer->start();
	while (!txDummy->flagDone());
	DMA0->DMA[txDummy->getChannel()].DCR &= ~ DMA_DCR_ERQ_MASK;
	txDummy->clearFlags();
	driver->disableDma(Spi::dma::transmit);
	cs.set();
	timer->stop();
	driver->setFrameSize (Spi::Size::bit8);

}
void Flash::eraseSector (uint32_t addr)
{
	writeEnable();
	while (flagBusy());
	cs.clear();
	command(flashCommands::SectorErase);
	command(addr>>16);
	command(addr>>8);
	command(addr);
	cs.set();
	while (flagBusy());
}

void Flash::eraseChip ()
{
	writeEnable ();
	cs.clear();
	command(flashCommands::ChipErase);
	cs.set();
	while (flagBusy());
}

bool Flash::flagBusy ()
{
	return readStatus1()&0x01;
}

uint8_t Flash::command (uint8_t c)
{
	while (!driver->flagSptef());
	driver->putDataDl(c);
	while (!driver->flagSprf());
	return driver->getDataDl();
}

uint16_t Flash::command16 (uint8_t h, uint8_t l)
{
	while (!driver->flagSptef());
	driver->putDataDh(h);
	driver->putDataDl(l);
	while (!driver->flagSprf());
	uint16_t res = driver->getDataDh() << 8;
	res |= driver->getDataDl();
	return res;
}

void Flash::powerDown ()
{
	cs.clear();
	command(flashCommands::PowerDown);
	cs.set();
	delay_us(5);
}

void Flash::powerUp ()
{
	cs.clear();
	command(flashCommands::ReleasePowerDown);
	cs.set();
	delay_us(5);
}

void Flash::getCapacity ()
{

}

void Flash::setDma (Dma &d, Dma &tx)
{
	//Settings transmitter
	transmitter = &d;
	transmitter->setDsize(Dma::size::bit16);
	transmitter->setSsize(Dma::size::bit16);
	transmitter->setSource((uint32_t)&driver->getSpiPtr()->DL);
	transmitter->enableDmaMux(Dma::dmaMux::spi0Rx);
	transmitter->setIncSource(false);
	DMA0->DMA[transmitter->getChannel()].DCR |= DMA_DCR_CS_MASK;
	DMA0->DMA[2].DSR_BCR |= DMA_DCR_LINKCC(2);
	DMA0->DMA[2].DSR_BCR |= DMA_DCR_LCH1(1);

	//Settings txDummy
	txDummy = &tx;
	txDummy->setDsize(Dma::size::bit16);
	txDummy->setSsize(Dma::size::bit16);
	txDummy->setSource((uint32_t)&dummy);
	txDummy->setDestination((uint32_t)&driver->getSpiPtr()->DL);
	txDummy->setIncSource(false);
	txDummy->setIncDestination(false);
	DMA0->DMA[1].DSR_BCR |= DMA_DCR_LINKCC(2);
	DMA0->DMA[1].DSR_BCR |= DMA_DCR_LCH1(2);
	DMA0->DMA[txDummy->getChannel()].DCR |= DMA_DCR_CS_MASK;
	txDummy->enableDmaMux(Dma::dmaMux::spi0Tx);
	//DMAMUX0->CHCFG[1] |= DMAMUX_CHCFG_TRIG_MASK;

	//Settings Pit
	//timer = &t;
}
