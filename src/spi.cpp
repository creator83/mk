#include "spi.h"

Spi::Spi(numberSpi nSpi, ctarNumber n, csNumber cs, role r)
{
	uint8_t nSpi_ = static_cast<uint8_t>(nSpi);
	spiPtr = ((SPI_Type *)spiAddress[nSpi_]);
	//Turn on tacting
	SIM->SCGC6 |= 1 << spiClockShift[nSpi_];
	txCommand.txCommandBits.pcs = 1 << static_cast<uint8_t>(cs);
	txFifoDisable();
	rxFifoDisable();
	//set ctar number
	setCtar(n);
	//Settings role
	spiPtr->MCR |= (static_cast<uint8_t>(r) << SPI_MCR_MSTR_SHIFT)|SPI_MCR_DIS_TXF_MASK|SPI_MCR_DIS_RXF_MASK;//|SPI_MCR_PCSIS(1<<0);
	//Start
	spiPtr->SR |= SPI_SR_EOQF_MASK;
	spiPtr->MCR &= ~(SPI_MCR_HALT_MASK|SPI_MCR_FRZ_MASK);
}

void Spi::setCpol (cpol c)
{
	ctar.ctarBits.cpol = static_cast<uint8_t>(c);
	updateCtar ();
}

void Spi::setCpha (cpha c)
{
    ctar.ctarBits.cpha = static_cast<uint8_t>(c);
    updateCtar ();
}

void Spi::setFrameSize (fSize f)
{
	ctar.ctarBits.fmsz = static_cast<uint8_t>(f);
	updateCtar ();
}

void Spi::setBaudrate (division d)
{
	ctar.ctarBits.br = static_cast<uint8_t>(d);
	updateCtar ();
}
void Spi::setCtar (ctarNumber c){
	txCommand.txCommandBits.ctas = static_cast<uint8_t>(c);
}
void Spi::txFifoEnable(){
	spiPtr->MCR &=~ SPI_MCR_DIS_TXF_MASK;
}
void Spi::txFifoDisable(){
	spiPtr->MCR |= SPI_MCR_DIS_TXF_MASK;
}

void Spi::rxFifoEnable(){
	spiPtr->MCR &=~ SPI_MCR_DIS_RXF_MASK;
}

void Spi::rxFifoDisable(){
	spiPtr->MCR |= SPI_MCR_DIS_RXF_MASK;
}

void Spi::updateCtar (){
	spiPtr->CTAR[nCtar] = ctar.set;
}

void Spi::transmit (uint16_t data)
{
    while (!flag_tfff());
	spiPtr->PUSHR = txCommand.set|data;
}

void Spi::transmit (uint16_t * data, uint32_t n)
{
    while (!flag_tfff());
	spiPtr->PUSHR = txCommand.set|*data;
}

uint16_t Spi::receive ()
{
	while (!flag_rfof());
	return spiPtr->POPR;	
}

uint16_t Spi::exchange (uint16_t data)
{
    transmit(data);
	return receive();
}

uint16_t Spi::get_data ()
{
	return SPI0->POPR;
}

bool Spi::flag_tcf ()
{
	return SPI0->SR&SPI_SR_TCF_MASK;
}

bool Spi::flag_tfff ()
{
	return SPI0->SR&SPI_SR_TFFF_MASK;
}

bool Spi::flag_tfuf ()
{
	return SPI0->SR&SPI_SR_TFUF_MASK;
}

bool Spi::flag_txctr ()
{
	return SPI0->SR&SPI_SR_TXCTR_MASK;
}

bool Spi::flag_rfof ()
{
	return SPI0->SR&SPI_SR_RFOF_MASK;
}

bool Spi::flag_rfdf ()
{
	return SPI0->SR&SPI_SR_RFDF_MASK;
}

void Spi::clear_flag_tcf()
{
	SPI0->SR |= SPI_SR_TCF_MASK;
}

void Spi::clear_flag_tfuf()
{
	SPI0->SR |= SPI_SR_TFUF_MASK;
}

void Spi::clear_flag_rfof()
{
	SPI0->SR |= SPI_SR_RFOF_MASK;
}

void Spi::clear_flag_rfdf()
{
	SPI0->SR |= SPI_SR_RFDF_MASK;
}

