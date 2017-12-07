#include "spi.h"

Spi::Spi(numberSpi nSpi, ctarNumber nCtar_, csNumber cs, role r)
{
  uint8_t nSpi_ = static_cast<uint8_t>(nSpi);
  spiPtr = ((SPI_Type *)spiAddress[nSpi_]);
  txCommand.txCommandBits.ctas = static_cast<uint8_t>(nCtar_);
  //Turn on tacting Spi1
  SIM->SCGC6 |= 1 << spiClockShift[nSpi_];
  txCommand.txCommandBits.pcs = 1 << static_cast<uint8_t>(cs);
  //Settings role and turn off tx and rx fifo
  spiPtr->MCR &= ~ (SPI_MCR_MSTR_MASK|SPI_MCR_MDIS_MASK);
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

void Spi::setfSize (fSize f)
{
	ctar.ctarBits.fmsz = static_cast<uint8_t>(f);
 updateCtar ();
}

void Spi::setBaudrate (division d)
{
	ctar.ctarBits.br = static_cast<uint8_t>(d);
 updateCtar ();
}

void Spi::updateCtar (){
 spiPtr->CTAR[nCtar] = ctar.set;
}

void Spi::transmit (uint16_t data)
{
 spiPtr->PUSHR = txCommand.set|data;
}


uint8_t Spi::receive ()
{
	return 0;	
}

uint8_t Spi::exchange (uint8_t data)
{

}
/*
void Spi::putData (uint16_t data, CS_number cs, CTAR_number ctar, State cont)
{
	SPI0->PUSHR = SPI_PUSHR_PCS(1<<(uint8_t)cs)|SPI_PUSHR_TXDATA(data)|SPI_PUSHR_CTAS(ctar)|(static_cast<uint8_t>(cont));
}*/

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

