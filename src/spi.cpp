#include "spi.h"

SPI_Type * Spi::spiAdr [2]={SPI0, SPI1};

PotMemFn Spi::ptr_receive[2] = {};

PotMemF Spi::ptr_transmite[2] = {};
ptr_ex Spi::ptr_exchange[2] =  {};



Spi::Spi(SPI_N n, Division d, Cpol cpol, Cpha cpha, Size s, Mode m, Role r)
{
  n_spi = (uint8_t) n;
  size_ = (uint8_t)s;
  cpol_ = (uint8_t)cpol;
  cpha_ = (uint8_t)cpha;
  role_ = (uint8_t)r;
  div_ = (uint8_t)d;
  mode_ = (uint8_t)m;

  //Turn on tacting SPI1
  SIM->SCGC4 |= 1 << (SIM_SCGC4_SPI0_SHIFT + n_spi);

  //===Settings SPI1===//
  spiAdr [n_spi]->S = 0x00;
  spiAdr [n_spi]->C2 &= ~SPI_C2_MODFEN_MASK;
  spiAdr [n_spi]->BR = SPI_BR_SPR(div_);
  spiAdr [n_spi]->C2 |= mode_ << SPI_C2_MODFEN_SHIFT|(size_<<SPI_C2_SPIMODE_SHIFT);
  spiAdr [n_spi]->C1 &= ~(SPI_C1_MSTR_MASK |SPI_C1_CPHA_MASK|SPI_C1_CPOL_MASK|SPI_C1_SSOE_MASK);
  spiAdr [n_spi]->C1 |= (role_ << SPI_C1_MSTR_SHIFT|cpol_ << SPI_C1_CPOL_SHIFT |cpha_ << SPI_C1_CPHA_SHIFT|mode_ <<  SPI_C1_SSOE_SHIFT);
}

Spi::Spi (SPI_N n, Role r )
{
	n_spi = (uint8_t) n;
	role_ = (uint8_t)r;
	//Turn on tacting SPI1
	SIM->SCGC4 |= 1 << (SIM_SCGC4_SPI0_SHIFT + n_spi);

	//===Settings SPI1===//
	spiAdr [n_spi]->C1 &= ~(SPI_C1_MSTR_MASK |SPI_C1_CPHA_MASK|SPI_C1_CPOL_MASK);
	spiAdr [n_spi]->C1 |= role_ << SPI_C1_MSTR_SHIFT;
}

void Spi::start ()
{
	spiAdr [n_spi]->C1 |= SPI_C1_SPE_MASK;
}

void Spi::stop ()
{
	spiAdr [n_spi]->C1 &= ~ SPI_C1_SPE_MASK;
}

void Spi::setCpol (Cpol c)
{
	spiAdr [n_spi]->C1 &= ~SPI_C1_CPOL_MASK;
	spiAdr [n_spi]->C1 |= (uint8_t)c << SPI_C1_CPOL_SHIFT;
}

void Spi::setCpha (Cpha c)
{
	spiAdr [n_spi]->C1 &= ~SPI_C1_CPHA_MASK;
	spiAdr [n_spi]->C1 |= (uint8_t)c << SPI_C1_CPHA_SHIFT;
}

void Spi::setDivision (Division d)
{
	spiAdr [n_spi]->BR = SPI_BR_SPR(uint8_t (d));
}

void Spi::setFrameSize (Size s)
{
	spiAdr [n_spi]->C2 &= ~SPI_C2_SPIMODE_MASK;
	spiAdr [n_spi]->C2 |= (uint8_t)s << SPI_C2_SPIMODE_SHIFT;
}

void Spi::setMode (Mode m)
{
	spiAdr [n_spi]->C2 &= ~SPI_C2_MODFEN_MASK;
	spiAdr [n_spi]->C2 |= (uint8_t)m << SPI_C2_MODFEN_SHIFT;
	spiAdr [n_spi]->C1 &= ~SPI_C1_SSOE_MASK;
	spiAdr [n_spi]->C1 |= (uint8_t)m << SPI_C1_SSOE_SHIFT;
}

void Spi::setDma (Dma &d)
{
	driverDma = &d;
	DMA0->DMA[d.getChannel()].DCR |= DMA_DCR_CS_MASK;
	DMA0->DMA[d.getChannel()].DCR |= DMA_DCR_EADREQ_MASK| DMA_DCR_ERQ_MASK;
}

void Spi::enableDma (dma d)
{
	spiAdr [n_spi]->C2 |= (1 << (uint8_t)d);
}

void Spi::disableDma (dma d)
{
	spiAdr [n_spi]->C2 &= ~(1 << (uint8_t)d);
}

void Spi::putDataDh (uint8_t data)
{
	spiAdr [n_spi]->DH = data;
}

void Spi::putDataDl (uint8_t data)
{
	spiAdr [n_spi]->DL = data;
}
uint8_t Spi::getDataDh ()
{
	return spiAdr [n_spi]->DH;
}
uint8_t Spi::getDataDl ()
{
	return spiAdr [n_spi]->DL;
}

uint8_t Spi::exhange (uint8_t data)
{

}

uint8_t Spi::exhange8 (uint8_t data)
{

}

uint16_t Spi::exhange16 (uint8_t data)
{

}

bool Spi::flagSptef ()
{
	return spiAdr [n_spi]->S & SPI_S_SPTEF_MASK;
}

bool Spi::flagSprf ()
{
	return spiAdr [n_spi]->S & SPI_S_SPRF_MASK;
}

SPI_Type * Spi::getSpiPtr ()
{
	return spiAdr[n_spi];
}
