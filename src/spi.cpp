#include "spi.h"

Spi::Ctar_set Spi::C1;
Spi::Ctar_set Spi::C0;

//Spi::Ctar_set* Spi::set_ctar [2] = {&Spi::C0, &Spi::C1};
Spi::ctarPtr Spi::s_ctar [2] = {&Spi::C0, &Spi::C1};

void Spi::set_cpol (Spi &s, Cpol c)
{
	s.set_cpol(c);
}

void Spi::set_cpha (Spi &s, Cpha c)
{
	s.set_cpha(c);
}

void Spi::update_ctar ()
{

}

void Spi::set_ctar (Spi &s, CTAR_number c)
{
	s.set_ctar(c);
}

void Spi::set_baudrate (Spi &s, Division d)
{
	s.set_baudrate(d);
}

void Spi::set_f_size (Spi &s, Fsize f)
{
	s.set_f_size(f);
}

void Spi::set_ctar (CTAR_number n)
{
	ctar_ = (uint8_t)n;
}


Spi::Spi( Role r)
{
  //Turn on tacting Spi1
  SIM->SCGC6 |= SIM_SCGC6_SPI0_MASK;

  //Settings role and turn off tx and rx fifo
  SPI0->MCR &= ~ (SPI_MCR_MSTR_MASK|SPI_MCR_MDIS_MASK);
  SPI0->MCR |= (static_cast<uint8_t>(r) << SPI_MCR_MSTR_SHIFT)|SPI_MCR_DIS_TXF_MASK|SPI_MCR_DIS_RXF_MASK;//|SPI_MCR_PCSIS(1<<0);
  //сделать настройку
  SPI0_CTAR(0) |= SPI_CTAR_PCSSCK(1)|SPI_CTAR_PASC(1);
  //Start
  SPI0->SR |= SPI_SR_EOQF_MASK;
  SPI0->MCR &= ~(SPI_MCR_HALT_MASK|SPI_MCR_FRZ_MASK);
}

void Spi::set_cpol (Cpol c)
{
	s_ctar [ctar_]->cpol = static_cast<uint8_t>(c);
	SPI0_CTAR(ctar_) &= ~ SPI_CTAR_CPOL_MASK;
	SPI0_CTAR(ctar_) |= s_ctar [ctar_]->cpol << SPI_CTAR_SLAVE_CPOL_SHIFT;
}

void Spi::set_cpha (Cpha c)
{
	s_ctar [ctar_]->cpha = static_cast<uint8_t>(c);
	SPI0_CTAR(ctar_) &= ~ SPI_CTAR_CPHA_MASK;
	SPI0_CTAR(ctar_) |= s_ctar [ctar_]->cpha << SPI_CTAR_SLAVE_CPHA_SHIFT;
}

void Spi::set_f_size (Fsize f)
{
	s_ctar [ctar_]->f_size = static_cast<uint8_t>(f);
	SPI0_CTAR(ctar_) &= ~ SPI_CTAR_FMSZ(0x0F);
	SPI0_CTAR(ctar_) |= SPI_CTAR_FMSZ(s_ctar [ctar_]->f_size);
}

void Spi::set_baudrate (Division d)
{
	s_ctar [ctar_]->br = static_cast<uint8_t>(d);
	SPI0_CTAR(ctar_) &= ~ SPI_CTAR_BR(0x0F);
	SPI0_CTAR(ctar_) |= SPI_CTAR_BR (s_ctar [ctar_]->br);
}

void Spi::transmit (uint16_t data)
{

}


uint8_t Spi::receive ()
{
	return 0;	
}

uint8_t Spi::exchange (uint8_t data)
{

}

void Spi::put_data (uint16_t data, CS_number cs, CTAR_number ctar, State cont)
{

	SPI0->PUSHR = SPI_PUSHR_PCS(1<<(uint8_t)cs)|SPI_PUSHR_TXDATA(data)|SPI_PUSHR_CTAS(ctar)|(static_cast<uint8_t>(cont));
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