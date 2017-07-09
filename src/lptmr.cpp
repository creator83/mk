#include "lptmr.h"

Lptmr::Lptmr(division d, source s_)
{
	s = static_cast<uint8_t>(s_);
	SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;
	LPTMR0->CSR = 0;
	LPTMR0->PSR =  (LPTMR_PSR_PRESCALE(static_cast<uint8_t>(d))|s << LPTMR_PSR_PCS_SHIFT);
}

Lptmr::Lptmr(source s_)
{
	s = static_cast<uint8_t>(s_);
	SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK;
	LPTMR0->CSR = 0;
	LPTMR0->PSR =  (LPTMR_PSR_PBYP_MASK|s << LPTMR_PSR_PCS_SHIFT);
}
void Lptmr::start ()
{
	LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;
}

void Lptmr::stop ()
{
	LPTMR0->CSR &= ~ LPTMR_CSR_TEN_MASK;
}

void Lptmr::setComp (uint16_t val)
{
	LPTMR0->CMR = val;
}

void Lptmr::setCnt (uint16_t val)
{
	LPTMR0->CNR = val;
}

void Lptmr::clearTCF ()
{
	LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
}

void Lptmr::interruptEnable ()
{
	LPTMR0->CSR |= LPTMR_CSR_TIE_MASK;
	NVIC_EnableIRQ(LPTimer_IRQn);
}

void Lptmr::interruptDisable ()
{
	LPTMR0->CSR &= ~ LPTMR_CSR_TIE_MASK;
	NVIC_DisableIRQ(LPTimer_IRQn);
}
