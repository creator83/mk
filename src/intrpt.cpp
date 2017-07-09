#include "intrpt.h"

//IRQn Intrpt::source [4] {};

Intrpt::Intrpt (Gpio::Port p, uint8_t pi_, mode m, Gpio::PP f)
:pin_(p, pi_, f)
{
	pin_.getPort()->PCR[pi_]|= PORT_PCR_IRQC((uint8_t)m);
	if (pin_.getPort()== PORTA)
	{
		NVIC_EnableIRQ(PORTA_IRQn);
	}
	else if (pin_.getPort()== PORTC||PORTD)
	{
		//NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	}
}

void Intrpt::clearFlag ()
{
	pin_.getPort()->ISFR|= 1 << pin_.getPin();
}
