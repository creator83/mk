#include "tact.h"

uint16_t Tact::cpuClock;
uint16_t Tact::busClock;
uint16_t Tact::mcgirClock;
uint16_t Tact::mcgpllClock;
uint16_t Tact::mcgfllClock;

Tact::Tact ()
{
	init ();
	initFei();
	cpuClock = 20971;
	busClock = 20971;
}

void Tact::init ()
{
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) //core clock divider
								|SIM_CLKDIV1_OUTDIV2(0x00) //bus clock divider
								|SIM_CLKDIV1_OUTDIV4(0x01); //flash clock divider
	setPllFllSource (Tact::pllFllSource::mcgFllClk);
	setLptmrSource (Tact::lptmrSource::lpo);
 
}

void Tact::setPllFllSource (pllFllSource s)
{
	nPllFllSource = static_cast<uint8_t>(s);
	SIM->SOPT2 &= ~ SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 = nPllFllSource << SIM_SOPT2_PLLFLLSEL_SHIFT;
}

void Tact::setLptmrSource (lptmrSource s)
{
	nLptmrSource = static_cast<uint8_t>(s);
	SIM->SOPT1 &= ~ SIM_SOPT1_OSC32KSEL_MASK;
	SIM->SOPT1 |= nLptmrSource << SIM_SOPT1_OSC32KSEL_SHIFT;
}

void Tact::initFei ()
{
	mcg.setMcgOutClk (Mcg::mcgOutClk::fll);
	mcg.setFllDivider (Mcg::fllDivider::div1_32);
	mcg.setFllSource (Mcg::fllSource::internal);
	mcg.mcgIrclkEnable ();
	mcg.setFreqRange (Mcg::freqRange::vHighFreq);
	mcg.setExtSource (Mcg::extSource::oscillator);
	mcg.setIntSource (Mcg::intSource::slow);
	MCG->C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
	while (!mcg.getFllSource());
	while (mcg.getMcgOutClk()!= 0);
	
}

