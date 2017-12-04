#include "mcg.h"

Mcg::Mcg ()
{
}

void Mcg::mcgIrclkEnable ()
{
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;
}
void Mcg::mcgIrclkDisable ()
{
	MCG->C1 &=~ MCG_C1_IRCLKEN_MASK;
}

void Mcg::setMcgOutClk (mcgOutClk s)
{
	
	MCG->C1 &=~ MCG_C1_CLKS(3);
	MCG->C1 &=~ MCG_C1_CLKS(s);
}

void Mcg::setFllDivider (fllDivider d)
{
	MCG->C1 &=~ MCG_C1_FRDIV(3);
	MCG->C1 |= MCG_C1_FRDIV(d);
}

void Mcg::setFllSource (fllSource s)
{
	MCG->C1 &=~ MCG_C1_IREFS_MASK;
	MCG->C1 |= static_cast <uint8_t>(s) << MCG_C1_IREFS_SHIFT;
}

void Mcg::setFreqRange (freqRange r)
{
	MCG->C2 &=~ MCG_C2_RANGE(3);
	MCG->C2 |= MCG_C2_RANGE(r);
}

void Mcg::setExtSource (extSource s)
{
	MCG->C2 &=~ MCG_C2_EREFS_MASK;
	MCG->C2 |= static_cast <uint8_t>(s) << MCG_C2_EREFS_SHIFT;
}

void Mcg::setIntSource (intSource s)
{
	MCG->C2 &=~ MCG_C2_IRCS_MASK ;
	MCG->C2 |= static_cast <uint8_t>(s) << MCG_C2_IRCS_SHIFT;
}

uint8_t Mcg::getFllSource ()
{
	return MCG->S & MCG_S_IREFST_MASK;
}

uint8_t Mcg::getMcgOutClk ()
{
	return MCG->S & MCG_S_CLKST(1);
}

uint8_t Mcg::getIntSource ()
{
	return MCG->S & MCG_S_IRCST_MASK;
}
