#include "mcg.h"

uint32_t Mcg::mcgOutClkDividerArr[2][8] = {{1, 2, 4, 8, 16, 32, 64, 128},
                                        {32, 64, 128, 256, 512, 1024, 1280, 1536}};

Mcg::Mcg (){
    mcgOutSourceArr[0] = &fllClock;
    mcgOutSourceArr[1] = &mcgIRClock;
    mcgOutSourceArr[2] = &extClock;
    rtc = 32000;
    fllSourceArr[0] = &rtc;
    fllSourceArr[1] = &extClock;
}

void Mcg::mcgIrclkEnable (){
	MCG->C1 |= MCG_C1_IRCLKEN_MASK;
}
void Mcg::mcgIrclkDisable (){
	MCG->C1 &=~ MCG_C1_IRCLKEN_MASK;
}

void Mcg::setMcgOutClk (mcgOutSource s){
    mcgOutValue = static_cast <uint8_t>(s);
	MCG->C1 &=~ MCG_C1_CLKS_MASK;
	MCG->C1 &=~ MCG_C1_CLKS(s);
    mcgOutClk = *mcgOutSourceArr[mcgOutValue];
    while (getMcgOutSource()!=mcgOutValue);
}

void Mcg::setFllDivider (fllDivider d){
    fllDividerValue = static_cast <uint8_t>(d);
	MCG->C1 &=~ MCG_C1_FRDIV_MASK;
	MCG->C1 |= MCG_C1_FRDIV(fllDividerValue);
    fllClock /= mcgOutClkDividerArr[fllSourceValue][fllDividerValue];
}

void Mcg::setFllSource (fllSource s){
    fllSourceValue = static_cast <uint8_t>(s); 
	MCG->C1 &=~ MCG_C1_IREFS_MASK;
    MCG->C1 |= MCG_C1_IREFS_MASK;
	MCG->C1 |= fllSourceValue << MCG_C1_IREFS_SHIFT;
    fllClock = *fllSourceArr[static_cast <uint8_t>(s)];
    while (getFllSource()!= static_cast<uint8_t>(s));
}

void Mcg::setFreqRange (freqRange r){
	MCG->C2 &=~ MCG_C2_RANGE_MASK;
	MCG->C2 |= MCG_C2_RANGE(r);
}

void Mcg::setExtSource (extSource s){
	MCG->C2 &=~ MCG_C2_EREFS_MASK;
	MCG->C2 |= static_cast <uint8_t>(s) << MCG_C2_EREFS_SHIFT;
}

void Mcg::setIntSource (intSource s){
	MCG->C2 &=~ MCG_C2_IRCS_MASK ;
	MCG->C2 |= static_cast <uint8_t>(s) << MCG_C2_IRCS_SHIFT;
    while(getIntSource ()!= static_cast <uint8_t>(s));
}

void Mcg::setCoreDivider (mcgOutClkDivider d){
    coreDivider = static_cast<uint8_t>(d);
    SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV1_MASK;
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(coreDivider);
}

void Mcg::setBusDivider (mcgOutClkDivider d){
    busDivider = static_cast<uint8_t>(d);
    SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV2_MASK;
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(busDivider);
}

void Mcg::setFlashDivider (mcgOutClkDivider d){
    flashDivider = static_cast<uint8_t>(d);
    SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV4_MASK;
    SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV4(flashDivider);
}

void Mcg::setFllMultiplication(dcoRange dco, multiplicationRangeFll m){
    MCG->C4 &= ~(MCG_C4_DRST_DRS_MASK);
    MCG->C4 |= MCG_C4_DMX32_MASK;
    MCG->C4 &= ~ MCG_C4_DMX32_MASK;
    MCG->C4 |= (static_cast <uint8_t>(dco) << MCG_C4_DMX32_SHIFT)|MCG_C4_DRST_DRS(static_cast<uint8_t>(m));
    while ((MCG->C4&MCG_C4_DRST_DRS_MASK)!=MCG_C4_DRST_DRS(static_cast<uint8_t>(m)));
}

void Mcg::setOscilator (oscilator o){
    MCG->C7 = MCG_C7_OSCSEL(o);
}
void Mcg::setGain (gainOsc g){
    MCG->C2 &= ~ MCG_C2_HGO_MASK;
    MCG->C2 |= (static_cast <uint8_t>(g)) << MCG_C2_HGO_SHIFT;
}
uint8_t Mcg::getFllSource ()
{
	return MCG->S & MCG_S_IREFST_MASK;
}

uint8_t Mcg::getMcgOutSource (){
	return MCG->S & MCG_S_CLKST_MASK;
}

uint8_t Mcg::getIntSource (){
	return MCG->S & MCG_S_IRCST_MASK;
}

uint32_t & Mcg::getMcgOutClk (){
    return mcgOutClk;
}

uint32_t & Mcg::getFllClk (){
    return fllClock;
}

uint8_t & Mcg::getCoreDivider(){
    return coreDivider;
}

uint8_t & Mcg::getBusDivider(){
    return busDivider;
}

uint8_t & Mcg::getFlashDivider(){
    return flashDivider;
}

uint8_t & Mcg::getFllSourceValue(){
    return fllSourceValue;
}

