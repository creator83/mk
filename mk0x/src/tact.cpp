#include "tact.h"

uint32_t Tact::cpuClock;
uint32_t Tact::busClock;
uint32_t Tact::mcgIRClock;
uint32_t Tact::mcgFFClock;
uint32_t Tact::mcgFLLClock;
Tact * Tact::p_instance = nullptr;
//Tact * Tact::_instance = nullptr;

TactDestroyer Tact::destroyer;

TactDestroyer::~TactDestroyer(){
    delete p_instance;
}
void TactDestroyer::initialize( Tact* p ){
    p_instance = p;
}
Tact::Tact ()
{
	initFei();
	cpuClock = 20971;
	busClock = 20971;
    mcg.setFlashDivider(Mcg::mcgOutClkDivider::div2);
    mcg.setBusDivider (Mcg::mcgOutClkDivider::div1);
    MCG->C4 |= MCG_C4_DMX32_MASK;
    MCG->C4 |= MCG_C4_DRST_DRS(0x02);
    MCG->C4 &= ~ MCG_C4_DMX32_MASK;
    while ((MCG->C4&MCG_C4_DRST_DRS_MASK)!=MCG_C4_DRST_DRS(0x02));
    //mcg.setFllMultiplication(Mcg::dcoRange::notFine, Mcg::multiplicationRangeFll::midRange);
    //SMC->PMPROT |= SMC_PMPROT_AHSRUN_MASK;
    //SMC->PMCTRL |= SMC_PMCTRL_RUNM_MASK;
    
    
    /*SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01)  //core clock divider
                  |SIM_CLKDIV1_OUTDIV2(0x01)  //bus clock divider
                  |SIM_CLKDIV1_OUTDIV4(0x03); //flash clock divider
    MCG->C4 &= ~(MCG_C4_DMX32_MASK|MCG_C4_DRST_DRS_MASK);
    MCG->C4 |= MCG_C4_DRST_DRS(2);*/
}
Tact& Tact::getInstance()
{
	if (p_instance == nullptr)
	{
		p_instance = new Tact ();
        destroyer.initialize(p_instance);
	}
	return *p_instance;
}

uint32_t & Tact::getFrqBus ()
{
	return busClock;
}
uint32_t & Tact::getFrqCpu (){
    return cpuClock;
}
uint32_t & Tact::getFrqFlash (){
    return busClock;
}
void Tact::setPllFllSource (pllFllSource s){
	nPllFllSource = static_cast<uint8_t>(s);
	SIM->SOPT2 &= ~ SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 = nPllFllSource << SIM_SOPT2_PLLFLLSEL_SHIFT;
}

void Tact::setLptmrSource (lptmrSource s){
	nLptmrSource = static_cast<uint8_t>(s);
	SIM->SOPT1 &= ~ SIM_SOPT1_OSC32KSEL_MASK;
	SIM->SOPT1 |= nLptmrSource << SIM_SOPT1_OSC32KSEL_SHIFT;
}

void Tact::initFei (){
    //uint8_t 
    mcg.setCoreDivider(Mcg::mcgOutClkDivider::div1);
    mcg.setBusDivider (Mcg::mcgOutClkDivider::div1);
    mcg.setFlashDivider(Mcg::mcgOutClkDivider::div2);
    mcg.setOscilator(Mcg::oscilator::rtc);
    mcg.setFreqRange (Mcg::freqRange::lowFreq);
    mcg.setGain (Mcg::gainOsc::highGain);
	setPllFllSource (Tact::pllFllSource::mcgFllClk);
    mcg.setFllSource (Mcg::fllSource::internal);
    mcg.setMcgOutClk (Mcg::mcgOutSource::fll);
	setLptmrSource (Tact::lptmrSource::lpo); 
	mcg.setFllDivider (Mcg::fllDivider::div1_32);
	mcg.mcgIrclkEnable ();
	mcg.setExtSource (Mcg::extSource::oscillator);
	mcg.setIntSource (Mcg::intSource::slow);	
}



