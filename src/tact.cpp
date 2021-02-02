#include "tact.h"

Tact * Tact::_instance = nullptr;
uint32_t Tact::cpuClock;
uint32_t Tact::busClock;
Tact::Tact ()
:ircSlowClock(32), ircFastClock(4000){
	mcgOutClockPtr[0] = &fllClock;
    mcgOutClockPtr[1] = &mcgiIrClock;
    mcgOutClockPtr[2] = &extClock;
	init ();
	initFei();
}
Tact * Tact::getInstance()
{
	if (_instance == nullptr)
	{
		_instance = new Tact ();
	}
	return _instance;
}

void Tact::init ()
{
	setClockDivider (dividers::coreClock, clockDivider::div1);
	setClockDivider (dividers::busClock, clockDivider::div1);
	setClockDivider (dividers::flexBusClock, clockDivider::div2);
	setClockDivider (dividers::flashClock, clockDivider::div2);
	/*SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) //core clock divider - 1
								|SIM_CLKDIV1_OUTDIV2(0x00) //bus clock divider - 1
								|SIM_CLKDIV1_OUTDIV3(0x01) //flexBus clock divider - 2
								|SIM_CLKDIV1_OUTDIV4(0x01); //flash clock divider - 2*/
	setPeriphSource (periphSource::fllClock);
	setLptmrSource (lptmrSource::lpo);
}

uint32_t & Tact::getFrqBus (){
	return busClock;
}
void Tact::setPeriphSource (periphSource s){
	periphSourceValue = static_cast<uint8_t>(s);
	SIM->SOPT2 &= ~ SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 = periphSourceValue << SIM_SOPT2_PLLFLLSEL_SHIFT;
}
uint8_t Tact::getPeriphSource (){
	return SIM->SOPT2 & SIM_SOPT2_PLLFLLSEL_MASK >> SIM_SOPT2_PLLFLLSEL_SHIFT;
}
void Tact::setLptmrSource (lptmrSource s)
{
	nLptmrSource = static_cast<uint8_t>(s);
	SIM->SOPT1 &= ~ SIM_SOPT1_OSC32KSEL_MASK;
	SIM->SOPT1 |= nLptmrSource << SIM_SOPT1_OSC32KSEL_SHIFT;
}

void Tact::initFei ()
{
	setClockSourceSelect (clockSourceSelect::fllPll);
	setReferenceClockFll (fllSource::internal);
	setPll (false);
	/*mcgDriver.setFreqRange (Mcg::freqRange::vHighFreq);
	mcgDriver.setExtSource (Mcg::extSource::oscillator);
	mcgDriver.setIntSource (Mcg::intSource::slow);
	while (!mcgDriver.getFllSource());
	while (mcgDriver.getMcgOutClk()!= 0);*/
	
}
void Tact::setClockSourceSelect (clockSourceSelect source){
	clockSource = static_cast <uint8_t>(source);
	MCG->C1 &=~ MCG_C1_CLKS_MASK;
	MCG->C1 |= MCG_C1_CLKS(clockSource);
    mcgOutClock = *mcgOutClockPtr[clockSource];
    while (getClockSourceSelect()!=clockSource);
}
uint8_t Tact::getClockSourceSelect (){
	return (MCG->S & MCG_S_CLKST_MASK)>>MCG_S_CLKST_SHIFT;
}
void Tact::setExtFllDivider(fllDivider d){
	extFllDividerValue = static_cast <uint8_t>(d);
	MCG->C1 &=~ MCG_C1_FRDIV_MASK;
	MCG->C1 |= MCG_C1_FRDIV(extFllDividerValue);
   // fllClock /= extFllDividerArr[fllSourceValue][fllDividerValue];
}

void Tact::setClockDivider (dividers div, clockDivider clk){
	SIM->CLKDIV1 &= ~ 0x0F << static_cast<uint8_t>(div);
	SIM->CLKDIV1 |= static_cast<uint8_t>(clk) << static_cast<uint8_t>(div);
}
void Tact::setReferenceClockFll (fllSource s){
	MCG->C1 &= ~ MCG_C1_IREFS_MASK;
	MCG->C1 |= static_cast<uint8_t>(s) << MCG_C1_IREFS_SHIFT;
	while (getReferenceClockFll()!= static_cast<uint8_t>(s));
}
uint8_t Tact::getReferenceClockFll (){
	return (MCG->S & MCG_S_IREFST_MASK)>>MCG_S_IREFST_SHIFT;
}
void Tact::setPll (bool state){
	MCG->C6 &= ~MCG_C6_PLLS_MASK;
	MCG->C6 |= state << MCG_C6_PLLS_SHIFT;
	while (getPllstate()!=state);
}
uint8_t Tact::getPllstate (){
	return (MCG->S & MCG_S_PLLST_MASK)>>MCG_S_PLLST_SHIFT;
}
