#include "tact.h"

uint32_t Tact::cpuClock = 0;
uint32_t Tact::busClock = 0;
uint32_t Tact::flexBusClock = 0;
uint32_t Tact::flashClock = 0;
uint32_t Tact::mcgIRClock = 0;
uint32_t Tact::mcgFFClock = 0;
uint32_t Tact::mcgFLLClock = 0;

Tact::ptrModeFunc Tact::modeFunc [6] = {&Tact::initFei, &Tact::initFee,
										&Tact::initFbi, &Tact::initFbe,
										&Tact::initBlpi, &Tact::initBlpe};

Tact::Tact ()
{
	initFei();
}
Tact::Tact (clockMode mode){
	clockMode_ = static_cast<uint8_t>(mode);
	(this->*(Tact::modeFunc[clockMode_]))();
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
	/*nPllFllSource = static_cast<uint8_t>(s);
	SIM->SOPT2 &= ~ SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 = nPllFllSource << SIM_SOPT2_PLLFLLSEL_SHIFT;*/
}
/*
void Tact::setLptmrSource (lptmrSource s){
	nLptmrSource = static_cast<uint8_t>(s);
	SIM->SOPT1 &= ~ SIM_SOPT1_OSC32KSEL_MASK;
	SIM->SOPT1 |= nLptmrSource << SIM_SOPT1_OSC32KSEL_SHIFT;
}*/

void Tact::initFei (){
    //uint8_t 
	mcg.setMcgOutClk(Mcg::mcgOutSource::fll);
	mcg.setFllSource (Mcg::fllSource::internal);
	mcg.setIntSource (Mcg::intSource::slow);
	cpuClock = mcg.getInternalSlowClock();
	busClock = cpuClock;
	altAdcClock = cpuClock;
	mcg.setFllDivider (Mcg::fllDivider::div1_32);

    mcg.setFreqRange (Mcg::freqRange::lowFreq);
    mcg.setGain (Mcg::gainOsc::highGain);


	//setLptmrSource (Tact::lptmrSource::lpo);
    //enable MCGIRCLK, setOut 4Mhz
	mcg.mcgIrclkEnable ();
	mcg.setIntSource (Mcg::intSource::fast);

	mcg.setExtSource (Mcg::extSource::oscillator);
	setCoreClockDvider(dividers::div1);
	cpuClock = cpuClock/coreClockDivider;
	setBusClockDvider(dividers::div2);
	busClock = busClock/busClockDivider;
	setFlashClockDvider(dividers::div4);
	flashClock = flashClock/flashClockDivider;

	mcg.setFllMultiplication(Mcg::dcoRange::notFine, Mcg::multiplicationRangeFll::midHighRange);
	cpuClock = cpuClock*1920;
	busClock = busClock *1920;
	altAdcClock = altAdcClock*1920;
}

void Tact::initFee (){
	//OSC0->CR |= OSC_CR_ERCLKEN_MASK;
	uint8_t temp_register;
	OSC->CR = OSC_CR_ERCLKEN_MASK
	|OSC_CR_EREFSTEN_MASK
	|OSC_CR_SC16P_MASK;
	temp_register = MCG->C2;
	temp_register &=~ (MCG_C2_RANGE_MASK | MCG_C2_HGO_MASK | MCG_C2_EREFS_MASK);

	temp_register |= (MCG_C2_RANGE(2) | MCG_C2_HGO_MASK | MCG_C2_EREFS(1));
	MCG->C2 = temp_register;
	while (!(MCG->S & MCG_S_OSCINIT0_MASK));

	mcg.setExtSource (Mcg::extSource::external);
	mcg.setMcgOutClk(Mcg::mcgOutSource::fll);
	mcg.setFllDivider (Mcg::fllDivider::div8_256);
	mcg.setFllSource (Mcg::fllSource::external);
	mcg.setIntSource (Mcg::intSource::fast);

	cpuClock = 39062;
	busClock = cpuClock;
	altAdcClock = cpuClock;
	mcg.setFreqRange (Mcg::freqRange::vHighFreq);
	mcg.setGain (Mcg::gainOsc::highGain);


	//setLptmrSource (Tact::lptmrSource::lpo);

	mcg.mcgIrclkEnable ();

	setCoreClockDvider(dividers::div1);
	cpuClock = cpuClock/coreClockDivider;
	setBusClockDvider(dividers::div2);
	busClock = busClock/busClockDivider;
	setFlashClockDvider(dividers::div4);
	flashClock = flashClock/flashClockDivider;
	setFlexBusClockDvider (dividers::div2);
	flexBusClock = flexBusClock/flexBusDivider;

	mcg.setFllMultiplication(Mcg::dcoRange::notFine, Mcg::multiplicationRangeFll::midHighRange);
	cpuClock = cpuClock*1920;
	busClock = busClock *1920;
}

void Tact::setCoreClockDvider (dividers d){
	coreClockDivider = static_cast<uint8_t>(d)+1;
	SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV1_MASK;
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV1(coreClockDivider);
}
void Tact::setBusClockDvider (dividers d){
	busClockDivider = static_cast<uint8_t>(d)+1;
	SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV2_MASK;
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV2(busClockDivider);
}
void Tact::setFlexBusClockDvider (dividers d){
	flexBusDivider = static_cast<uint8_t>(d)+1;
	SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV3_MASK;
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV3(flexBusDivider);
}
void Tact::setFlashClockDvider (dividers d){
	flashClockDivider = static_cast<uint8_t>(d)+1;
	SIM->CLKDIV1 &= ~ SIM_CLKDIV1_OUTDIV4_MASK;
	SIM->CLKDIV1 |= SIM_CLKDIV1_OUTDIV4(flashClockDivider);
}
void Tact::initFbi (){
	/*mcg.setMcgOutClk(Mcg::mcgOutSource::internal);
	mcg.setFllSource (Mcg::fllSource::internal);
	mcg.setIntSource (Mcg::intSource::fast);
	cpuClock = 4000000;
	busClock = cpuClock;
	altAdcClock = cpuClock;
	mcg.mcgIrclkEnable ();
	mcg.setIntSource (Mcg::intSource::fast);

	mcg.setExtSource (Mcg::extSource::oscillator);
	setCoreClockDvider(dividers::div1);
	cpuClock = cpuClock/coreClockDevider;
	setBusClockDvider(dividers::div1);
	busClock = busClock/busClockDevider;
	setAltAdcClockDvider(dividers::div1);
	altAdcClock = altAdcClock/altAdcClockDevider;*/
}
void Tact::initFbe (){

}
void Tact::initBlpi (){

}
void Tact::initBlpe (){

}

