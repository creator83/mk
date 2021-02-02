#include "state.h"
uint32_t State::cpuClock;
uint32_t State::busClock;
uint32_t State::mcgIRClock;
uint32_t State::mcgFFClock;
uint32_t State::mcgFLLClock;

State * State::instancePtr = nullptr;

State::State ()
{
      SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                    SIM_CLKDIV1_OUTDIV2(0x00) |
                    SIM_CLKDIV1_OUTDIV4(0x01); /* Update system prescalers */
  /* SIM_SOPT2: PLLFLLSEL&=~1 */
  SIM_SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x01)); /* Select FLL as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=3 */
  SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03); /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
    /* Switch to FEI Mode */
    /* MCG_C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
    MCG_C1 = MCG_C1_CLKS(0x00) |
             MCG_C1_FRDIV(0x00) |
             MCG_C1_IREFS_MASK |
             MCG_C1_IRCLKEN_MASK;
    /* MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
    MCG_C2 = (uint8_t)((MCG_C2 & (uint8_t)~(uint8_t)(
              MCG_C2_LOCRE0_MASK |
              MCG_C2_RANGE(0x01) |
              MCG_C2_HGO_MASK |
              MCG_C2_LP_MASK |
              MCG_C2_IRCS_MASK
             )) | (uint8_t)(
              MCG_C2_RANGE(0x02) |
              MCG_C2_EREFS_MASK
             ));
    /* MCG_C4: DMX32=0,DRST_DRS=0 */
    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
    //MCG_C4 |= MCG_C4_DRST_DRS(0x01);
    /* OSC_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    OSC_CR = OSC_CR_ERCLKEN_MASK;
    /* MCG_C7: OSCSEL=0 */
    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
    }
    while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
    }
	//initFei();
    //SMC->PMPROT |= SMC_PMPROT_AHSRUN_MASK;
    //SMC->PMCTRL |= SMC_PMCTRL_RUNM_MASK;
   /* SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01)  //core clock divider
                  |SIM_CLKDIV1_OUTDIV2(0x01)  //bus clock divider
                  |SIM_CLKDIV1_OUTDIV4(0x03); //flash clock divider
    MCG->C4 &= ~(MCG_C4_DMX32_MASK|MCG_C4_DRST_DRS_MASK);
    MCG->C4 |= MCG_C4_DRST_DRS(3);*/
    busClock = 20480;
    cpuClock = 20480;
    /*busClock = 40960;
    cpuClock = 40960;*/
}

State & State::getInstance(){
    if (instancePtr == nullptr){
        State();
    }
    return *instancePtr;
}

uint32_t & State::getFrqBus ()
{
	return busClock;
}
uint32_t & State::getFrqCpu (){
    return cpuClock;
}
uint32_t & State::getFrqFlash (){
    return busClock;
}
void State::setPllFllSource (pllFllSource s)
{
	nPllFllSource = static_cast<uint8_t>(s);
	SIM->SOPT2 &= ~ SIM_SOPT2_PLLFLLSEL_MASK;
	SIM->SOPT2 = nPllFllSource << SIM_SOPT2_PLLFLLSEL_SHIFT;
}

void State::setLptmrSource (lptmrSource s){
	nLptmrSource = static_cast<uint8_t>(s);
	SIM->SOPT1 &= ~ SIM_SOPT1_OSC32KSEL_MASK;
	SIM->SOPT1 |= nLptmrSource << SIM_SOPT1_OSC32KSEL_SHIFT;
}

void State::initFei (){
    mcg.setCoreDivider(Mcg::mcgOutClkDivider::div1);
    mcg.setBusDivider (Mcg::mcgOutClkDivider::div1);
    mcg.setFlashDivider(Mcg::mcgOutClkDivider::div2);

	setPllFllSource (State::pllFllSource::mcgFllClk);
	setLptmrSource (State::lptmrSource::lpo); 
    mcg.setFllSource (Mcg::fllSource::internal);
	mcg.setMcgOutClk (Mcg::mcgOutSource::fll);
	mcg.setFllDivider (Mcg::fllDivider::div1_32);
	//while (mcg.getFllSource()!= mcg.getFllSourceValue());
	mcg.mcgIrclkEnable ();
	mcg.setFreqRange (Mcg::freqRange::vHighFreq);
	mcg.setExtSource (Mcg::extSource::oscillator);
	mcg.setIntSource (Mcg::intSource::slow);
    volatile uint32_t clock = mcg.getMcgOutClk();
    volatile uint32_t divider = mcg.getCoreDivider();
    cpuClock = mcg.getMcgOutClk()/(mcg.getCoreDivider()+1);
    busClock = mcg.getMcgOutClk()/mcg.getBusDivider();
}



