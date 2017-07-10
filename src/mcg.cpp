#include "mcg.h"

uint16_t Tact::cpuClock;
uint16_t Tact::busClock;
uint16_t Tact::mcgirClock;
uint16_t Tact::mcgpllClock;
uint16_t Tact::mcgfllClock;

Tact::Tact ()
{
	initFei ();
	cpuClock = 20971;
	busClock = 20971;
	mcgirClock = 32;
}

void Tact::initFei ()
{
	/* Predefined clock setups
	   0 ... Default  part configuration
	         Multipurpose Clock Generator (MCG) in FEI mode.
	         Reference clock source for MCG module: Slow internal reference clock
	         Core clock = 20.97152MHz
	         Bus clock  = 20.97152MHz*/
	// SMC_PMPROT: AHSRUN=1,??=0,AVLP=1,??=0,ALLS=0,??=0,AVLLS=0,??=0
	//Setup Power mode protection register
	  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK);
	  // SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0
	  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
	                SIM_CLKDIV1_OUTDIV2(0x00) |
	                SIM_CLKDIV1_OUTDIV4(0x01);
	  // SIM_SOPT2: PLLFLLSEL&=~1
	  // Select FLL as a clock source for various peripherals
	  SIM_SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x01));
	  // SIM_SOPT1: OSC32KSEL=3
	  // LPO 1kHz oscillator drives 32 kHz clock for various peripherals
	  SIM_SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03);
	 // Switch to FEI Mode
	 // MCG_C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0
	    MCG_C1 = MCG_C1_CLKS(0x00) |
	             MCG_C1_FRDIV(0x00) |
	             MCG_C1_IREFS_MASK |
	             MCG_C1_IRCLKEN_MASK;
	 // MCG_C2: LOCRE0=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0
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
	    // MCG_C4: DMX32=0,DRST_DRS=0
	    MCG_C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
	    /* OSC_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	    OSC_CR = OSC_CR_ERCLKEN_MASK;
	    /* MCG_C7: OSCSEL=0 */
	MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
	while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	}
	while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
	}
}


void Tact::initFee ()
{
	/*1 ... Maximum achievable clock frequency configuration
	         Multipurpose Clock Generator (MCG) in FEE mode.
	         Reference clock source for MCG module: System oscillator 0 reference clock
	         Core clock = 80MHz
	         Bus clock  = 40MHz*/
	  /* SMC_PMPROT: AHSRUN=1,??=0,AVLP=1,??=0,ALLS=0,??=0,AVLLS=0,??=0 */
	  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
	  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=1,??=0,??=0,??=0,??=0,OUTDIV4=3,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
	  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
	                SIM_CLKDIV1_OUTDIV2(0x01) |
	                SIM_CLKDIV1_OUTDIV4(0x03); /* Update system prescalers */
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
	    /* OSC_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	    OSC_CR = OSC_CR_ERCLKEN_MASK;
	    /* MCG_C7: OSCSEL=0 */
	    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
	    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	    }
	    while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
	    }
}

void Tact::initBlpi ()
{
	 /* SMC_PMPROT: AHSRUN=1,??=0,AVLP=1,??=0,ALLS=0,??=0,AVLLS=0,??=0 */
	  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
	  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=0,??=0,??=0,??=0,??=0,OUTDIV4=4,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
	  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
	                SIM_CLKDIV1_OUTDIV2(0x00) |
	                SIM_CLKDIV1_OUTDIV4(0x04); /* Update system prescalers */
	  /* SIM_SOPT2: PLLFLLSEL|=1 */
	  SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(0x01); /* Select PLL as a clock source for various peripherals */
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
	    /* OSC_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	    OSC_CR = OSC_CR_ERCLKEN_MASK;
	    /* MCG_C7: OSCSEL=0 */
	    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
	    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	  }
	    while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
	    }
}

void Tact::initBlpe ()
{
	/* SMC_PMPROT: AHSRUN=1,??=0,AVLP=1,??=0,ALLS=0,??=0,AVLLS=0,??=0 */
	  SMC_PMPROT = (SMC_PMPROT_AHSRUN_MASK | SMC_PMPROT_AVLP_MASK); /* Setup Power mode protection register */
	  /* SIM_CLKDIV1: OUTDIV1=1,OUTDIV2=1,??=0,??=0,??=0,??=0,OUTDIV4=7,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
	  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x01) |
	                SIM_CLKDIV1_OUTDIV2(0x01) |
	                SIM_CLKDIV1_OUTDIV4(0x07); /* Update system prescalers */
	  /* SIM_SOPT2: PLLFLLSEL|=1 */
	  SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL(0x01); /* Select PLL as a clock source for various peripherals */
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
	    /* OSC_CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	    OSC_CR = OSC_CR_ERCLKEN_MASK;
	    /* MCG_C7: OSCSEL=0 */
	    MCG_C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL(0x03));
	    while((MCG_S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
	    }
	    while((MCG_S & 0x0CU) != 0x00U) {  /* Wait until output of the FLL is selected */
	    }
}
