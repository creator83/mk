#include "tact.h"

uint16_t Tact::cpuClock;
uint16_t Tact::busClock;
uint16_t Tact::mcgirClock;
uint16_t Tact::mcgpllClock;
uint16_t Tact::mcgfllClock;

Tact::Tact ()
{
	initFee ();
	cpuClock = 40960;
	busClock = 20480;
	mcgirClock = 32;
}

void Tact::initFee ()
{
	/* SIM->CLKDIV1: OUTDIV1=0OUTDIV4=2 */
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV4(1); /* Update system prescalers */

	/* Switch to FEI Mode */
	/* MCG->C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
	MCG->C1 = MCG_C1_IREFS_MASK|MCG_C1_IRCLKEN_MASK;

	/* MCG->C2: LOCRE0=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
	MCG->C2 = MCG_C2_FCFTRIM_MASK;
	/* MCG->C4: DMX32=0,DRST_DRS=1 */
	MCG->C4 &= ~(MCG_C4_DRST_DRS_MASK|MCG_C4_DMX32_MASK);
	MCG->C4 |= MCG_C4_DRST_DRS(1);

	//=== Use FLL ===//
	MCG->C5 = 0;
	MCG->C6 = 0;

	while((MCG->S & MCG_S_IREFST_MASK) == 0x00U);

	while((MCG->S & MCG_S_CLKST_MASK) != 0) ;
}

