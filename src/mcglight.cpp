#include "mcglight.h"

uint16_t Tact::cpuClock;
uint16_t Tact::busClock;
uint16_t Tact::mcgirClock;

Tact::Tact ()
{
	initHIRC ();
}

void Tact::initHIRC ()
{
	/* SIM->CLKDIV1: OUTDIV1=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |//set prescaler for system and busclock
	SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers for busclock */
	/* MCG->SC: FCRDIV=1 */
	/* MCG->MC: HIRC=1,LIRC_DIV2=0 */
	MCG->MC = MCG_MC_HIRCEN_MASK;    /* Enable HIRC clock source*/
	/* OSC0->CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	OSC0->CR = 0;         /* Disable External reference */
	/* MCG->C2: RANGE0=0,HGO0=0,EREFS0=0,IRCS=1 */
	/* Switch to HIRC Mode */
	/* MCG->C1: CLKS=0,IRCLKEN=1,IREFSTEN=0 */
	MCG->C1 = MCG_C1_CLKS(0x00) |
	MCG_C1_IRCLKEN_MASK;       /* Leave LIRC enabled and select HIRC as a clock source */
	while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) ; /* Check that the clock source is the HIRC clock. */

	cpuClock = 48000;
	busClock = 24000;
	mcgirClock = 8000;
}

void Tact::get_LIRC_8 (divider div_1, divider div_2)
{
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |//set prescaler for system and busclock
	SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers for busclock */
	/* MCG->SC: FCRDIV=1 */
	/* MCG->MC: HIRC=1,LIRC_DIV2=0 */

	// Enable HIRC clock source
	MCG->MC = MCG_MC_HIRCEN_MASK;

	//select HIRC as a clock source
	MCG->C1 = MCG_C1_CLKS(0x00);

	// Check that the clock source is the HIRC clock
	while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) ;

	//Turn on LIRC8
	MCG->C2 = MCG_C2_IRCS_MASK;
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;       /* Cler LIRC1 divider */
	MCG->SC |= MCG_SC_FCRDIV(div_1);	  /* Set LIRC1 divider */
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;	  /* Cler LIRC2 divider */
	MCG->MC |= MCG_MC_LIRC_DIV2(div_2);   /* Set LIRC2 divider */
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
	SIM_CLKDIV1_OUTDIV4(0x00); /* Set system prescalers */
	OSC0->CR = 0;         /* Disable External reference */

	//Select LIRC as a clock source
	MCG->C1 = MCG_C1_CLKS(0x01) |MCG_C1_IRCLKEN_MASK;
	while((MCG->S & MCG_S_CLKST_MASK) != 0x01) ; /* Check that the clock source is the LIRC clock. */
	cpuClock = 8/(1<<div_1);
	busClock = 8/(1<<div_1);
	mcgirClock = 8/((1<<div_1)*(1<<div_2));
}

void Tact::get_LIRC_2 (divider div_1, divider div_2)
{
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |//set prescaler for system and busclock
	SIM_CLKDIV1_OUTDIV4(0x01); /* Set system prescalers for busclock */
	/* MCG->SC: FCRDIV=1 */
	/* MCG->MC: HIRC=1,LIRC_DIV2=0 */

	// Enable HIRC clock source
	MCG->MC = MCG_MC_HIRCEN_MASK;

	//select HIRC as a clock source
	MCG->C1 = MCG_C1_CLKS(0x00);

	// Check that the clock source is the HIRC clock
	while((MCG->S & MCG_S_CLKST_MASK) != 0x00u) ;

	//Turn on LIRC2
	MCG->C2 &= ~MCG_C2_IRCS_MASK;
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;       /* Cler LIRC1 divider */
	MCG->SC |= MCG_SC_FCRDIV(div_1);	  /* Set LIRC1 divider */
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;	  /* Cler LIRC2 divider */
	MCG->MC |= MCG_MC_LIRC_DIV2(div_2);   /* Set LIRC2 divider */
	SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
	SIM_CLKDIV1_OUTDIV4(0x00); /* Set system prescalers */
	OSC0->CR = 0;         /* Disable External reference */

	//Select LIRC as a clock source
	MCG->C1 = MCG_C1_CLKS(0x01) |MCG_C1_IRCLKEN_MASK;
	while((MCG->S & MCG_S_CLKST_MASK) != 0x01) ; /* Check that the clock source is the LIRC clock. */
	cpuClock = 2/(1<<div_1);
	busClock = 2/(1<<div_1);
	mcgirClock = 2/((1<<div_1)*(1<<div_2));

}

void Tact::set_LIRC_div (mode m, divider div_1, divider div_2)
{
	//Clear LIRC source
	MCG->C2 &= ~MCG_C2_IRCS_MASK;
	//Set LIRC source
	MCG->C2 |= m << MCG_C2_IRCS_SHIFT;
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;       /* Cler LIRC1 divider */
	MCG->SC |= MCG_SC_FCRDIV(div_1);	  /* Set LIRC1 divider */
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;	  /* Cler LIRC2 divider */
	MCG->MC |= MCG_MC_LIRC_DIV2(div_2);   /* Set LIRC2 divider */
	if (m) mcgirClock = 8/((1<<div_1)*(1<<div_2));
	else mcgirClock = 2/((1<<div_1)*(1<<div_2));
}
