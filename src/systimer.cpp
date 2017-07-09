#include "systimer.h"


Systimer::Systimer(mode m, uint16_t val)
{
	switch (static_cast <uint8_t> (m))
	{
		case 0: 
			SysTick_Config (Tact::getFrqCpu()*val);
		break;
		case 1:
			SysTick_Config (Tact::getFrqCpu()*1000*val);
		break;
		case 2:
			SysTick_Config (Tact::getFrqCpu()*1000000*val);
		break;	
	}
}

void Systimer::interruptEnable ()
{
	
}

void Systimer::interruptDisable ()
{
}
