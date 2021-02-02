#include "delay.h"

void delay_ms (uint16_t del)
{
	Pit ms (pitChannel::ch3, del, Pit::mode::ms);
	ms.start();
	while (!(ms.flag_TIF()));
	ms.stop();
	ms.clear_flag();
}
void delay_us (uint16_t del)
{
	Pit us (pitChannel::ch3, del, Pit::mode::us);
	us.start();
	while (!(us.flag_TIF()));
	us.stop();
	us.clear_flag();
}
