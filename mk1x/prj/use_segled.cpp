#include "MKL17Z4.h"                    // Device header
#include "gpio.h"
#include "delay.h"
#include "pit.h"
#include "tact.h"
#include "segled.h"

tact frq;
const char led = 0;
gpio D(gpio::D);
pit del(pit::ch0, 1, pit::ms);
segled indicator(segled::B);

struct flags
{
	unsigned led_indicator_delay : 1;
}flag;


extern "C"
{
	void PIT_IRQHandler();
}

void PIT_IRQHandler()
{
	del.clear_flag();
	if (flag.led_indicator_delay)flag.led_indicator_delay = 0;
	else
	{
		indicator.digit();
		flag.led_indicator_delay = 1;
	}

}


int main()
{

	D.setOutPin(led);
	indicator.get_buffer(2546);
	while (1)
	{

	}
}
