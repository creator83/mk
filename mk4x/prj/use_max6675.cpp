#include "MKL17Z4.h"                    // Device header
#include "gpio.h"
#include "delay.h"
#include "pit.h"
#include "tact.h"
#include "segled.h"
#include "max6675.h"

tact frq;
const char led = 0;
gpio D(gpio::D);
pit del(pit::ch0, 1, pit::ms);
segled indicator(segled::B);


struct
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

uint16_t result [4];

int main()
{
	uint16_t abs_result;
	D.setOutPin(led);
	max6675 sensor;

	while (1)
	{
		for (uint8_t i=0;i<4;++i)
		{
			result [i]=sensor.readCelsius();
			delay_ms(200);
		}
		abs_result=0;
		for (uint8_t i=0;i<4;++i)
		{
			abs_result += result [i];
		}
		abs_result >>=2;
		indicator.get_buffer (abs_result);
		//delay_ms(250);
	}
}
