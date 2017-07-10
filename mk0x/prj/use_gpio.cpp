#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"
#include "port.h"
#include "mcg.h"
#include "delay.h"

Tact frq;
const char ledPin = 16;


int main ()
{
	Pin led (Gpio::Port::A, ledPin);

	while (1)
	{
		led.togle();
		//delay_ms(1000);
	}
}
