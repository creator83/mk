#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"
#include "tact.h"


Pin led1 (Gpio::Port::D, 5);
Pin led2 (Gpio::Port::D, 6);
Pin led3 (Gpio::Port::D, 7);


int main ()
{
	Tact::getInstance();
	while (1)
	{
		
	}
}
