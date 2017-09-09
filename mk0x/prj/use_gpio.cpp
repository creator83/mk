#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"
#include "port.h"
#include "tact.h"
#include "delay.h"
#include "dispatcher.h"
Tact frq;

Pin led1 (Gpio::Port::D, 5);
Pin led2 (Gpio::Port::D, 6);
Pin led3 (Gpio::Port::D, 7);

void f1 ();
void f2 ();
void f3 ();
void f4 ();

int main ()
{

	Dispatcher turn;
	turn.addTask (f1, 10);
	turn.addTask (f2, 15);
	turn.addTask (f3, 20);
	turn.addTask (f4, 25);

	while (1)
	{
		turn.tickTask();
		turn.checkTasks();
		turn.checkQueue();
	}
}

void f1 ()
{
	led1.togle();
}
void f2 ()
{
	led2.togle();
}
void f3 ()
{
	led3.togle();
}
void f4 ()
{
}
