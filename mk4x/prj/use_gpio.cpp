#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"
#include "port.h"
#include "tact.h"
#include "delay.h"
#include "dispatcher.h"

//Tact frq;
Pin led1 (Gpio::Port::C, 7);
Pin led2 (Gpio::Port::C, 8);
Pin led3 (Gpio::Port::C, 9);
Pin led4 (Gpio::Port::B, 11);

void f1 ();
void f2 ();
void f3 ();
void f4 ();


int main ()
{

	Dispatcher turn;
	turn.addTask (f1, 4);
	turn.addTask (f2, 6);
	turn.addTask (f4, 3);
	turn.addTask (f3, 2);

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
	led4.togle();
}
