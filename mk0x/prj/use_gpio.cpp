#include "device.h"                   // Device header
#include "gpio.h"
#include "pin.h"
#include "port.h"
#include "tact.h"
#include "delay.h"
#include "dispatcher.h"


Pin led1 (Gpio::Port::D, 5);
Pin led2 (Gpio::Port::D, 6);
Pin led3 (Gpio::Port::D, 7);

void f1 ();
void f2 ();
void f3 ();
void f4 ();
class Item
{
	public:
	void (*ptrF)();
	Item * next;
	Item * prev;
	uint16_t counter, set;
	Item (void (*f)(), uint16_t c, Item * n = nullptr,  Item * p = nullptr);
	void decrCounter ();
	uint16_t & getCounter ();
	uint16_t & getSet ();
	void (*getPtrF(void)) ();
};

int main ()
{
	Tact::getInstance ();
	Dispatcher turn;
	turn.addTask (f1, 4);
	turn.addTask (f2, 6);
	turn.addTask (f3, 2);
	turn.addTask (f4, 3);

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

Item::Item (void (*f)(), uint16_t c, Item * n,  Item * p)
	:ptrF(f), next(n), prev(p), counter(c), set(c)
{
}
