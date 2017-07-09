#include "tbutton.h"

Tbutton::Tbutton ( Tgrid & g)
:count (0), first (nullptr), last (nullptr), current (nullptr)
{
	grid = &g;
}

Tbutton::~Tbutton ()
{
	Item * current = nullptr;
	Item * next = first;
	while (1)
	{
		current = next;
		next = next->next;
		delete current;
	}
}

void Tbutton::addButton (uint16_t k,  void (*f)())
{
	Item * newItem = new Item (k, f);
	if (last == nullptr)
	{
		first = newItem;
	}
	else
	{
		last->next = newItem;
	}
	last = newItem;
	++count;
}



void Tbutton::calculateTouch ()
{
	result = grid->getDriver()->getX()/grid->getXgrid();
	result *= grid->getV();
	uint8_t temp = grid->getDriver()->getY()/grid->getYgrid();
	result += temp;
	searchKey (result);
}

uint8_t & Tbutton::getResult ()
{
	return result;
}


void Tbutton::searchKey (uint8_t k)
{
	Item * next = first;
	for (uint16_t i=0;i<count;++i)
	{
		if (next->key == k)
		{
			next->function ();
			return;
		}
		next = next->next;
	}
}

void Tbutton::setCurrent ()
{
	current = first;
	for (uint16_t i=0;i<count;++i)
	{
		if (current->key == result)
		{
			return;
		}
		current = current->next;
	}
}



