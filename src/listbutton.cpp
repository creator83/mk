#include "listbutton.h"

ListButton::ListButton ( Tgrid & g)
:count (0), first (nullptr), last (nullptr), current (nullptr)
{
	grid = &g;
}

ListButton::~ListButton ()
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

void ListButton::addButton (uint16_t k,  void (*f)())
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



void ListButton::calculateTouch ()
{
	result = grid->getDriver()->getX()/grid->getXgrid();
	result *= grid->getV();
	uint8_t temp = grid->getDriver()->getY()/grid->getYgrid();
	result += temp;
	searchKey (result);
}

uint8_t & ListButton::getResult ()
{
	return result;
}


void ListButton::searchKey (uint8_t k)
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

void ListButton::setCurrent ()
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



