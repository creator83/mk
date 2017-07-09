#include "list.h"

List::List(ListButton * b)
:buttons(b)
{
	count = 0;
	first = last = nullptr;
}

List::~List()
{
	Item * current = nullptr;
	Item * next = first;
	while (next->next != nullptr)
	{
		current = next;
		next = next->next;
		delete current;
	}
	current = last;
	delete current;
}



void List::addFirst (Shape * o)
{
	Item * newItem = new Item (o, first);
	if (first == nullptr)
	{
		last = newItem;
	}
	first = newItem;
	++count;
}

void List::addLast (Shape * o)
{
	Item * newItem = new Item (o);
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

void List::iterate ()
{
	Item * next = first;
	//uint16_t counter = count;
	function ();
	for (uint16_t i=0;i<count;++i)
	{
		next->object->draw();
		next = next->next;
	}
}

void List::iterateData ()
{
	Item * next = first;
	for (uint16_t i=0;i<count;++i)
	{
		if (Data * ptr = dynamic_cast <Data *>(next->object)) next->object->draw();
		next = next->next;
	}
}

void List::print (uint16_t n)
{
	Item * current = nullptr;
	Item * next = first;
	while (n--)
	{
		current = next;
		next = next->next;
	}
	next->object->draw();
}

void List::setFunction (void (*f)())
{
	function = f;
}

