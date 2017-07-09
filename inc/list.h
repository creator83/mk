#include "device.h"
#include "shape.h"
#include "data.h"
#include "listbutton.h"


#ifndef LIST_H
#define LIST_H

class List {
struct Item
{
	Shape * object;
	Item * next;
	Item (Shape *o, Item * n = nullptr){object = o; next = n;}
};

	Item * first;
	Item * last;
	ListButton * buttons;
	void (*function)();
	uint16_t count;
public:
	List(ListButton *);
	~List();
	Shape* head () const {return first->object;}
	Shape* tail () const {return last->object;}
	void addFirst (Shape *);
	void addLast (Shape *);
	void iterate ();
	void iterateData ();
	void print (uint16_t);
	uint16_t & getCount (){return count;}
	void setFunction (void (*f)());
};



#endif /* LIST_H_ */
