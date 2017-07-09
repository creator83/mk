#include "device.h"                 // Device header
#include "tgrid.h"




#ifndef LISTBUTTON_H
#define LISTBUTTON_H

class ListButton
{

struct Item
{
	uint16_t key;
	void (*function)();
	Item * next;
	Item (uint16_t k, void (*f)(), Item * n = nullptr){key = k; function = f; next = n;}
};
//variables
public:
private:
  uint16_t xValue, yValue;
  uint8_t hValue, vValue, result;
  Item * first;
  Item * last;
  uint16_t count;
  Tgrid * grid;
  Item * current;
//func
public:
  ListButton (Tgrid &);
  ~ListButton ();
  void addButton (uint16_t,  void (*f)());
  void setCount (uint8_t hor, uint8_t ver);
  void calculateTouch ();
  uint8_t & getResult ();
  void setCurrent ();
private:
  void searchKey (uint8_t k);

};

#endif
