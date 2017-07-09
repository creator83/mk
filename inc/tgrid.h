#include "device.h"                 // Device header
#include "xpt2046.h"



#ifndef TGRID_H
#define TGRID_H

class Tgrid
{

//variables
public:
private:
  uint16_t xGrid, yGrid;
  uint8_t h,v;
  Xpt2046 * driver;
//func
public:
  Tgrid ( Xpt2046 &, uint8_t hor, uint8_t ver);
  uint16_t & getXgrid ();
  uint16_t & getYgrid ();
  uint8_t & getV ();
  Xpt2046 * getDriver ();
};

#endif
