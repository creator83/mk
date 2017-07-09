#include "horline.h"

Horline::Horline(uint16_t x_, uint16_t y_, uint16_t * color_, uint16_t length_, uint8_t thick_)
:x(x_), y(y_), colorPtr (color_), length (length_), thick (thick_)
{

}
void Horline::draw () const
{
	displayDriver->horLine (x, y, colorPtr, length, thick);
}

void Horline::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
