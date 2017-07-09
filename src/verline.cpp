#include "verline.h"

Verline::Verline (uint16_t x_, uint16_t y_, uint16_t * color_, uint16_t length_, uint8_t thick_)
:x(x_), y(y_), color (color_), length (length_), thick (thick_)
{

}

void Verline::draw () const
{
	displayDriver->verLine(x, y, color, length, thick);
}

void Verline::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
