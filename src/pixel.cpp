#include "pixel.h"

Pixel::Pixel (uint16_t x_, uint16_t y_, const uint16_t color_)
:x(x_), y (y_), color (color_)
{

}

void Pixel::draw () const
{
	displayDriver->pixel(x,y,color);
}

void Pixel::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
