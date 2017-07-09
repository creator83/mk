#include "rectangle.h"


Rectangle::Rectangle (uint16_t x_, uint16_t y_, uint16_t * color_, uint16_t length_, uint8_t width_, uint8_t thick_)
:x(x_), y(y_), color (color_), length (length_), width (width_), thick (thick_)
{

}

void Rectangle::draw() const
{
	displayDriver->rectangle(x, y, color, length, width, thick);
}

