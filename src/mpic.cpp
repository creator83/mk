#include "mpic.h"

MonoPicture::MonoPicture (uint16_t x_, uint16_t y_, const uint16_t color_, uint16_t fon_, const uint8_t *arr_, uint16_t l_, uint16_t width_)
:x(x_), y(y_), color (color_), fon(fon_), arr (arr_), l (l_), width (width_)
{

}

void MonoPicture::draw () const
{
	driver->drawArr (x, y, color, fon, arr, l, width);
}

void MonoPicture::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
