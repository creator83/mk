#include "data.h"

const uint8_t Data::point [8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


Data::Data (uint16_t d, uint8_t n, Font * f)
:value (d)
{
	font_= f;
	nElements = n+2;
	data = new uint8_t [nElements];
	data [nElements-2] = '.';
}

Data::Data (uint16_t x_, uint16_t y_, uint16_t color_,uint16_t fon_, uint16_t d, uint8_t n, Font * f)
:value (d), x (x_), y (y_), color(color_), fon(fon_)
{
	font_ = f;
	nElements = n+1;
	data = new uint8_t [nElements];
	data [nElements-2] = '.';
	convert ();
}

Data::Data (uint16_t x_, uint16_t y_, uint16_t color_,uint16_t fon_, uint8_t n, Font *f)
:x (x_), y (y_), color(color_), fon(fon_)
{
	font_ = f;
	nElements = n+1;
	data = new uint8_t [nElements];
}
void Data::convert ()
{
	uint16_t temp = value;
	for (data[0]=0;temp>=10;++data[0])temp -=10;
	data[2] = temp%10;
}

void Data::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}

void Data::setValue (uint16_t val)
{
	value = val;
}

void Data::draw () const
{
	displayDriver->symbol (x, y, color, fon, data[0], *font_);
	displayDriver->drawArr(x+26, y-27, color, fon, point, 1, 8);
	displayDriver->symbol(x+36, y, color, fon, data[2], *font_);
}

Data::~Data ()
{
	delete[] data;
}
