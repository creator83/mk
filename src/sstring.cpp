#include "sstring.h"

Sstring::Sstring (uint16_t x_, uint16_t y_, uint16_t color_, uint16_t fon_, char *str_, Font * f, uint8_t interval_)
:x(x_), y(y_), color (color_), fon (fon_), interval (interval_)
{
	font_ = f;
	uint8_t l=0;
	while (*str_--)++l;
	str = new char [l+1];
	while (*str++=*str_++);
	str[l] = 0;
	nChar = l+1;
}

Sstring::Sstring (uint16_t x_, uint16_t y_, uint16_t color_, uint16_t fon_, uint8_t n, Font *f, uint8_t interval_)
:x(x_), y(y_), color (color_), fon (fon_), interval (interval_)
{
	font_ = f;
	str = new char [n+1];
	str[n] = 0;
	nChar = n+1;
}

void Sstring::setElement (uint8_t n, char val)
{
	char * ptr = &str[n];
	*ptr = val;
}

void Sstring::setFont (Font *f)
{
	font_ = f;
}

void Sstring::copy (const char *s)
{
	uint8_t l=0;
	while (*s--)++l;
	if (l<=nChar)
	{
		while (*str++=*s++);
	}

}

Sstring::~Sstring ()
{
	delete [] str;
}

void Sstring::draw () const
{
	displayDriver->string(x, y, color, fon, str, *font_, interval);
}

void Sstring::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
