#include "font.h"

Font::Font (const uint8_t *f)
:font(f)
{

}

void Font::setWidth (uint8_t w)
{
	width = w;
}

void Font::setHeight (uint8_t h)
{
	height = h;
}

void Font::setShift (uint8_t s)
{
	shift = s;
}

uint8_t & Font::getWidth ()
{
	return width;
}

uint8_t & Font::getHeight ()
{
	return height;
}

uint8_t & Font::getShift ()
{
	return shift;
}

const uint8_t * Font::getFont ()
{
	return font;
}

const uint8_t * Font::getFont (uint16_t ch)
{
	return &font[ch];
}
