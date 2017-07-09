#include "device.h"


#ifndef FONT_H
#define FONT_H


class Font
{
	const uint8_t * font;
	uint8_t width;
	uint8_t height;
	uint8_t shift;
public:
	Font (const uint8_t *f);
	void setWidth (uint8_t);
	void setHeight (uint8_t);
	void setShift (uint8_t);
	uint8_t & getWidth ();
	uint8_t & getHeight ();
	uint8_t & getShift ();
	const uint8_t * getFont ();
	const uint8_t * getFont (uint16_t);
};


#endif
