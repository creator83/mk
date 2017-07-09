#include "cpic.h"

ColorPicture::setF ColorPicture::fMode [2] = {&ColorPicture::drawPtr, &ColorPicture::drawAddr};

ColorPicture::ColorPicture (uint16_t x_, uint16_t y_, const uint16_t * pic, uint16_t length_, uint16_t width_)
:x(x_), y (y_), length (length_), width (width_), arr (pic), ptrF(0)
{
}

ColorPicture::ColorPicture (uint16_t x_, uint16_t y_, uint32_t pic, uint16_t length_, uint16_t width_)
:x(x_), y (y_), length (length_), width (width_), picAddress (pic), ptrF(1)
{
}
void ColorPicture::draw () const
{
	(this->*(ColorPicture::fMode[ptrF]))();
}

void ColorPicture::drawPtr() const
{
	displayDriver->drawPic (x, y, arr, length, width);
}

void ColorPicture::drawAddr() const
{
	displayDriver->drawPic (x, y, picAddress, length, width);
}

void ColorPicture::setPosition (uint16_t x_, uint16_t y_)
{
	x = x_;
	y = y_;
}
