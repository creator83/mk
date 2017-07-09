#include "device.h"                // Device header
#include "shape.h"
#include "colors16bit.h"

#ifndef PIXEL_H
#define PIXEL_H

class Pixel : public Shape
{
private:
	uint16_t x, y, color;
public:

	Pixel (uint16_t x_, uint16_t y_, const uint16_t color_);
	void draw () const override;
	void setPosition (uint16_t, uint16_t) override;
};

#endif
