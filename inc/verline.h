#include "device.h"               // Device header
#include "shape.h"
#include "colors16bit.h"

#ifndef VERLINE_H
#define VERLINE_H

class Verline : public Shape
{
private:
	uint16_t x, y, length;
	uint8_t thick;
	uint16_t * color;
public:

	Verline (uint16_t x_, uint16_t y_, uint16_t * color_, uint16_t length_, uint8_t thick_);
	void draw () const override;
	void setPosition (uint16_t x_, uint16_t y_) override;
};

#endif
