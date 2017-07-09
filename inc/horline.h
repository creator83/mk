#include "device.h"                  // Device header
#include "shape.h"
#include "colors16bit.h"

#ifndef HORLINE_H
#define HORLINE_H

class Horline : public Shape
{
private:
	uint16_t x, y, length;
	uint8_t thick;
	uint16_t * colorPtr;
public:

	Horline (uint16_t x_, uint16_t y_, uint16_t * color_, uint16_t length_, uint8_t thick_);
	void draw () const override;
	void setPosition (uint16_t, uint16_t) override;
};

#endif
