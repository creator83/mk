#include "device.h"               // Device header
#include "shape.h"
#include "colors16bit.h"

#ifndef MPIC_H
#define MPIC_H

class MonoPicture : public Shape
{
private:
	uint16_t x, y, color, fon, l, width;
	const uint8_t * arr;

public:

	MonoPicture (uint16_t x_, uint16_t y_, const uint16_t color_, uint16_t fon_, const uint8_t *arr_, uint16_t l_, uint16_t width_);
	void draw () const override;
	void setPosition (uint16_t, uint16_t) override;
};

#endif
