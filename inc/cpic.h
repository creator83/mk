#include "device.h"               // Device header
#include "shape.h"
#include "colors16bit.h"

#ifndef CPIC_H
#define CPIC_H

class ColorPicture : public Shape
{
private:
	uint16_t x, y, length, width;
	const uint16_t * arr;
	uint32_t picAddress;
	uint8_t ptrF;
	using setF = void (ColorPicture::*) ()const;
	static setF fMode [2];
public:

	ColorPicture (uint16_t x_, uint16_t y_, const uint16_t * pic, uint16_t length_, uint16_t width_);
	ColorPicture (uint16_t x_, uint16_t y_, uint32_t pic, uint16_t length_, uint16_t width_);
	void draw () const override;
	void drawPtr()const;
	void drawAddr()const;
	void setPosition (uint16_t, uint16_t) override;


};

#endif
