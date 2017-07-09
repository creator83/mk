#include "device.h"                // Device header
#include "font.h"


#ifndef TFTDRIVER_H
#define TFTDRIVER_H

class Tftdriver
{
public:
	virtual void pixel (uint16_t x , uint16_t y, const uint16_t color) = 0;
	virtual void drawPic (uint16_t x,uint16_t y, const uint16_t * arr,uint16_t length,uint16_t width) = 0;
	virtual void drawPic (uint16_t x , uint16_t y, uint32_t address, uint16_t length, uint16_t height) = 0;
	virtual void drawArr (uint16_t x , uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width) = 0;
	virtual void symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, Font & s) = 0;
	virtual void string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, Font &f, int8_t interval) = 0;
	virtual void horLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick) = 0;
	virtual void verLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick) = 0;
	virtual void rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height, uint8_t thick) = 0;
	virtual void rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height) = 0;
	virtual ~Tftdriver(){}
};




#endif
