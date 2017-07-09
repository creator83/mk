#include "tftdriver.h"
#include "ili9341.h"
#include "flashspi.h"


#ifndef LCDFLASH_H
#define LCDFLASH_H


class Lcdflash: public Tftdriver
{
//variables
private:
	Ili9341 * lcdDriver;
	Flash * memory;
//functions
public:
	Lcdflash(Ili9341 &, Flash &);

	void drawArr (uint16_t x , uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width) override;
	void drawPic (uint16_t x , uint16_t y, const uint16_t *arr, uint16_t length, uint16_t height) override;
	void drawPic (uint16_t x , uint16_t y, uint32_t address, uint16_t length, uint16_t height) override;
	void symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, Font & s) override;
	void string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, Font &f, int8_t interval) override;
	void horLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick) override;
	void verLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick) override;
	void rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height, uint8_t thick) override;
	void rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height) override;
	void pixel (uint16_t x , uint16_t y, const uint16_t color)override;
};

#endif
