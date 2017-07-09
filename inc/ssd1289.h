#include "MKL26Z4.h"
#include "delay.h"
#include "i8080.h"

#ifndef SSD1289_H
#define SSD1289_H

namespace ssd1289Reg
{
	const uint16_t ram = 0x0022;
	const uint16_t gddramX = 0x004E;
	const uint16_t gddramY = 0x004F;
	const uint16_t scanPosition = 0x000F;
	const uint16_t entryMode = 0x0011;
	const uint16_t verticalScroll1 = 0x0041;
	const uint16_t verticalScroll2 = 0x0042;

}


class Ssd1289
{
//variables
public:
	struct sFont
	{
		const uint8_t * font;
		uint8_t width;
		uint8_t height;
		uint8_t shift;
	};
protected:
private:
	I8080 driver;

//functions
public:
	Ssd1289();
	void pixel (uint16_t x , uint16_t y, const uint16_t color);
	void fillScreen (uint16_t color);
	void symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, sFont & s);
	//void symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, uint8_t ch, sFont & s);
	void string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, sFont &f, uint8_t interval);
	void string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, sFont &f, uint16_t n,uint8_t interval);
	void setCursor (uint16_t x , uint16_t y);
	void drawArr (uint16_t x , uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width);
	void drawPic (uint16_t x , uint16_t y, const uint16_t *arr, uint16_t length, uint16_t width);
	void horLine (uint16_t x, uint16_t y, const uint16_t color, uint16_t length, uint8_t thick);
	void verLine (uint16_t x, uint16_t y, const uint16_t color, uint16_t length, uint8_t thick);
	void line (uint16_t x, uint16_t y, uint16_t color, uint16_t length, uint8_t thick);
	void rectangle (uint16_t x, uint16_t y, uint16_t color, uint16_t length, uint8_t width, uint8_t thick);
protected:
private:
	void init();

}; //ssd1289

#endif //__SSD1289_H__
