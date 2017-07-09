#include "lcdflash.h"


Lcdflash::Lcdflash(Ili9341 &lcd, Flash &mem)
{
	lcdDriver = &lcd;
	memory = &mem;
}

void Lcdflash::drawArr (uint16_t x , uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width)
{
	lcdDriver->drawArr(x, y, color, fon, arr, l, width);
}

void Lcdflash::drawPic (uint16_t x , uint16_t y, const uint16_t *arr, uint16_t length, uint16_t height)
{

}

void Lcdflash::drawPic (uint16_t x , uint16_t y, uint32_t address, uint16_t length, uint16_t height)
{

}

void Lcdflash::symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, Font & s)
{

}

void Lcdflash::string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, Font &f, int8_t interval)
{

}

void Lcdflash::horLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick)
{

}

void Lcdflash::verLine (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t thick)
{

}

void Lcdflash::rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height, uint8_t thick)
{

}

void Lcdflash::rectangle (uint16_t x, uint16_t y, const uint16_t * color, uint16_t length, uint8_t height)
{

}

void Lcdflash::pixel (uint16_t x , uint16_t y, const uint16_t color)
{

}
