#include "ssd1289.h"


Ssd1289::Ssd1289()
{
	delay_ms(100);
	driver.rst.set();
	driver.wReg(0x00,0x0001);
	driver.wReg(0x03,0xA8A4);
	driver.wReg(0x0C,0x0000);
	driver.wReg(0x0D,0x080C);
	driver.wReg(0x0E,0x2B00);
	driver.wReg(0x1E,0x00B7);
	driver.wReg(0x01,0x2B3F);
	driver.wReg(0x02,0x0600);
	driver.wReg(0x10,0x0000);
	delay_ms(30);
	driver.wReg(0x11,0x6068);
	driver.wReg(0x05,0x0000);
	driver.wReg(0x06,0x0000);
	driver.wReg(0x16,0xEF1C);
	driver.wReg(0x17,0x0003);
	driver.wReg(0x07,0x0233);
	driver.wReg(0x0B,0x0000);
	driver.wReg(0x0F,0x0000);
	driver.wReg(0x41,0x0000);
	driver.wReg(0x42,0x0000);
	driver.wReg(0x48,0x0000);
	driver.wReg(0x49,0x013F);
	driver.wReg(0x4A,0x0000);
	driver.wReg(0x4B,0x0000);
	driver.wReg(0x44,0xEF00);
	driver.wReg(0x45,0x0000);
	driver.wReg(0x46,0x013F);
	driver.wReg(0x30,0x0707);
	driver.wReg(0x31,0x0204);
	driver.wReg(0x32,0x0204);
	driver.wReg(0x33,0x0502);
	driver.wReg(0x34,0x0507);
	driver.wReg(0x35,0x0204);
	driver.wReg(0x36,0x0204);
	driver.wReg(0x37,0x0502);
	driver.wReg(0x3A,0x0302);
	driver.wReg(0x3B,0x0302);
	driver.wReg(0x23,0x0000);
	driver.wReg(0x24,0x0000);
	driver.wReg(0x25,0x8000);
	driver.wReg(0x4f,0x0000);
	driver.wReg(0x4e,0x0000);
	driver.index(0x22);
}

void Ssd1289::pixel (uint16_t x , uint16_t y, const uint16_t color)
{
	setCursor(x, y);
	driver.index(0x0022);
	driver.data (color);
}

void Ssd1289::fillScreen (uint16_t color)
{
	setCursor(0, 0);
	driver.index(0x0022);
	driver.cs.clear();
	//data
	driver.rs.set();
	driver.low.clear();
	driver.high.clear();
	driver.low.set(color&0x00FF);
	driver.high.set((color&0xFF00)<<8);
	for (uint32_t i=0;i<76800;++i)
	{
		driver.strob();
	}
	driver.cs.set();
}

void Ssd1289::symbol (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const uint8_t ch, sFont & s)
{
	const uint8_t * ptrFont = s.font;
	uint8_t byte = s.width>>3;
	uint8_t pix = s.width%8;
	uint16_t num;
	if (pix)  num = (ch-s.shift)*(byte+1)*s.height;
	else num = (ch-s.shift)*byte*s.height;
	ptrFont += num;
	uint16_t colors [2] = { color, fon};

	for (uint8_t i=0;i<s.height;++i)
	{
		driver.cs.set();
		setCursor(y-i, x);
		driver.index(0x0022);
		driver.cs.clear();
		//data
		driver.rs.set();
		for (uint8_t j=0;j<byte;++j)
		{
			for (int8_t k=7;k>=0;--k)
			{
				bool temp = (*ptrFont)&(1 << k);
				driver.putData(colors [temp]);
			}
			++ptrFont;
		}
		for (int8_t k=7, j=0;j<pix;--k, ++j)
		{
			bool temp = (*ptrFont)&(1 << k);
			driver.putData(colors [temp]);
		}
		if (pix) ++ptrFont;
	}
}

void Ssd1289::string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, sFont &s, uint8_t interval)
{
	uint16_t tempX = x;
	while (*str)
	{
		symbol (tempX, y, color, fon, *str++, s);
		tempX +=  s.width + interval;
	}
}

void Ssd1289::string (uint16_t x, uint16_t y, const uint16_t color, const uint16_t fon, const char *str, sFont &s, uint16_t n, uint8_t interval)
{
	uint16_t tempX = x;
	while (n--)
	{
		symbol (tempX, y, color, fon, *str++, s);
		tempX +=  s.width + interval;
	}
}
void Ssd1289::setCursor (uint16_t x , uint16_t y)
{
	driver.wReg(0x004e, x);
	driver.wReg(0x004f, y);
}

void Ssd1289::drawArr (uint16_t x , uint16_t y, const uint16_t color, uint16_t fon, const uint8_t *arr, uint16_t l, uint16_t width)
{
	uint16_t colors [2] = {fon, color};
	for (uint16_t i=0;i<width;++i)
	{
		driver.cs.set();
		setCursor(y-i, x);
		driver.index(0x0022);
		driver.cs.clear();
		//data
		driver.rs.set();
		for (uint16_t j=0;j<l;++j)
		{
			for (int8_t k=7;k>=0;--k)
			{
				bool temp = (*arr)&(1 << k);
				driver.putData(colors [temp]);
			}
		++arr;
		}
	}
	driver.cs.set();
}

void Ssd1289::drawPic (uint16_t x , uint16_t y, const uint16_t *arr, uint16_t length, uint16_t width)
{
	for (uint8_t i=0;i<width;++i)
	{
		driver.cs.set();
		setCursor(y-i, x);
		driver.index(0x0022);
		driver.cs.clear();
		//data
		driver.rs.set();

		for (uint8_t j=0;j<length;++j)
		{
			driver.putData(*arr++);
		}
	}
	driver.cs.set();
}

void Ssd1289::horLine (uint16_t x, uint16_t y, const uint16_t color, uint16_t length, uint8_t thick)
{

	for (uint16_t i=0;i<thick;++i)
	{
		driver.cs.set();
		setCursor(y+i, x);
		driver.index(0x0022);
		driver.cs.clear();
		//data
		driver.rs.set();
		driver.low.clear();
		driver.high.clear();
		driver.low.set(color&0x00FF);
		driver.high.set((color&0xFF00)<<8);
    		for (uint16_t j=0;j<length;++j)
		{
   			driver.strob();
		}
	}
	driver.cs.set();
}

void Ssd1289::verLine (uint16_t x, uint16_t y, const uint16_t color, uint16_t length, uint8_t thick)
{
	for (uint16_t i=0;i<length;++i)
	{
		driver.cs.set();
		setCursor(y+i, x);
		driver.index(0x0022);
		driver.cs.clear();
		//data
		driver.rs.set();
		driver.low.clear();
		driver.high.clear();
		driver.low.set(color&0x00FF);
		driver.high.set((color&0xFF00)<<8);
		for (uint8_t j=0;j<thick;++j)
		{
			driver.strob();
		}
	}
	driver.cs.set();
}

void Ssd1289::rectangle (uint16_t x, uint16_t y, uint16_t color, uint16_t length, uint8_t width, uint8_t thick)
{
	horLine(x, y,color, length, thick);
	horLine(x, y+width, color, length, thick);
	verLine(x, y, color, width, thick);
	verLine(x+length, y, color, width, thick);
}

