#include "device.h"
#include "delay.h"
#include "spi.h"
#include "pin.h"
#include "dma.h"
#include "font.h"
#include "tftdriver.h"

#ifndef ILI9341_H
#define ILI9341_H

//Commands
#define ILI9341_RESET				0x01
#define ILI9341_SLEEP_OUT			0x11
#define ILI9341_GAMMA				0x26
#define ILI9341_DISPLAY_OFF			0x28
#define ILI9341_DISPLAY_ON			0x29
#define ILI9341_COLUMN_ADDR			0x2A
#define ILI9341_PAGE_ADDR			0x2B
#define ILI9341_GRAM				0x2C
#define ILI9341_MAC			        0x36
#define ILI9341_PIXEL_FORMAT        0x3A
#define ILI9341_WDB			    	0x51
#define ILI9341_WCD				    0x53
#define ILI9341_RGB_INTERFACE       0xB0
#define ILI9341_FRC				    0xB1
#define ILI9341_BPC				    0xB5
#define ILI9341_DFC				    0xB6
#define ILI9341_POWER1				0xC0
#define ILI9341_POWER2				0xC1
#define ILI9341_VCOM1				0xC5
#define ILI9341_VCOM2				0xC7
#define ILI9341_POWERA				0xCB
#define ILI9341_POWERB				0xCF
#define ILI9341_PGAMMA				0xE0
#define ILI9341_NGAMMA				0xE1
#define ILI9341_DTCA				0xE8
#define ILI9341_DTCB				0xEA
#define ILI9341_POWER_SEQ			0xED
#define ILI9341_3GAMMA_EN			0xF2
#define ILI9341_INTERFACE			0xF6
#define ILI9341_PRC				    0xF7

namespace ili9341Commands
{
	const uint8_t softwareReset = 0x01;
	const uint8_t powerControl1 = 0xC0;
	const uint8_t powerControl2 = 0xC1;
	const uint8_t vcomControl1  = 0xC5;
	const uint8_t vcomControl2  = 0xC7;
	const uint8_t frameControl  = 0xB1;
	const uint8_t memryAccessControl = 0x36;
	const uint8_t pixelFormatSet = 0x3A;
	const uint8_t displayFunctionControl = 0xB6;
	const uint8_t sleepOut      = 0x11;
	const uint8_t displayOn     = 0x29;
	const uint8_t coloumnAddressSet = 0x2A;
	const uint8_t pageAddressSet = 0x2B;
	const uint8_t memoryWrite   = 0x2C;

}


class Ili9341
{
//variables
public:

protected:
private:
	Spi * driver;
	//Dma * dma;
	Pin dc, rst;
//functions
public:
	Ili9341(Spi &, port po, uint8_t p, port rstpo, uint8_t rstpi);

	void fillScreen (uint16_t color);
	void setCursor (uint16_t x , uint16_t y);
	void setArea (uint16_t x1 , uint16_t y1, uint16_t x2, uint16_t y2);
protected:
private:
	void data8 (uint8_t);
	void data16 (uint16_t);
	void command (uint8_t);
	void write (uint8_t);
	void init ();
	void setPage (uint16_t x1, uint16_t x2);
	void setColoumn (uint16_t y1, uint16_t y2);
	void setPosition (uint16_t x, uint16_t y);
};

#endif
